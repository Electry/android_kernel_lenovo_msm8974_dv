/*
 * arch/arm/hotplug/ele_plug.c
 *
 * Custom Hotplug driver for Quad-Core ARM Symmetric Multiprocessor SoCs
 *
 * Features:
 * - CPU cores auto (un)plugging based on system load
 * - WQ Suspension during screen-off state (only if CONFIG_FB)
 * - Extensive sysfs tuneables
 *
 * Copyright (c) 2015, Michal Chvila (Electry) <electrydev@gmail.com>.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#ifdef CONFIG_FB
#include <linux/fb.h>
#endif

#define ELE_PLUG			"ele_plug"
#define ELE_PLUG_MAJOR_VERSION		1
#define ELE_PLUG_MINOR_VERSION		6

#define ELE_PLUG_ENABLED		1

//#define ELE_PLUG_DEBUG 1
#undef ELE_PLUG_DEBUG

#define DEFAULT_ALIVE_THRESHOLD		10
#define DEFAULT_LOAD_THRESHOLD		20
#define DEFAULT_SPIKE_THRESHOLD		70
#define DEFAULT_MAX_CORES		4

#define DEFAULT_CPUFREQ_UNPLUG_LIMIT	1800000
#define DEFAULT_MIN_CPU_ONLINE_COUNTER	8 // [ticks] =2sec

#define DEFAULT_TIMER			250

extern unsigned int get_rq_info(void);

struct cpu_stats {
	unsigned int counter;
} stats = {
	.counter = 0,
};

struct hotplug_tunables {
	/*
	 * Enable/Disable Hotplug Work
	 */
	unsigned int enabled;

	/*
	 * system load threshold to decide when no un/plugging is needed
	 * (from 0 to 100)
	 */
	unsigned int alive_threshold;

	/*
	 * system load threshold to decide when online or offline one core
	 * (from 0 to 100)
	 */
	unsigned int load_threshold;

	/*
	 * system load threshold to decide when online all cores at once
	 * (from 0 to 100)
	 */
	unsigned int spike_threshold;

	/*
	 * minimum time in samples that cores have to stay online before they
	 * can be unplugged
	 */
	unsigned int min_cpu_online_counter;

	/*
	 * if the current CPU freq is above this limit don't offline the cores
	 * during current sample
	 */
	unsigned int cpufreq_unplug_limit;

	/*
	 * Number of maximum cores online
	 */
	unsigned int max_cores;

	/*
	 * sample timer in milliseconds. The default value of 250 equals to 4
	 * samples every second. The higher the value the less samples
	 * per second it runs
	 */
	unsigned int timer;
} tunables;

unsigned int hotplug_state = 1;

static struct workqueue_struct *wq;
static struct delayed_work decide_hotplug;
#ifdef CONFIG_FB
static struct notifier_block fb_notifier;
#endif

/*
 * Plug x cores (eg. cpu_plug(2) will online 2 cores (if possible) )
 */
static void __cpuinit cpu_plug(unsigned int x)
{
	unsigned int cpu;
	struct hotplug_tunables *t = &tunables;

#ifdef ELE_PLUG_DEBUG
	pr_info("%s: onlining %d cores\n", ELE_PLUG, x);
#endif

	for (cpu = 1; cpu < t->max_cores; cpu++) {
		if (cpu_is_offline(cpu)) {
			cpu_up(cpu);
			x--;
		}

		if ( x == 0 )
			break;
	}
}

/*
 * Unplug x cores (eg. cpu_unplug(2) will offline 2 cores (if possible) )
 */
static void __cpuinit cpu_unplug(unsigned int x)
{
	unsigned int cpu;
	struct hotplug_tunables *t = &tunables;

#ifdef ELE_PLUG_DEBUG
	pr_info("%s: offlining %d cores\n", ELE_PLUG, x);
#endif

	for (cpu = t->max_cores-1; cpu > 0; cpu--) {
		if (cpu_online(cpu)) {
			cpu_down(cpu);
			x--;
		}

		if ( x == 0 )
			break;
	}
}


/*
 * Returns true if average of (1-3cores) freqs is > than cpufreq_unplug_limit
 */
static inline bool cpus_freq_overlimit(void)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	struct hotplug_tunables *t = &tunables;
	unsigned int current_freq = 0;
	unsigned int cpu;

	if (policy) {
		if (policy->min > t->cpufreq_unplug_limit)
			return false;
	}

	for (cpu = 1; cpu < t->max_cores; cpu++)
		current_freq += cpufreq_quick_get(cpu);

	current_freq /= t->max_cores-1;

	return current_freq > t->cpufreq_unplug_limit;
}

static void __ref decide_hotplug_func(struct work_struct *work)
{
	struct hotplug_tunables *t = &tunables;
	unsigned int cur_load = 0;
	unsigned int cpu;
	unsigned int online_cpus = num_online_cpus();

	cur_load = get_rq_info();

	if (online_cpus > 1)
		stats.counter++;

#ifdef ELE_PLUG_DEBUG
	pr_info("%s: LOAD %d, ACTIVE CORES %d, COUNTER %d\n", ELE_PLUG, cur_load, online_cpus, stats.counter);
#endif

	if (unlikely(!hotplug_state)) {
		/* Suspending, Unplug all cores */
		cpu_unplug(3);
		stats.counter = 0; //Reset

	} else if (cur_load >= t->spike_threshold) {
		/* Plug all cores */
		if (online_cpus < t->max_cores)
			cpu_plug(t->max_cores-1);

	} else if (cur_load >= t->load_threshold) {
		/* Plug one more core */
		if (online_cpus < t->max_cores)
			cpu_plug(1);

	} else if (cur_load < t->alive_threshold
		   && online_cpus > 1
		   && !cpus_freq_overlimit()
		   && stats.counter > t->min_cpu_online_counter) {

			if (online_cpus < 3) //Last unplug possible (online == 2 [1 after this unplug]) -> reset counter
				stats.counter = 0;

			cpu_unplug(1);
	}

	if (likely(t->enabled) && likely(hotplug_state))
		queue_delayed_work(wq, &decide_hotplug,
			msecs_to_jiffies(t->timer));

	return;
}

static void hotplug_suspend(void)
{
	pr_info("%s: Screen off. Suspending.\n", ELE_PLUG);

	/* Suspend hotplug state */
	hotplug_state = 0;
}

static void hotplug_resume(void)
{
	pr_info("%s: Screen on. Resuming.\n", ELE_PLUG);

	/* Resume hotplug state */
	hotplug_state = 1;

	/* Resume main work thread */
	queue_delayed_work(wq, &decide_hotplug, 0);
}


/*
 * Start of FrameBuffer handling
 */
#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *this,
				unsigned long event, void *data)
{
	int blank_mode;

	if (event != FB_EVENT_BLANK || data == NULL)
		return 0;

	blank_mode = *(int*)(((struct fb_event*)data)->data);

#ifdef ELE_PLUG_DEBUG
	pr_info("%s: FB_CB: event = %lu, blank mode = %d\n", ELE_PLUG, event, blank_mode);
#endif

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		hotplug_resume();
		break;
	case FB_BLANK_POWERDOWN:
		hotplug_suspend();
		break;
	default:
		break;
	}

	return 0;
}
#endif
/*
 * End of FrameBuffer handling
 */

/*
 * Start of sysfs get/set entries
 */
static ssize_t enabled_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct hotplug_tunables *t = &tunables;

	return snprintf(buf, 10, "%u\n", t->enabled);
}

static ssize_t enabled_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct hotplug_tunables *t = &tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	/* Handle disabled->enabled and enabled->disabled change */
	if (!t->enabled && new_val) {
		/* Queue hotplug work 1s after enabling */
		queue_delayed_work(wq, &decide_hotplug, HZ);
	} else if (t->enabled && !new_val) {
		/* Flush existing hotplug work when disabling */
		flush_workqueue(wq);
		cancel_delayed_work_sync(&decide_hotplug);
	}
	t->enabled = new_val > 1 ? 1 : 0;
	return size;
}

static ssize_t alive_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hotplug_tunables *t = &tunables;

	return snprintf(buf, 10, "%u\n", t->alive_threshold);
}

static ssize_t alive_threshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct hotplug_tunables *t = &tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	t->alive_threshold = new_val > 100 ? 100 : new_val;

	return size;
}

static ssize_t load_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hotplug_tunables *t = &tunables;

	return snprintf(buf, 10, "%u\n", t->load_threshold);
}

static ssize_t load_threshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct hotplug_tunables *t = &tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	t->load_threshold = new_val > 100 ? 100 : new_val;

	return size;
}

static ssize_t spike_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hotplug_tunables *t = &tunables;

	return snprintf(buf, 10, "%u\n", t->spike_threshold);
}

static ssize_t spike_threshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct hotplug_tunables *t = &tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	t->spike_threshold = new_val > 100 ? 100 : new_val;

	return size;
}

static ssize_t min_cpu_online_counter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hotplug_tunables *t = &tunables;

	return snprintf(buf, 10, "%u\n", t->min_cpu_online_counter);
}

static ssize_t min_cpu_online_counter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct hotplug_tunables *t = &tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	/* Max = 100 */
	t->min_cpu_online_counter = new_val > 100 ? 100 : new_val;

	return size;
}

static ssize_t cpufreq_unplug_limit_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hotplug_tunables *t = &tunables;

	return snprintf(buf, 10, "%u\n", t->cpufreq_unplug_limit);
}

static ssize_t cpufreq_unplug_limit_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct hotplug_tunables *t = &tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	t->cpufreq_unplug_limit = new_val > ULONG_MAX ? ULONG_MAX : new_val;

	return size;
}

static ssize_t max_cores_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hotplug_tunables *t = &tunables;

	return snprintf(buf, 10, "%u\n", t->max_cores);
}

static ssize_t max_cores_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct hotplug_tunables *t = &tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	if (new_val > 4)
		new_val = 4;
	else if (new_val < 1)
		new_val = 1;

	t->max_cores = new_val;

	return size;
}

static ssize_t timer_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct hotplug_tunables *t = &tunables;

	return snprintf(buf, 10, "%u\n", t->timer);
}

static ssize_t timer_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct hotplug_tunables *t = &tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	t->timer = new_val > 2000 ? 2000 : new_val;

	return size;
}

static DEVICE_ATTR(enabled, 0664, enabled_show, enabled_store);
static DEVICE_ATTR(alive_threshold, 0664, alive_threshold_show,
		alive_threshold_store);
static DEVICE_ATTR(load_threshold, 0664, load_threshold_show,
		load_threshold_store);
static DEVICE_ATTR(spike_threshold, 0664, spike_threshold_show,
		spike_threshold_store);
static DEVICE_ATTR(min_cpu_online_counter, 0664, min_cpu_online_counter_show,
		min_cpu_online_counter_store);
static DEVICE_ATTR(cpufreq_unplug_limit, 0664, cpufreq_unplug_limit_show,
		cpufreq_unplug_limit_store);
static DEVICE_ATTR(max_cores, 0664, max_cores_show,
		max_cores_store);
static DEVICE_ATTR(timer, 0664, timer_show, timer_store);

static struct attribute *ele_plug_control_attributes[] = {
	&dev_attr_enabled.attr,
	&dev_attr_alive_threshold.attr,
	&dev_attr_load_threshold.attr,
	&dev_attr_spike_threshold.attr,
	&dev_attr_min_cpu_online_counter.attr,
	&dev_attr_cpufreq_unplug_limit.attr,
	&dev_attr_max_cores.attr,
	&dev_attr_timer.attr,
	NULL
};

static struct attribute_group ele_plug_control_group = {
	.attrs  = ele_plug_control_attributes,
};

static struct miscdevice ele_plug_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ele_plug_control",
};

/*
 * End of sysfs get/set entries
 */

static int ele_plug_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct hotplug_tunables *t = &tunables;

	wq = alloc_workqueue("ele_plug_workqueue",
		WQ_FREEZABLE |
		WQ_UNBOUND, 1);

	if (!wq) {
		ret = -ENOMEM;
		goto err;
	}

	t->enabled = ELE_PLUG_ENABLED;
	t->alive_threshold = DEFAULT_ALIVE_THRESHOLD;
	t->load_threshold = DEFAULT_LOAD_THRESHOLD;
	t->spike_threshold = DEFAULT_SPIKE_THRESHOLD;
	t->min_cpu_online_counter = DEFAULT_MIN_CPU_ONLINE_COUNTER;
	t->cpufreq_unplug_limit = DEFAULT_CPUFREQ_UNPLUG_LIMIT;
	t->max_cores = DEFAULT_MAX_CORES;
	t->timer = DEFAULT_TIMER;

	ret = misc_register(&ele_plug_control_device);
	if (ret) {
		ret = -EINVAL;
		goto err;
	}

	ret = sysfs_create_group(&ele_plug_control_device.this_device->kobj,
			&ele_plug_control_group);
	if (ret) {
		ret = -EINVAL;
		goto err;
	}

	INIT_DELAYED_WORK(&decide_hotplug, decide_hotplug_func);

	if (t->enabled) {
		cpu_down(3); //Reset
		queue_delayed_work(wq, &decide_hotplug, 0);
	}
err:
	return ret;
}

static struct platform_device ele_plug_device = {
	.name = ELE_PLUG,
	.id = -1,
};

static int ele_plug_remove(struct platform_device *pdev)
{
	destroy_workqueue(wq);

	return 0;
}

static struct platform_driver ele_plug_driver = {
	.probe = ele_plug_probe,
	.remove = ele_plug_remove,
	.driver = {
		.name = ELE_PLUG,
		.owner = THIS_MODULE,
	},
};

/*
 * Initialization
 */
static int __init ele_plug_init(void)
{
	int ret;

	pr_info("%s: init\n", ELE_PLUG);
	pr_info("%s: version %d.%d by Electry\n",
		 ELE_PLUG,
		 ELE_PLUG_MAJOR_VERSION,
		 ELE_PLUG_MINOR_VERSION);

	ret = platform_driver_register(&ele_plug_driver);
	if (ret) {
		pr_err("%s: Driver register failed: $d\n", ELE_PLUG, ret);
		return ret;
	}

	ret = platform_device_register(&ele_plug_device);
	if (ret) {
		pr_err("%s: Device register failed: $d\n", ELE_PLUG, ret);
		return ret;
	}

#ifdef CONFIG_FB
	fb_notifier.notifier_call = fb_notifier_callback;
	if (fb_register_client(&fb_notifier) != 0) {
		pr_err("%s: FB callback register failed\n", ELE_PLUG);
			return -EINVAL;
	}
#endif

	pr_info("%s: init complete\n", ELE_PLUG);

	return ret;
}

static void __exit ele_plug_exit(void)
{
	pr_info("%s: exiting\n", ELE_PLUG);

	platform_device_unregister(&ele_plug_device);
	platform_driver_unregister(&ele_plug_driver);
}

late_initcall(ele_plug_init);
module_exit(ele_plug_exit);

MODULE_AUTHOR("Michal Chvila (Electry) <electrydev@gmail.com>");
MODULE_DESCRIPTION("Ele_Plug Hotplug Driver");
MODULE_LICENSE("GPLv2");
