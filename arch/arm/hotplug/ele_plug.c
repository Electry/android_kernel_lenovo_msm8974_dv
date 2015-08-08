/*
 * Copyright (c) 2013-2015, Francisco Franco <franciscofranco.1990@gmail.com>,
 *                    2015, Michal Chvíla aka Electry <electrydev@gmail.com>.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/jiffies.h>

#define ELE_PLUG			"ele_plug"
#define ELE_PLUG_MAJOR_VERSION		1
#define ELE_PLUG_MINOR_VERSION		0

#define ELE_PLUG_ENABLED		1
#undef ELE_PLUG_DEBUG

#define DEFAULT_PLUSONE_THRESHOLD	25
#define DEFAULT_PLUSTWO_THRESHOLD	50
#define DEFAULT_PLUSTHREE_THRESHOLD	80

#define DEFAULT_CPUFREQ_UNPLUG_LIMIT	1800000
#define DEFAULT_MIN_CPU_ONLINE_COUNTER	10

/* Careful with this value */
#define DEFAULT_TIMER			1

extern unsigned int get_rq_info(void);

struct cpu_stats {
	unsigned int counter;
	u64 timestamp;
} stats = {
	.counter = 0,
	.timestamp = 0,
};

struct hotplug_tunables {
	/*
	 * Enable/Disable Hotplug Work
	 */
	unsigned int enabled;

	/*
	 * system load threshold to decide when online or offline one core
	 * (from 0 to 100)
	 */
	unsigned int plusone_threshold;

	/*
	 * system load threshold to decide when online two cores at once
	 * (from 0 to 100)
	 */
	unsigned int plustwo_threshold;

	/*
	 * system load threshold to decide when online three cores at once
	 * (from 0 to 100)
	 */
	unsigned int plusthree_threshold;

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
	 * sample timer in seconds. The default value of 1 equals to 10
	 * samples every second. The higher the value the less samples
	 * per second it runs
	 */
	unsigned int timer;
} tunables;

static struct workqueue_struct *wq;
static struct delayed_work decide_hotplug;

/*
 * Plug x cores (eg. cpu_plug(2) will online 2 cores (if possible) )
 */
static void __cpuinit cpu_plug(unsigned int x)
{
	unsigned int cpu;

#ifdef ELE_PLUG_DEBUG
	pr_info("%s: onlining %d cores\n", ELE_PLUG, x);
#endif

	for (cpu = 1; cpu < 4; cpu++) {
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

#ifdef ELE_PLUG_DEBUG
	pr_info("%s: offlining %d cores\n", ELE_PLUG, x);
#endif

	for (cpu = 3; cpu > 0; cpu--) {
		if (cpu_online(cpu)) {
			cpu_down(cpu);
			x--;
		}

		if ( x == 0 )
			break;
	}
}


/*
 * Returns true if average of (1-3cores) freqs is >= than cpufreq_unplug_limit
 */
static inline bool cpus_freq_overlimit(void)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	struct hotplug_tunables *t = &tunables;
	unsigned int current_freq = 0;
	unsigned int cpu;

	if (policy) {
		if (policy->min >= t->cpufreq_unplug_limit)
			return false;
	}

	for (cpu = 1; cpu < 4; cpu++)
		current_freq += cpufreq_quick_get(cpu);

	current_freq /= 3;

	return current_freq >= t->cpufreq_unplug_limit;
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

	if (cur_load >= t->plusthree_threshold) {
		if (online_cpus < 4)
			cpu_plug(3);

	} else if (cur_load >= t->plustwo_threshold) {
		if (online_cpus < 4)
			cpu_plug(2);

	} else if (cur_load >= t->plusone_threshold) {
		if (online_cpus < 4)
			cpu_plug(1);
	} else {
		if (online_cpus > 1 && !cpus_freq_overlimit() && stats.counter > t->min_cpu_online_counter) {
			if (online_cpus <= 2) //Last unplug -> reset counter
				stats.counter = 0;

			cpu_unplug(1);
		}
	}

	if (t->enabled)
		queue_delayed_work(wq, &decide_hotplug,
			msecs_to_jiffies(t->timer * HZ));

	return;
}

/*
 * Sysfs get/set entries start
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

static ssize_t plusone_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hotplug_tunables *t = &tunables;

	return snprintf(buf, 10, "%u\n", t->plusone_threshold);
}

static ssize_t plusone_threshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct hotplug_tunables *t = &tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	t->plusone_threshold = new_val > 100 ? 100 : new_val;

	return size;
}

static ssize_t plustwo_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hotplug_tunables *t = &tunables;

	return snprintf(buf, 10, "%u\n", t->plustwo_threshold);
}

static ssize_t plustwo_threshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct hotplug_tunables *t = &tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	t->plustwo_threshold = new_val > 100 ? 100 : new_val;

	return size;
}

static ssize_t plusthree_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hotplug_tunables *t = &tunables;

	return snprintf(buf, 10, "%u\n", t->plusthree_threshold);
}

static ssize_t plusthree_threshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct hotplug_tunables *t = &tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	t->plusthree_threshold = new_val > 100 ? 100 : new_val;

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

	t->timer = new_val > 100 ? 100 : new_val;

	return size;
}

static DEVICE_ATTR(enabled, 0664, enabled_show, enabled_store);
static DEVICE_ATTR(plusone_threshold, 0664, plusone_threshold_show,
		plusone_threshold_store);
static DEVICE_ATTR(plustwo_threshold, 0664, plustwo_threshold_show,
		plustwo_threshold_store);
static DEVICE_ATTR(plusthree_threshold, 0664, plusthree_threshold_show,
		plusthree_threshold_store);
static DEVICE_ATTR(min_cpu_online_counter, 0664, min_cpu_online_counter_show,
		min_cpu_online_counter_store);
static DEVICE_ATTR(cpufreq_unplug_limit, 0664, cpufreq_unplug_limit_show,
		cpufreq_unplug_limit_store);
static DEVICE_ATTR(timer, 0664, timer_show, timer_store);

static struct attribute *ele_plug_control_attributes[] = {
	&dev_attr_enabled.attr,
	&dev_attr_plusone_threshold.attr,
	&dev_attr_plustwo_threshold.attr,
	&dev_attr_plusthree_threshold.attr,
	&dev_attr_min_cpu_online_counter.attr,
	&dev_attr_cpufreq_unplug_limit.attr,
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
 * Sysfs get/set entries end
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
	t->plusone_threshold = DEFAULT_PLUSONE_THRESHOLD;
	t->plustwo_threshold = DEFAULT_PLUSTWO_THRESHOLD;
	t->plusthree_threshold = DEFAULT_PLUSTHREE_THRESHOLD;
	t->min_cpu_online_counter = DEFAULT_MIN_CPU_ONLINE_COUNTER;
	t->cpufreq_unplug_limit = DEFAULT_CPUFREQ_UNPLUG_LIMIT;
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

	if (t->enabled)
		queue_delayed_work(wq, &decide_hotplug, HZ * 30);
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

static int __init ele_plug_init(void)
{
	int ret;

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

	pr_info("%s: init\n", ELE_PLUG);
	pr_info("%s: version %d.%d by Electry\n",
		 ELE_PLUG,
		 ELE_PLUG_MAJOR_VERSION,
		 ELE_PLUG_MINOR_VERSION);

	return ret;
}

static void __exit ele_plug_exit(void)
{
	platform_device_unregister(&ele_plug_device);
	platform_driver_unregister(&ele_plug_driver);
}

late_initcall(ele_plug_init);
module_exit(ele_plug_exit);

MODULE_AUTHOR("Michal Chvíla aka Electry <electrydev@gmail.com>");
MODULE_DESCRIPTION("Ele-Plug Hotplug Driver");
MODULE_LICENSE("GPLv2");
