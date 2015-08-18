/*
 * drivers/thermal/ele_thermal.c
 *
 * Custom Thermal throttle driver for Quad-Core SoCs
 *
 * Features:
 * - CPU Frequency throttling based on thermal zone readings
 * - Configurable frequency stepping (up/down)
 * - Emergency shutdown on device overheat (default 100C)
 * - Extensive sysfs tuneables
 *
 * Copyright (c) 2015, Michal Chvila aka Electry <electrydev@gmail.com>.
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
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/msm_tsens.h>
#include <linux/jiffies.h>

#define ELE_THERMAL			"ele_thermal"
#define ELE_THERMAL_MAJOR_VERSION	1
#define ELE_THERMAL_MINOR_VERSION	0

#define ELE_THERMAL_ENABLED		1

//#define ELE_THERMAL_DEBUG 1
#undef ELE_THERMAL_DEBUG

#define DEFAULT_TIMER			250
#define DEFAULT_TEMP_HIGH		75
#define DEFAULT_TEMP_HYSTERESIS		5
#define DEFAULT_FREQ_STEPDOWN		1
#define DEFAULT_FREQ_STEPUP		1
#define DEFAULT_FREQ_LAZY		2 //0.5sec

unsigned int thermal_zones[4] = { 5, 6, 7, 8 };

unsigned int thermal_min_freq_id = 8;

unsigned int thermal_temp_shutdown = 100;

/* Spent time-in-temp monitoring */
//unsigned int monitor_temp_start = 0;
//unsigned int monitor_temp_count = 99;

struct thermal_tunables {
	unsigned int enabled;
	unsigned int timer;
	unsigned int temp_high;
	unsigned int temp_hysteresis;
	unsigned int freq_stepdown;
	unsigned int freq_stepup;
	unsigned int freq_lazy;
} therm_tunables;

struct cpu_info_struct {
	uint32_t device_max_freq;
	uint32_t user_max_freq;
	uint32_t limited_max_freq;
	unsigned int lazy_counter;
	unsigned int throttling;
//	uint32_t monitor_secs_in_state[monitor_temp_count];
} cpu_info;

enum thermal_state {
	STATE_THROTTLE_NOW,
	STATE_RESUME_NOW,
	STATE_THROTTLE_LAZY,
	STATE_RESUME_LAZY,
	STATE_KEEP,
	STATE_SHUTDOWN,
};

static struct workqueue_struct *wq;
static struct delayed_work thermal_tick;

static struct cpufreq_frequency_table *freq_table;

static DEFINE_MUTEX(emergency_shutdown_mutex);

/*
 * Get temp of specified zone
 */
static int thermal_get_temp(uint32_t zone, long *ttemp)
{
	int ret = 0;
	struct tsens_device tsens_dev;

	if (!ttemp) {
		pr_err("%s: Invalid *ttemp value\n", ELE_THERMAL);
		ret = -EINVAL;
		goto get_temp_exit;
	}

	tsens_dev.sensor_num = zone;

	ret = tsens_get_temp(&tsens_dev, ttemp);
	if (ret) {
		pr_err("%s: Unable to read TSENS sensor %d\n",
			ELE_THERMAL,
			tsens_dev.sensor_num);
		goto get_temp_exit;
	}

get_temp_exit:
	return ret;
}

/*
 * Returns highest measured cpu temp out of 4 zones specified
 */
static int thermal_get_cpu_temp(uint32_t cpu_zones[4])
{
	int temp = -99; //Start with way-off temp
	int cpu;

	long ttemp = 0; //Temporary temp
	int ret = 0;

	for (cpu = 0; cpu < 4; cpu++) {
		/* Read thermal temp */
		ret = thermal_get_temp(cpu_zones[cpu], &ttemp);
		if (ret) {
			pr_info("%s: Unable to read TSENS sensor %d\n", ELE_THERMAL, cpu_zones[cpu]);
			return temp;
		}

		if (ttemp > temp)
			temp = ttemp;
	}

	return temp;
}

/*
 * Read and store frequency table (called in _probe)
 */
static int thermal_get_freq_table(void)
{
	int i = 0;

	freq_table = cpufreq_frequency_get_table(0); //cpu0

	if (freq_table == NULL) {
		pr_info("%s: Error reading cpufreq table\n", ELE_THERMAL);
		return -EINVAL;
	}

	while (freq_table[i].frequency != CPUFREQ_TABLE_END) {
		i++;
	}

	/* Store device maximum frequency */
	cpu_info.device_max_freq = freq_table[i-1].frequency;

	pr_info("%s: CPUFreq table loaded, min=%d, max=%d\n", ELE_THERMAL, 
				freq_table[0].frequency, freq_table[i-1].frequency);

	return;
}

/*
 * Return frequency ID (in the freq_table) by frequency (loop)
 */
static int thermal_get_freq_id(long freq)
{
	int i = 0;

	while (freq_table[i].frequency != CPUFREQ_TABLE_END) {
		if (freq_table[i].frequency == freq)
			return i;
		i++;
	}

	return -1;
}

/*
 * Return state (action to do)
 */
enum thermal_state thermal_get_state(int temp)
{
	struct thermal_tunables *t = &therm_tunables;

	if (temp > thermal_temp_shutdown) {

		return STATE_SHUTDOWN;

	} else if (temp >= t->temp_high &&
		   cpu_info.lazy_counter < t->freq_lazy) {

		return STATE_THROTTLE_LAZY;

	} else if (temp >= t->temp_high &&
		   cpu_info.lazy_counter >= t->freq_lazy) {

		return STATE_THROTTLE_NOW;

	} else if (temp <= t->temp_high - t->temp_hysteresis &&
		   cpu_info.throttling &&
		   cpu_info.lazy_counter < t->freq_lazy) {

		return STATE_RESUME_LAZY;

	} else if (temp <= t->temp_high - t->temp_hysteresis &&
		   cpu_info.throttling &&
		   cpu_info.lazy_counter >= t->freq_lazy) {

		return STATE_RESUME_NOW;

	}

	return STATE_KEEP;
}

/*
 * Update max frequency of all cores
 */
static void thermal_update_max_freq(long freq)
{
	/* Update max_freq for all cores */
	int cpu;
	struct cpufreq_policy *policy = NULL;

	for_each_possible_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy) {
       			pr_debug("%s: NULL policy on cpu %d\n", ELE_THERMAL, cpu);
        		continue;
       		}

		cpufreq_verify_within_limits(policy, policy->min, freq);

		policy->user_policy.max = freq;

		cpufreq_update_policy(cpu);
		cpufreq_cpu_put(policy);
	}
}

/*
 * Power off the device now
 */
static void thermal_shutdown_now(int temp) 
{
	mutex_lock(&emergency_shutdown_mutex);
        pr_warn("###############################\n");
        pr_warn("###############################\n");
        pr_warn("-   OVERHEAT! SHUTTING DOWN!  -\n");
        pr_warn("-        cur temp: %dC        -\n", temp);
        pr_warn("###############################\n");
        pr_warn("###############################\n");
	/* orderly poweroff tries to power down gracefully
           if it fails it will force it. */
        orderly_poweroff(true);
	mutex_unlock(&emergency_shutdown_mutex);
}

/*
 * Thermal Tick
 */
static void __ref thermal_tick_func(struct work_struct *work)
{
	int temp = -99;
	long cur_freq_id;
	long target_freq_id;
	struct thermal_tunables *t = &therm_tunables;
	enum thermal_state state = STATE_KEEP;

	struct cpufreq_policy *policy = NULL;

	/* Get highest CPU temp */
	temp = thermal_get_cpu_temp(thermal_zones);
	if (unlikely(temp == -99)) {
		pr_err("%s: Unable to retreive valid CPU temperature\n", ELE_THERMAL);
		goto delay;
	}

#ifdef ELE_THERMAL_DEBUG
	policy = cpufreq_cpu_get(0); //cpu0
	pr_info("%s: temp %dC, limit %dkHz, cpu0 %dkHz\n", ELE_THERMAL, temp, cpu_info.limited_max_freq, policy->cur);
#endif

	/* Get thermal state */
	state = thermal_get_state(temp);
	switch(state) {
		case STATE_THROTTLE_NOW:
			cur_freq_id = thermal_get_freq_id(cpu_info.limited_max_freq);
			if (cur_freq_id <= thermal_min_freq_id)
				break; //We are too low

			if (!cpu_info.throttling) {
				policy = cpufreq_cpu_get(0); //cpu0
				cpu_info.user_max_freq = policy->max; //First throttle? Save user_max_freq then
			}

			cpu_info.throttling = 1;

			target_freq_id = cur_freq_id - t->freq_stepdown;
			cpu_info.limited_max_freq = freq_table[target_freq_id].frequency;

			/* In case limited_max_freq resulted in lower freq than thermal_min_freq (due to freq_stepdown diff)*/
			if (cpu_info.limited_max_freq < freq_table[thermal_min_freq_id].frequency)
				cpu_info.limited_max_freq = freq_table[thermal_min_freq_id].frequency;

			pr_info("%s: Thermal throttle @%dC, max_freq %dkHz\n", ELE_THERMAL, temp, cpu_info.limited_max_freq);

			thermal_update_max_freq(cpu_info.limited_max_freq);

			cpu_info.lazy_counter = 0; //reset

			break;
		case STATE_RESUME_NOW:
			cur_freq_id = thermal_get_freq_id(cpu_info.limited_max_freq);
			if (cur_freq_id >= thermal_get_freq_id(cpu_info.user_max_freq)) {
				break; //We are already at top freq, no need for resume
			}

			target_freq_id = cur_freq_id + t->freq_stepup;
			cpu_info.limited_max_freq = freq_table[target_freq_id].frequency;

			/* In case limited_max_freq resulted in higher freq than user_max_freq (due to freq_stepup diff)*/
			if (cpu_info.limited_max_freq > cpu_info.user_max_freq)
				cpu_info.limited_max_freq = cpu_info.user_max_freq;

			pr_info("%s: Thermal resume @%dC, max_freq %dkHz\n", ELE_THERMAL, temp, cpu_info.limited_max_freq);

			thermal_update_max_freq(cpu_info.limited_max_freq);

			/* If this was last resume, reset throttling to 0 */
			if (cpu_info.limited_max_freq == cpu_info.user_max_freq) {
				cpu_info.throttling = 0;
			}

			cpu_info.lazy_counter = 0; //reset

			break;
		case STATE_THROTTLE_LAZY:
			cpu_info.lazy_counter++;

			break;
		case STATE_RESUME_LAZY:
			cpu_info.lazy_counter++;

			break;
		case STATE_SHUTDOWN:
			thermal_shutdown_now(temp);

			break;
		default: break;

	}


	/* Next tick */
	goto delay;

	return;
delay:
	if (likely(t->enabled))
		queue_delayed_work(wq, &thermal_tick,
			msecs_to_jiffies(t->timer));
	return;
}


/*
 * Sysfs Entries
 */

/* t->enabled */
static ssize_t sysfs_enabled_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct thermal_tunables *t = &therm_tunables;

	return snprintf(buf, 10, "%u\n", t->enabled);
}
static ssize_t sysfs_enabled_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct thermal_tunables *t = &therm_tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	/* Handle disabled->enabled and enabled->disabled change */
	if (!t->enabled && new_val) {
		/* Queue work immediately */
		queue_delayed_work(wq, &thermal_tick, 0);

		pr_info("%s: Thermal control enabled!\n", ELE_THERMAL);
	} else if (t->enabled && !new_val) {
		/* Flush existing work when disabling */
		flush_workqueue(wq);
		cancel_delayed_work_sync(&thermal_tick);

		pr_info("%s: Thermal control disabled! Your device might get hot!\n", ELE_THERMAL);
	}
	t->enabled = new_val > 0 ? 1 : 0;
	return size;
}

/* t->timer */
static ssize_t sysfs_timer_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct thermal_tunables *t = &therm_tunables;

	return snprintf(buf, 10, "%u\n", t->timer);
}
static ssize_t sysfs_timer_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct thermal_tunables *t = &therm_tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);
	if (ret < 0)
		return ret;

	t->timer = new_val > 500 ? 500 : new_val; //0.5sec max

	return size;
}

/* t->temp_high */
static ssize_t sysfs_temp_high_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct thermal_tunables *t = &therm_tunables;

	return snprintf(buf, 10, "%u\n", t->temp_high);
}
static ssize_t sysfs_temp_high_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct thermal_tunables *t = &therm_tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);

	if (ret < 0)
		return ret;

	// 0C - 90C
	if (new_val > 90 || new_val < 0)
		return size;

	t->temp_high = new_val;

	return size;
}

/* t->temp_hysteresis */
static ssize_t sysfs_temp_hysteresis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct thermal_tunables *t = &therm_tunables;

	return snprintf(buf, 10, "%u\n", t->temp_hysteresis);
}
static ssize_t sysfs_temp_hysteresis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct thermal_tunables *t = &therm_tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);

	if (ret < 0)
		return ret;

	// 1C - 20C
	if (new_val > 20 || new_val < 1)
		return size;

	t->temp_hysteresis = new_val;

	return size;
}

/* t->freq_stepdown */
static ssize_t sysfs_freq_stepdown_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct thermal_tunables *t = &therm_tunables;

	return snprintf(buf, 10, "%u\n", t->freq_stepdown);
}
static ssize_t sysfs_freq_stepdown_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct thermal_tunables *t = &therm_tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);

	if (ret < 0)
		return ret;

	// 1 - 4 steps
	if (new_val > 4 || new_val < 1)
		return size;

	t->freq_stepdown = new_val;

	return size;
}

/* t->freq_stepup */
static ssize_t sysfs_freq_stepup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct thermal_tunables *t = &therm_tunables;

	return snprintf(buf, 10, "%u\n", t->freq_stepup);
}
static ssize_t sysfs_freq_stepup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct thermal_tunables *t = &therm_tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);

	if (ret < 0)
		return ret;

	// 1 - 4 steps
	if (new_val > 4 || new_val < 1)
		return size;

	t->freq_stepup = new_val;

	return size;
}

/* t->freq_lazy */
static ssize_t sysfs_freq_lazy_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct thermal_tunables *t = &therm_tunables;

	return snprintf(buf, 10, "%u\n", t->freq_lazy);
}
static ssize_t sysfs_freq_lazy_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct thermal_tunables *t = &therm_tunables;
	int ret;
	unsigned long new_val;

	ret = kstrtoul(buf, 0, &new_val);

	if (ret < 0)
		return ret;

	// 0 - 4 steps
	if (new_val > 4 || new_val < 0)
		return size;

	t->freq_lazy = new_val;

	return size;
}

/*
 * Control device
 */
static DEVICE_ATTR(enabled, 0664, sysfs_enabled_show, sysfs_enabled_store);
static DEVICE_ATTR(timer, 0664, sysfs_timer_show, sysfs_timer_store);
static DEVICE_ATTR(temp_high, 0664, sysfs_temp_high_show, sysfs_temp_high_store);
static DEVICE_ATTR(temp_hysteresis, 0664, sysfs_temp_hysteresis_show, sysfs_temp_hysteresis_store);
static DEVICE_ATTR(freq_stepdown, 0664, sysfs_freq_stepdown_show, sysfs_freq_stepdown_store);
static DEVICE_ATTR(freq_stepup, 0664, sysfs_freq_stepup_show, sysfs_freq_stepup_store);
static DEVICE_ATTR(freq_lazy, 0664, sysfs_freq_lazy_show, sysfs_freq_lazy_store);

static struct attribute *ele_thermal_control_attributes[] = {
	&dev_attr_enabled.attr,
	&dev_attr_timer.attr,
	&dev_attr_temp_high.attr,
	&dev_attr_temp_hysteresis.attr,
	&dev_attr_freq_stepdown.attr,
	&dev_attr_freq_stepup.attr,
	&dev_attr_freq_lazy.attr,
	NULL
};

static struct attribute_group ele_thermal_control_group = {
	.attrs  = ele_thermal_control_attributes,
};

static struct miscdevice ele_thermal_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ele_thermal_control",
};

/*
 * Driver
 */
static int ele_thermal_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct thermal_tunables *t = &therm_tunables;
	struct cpufreq_policy *policy;

	wq = alloc_workqueue("ele_thermal_workqueue",
		WQ_FREEZABLE |
		WQ_UNBOUND, 1);

	if (!wq) {
		return -ENOMEM;
	}

	/* Set default tunable variables */
	t->enabled = ELE_THERMAL_ENABLED;
	t->timer = DEFAULT_TIMER;
	t->temp_high = DEFAULT_TEMP_HIGH;
	t->temp_hysteresis = DEFAULT_TEMP_HYSTERESIS;
	t->freq_stepdown = DEFAULT_FREQ_STEPDOWN;
	t->freq_stepup = DEFAULT_FREQ_STEPUP;
	t->freq_lazy = DEFAULT_FREQ_LAZY;

	policy = cpufreq_cpu_get(0); //cpu0
	cpu_info.device_max_freq = 0;
	cpu_info.user_max_freq = policy->max;
	cpu_info.limited_max_freq = policy->max;
	cpu_info.throttling = 0;
	cpu_info.lazy_counter = 0;

	/* Read frequency table */
	thermal_get_freq_table();

	/* Register control device */
	ret = misc_register(&ele_thermal_control_device);
	if (ret) {
		return -EINVAL;
	}
	ret = sysfs_create_group(&ele_thermal_control_device.this_device->kobj,
			&ele_thermal_control_group);
	if (ret) {
		return -EINVAL;
	}

	/* Initialize delayed work */
	INIT_DELAYED_WORK(&thermal_tick, thermal_tick_func);

	/* Run delayed work */
	if (t->enabled) {
		queue_delayed_work(wq, &thermal_tick, 0);
	}

	return ret;
}

static int ele_thermal_remove(struct platform_device *pdev)
{
	destroy_workqueue(wq);

	return 0;
}

static struct platform_driver ele_thermal_driver = {
	.probe = ele_thermal_probe,
	.remove = ele_thermal_remove,
	.driver = {
		.name = ELE_THERMAL,
		.owner = THIS_MODULE,
	},
};


/*
 * Device
 */
static struct platform_device ele_thermal_device = {
	.name = ELE_THERMAL,
	.id = -1,
};

/*
 * Initialization
 */
static int __init ele_thermal_init(void)
{
	int ret;

	pr_info("%s: init\n", ELE_THERMAL);
	pr_info("%s: version %d.%d by Electry\n",
		 ELE_THERMAL,
		 ELE_THERMAL_MAJOR_VERSION,
		 ELE_THERMAL_MINOR_VERSION);

	/* Register driver */
	ret = platform_driver_register(&ele_thermal_driver);
	if (ret) {
		pr_err("%s: Driver register failed: $d\n", ELE_THERMAL, ret);
		return ret;
	}

	/* Register device */
	ret = platform_device_register(&ele_thermal_device);
	if (ret) {
		pr_err("%s: Device register failed: $d\n", ELE_THERMAL, ret);
		return ret;
	}

	pr_info("%s: init complete\n", ELE_THERMAL);

	return ret;
}

late_initcall(ele_thermal_init);

MODULE_AUTHOR("Michal Chvíla aka Electry <electrydev@gmail.com>");
MODULE_DESCRIPTION("Ele_Thermal Throttling Driver");
MODULE_LICENSE("GPLv2");
