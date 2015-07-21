/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/input/synaptics_dsx_rmi4.h>
#include <linux/of_gpio.h>
#include "synaptics_dsx_i2c.h"
#include <linux/string.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif

static unsigned char config_id[4];
static int init_done = 0;
static int SYNA_WORK_DONE = 0;
static int synaptics_suspend_flag = 0;
unsigned int SYNA_REG_ADDR = 0x0000;
int SYNA_COUNT = 1;
int SYNA_VENDOR_ID = 0;
int SYNA_FW_VERSION = 0;
static struct wake_lock gesture_wakelock;
static volatile int synaptics_glove = 0;
//static int finger_f = 0;
#define SYNAP_MAX_X 1440
#define SYNAP_MAX_Y 2560
#define SYNAP_MT_TOUCH 26
#define SYNAP_MAX_FINGER 10
#if SYNA_SLIDE_WAKEUP
enum doze {
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
};
static enum doze doze_status = DOZE_DISABLED;
static void synaptics_rmi4_sensor_slidewake(struct synaptics_rmi4_data *rmi4_data);
static void synaptics_rmi4_sensor_doze(struct synaptics_rmi4_data *rmi4_data);
static int syna_wakeup_flag = 0;
static int GESTURE_DATA_OFFSET = 0;
static int BUTTON_DATA_OFFSET = 27;
//static int GESTURE_SWIPID_OFFSET = 18;
#endif
int X_DEL = 10;
int Y_DEL = 10;
static atomic_t device_count;
//static int no_touch = 0;
/*extern int is_charger_plug_in(void);*/
//static int glove = 0;
#define RMI4_VTG_MIN_UV		2700000
#define RMI4_VTG_MAX_UV		3300000
#define RMI4_ACTIVE_LOAD_UA	15000
#define RMI4_LPM_LOAD_UA	10

#define RMI4_I2C_VTG_MIN_UV	1800000
#define RMI4_I2C_VTG_MAX_UV	1800000
#define RMI4_I2C_LOAD_UA	10000
#define RMI4_I2C_LPM_LOAD_UA	10

#define RMI4_GPIO_SLEEP_LOW_MS 10
#define RESET_DELAY 100

static volatile int synaptics_debug = 0;
static struct device *t_dev;
#define DRIVER_NAME "synaptics_dsx_i2c"
#define INPUT_PHYS_NAME "synaptics_dsx_i2c/input0"

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif

#define NO_0D_WHILE_2D
/*
#define REPORT_2D_Z
*/
#define REPORT_2D_W

/*
#define IGNORE_FN_INIT_FAILURE
*/

#define RPT_TYPE (1 << 0)
#define RPT_X_LSB (1 << 1)
#define RPT_X_MSB (1 << 2)
#define RPT_Y_LSB (1 << 3)
#define RPT_Y_MSB (1 << 4)
#define RPT_Z (1 << 5)
#define RPT_WX (1 << 6)
#define RPT_WY (1 << 7)
#define RPT_DEFAULT (RPT_TYPE | RPT_X_LSB | RPT_X_MSB | RPT_Y_LSB | RPT_Y_MSB)

#define EXP_FN_WORK_DELAY_MS 1000 /* ms */
#define SYN_I2C_RETRY_TIMES 10
#define MAX_F11_TOUCH_WIDTH 15

#define CHECK_STATUS_TIMEOUT_MS 100

#define F01_STD_QUERY_LEN 21
#define F01_BUID_ID_OFFSET 18
#define F11_STD_QUERY_LEN 9
#define F11_STD_CTRL_LEN 10
#define F11_STD_DATA_LEN 12

#define STATUS_NO_ERROR 0x00
#define STATUS_RESET_OCCURRED 0x01
#define STATUS_INVALID_CONFIG 0x02
#define STATUS_DEVICE_FAILURE 0x03
#define STATUS_CONFIG_CRC_FAILURE 0x04
#define STATUS_FIRMWARE_CRC_FAILURE 0x05
#define STATUS_CRC_IN_PROGRESS 0x06

#define NORMAL_OPERATION (0 << 0)
#define SENSOR_SLEEP (1 << 0)
#define NO_SLEEP_OFF (0 << 2)
#define NO_SLEEP_ON (1 << 2)
#define CONFIGURED (1 << 7)

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length);

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length);

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28);

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable);
#ifdef TYPE_B_PROTOCOL
static int synaptics_rmi4_free_abnormall_fingers(struct synaptics_rmi4_data *rmi4_data);
#endif
static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data,
		unsigned short f01_cmd_base_addr);

static int synaptics_rmi4_suspend(struct device *dev);

static int synaptics_rmi4_resume(struct device *dev);

static int synaptics_rmi4_power_on(struct synaptics_rmi4_data *rmi4_data,bool on);
static int synaptics_rmi4_regulator_configure(struct synaptics_rmi4_data*rmi4_data, bool on);
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void synaptics_rmi4_early_suspend(struct early_suspend *h);

static void synaptics_rmi4_late_resume(struct early_suspend *h);
static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#endif
static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_regaddress_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_regaddress_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_gloveon_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_gloveon_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_gpio_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_power_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_regvalue_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_regvalue_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t synaptics_rmi4_discardx_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_discardx_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t synaptics_rmi4_work_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t synaptics_rmi4_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_count_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_information_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t synaptics_rmi4_sensorinfor_show(struct device *dev,
		struct device_attribute *attr, char *buf);
#if SYNA_SLIDE_WAKEUP
static ssize_t synaptics_rmi4_gesture_wakeup_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_gesture_wakeup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#endif
struct synaptics_rmi4_f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query6;
			struct {
				unsigned char ctrl0_is_present:1;
				unsigned char ctrl1_is_present:1;
				unsigned char ctrl2_is_present:1;
				unsigned char ctrl3_is_present:1;
				unsigned char ctrl4_is_present:1;
				unsigned char ctrl5_is_present:1;
				unsigned char ctrl6_is_present:1;
				unsigned char ctrl7_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl8_is_present:1;
				unsigned char ctrl9_is_present:1;
				unsigned char ctrl10_is_present:1;
				unsigned char ctrl11_is_present:1;
				unsigned char ctrl12_is_present:1;
				unsigned char ctrl13_is_present:1;
				unsigned char ctrl14_is_present:1;
				unsigned char ctrl15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl16_is_present:1;
				unsigned char ctrl17_is_present:1;
				unsigned char ctrl18_is_present:1;
				unsigned char ctrl19_is_present:1;
				unsigned char ctrl20_is_present:1;
				unsigned char ctrl21_is_present:1;
				unsigned char ctrl22_is_present:1;
				unsigned char ctrl23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl24_is_present:1;
				unsigned char ctrl25_is_present:1;
				unsigned char ctrl26_is_present:1;
				unsigned char ctrl27_is_present:1;
				unsigned char ctrl28_is_present:1;
				unsigned char ctrl29_is_present:1;
				unsigned char ctrl30_is_present:1;
				unsigned char ctrl31_is_present:1;
			} __packed;
		};
		unsigned char data[5];
	};
};

struct synaptics_rmi4_f12_query_8 {
	union {
		struct {
			unsigned char size_of_query9;
			struct {
				unsigned char data0_is_present:1;
				unsigned char data1_is_present:1;
				unsigned char data2_is_present:1;
				unsigned char data3_is_present:1;
				unsigned char data4_is_present:1;
				unsigned char data5_is_present:1;
				unsigned char data6_is_present:1;
				unsigned char data7_is_present:1;
			} __packed;
			struct {
				unsigned char data8_is_present:1;
				unsigned char data9_is_present:1;
				unsigned char data10_is_present:1;
				unsigned char data11_is_present:1;
				unsigned char data12_is_present:1;
				unsigned char data13_is_present:1;
				unsigned char data14_is_present:1;
				unsigned char data15_is_present:1;
			} __packed;
		};
		unsigned char data[3];
	};
};

struct synaptics_rmi4_f12_ctrl_8 {
	union {
		struct {
			unsigned char max_x_coord_lsb;
			unsigned char max_x_coord_msb;
			unsigned char max_y_coord_lsb;
			unsigned char max_y_coord_msb;
			unsigned char rx_pitch_lsb;
			unsigned char rx_pitch_msb;
			unsigned char tx_pitch_lsb;
			unsigned char tx_pitch_msb;
			unsigned char low_rx_clip;
			unsigned char high_rx_clip;
			unsigned char low_tx_clip;
			unsigned char high_tx_clip;
			unsigned char num_of_rx;
			unsigned char num_of_tx;
		};
		unsigned char data[14];
	};
};

struct synaptics_rmi4_f12_ctrl_9 {
	union {
		struct {
			unsigned char ctrl0_0;
			unsigned char ctrl0_1;
			unsigned char ctrl0_2;
			unsigned char ctrl0_3;
			unsigned char ctrl0_4;
			unsigned char ctrl0_5;
			unsigned char ctrl0_6;
			unsigned char ctrl1_0;
			unsigned char ctrl1_1;
			unsigned char ctrl1_2;
			unsigned char ctrl1_3;
			unsigned char ctrl2_0;
			unsigned char ctrl2_1;
			unsigned char ctrl2_2;
			unsigned char ctrl2_3;
			unsigned char ctrl3_0;
			unsigned char ctrl4_0;
			unsigned char ctrl4_1;
			unsigned char ctrl4_2;
			unsigned char ctrl4_3;
			unsigned char ctrl4_4;
		};
		unsigned char data[21];
	};
};
struct synaptics_rmi4_f12_ctrl_23 {
	union {
		struct {
			unsigned char obj_type_enable;
			unsigned char max_reported_objects;
		};
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f12_finger_data {
	unsigned char object_type_and_status;
	unsigned char x_lsb;
	unsigned char x_msb;
	unsigned char y_lsb;
	unsigned char y_msb;
#ifdef REPORT_2D_Z
	unsigned char z;
#endif
#ifdef REPORT_2D_W
	unsigned char wx;
	unsigned char wy;
#endif
};
#if SYNA_SLIDE_WAKEUP
struct synaptics_rmi4_f51_gesture_data {
	unsigned char lsb;
	unsigned char msb;
};
#endif
struct synaptics_rmi4_f1a_query {
	union {
		struct {
			unsigned char max_button_count:3;
			unsigned char reserved:5;
			unsigned char has_general_control:1;
			unsigned char has_interrupt_enable:1;
			unsigned char has_multibutton_select:1;
			unsigned char has_tx_rx_map:1;
			unsigned char has_perbutton_threshold:1;
			unsigned char has_release_threshold:1;
			unsigned char has_strongestbtn_hysteresis:1;
			unsigned char has_filter_strength:1;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f1a_control_0 {
	union {
		struct {
			unsigned char multibutton_report:2;
			unsigned char filter_mode:2;
			unsigned char reserved:4;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f1a_control {
	struct synaptics_rmi4_f1a_control_0 general_control;
	unsigned char button_int_enable;
	unsigned char multi_button;
	unsigned char *txrx_map;
	unsigned char *button_threshold;
	unsigned char button_release_threshold;
	unsigned char strongest_button_hysteresis;
	unsigned char filter_strength;
};

struct synaptics_rmi4_f1a_handle {
	int button_bitmask_size;
	unsigned char max_count;
	unsigned char valid_button_count;
	unsigned char *button_data_buffer;
	unsigned char *button_map;
	struct synaptics_rmi4_f1a_query button_query;
	struct synaptics_rmi4_f1a_control button_control;
};

struct synaptics_rmi4_exp_fn {
	enum exp_fn fn_type;
	bool inserted;
	int (*func_init)(struct synaptics_rmi4_data *rmi4_data);
	void (*func_remove)(struct synaptics_rmi4_data *rmi4_data);
	void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
			unsigned char intr_mask);
	struct list_head link;
};

struct synaptics_rmi4_exp_fn_data {
	bool initialized;
	bool queue_work;
	struct mutex mutex;
	struct list_head list;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct synaptics_rmi4_data *rmi4_data;
};

static struct synaptics_rmi4_exp_fn_data exp_data;

static struct device_attribute attrs[] = {
#ifdef CONFIG_HAS_EARLYSUSPEND
	__ATTR(full_pm_cycle, 0644,
			synaptics_rmi4_full_pm_cycle_show,
			synaptics_rmi4_full_pm_cycle_store),
#endif
	__ATTR(reset, 0200,
			synaptics_rmi4_show_error,
			synaptics_rmi4_f01_reset_store),
	__ATTR(productinfo, 0444,
			synaptics_rmi4_f01_productinfo_show,
			synaptics_rmi4_store_error),
	__ATTR(buildid, 0444,
			synaptics_rmi4_f01_buildid_show,
			synaptics_rmi4_store_error),
	__ATTR(flashprog, 0444,
			synaptics_rmi4_f01_flashprog_show,
			synaptics_rmi4_store_error),
	__ATTR(0dbutton, 0644,
			synaptics_rmi4_0dbutton_show,
			synaptics_rmi4_0dbutton_store),
	__ATTR(suspend, 0200,
			synaptics_rmi4_show_error,
			synaptics_rmi4_suspend_store),
	__ATTR(debug_switch, 0644,
			synaptics_rmi4_debug_show,
			synaptics_rmi4_debug_store),
	__ATTR(register_address, 0644,
			synaptics_rmi4_regaddress_show,
			synaptics_rmi4_regaddress_store),
	__ATTR(register_value, 0644,
			synaptics_rmi4_regvalue_show,
			synaptics_rmi4_regvalue_store),
	__ATTR(discard_x, 0644,
			synaptics_rmi4_discardx_show,
			synaptics_rmi4_discardx_store),
	__ATTR(irq_enable, 0644,
			synaptics_rmi4_irq_show,
			synaptics_rmi4_irq_store),
	__ATTR(work_done, 0444,
			synaptics_rmi4_work_show,
			NULL),
	__ATTR(register_count, 0644,
			synaptics_rmi4_count_show,
			synaptics_rmi4_count_store),
	__ATTR(glove_on, 0644,
			synaptics_rmi4_gloveon_show,
			synaptics_rmi4_gloveon_store),
	__ATTR(ic_info, 0444,
			synaptics_rmi4_information_show,
			NULL),
	__ATTR(sensor_info, 0444,
			synaptics_rmi4_sensorinfor_show,
			NULL),
#if SYNA_SLIDE_WAKEUP
	__ATTR(gesture_on, 0644,
			synaptics_rmi4_gesture_wakeup_show,
			synaptics_rmi4_gesture_wakeup_store),
#endif
	__ATTR(gpio_reset, 0644,
			NULL,
			synaptics_rmi4_gpio_store),
	__ATTR(power_reset, 0644,
			NULL,
			synaptics_rmi4_power_store),
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->full_pm_cycle);
}

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmi4_data->full_pm_cycle = input > 0 ? 1 : 0;

	return count;
}
#endif

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned short f01_cmd_base_addr;
	unsigned int reset;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	f01_cmd_base_addr = rmi4_data->f01_cmd_base_addr;

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	retval = synaptics_rmi4_reset_device(rmi4_data, f01_cmd_base_addr);
	if (retval < 0) {
		dev_err(dev,
				"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		return retval;
	}

	return count;
}

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
			(rmi4_data->rmi4_mod_info.product_info[0]),
			(rmi4_data->rmi4_mod_info.product_info[1]));
}

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int build_id;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	build_id = (unsigned int)rmi->build_id[0] +
			(unsigned int)rmi->build_id[1] * 0x100 +
			(unsigned int)rmi->build_id[2] * 0x10000;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			build_id);
}

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	struct synaptics_rmi4_f01_device_status device_status;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			device_status.data,
			sizeof(device_status.data));
	if (retval < 0) {
		dev_err(dev,
				"%s: Failed to read device status, error = %d\n",
				__func__, retval);
		return retval;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n",
			device_status.flash_prog);
}

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->button_0d_enabled);
}

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	unsigned char ii;
	unsigned char intr_enable;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (rmi4_data->button_0d_enabled == input)
		return count;

	if (list_empty(&rmi->support_fn_list))
		return -ENODEV;

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
			ii = fhandler->intr_reg_num;

			retval = synaptics_rmi4_i2c_read(rmi4_data,
					rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;

			if (input == 1)
				intr_enable |= fhandler->intr_mask;
			else
				intr_enable &= ~fhandler->intr_mask;

			retval = synaptics_rmi4_i2c_write(rmi4_data,
					rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;
		}
	}

	rmi4_data->button_0d_enabled = input;

	return count;
}

static ssize_t synaptics_rmi4_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",synaptics_debug);
}

static ssize_t synaptics_rmi4_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int error;

	error = sstrtoul(buf, 10, &val);
	if(error)
		return error;

	synaptics_debug =(unsigned int)val;
	SYNAP_DEBUG("synaptics_debug value = %ld", val);
	return count;
}
/*
exchange the register address
example:change the register addr to 0015
echo "0015">register_address
cat register_address
*/
static ssize_t synaptics_rmi4_regaddress_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	
	return snprintf(buf, PAGE_SIZE, "0X%x\n",SYNA_REG_ADDR);
}

static ssize_t synaptics_rmi4_regaddress_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int regaddress = 0x0000;
	int input_number = 0;
	
	input_number = count - 1;
	printk("input number is %d\n",input_number);
	if (sscanf(buf, "%x", &regaddress) != 1)
		return -EINVAL;
	printk("regaddress is 0x%x\n",regaddress);
	if (input_number == 4)
	{
	SYNA_REG_ADDR = regaddress;
	}
	else
	printk("input number is error,please input 4 charter\n");
	return count;
}
/*
 * read more than one register
example:read the value ,start addr is 0015,read count is 10
echo "0015">register_address
echo "10">register_count
cat register_value
 */
static ssize_t synaptics_rmi4_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	
	return snprintf(buf, PAGE_SIZE, "%d\n",SYNA_COUNT);
}

static ssize_t synaptics_rmi4_count_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int reg_count = 0;
	
	if (sscanf(buf, "%d", &reg_count) != 1)
		return -EINVAL;
	SYNAP_DEBUG("reg count is %d",reg_count);
	if(reg_count == 0)
	{
		SYNAP_DEBUG("reg count can not be 0");
		return count;
	}
	else
		SYNA_COUNT = reg_count;
	return count;
}
static ssize_t synaptics_rmi4_information_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	static char* vendor_id;
	unsigned char vendor = 0;
	int retval = 0;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f34_ctrl_base_addr,
			config_id,4);
	if (retval < 0) {
			dev_dbg(&rmi4_data->i2c_client->dev,
				"%s: Failed to read configID from device\n",
			__func__);
				} 
	retval = synaptics_rmi4_i2c_read(rmi4_data,
		rmi4_data->f01_query_base_addr + VENDOR_ID_OFFSET,
		&vendor,
		sizeof(vendor));
	if (retval < 0) {
		dev_dbg(&rmi4_data->i2c_client->dev,
		"%s: Failed to read vendor id from device\n",
		__func__);
				}
	SYNA_FW_VERSION = config_id[3];
	SYNA_VENDOR_ID = vendor;
	switch (SYNA_VENDOR_ID){
	case 1:
	vendor_id = "ofilm";
	break;
	case 4:
	vendor_id = "biel";
	break;
	case 5:
	vendor_id = "laibao";
	break;
	default:
	vendor_id = "unknown";
	}
	
	return snprintf(buf, PAGE_SIZE, "synaptics3508_%s_%d\n",vendor_id,SYNA_FW_VERSION);
}

static ssize_t synaptics_rmi4_sensorinfor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "GSensor_BOSCH_BMA280\nMSensor_BOSCH_BMM050\nGyro_BOSCH_BMG160\nPLSensor_AVAGO_apds9960\n");
}
static ssize_t synaptics_rmi4_gpio_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	const struct synaptics_rmi4_platform_data *pdata = rmi4_data->board;
	gpio_set_value(pdata->reset_gpio, 0);
	mdelay(RMI4_GPIO_SLEEP_LOW_MS);
	gpio_set_value(pdata->reset_gpio, 1);
	mdelay(RESET_DELAY);
	dev_err(&rmi4_data->i2c_client->dev,
				"%s:gpio reset\n",
				__func__);
	return count;
}
static ssize_t synaptics_rmi4_power_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	int retval;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	retval = synaptics_rmi4_power_on(rmi4_data, false);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power on\n");

	}
		mdelay(RMI4_GPIO_SLEEP_LOW_MS);
	retval = synaptics_rmi4_power_on(rmi4_data, true);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power on\n");
	}
		mdelay(RESET_DELAY);
		dev_err(&rmi4_data->i2c_client->dev,
				"%s:power reset\n",
				__func__);
	return count;
}
static ssize_t synaptics_rmi4_gloveon_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",synaptics_glove);
}
static ssize_t synaptics_rmi4_gloveon_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	int retval;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_f12_ctrl_9 ctrl_9;
	struct synaptics_rmi4_f12_query_5 query_5;
	unsigned char ctrl_9_offset;
	unsigned char sen_on[2]={0x10,0x05}; 
	unsigned char sen_off[2]={0x1e,0x05}; 
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f12_query_base_addr + 5,
			query_5.data,
			sizeof(query_5.data));
	if (retval < 0)
		return retval;
	ctrl_9_offset = query_5.ctrl0_is_present +
			query_5.ctrl1_is_present +
			query_5.ctrl2_is_present +
			query_5.ctrl3_is_present +
			query_5.ctrl4_is_present +
			query_5.ctrl5_is_present +
			query_5.ctrl6_is_present +
			query_5.ctrl7_is_present +
			query_5.ctrl8_is_present;
	if (sscanf(buf, "%d", &synaptics_glove) != 1)
		return -EINVAL;
	SYNAP_DEBUG("glove on  is 0x%x",synaptics_glove);
	SYNAP_DEBUG("ctrl base addr is %x,ctrl 9 offset is %d",rmi4_data->f12_ctrl_base_addr,ctrl_9_offset);
	if(synaptics_glove)
	{

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f12_ctrl_base_addr + ctrl_9_offset,
			sen_on,
			sizeof(sen_on));
	if (retval < 0)
	{
		SYNAP_DEBUG("%s,write to ctrl9 error",__func__);
		return retval;
	}
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f12_ctrl_base_addr + ctrl_9_offset,
			ctrl_9.data,
			sizeof(ctrl_9.data));
	if (retval < 0)
		return retval;
	SYNAP_DEBUG("write to 0x10,0x05,ctrl9 0-0 is 0x%x,0-1 is 0x%x",ctrl_9.ctrl0_0,ctrl_9.ctrl0_1);
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			0x0015,
			ctrl_9.data,
			sizeof(ctrl_9.data));
	if (retval < 0)
		return retval;
	SYNAP_DEBUG("write to 0x10,0x05,ctrl9 0-0 is 0x%x,0-1 is 0x%x",ctrl_9.ctrl0_0,ctrl_9.ctrl0_1);
	}
	else
	{

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f12_ctrl_base_addr + ctrl_9_offset,
			sen_off,
			sizeof(sen_off));
	if (retval < 0)
	{
		SYNAP_DEBUG("%s,write to ctrl9 error",__func__);
		return retval;
	}
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f12_ctrl_base_addr + ctrl_9_offset,
			ctrl_9.data,
			sizeof(ctrl_9.data));
	if (retval < 0)
		return retval;
	SYNAP_DEBUG("write to 0x1e,0x05,ctrl9 0-0 is 0x%x,0-1 is 0x%x",ctrl_9.ctrl0_0,ctrl_9.ctrl0_1);
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			0x0015,
			ctrl_9.data,
			sizeof(ctrl_9.data));
	if (retval < 0)
		return retval;
	SYNAP_DEBUG("write to 0x10,0x05,ctrl9 0-0 is 0x%x,0-1 is 0x%x",ctrl_9.ctrl0_0,ctrl_9.ctrl0_1);
	}
	return count;
}
static ssize_t synaptics_rmi4_discardx_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf,PAGE_SIZE,"Discard x:%d\n",X_DEL);
}

static ssize_t synaptics_rmi4_discardx_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	if (sscanf(buf, "%d", &X_DEL) != 1)
		return -EINVAL;
	return count;
}
static ssize_t synaptics_rmi4_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	int irq_enable;
	if(!rmi4_data->irq_enabled)
		irq_enable = 0;
	else
		irq_enable = 1;
	return snprintf(buf,PAGE_SIZE,"irq:%d\n",irq_enable);
}

static ssize_t synaptics_rmi4_work_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf,PAGE_SIZE,"work done:%d\n",SYNA_WORK_DONE);
}
static ssize_t synaptics_rmi4_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	int input;
	if (sscanf(buf, "%d", &input) != 1)
		return -EINVAL;
	if(input)
	{
		enable_irq(rmi4_data->irq);
		rmi4_data->irq_enabled = true;
	}
	else
	{
		disable_irq_nosync(rmi4_data->irq);
		rmi4_data->irq_enabled = false;
	}
	return count;
}
/*
  read and writethe value of the cuurrent register addr 
example:change 0015 value to 0x1e
echo "0015">register_address
echo "1e">register_value
cat register_value
*/
static ssize_t synaptics_rmi4_regvalue_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	unsigned char regvalue[30];
	int i = 0;
	//char reg[10];
	char string[100];
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			SYNA_REG_ADDR,
			regvalue,
			sizeof(regvalue));
			if (retval < 0)
			{
				SYNAP_DEBUG("%s,read error",__func__);
				return retval;
			}
	for(i=0;i<SYNA_COUNT;i++)
	{
		if(i== 0)
		sprintf(string,"%s%x",string,regvalue[i]);
		else
		sprintf(string,"%s,%x",string,regvalue[i]);
	//	sprintf(reg,"%x",regvalue[i]);
	//	strcat(string,reg);
	//	strcat(string,",");
	}
	SYNAP_DEBUG("%s\n",string);
	return snprintf(buf,PAGE_SIZE,"Addr:0X%x,Count:%d\n%s\n",SYNA_REG_ADDR,SYNA_COUNT,string);
}

static ssize_t synaptics_rmi4_regvalue_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	int input_number = 0;
	unsigned char regvalue[30] = {0x00};
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	input_number = count - 1;
	printk("input number is %d\n",input_number);
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			SYNA_REG_ADDR,
			regvalue,
			sizeof(regvalue));
			if (retval < 0)
			{
				SYNAP_DEBUG("%s,read error",__func__);
				return retval;
			}
	if (sscanf(buf, "%x", (int *)&regvalue[SYNA_COUNT - 1]) != 1)
		return -EINVAL;
	printk("regvalue is 0x%x\n",regvalue[SYNA_COUNT - 1]);
	if (input_number != 2)
	printk("input number is error,please input 2 charter\n");
	retval = synaptics_rmi4_i2c_write(rmi4_data,
			SYNA_REG_ADDR,
			regvalue,
			SYNA_COUNT*sizeof(unsigned char));
			if (retval < 0)
				return retval;
	return count;
}
/*
extern int set_suspend_from_lcd(int val)
{
	struct device *dev = t_dev;
	
	if((val == 0) && (no_touch == 0))
		synaptics_rmi4_suspend(dev);
	else if((val == 1) && (no_touch == 0))
		synaptics_rmi4_resume(dev);
	else
		return 0;
	return 1;
}
*/
static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input == 1)
		synaptics_rmi4_suspend(dev);
	else if (input == 0)
		synaptics_rmi4_resume(dev);
	else
		return -EINVAL;

	return count;
}
#if SYNA_SLIDE_WAKEUP
static ssize_t synaptics_rmi4_gesture_wakeup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",syna_wakeup_flag);
}

static ssize_t synaptics_rmi4_gesture_wakeup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int error;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	error = sstrtoul(buf, 10, &val);
	if(error)
		return error;
	if(val == 1)
	{
		if(!syna_wakeup_flag)
		{
		syna_wakeup_flag = 1;
		enable_irq_wake(rmi4_data->irq);
		SYNAP_DEBUG("%s,gesture flag is  = %ld",__func__,val);
		}else
			return count;
	}else
	{
		if(syna_wakeup_flag)
		{
		syna_wakeup_flag = 0;
		disable_irq_wake(rmi4_data->irq);
		SYNAP_DEBUG("%s,gesture flag is  = %ld",__func__,val);
		}else
			return count;
	}
	return count;
}
#endif
 /**
 * synaptics_rmi4_set_page()
 *
 * Called by synaptics_rmi4_i2c_read() and synaptics_rmi4_i2c_write().
 *
 * This function writes to the page select register to switch to the
 * assigned page.
 */
static int synaptics_rmi4_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned int address)
{
	int retval = 0;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = rmi4_data->i2c_client;

	page = ((address >> 8) & MASK_8BIT);
	if (page != rmi4_data->current_page) {
		buf[0] = MASK_8BIT;
		buf[1] = page;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
			if (retval != PAGE_SELECT_LEN) {
				dev_err(&i2c->dev,
						"%s: I2C retry %d\n",
						__func__, retry + 1);
				mdelay(20);
			} else {
				rmi4_data->current_page = page;
				break;
			}
		}
	} else {
		retval = PAGE_SELECT_LEN;
	}
	return retval;
}

 /**
 * synaptics_rmi4_i2c_read()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function reads data of an arbitrary length from the sensor,
 * starting from an assigned register address of the sensor, via I2C
 * with a retry mechanism.
 */
static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	int setpage_retry = 2;//add for set page retry.
	struct i2c_msg msg[] = {
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	buf = addr & MASK_8BIT;
	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));
	if(init_done)//ic init ok,then power reset when i2c err
	{
	for(retry = 0;retry < setpage_retry; retry++)//set page retry ,if setpage err,do power reset
	{
	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval == PAGE_SELECT_LEN)
		break;
	else//power reset 
	{
	retval = synaptics_rmi4_power_on(rmi4_data, false);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power off\n");
		retval = -EIO;
		goto exit;

	}
	mdelay(100);
	retval = synaptics_rmi4_power_on(rmi4_data, true);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power on\n");
		retval = -EIO;
		goto exit;
	}
		mdelay(200);
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: setpage over retry limit,power reset\n",
				__func__);
	}
	}
	if (retval != PAGE_SELECT_LEN)
	{
		dev_err(&rmi4_data->i2c_client->dev, "setpage retry over limit,exit i2c read\n");
		retval = -EIO;
		goto exit;
	}
	}else//in ic init process,do not reset
	{
	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN)
	{

		dev_err(&rmi4_data->i2c_client->dev, "setpage err,exit i2cread\n");
		goto exit;
	}
	}
	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		mdelay(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		if(init_done)
		{
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C read over retry limit,do power reset\n",
				__func__);
	retval = synaptics_rmi4_power_on(rmi4_data, false);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power off\n");
		retval = -EIO;
		goto exit;

	}
	mdelay(100);
	retval = synaptics_rmi4_power_on(rmi4_data, true);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power on\n");
		retval = -EIO;
		goto exit;
	}
		mdelay(200);
		}else
		{
		retval = -EIO;
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C read over retry limit\n",
				__func__);
		}
	}


exit:
	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

 /**
 * synaptics_rmi4_i2c_write()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function writes data of an arbitrary length to the sensor,
 * starting from an assigned register address of the sensor, via I2C with
 * a retry mechanism.
 */
static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	int setpage_retry = 2;
	struct i2c_msg msg[] = {
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));
if(init_done)
{
	for(retry = 0;retry < setpage_retry; retry++)//set page retry 5,if setpage err,do power reset
	{
	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval == PAGE_SELECT_LEN)
		break;
	else//power reset 
	{
	retval = synaptics_rmi4_power_on(rmi4_data, false);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power off\n");
		retval = -EIO;
		goto exit;

	}
		mdelay(100);
	retval = synaptics_rmi4_power_on(rmi4_data, true);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power on\n");
		retval = -EIO;
		goto exit;
	}
		mdelay(200);
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: setpage over retry limit,power reset\n",
				__func__);
	}
	}
	if (retval != PAGE_SELECT_LEN)
	{

		dev_err(&rmi4_data->i2c_client->dev, "setpage retry over limit,exit i2c read\n");
		retval = -EIO;
		goto exit;
	}
}else
{

	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN)
	{
		dev_err(&rmi4_data->i2c_client->dev, "setpage retry over limit,exit i2c read\n");
		goto exit;
	}
}

	buf[0] = addr & MASK_8BIT;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) 
		{
		if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		mdelay(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		if(init_done)
		{
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C read over retry limit,do power reset\n",
				__func__);
	retval = synaptics_rmi4_power_on(rmi4_data, false);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power off\n");
		retval = -EIO;
		goto exit;

	}
	mdelay(100);
	retval = synaptics_rmi4_power_on(rmi4_data, true);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power on\n");
		retval = -EIO;
		goto exit;
	}
		mdelay(200);
		}else
		{
		retval = -EIO;
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C read over retry limit\n",
				__func__);
		}
	}
exit:
	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

 /**
 * synaptics_rmi4_f11_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $11
 * finger data has been detected.
 *
 * This function reads the Function $11 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f11_abs_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char reg_index;
	unsigned char finger;
	unsigned char fingers_supported;
	unsigned char num_of_finger_status_regs;
	unsigned char finger_shift;
	unsigned char finger_status;
	unsigned char data_reg_blk_size;
	unsigned char finger_status_reg[3];
	unsigned char data[F11_STD_DATA_LEN];
	unsigned short data_addr;
	unsigned short data_offset;
	int x;
	int y;
	int wx;
	int wy;
	int temp;

	/*
	 * The number of finger status registers is determined by the
	 * maximum number of fingers supported - 2 bits per finger. So
	 * the number of finger status registers to read is:
	 * register_count = ceil(max_num_of_fingers / 4)
	 */
	fingers_supported = fhandler->num_of_data_points;
	num_of_finger_status_regs = (fingers_supported + 3) / 4;
	data_addr = fhandler->full_addr.data_base;
	data_reg_blk_size = fhandler->size_of_data_register_block;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr,
			finger_status_reg,
			num_of_finger_status_regs);
	if (retval < 0)
		return 0;

	for (finger = 0; finger < fingers_supported; finger++) {
		reg_index = finger / 4;
		finger_shift = (finger % 4) * 2;
		finger_status = (finger_status_reg[reg_index] >> finger_shift)
				& MASK_2BIT;

		/*
		 * Each 2-bit finger status field represents the following:
		 * 00 = finger not present
		 * 01 = finger present and data accurate
		 * 10 = finger present but data may be inaccurate
		 * 11 = reserved
		 */
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, finger_status);
#endif

		if (finger_status) {
			data_offset = data_addr +
					num_of_finger_status_regs +
					(finger * data_reg_blk_size);
			retval = synaptics_rmi4_i2c_read(rmi4_data,
					data_offset,
					data,
					data_reg_blk_size);
			if (retval < 0)
				return 0;

			x = (data[0] << 4) | (data[2] & MASK_4BIT);
			y = (data[1] << 4) | ((data[2] >> 4) & MASK_4BIT);
			wx = (data[3] & MASK_4BIT);
			wy = (data[3] >> 4) & MASK_4BIT;

			if (rmi4_data->board->swap_axes) {
				temp = x;
				x = y;
				y = temp;
				temp = wx;
				wx = wy;
				wy = temp;
			}

			if (rmi4_data->board->x_flip)
				x = rmi4_data->sensor_max_x - x;
			if (rmi4_data->board->y_flip)
				y = rmi4_data->sensor_max_y - y;
			input_report_key(rmi4_data->input_dev,
					BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,
					BTN_TOOL_FINGER, 1);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif
			touch_count++;
		}
	}

	if (touch_count == 0) {
		input_report_key(rmi4_data->input_dev,
				BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,
				BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
#endif
	}

	input_sync(rmi4_data->input_dev);

	return touch_count;
}

 /**
 * synaptics_rmi4_f12_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $12
 * finger data has been detected.
 *
 * This function reads the Function $12 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f12_abs_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	//unsigned char index;
	unsigned char finger;
	unsigned char fingers_to_process;
	unsigned char finger_status;
	unsigned char size_of_2d_data;
	unsigned short data_addr;
	int x;
	int y;
	int wx;
	int wy;
	int temp;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_finger_data *data;
	struct synaptics_rmi4_f12_finger_data *finger_data;
#if SYNA_SLIDE_WAKEUP
unsigned char gesture_type[5];
//unsigned char button_irq[2];
struct synaptics_rmi4_f51_gesture_data gesture_data[12];
int start_x,start_y,end_x,end_y;
/*int min_y_x,min_y,min_x,min_x_y,max_y_x,max_y,max_x,max_x_y;*/
#endif
	fingers_to_process = fhandler->num_of_data_points;
	data_addr = fhandler->full_addr.data_base;
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);
#if SYNA_SLIDE_WAKEUP
	if ((syna_wakeup_flag)&&((DOZE_ENABLED == doze_status ||DOZE_WAKEUP == doze_status))) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				data_addr + extra_data->data4_offset,
				gesture_type,
				sizeof(gesture_type));
		if (retval < 0)
		{
			SYNAP_DEBUG("READ GESTURE TYPE ERROR!!");
			synaptics_rmi4_sensor_doze(rmi4_data);
			return 0;
		}
		//SYNAP_DEBUG("GESTURE TYPE IS %d,%d,%d,%d,%d",gesture_type[0],gesture_type[1],gesture_type[2],gesture_type[3],gesture_type[4]);
	switch (gesture_type[0]){
		wake_lock_timeout(&gesture_wakelock,1*HZ);
		case 7:
			retval = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f51_data_base_addr + GESTURE_DATA_OFFSET,
				(unsigned char *)gesture_data,
				4*sizeof(struct synaptics_rmi4_f51_gesture_data));
			if (retval < 0)
			{
				SYNAP_DEBUG("READ GESTURE data ERROR!!");
				synaptics_rmi4_sensor_doze(rmi4_data);
				return 0;
			}
				start_x = (gesture_data[0].msb << 8)|(gesture_data[0].lsb);
				start_y = (gesture_data[1].msb << 8)|(gesture_data[1].lsb);
				end_x = (gesture_data[2].msb << 8)|(gesture_data[2].lsb);
				end_y = (gesture_data[3].msb << 8)|(gesture_data[3].lsb);
		//		SYNAP_DEBUG("start (x,y):%d,%d;end (x,y):%d,%d",start_x,start_y,end_x,end_y);
				doze_status = DOZE_WAKEUP;
				if(start_x < end_x)
				{
				SYNAP_DEBUG("Slide Left to Right detected!");
				input_report_key(
					rmi4_data->input_dev, KEY_GESTURE_LR, 1);
				input_sync(rmi4_data->input_dev);
				input_report_key(
					rmi4_data->input_dev, KEY_GESTURE_LR, 0);
				input_sync(rmi4_data->input_dev);
				}
				else
				{
				SYNAP_DEBUG("Slide Right to Left detected!");
				input_report_key(
					rmi4_data->input_dev, KEY_GESTURE_RL, 1);
				input_sync(rmi4_data->input_dev);
				input_report_key(
					rmi4_data->input_dev, KEY_GESTURE_RL, 0);
				input_sync(rmi4_data->input_dev);
				}
				break;
			case 3:
			    SYNAP_DEBUG("one finger double tap detected!");
				doze_status = DOZE_WAKEUP;
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_DT, 1);
				input_sync(rmi4_data->input_dev);
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_DT, 0);
				input_sync(rmi4_data->input_dev);
				break;
				/*
			case 8:
			    SYNAP_DEBUG("character o detected!");
				doze_status = DOZE_WAKEUP;
				break;
				
			case 9:
			    SYNAP_DEBUG("triangle detected!");
				doze_status = DOZE_WAKEUP;
				break;
				*/
			case 10:
				doze_status = DOZE_WAKEUP;
				if(gesture_type[2] != 4)
				{
			    SYNAP_DEBUG("character v detected!");
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_V, 1);
				input_sync(rmi4_data->input_dev);
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_V, 0);
				input_sync(rmi4_data->input_dev);
				}else
				{
			    SYNAP_DEBUG("character c detected!");
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_C, 1);
				input_sync(rmi4_data->input_dev);
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_C, 0);
				input_sync(rmi4_data->input_dev);
				}

				break;
			case 11:
				if(gesture_type[2] == 0x63)
				{
					/*
				retval = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f51_data_base_addr + GESTURE_DATA_OFFSET,
				(unsigned char *)gesture_data,
				12*sizeof(struct synaptics_rmi4_f51_gesture_data));
				if (retval < 0)
				{
				SYNAP_DEBUG("READ GESTURE data ERROR!!");
				return 0;
				}
				
				start_x = (gesture_data[0].msb << 8)|(gesture_data[0].lsb);
				start_y = (gesture_data[1].msb << 8)|(gesture_data[1].lsb);
				end_x = (gesture_data[2].msb << 8)|(gesture_data[2].lsb);
				end_y = (gesture_data[3].msb << 8)|(gesture_data[3].lsb);
				min_y_x = (gesture_data[4].msb << 8)|(gesture_data[4].lsb);
				min_y = (gesture_data[5].msb << 8)|(gesture_data[5].lsb);
				min_x = (gesture_data[6].msb << 8)|(gesture_data[6].lsb);
				min_x_y = (gesture_data[7].msb << 8)|(gesture_data[7].lsb);
				max_y_x = (gesture_data[8].msb << 8)|(gesture_data[8].lsb);
				max_y = (gesture_data[9].msb << 8)|(gesture_data[9].lsb);
				max_x = (gesture_data[10].msb << 8)|(gesture_data[10].lsb);
				max_x_y = (gesture_data[11].msb << 8)|(gesture_data[11].lsb);
				*/
		//		SYNAP_DEBUG("start:(x,y):%d,%d;end:(x,y):%d,%d;up:(x,y):%d,%d;left:(x,y):%d,%d;down:(x,y):%d,%d;right:(x,y):%d,%d",start_x,start_y,end_x,end_y,min_y_x,min_y,min_x,min_x_y,max_y_x,max_y,max_x,max_x_y);
				doze_status = DOZE_WAKEUP;
				SYNAP_DEBUG("character c detected!");
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_C, 1);
				input_sync(rmi4_data->input_dev);
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_C, 0);
				input_sync(rmi4_data->input_dev);
				}else if(gesture_type[2] == 0x65)
				{
				SYNAP_DEBUG("character e detected!");
				doze_status = DOZE_WAKEUP;
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_E, 1);
				input_sync(rmi4_data->input_dev);
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_E, 0);
				input_sync(rmi4_data->input_dev);
				}
				/*else if(gesture_type[2] == 0x77)
				{
				SYNAP_DEBUG("character w detected!");
				doze_status = DOZE_WAKEUP;
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_W, 1);
				input_sync(rmi4_data->input_dev);
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_W,0);
				input_sync(rmi4_data->input_dev);
				}*/else if(gesture_type[2] == 0x6d)
				{
				SYNAP_DEBUG("character m detected!");
				doze_status = DOZE_WAKEUP;
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_M, 1);
				input_sync(rmi4_data->input_dev);
				input_report_key(rmi4_data->input_dev, KEY_GESTURE_M, 0);
				input_sync(rmi4_data->input_dev);
				}
				else
				{	SYNAP_DEBUG("unkonwn character!");
					return 0;
				}
				break;
				/*
			case 12:
			    SYNAP_DEBUG("one finger three tap detected!");
				doze_status = DOZE_WAKEUP;
				break;
			case 15:
			    SYNAP_DEBUG("two finger double tap detected!");
				doze_status = DOZE_WAKEUP;
				break;
				*/
			default:
				SYNAP_DEBUG("UNKONWN gesture type!!");
	}
	return 0;
			}
#endif
#if 0
	/* Determine the total number of fingers to process */
	if (extra_data->data15_size) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				data_addr + extra_data->data15_offset,
				extra_data->data15_data,
				extra_data->data15_size);
		if (retval < 0)
			return 0;

		/* Start checking from the highest bit */
		index = extra_data->data15_size - 1; /* Highest byte */
		finger = (fingers_to_process - 1) % 8; /* Highest bit */
		do {
			if (extra_data->data15_data[index] & (1 << finger))
				break;

			if (finger) {
				finger--;
			} else {
				index--; /* Move to the next lower byte */
				finger = 7;
			}

			fingers_to_process--;
		} while (fingers_to_process);

		dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Number of fingers to process = %d\n",
			__func__, fingers_to_process);
	}

	if (!fingers_to_process) {
		synaptics_rmi4_free_fingers(rmi4_data);
		return 0;
	}
#endif
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr + extra_data->data1_offset,
			(unsigned char *)fhandler->data,
			fingers_to_process * size_of_2d_data);
	if (retval < 0)
		return 0;

	data = (struct synaptics_rmi4_f12_finger_data *)fhandler->data;

	for (finger = 0; finger < fingers_to_process; finger++) {
		finger_data = data + finger;
		finger_status = finger_data->object_type_and_status & MASK_2BIT;

		/*
		 * Each 2-bit finger status field represents the following:
		 * 00 = finger not present
		 * 01 = finger present and data accurate
		 * 10 = finger present but data may be inaccurate
		 * 11 = reserved
		 */

		if (finger_status) {
		x = (finger_data->x_msb << 8) | (finger_data->x_lsb);
		y = (finger_data->y_msb << 8) | (finger_data->y_lsb);
		if (rmi4_data->board->x_flip)
			x = rmi4_data->sensor_max_x - x;
		if (rmi4_data->board->y_flip)
			y = rmi4_data->sensor_max_y - y;
		
		if(x < X_DEL || x > (1440-X_DEL) ||  y > (2560-Y_DEL))
		{
		printk("F=%d x=%d y=%d dropped!\n",finger,x,y);
		continue;
		}
		
//			x = (finger_data->x_msb << 8) | (finger_data->x_lsb);
//			y = (finger_data->y_msb << 8) | (finger_data->y_lsb);
#ifdef REPORT_2D_W
			wx = finger_data->wx;
			wy = finger_data->wy;
#endif
			if (rmi4_data->board->swap_axes) {
				temp = x;
				x = y;
				y = temp;
				temp = wx;
				wx = wy;
				wy = temp;
			}
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, finger_status);
#endif
/*
			if (rmi4_data->board->x_flip)
				x = rmi4_data->sensor_max_x - x;
			if (rmi4_data->board->y_flip)
				y = rmi4_data->sensor_max_y - y;
*/
			input_report_key(rmi4_data->input_dev,
					BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,
					BTN_TOOL_FINGER, 1);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif
			if(synaptics_debug){
				printk("F=%d x=%d y=%d\n", finger, x, y);
			}


			touch_count++;
		}
		else
		{

#ifdef TYPE_B_PROTOCOL
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, finger_status);
#endif
		}
	}
#if 0
		if(finger_f < fingers_to_process) {
			finger_f = fingers_to_process;
		}
		if(finger_f >fingers_to_process){
			--finger_f;
			input_mt_slot(rmi4_data->input_dev, finger_f);
			input_mt_report_slot_state(rmi4_data->input_dev,
					MT_TOOL_FINGER, 0);
		}
#endif
	if (touch_count == 0) {
		input_report_key(rmi4_data->input_dev,
				BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,
				BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
#endif
	}

	input_sync(rmi4_data->input_dev);

	return touch_count;
}

static void synaptics_rmi4_f1a_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0;
	unsigned char button;
	unsigned char index;
	unsigned char shift;
	unsigned char status;
	unsigned char *data;
	unsigned short data_addr = fhandler->full_addr.data_base;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	static unsigned char do_once = 1;
	static bool current_status[MAX_NUMBER_OF_BUTTONS];
#if SYNA_SLIDE_WAKEUP
unsigned char button_type;
#endif
#ifdef NO_0D_WHILE_2D
	static bool before_2d_status[MAX_NUMBER_OF_BUTTONS];
	static bool while_2d_status[MAX_NUMBER_OF_BUTTONS];
#endif

#if SYNA_SLIDE_WAKEUP
	if ((syna_wakeup_flag)&&((DOZE_ENABLED == doze_status ||DOZE_WAKEUP == doze_status))) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f51_data_base_addr+BUTTON_DATA_OFFSET,
				&button_type,
				sizeof(button_type));
		SYNAP_DEBUG("BUTTON TYPE IS %d",button_type);
		if (retval < 0)
				{
					SYNAP_DEBUG("READ TYPE ERROR!!");
					synaptics_rmi4_sensor_doze(rmi4_data);
					return;
				}else
					{
						if(button_type == 1)
							{
								SYNAP_DEBUG("double tap menu detected!!");
								doze_status = DOZE_WAKEUP;
								/*
								input_report_key(rmi4_data->input_dev, KEY_POWER, 1);
								input_sync(rmi4_data->input_dev);
								input_report_key(rmi4_data->input_dev, KEY_POWER, 0);
								input_sync(rmi4_data->input_dev);
								*/
							}else if(button_type == 2)
							{
								SYNAP_DEBUG("double tap home detected!!");
								input_report_key(rmi4_data->input_dev, KEY_GESTURE_W, 1);
								input_sync(rmi4_data->input_dev);
								input_report_key(rmi4_data->input_dev, KEY_GESTURE_W, 0);
								input_sync(rmi4_data->input_dev);
								doze_status = DOZE_WAKEUP;
							}else if(button_type == 4)
							{
								SYNAP_DEBUG("double tap back detected!!");
								doze_status = DOZE_WAKEUP;
								/*
								input_report_key(rmi4_data->input_dev, KEY_POWER, 1);
								input_sync(rmi4_data->input_dev);
								input_report_key(rmi4_data->input_dev, KEY_POWER, 0);
								input_sync(rmi4_data->input_dev);
								*/
							}else
							{
								SYNAP_DEBUG("unknown button gesture!!");
								return;
							}
					}
		return;
	}
#endif
	if (do_once) {
		memset(current_status, 0, sizeof(current_status));
#ifdef NO_0D_WHILE_2D
		memset(before_2d_status, 0, sizeof(before_2d_status));
		memset(while_2d_status, 0, sizeof(while_2d_status));
#endif
		do_once = 0;
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr,
			f1a->button_data_buffer,
			f1a->button_bitmask_size);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read button data registers\n",
				__func__);
		return;
	}

	data = f1a->button_data_buffer;

	for (button = 0; button < f1a->valid_button_count; button++) {
		index = button / 8;
		shift = button % 8;
		status = ((data[index] >> shift) & MASK_1BIT);

		if (current_status[button] == status)
			continue;
		else
			current_status[button] = status;

		dev_dbg(&rmi4_data->i2c_client->dev,
				"%s: Button %d (code %d) ->%d\n",
				__func__, button,
				f1a->button_map[button],
				status);
if(synaptics_debug)
		printk("Button %d,status %d.\n",f1a->button_map[button], status);
#ifdef NO_0D_WHILE_2D
		if (rmi4_data->fingers_on_2d == false) {
			if (status == 1) {
				before_2d_status[button] = 1;
			} else {
				if (while_2d_status[button] == 1) {
					while_2d_status[button] = 0;
					continue;
				} else {
					before_2d_status[button] = 0;
				}
			}
			touch_count++;
			input_report_key(rmi4_data->input_dev,
					f1a->button_map[button],
					status);
		} else {
			if (before_2d_status[button] == 1) {
				before_2d_status[button] = 0;
				touch_count++;
				input_report_key(rmi4_data->input_dev,
						f1a->button_map[button],
						status);
			} else {
				if (status == 1)
					while_2d_status[button] = 1;
				else
					while_2d_status[button] = 0;
			}
		}
#else
		touch_count++;
		input_report_key(rmi4_data->input_dev,
				f1a->button_map[button],
				status);
#endif
	}

	if (touch_count)
		input_sync(rmi4_data->input_dev);

	return;
}

 /**
 * synaptics_rmi4_report_touch()
 *
 * Called by synaptics_rmi4_sensor_report().
 *
 * This function calls the appropriate finger data reporting function
 * based on the function handler it receives and returns the number of
 * fingers detected.
 */
static void synaptics_rmi4_report_touch(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	unsigned char touch_count_2d;

	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Function %02x reporting\n",
			__func__, fhandler->fn_number);

	switch (fhandler->fn_number) {
	case SYNAPTICS_RMI4_F11:
		touch_count_2d = synaptics_rmi4_f11_abs_report(rmi4_data,
				fhandler);

		if (touch_count_2d)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;
	case SYNAPTICS_RMI4_F12:
		touch_count_2d = synaptics_rmi4_f12_abs_report(rmi4_data,
				fhandler);

		if (touch_count_2d)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;
	case SYNAPTICS_RMI4_F1A:
		synaptics_rmi4_f1a_report(rmi4_data, fhandler);
		break;
	default:
		break;
	}

	return;
}

 /**
 * synaptics_rmi4_sensor_report()
 *
 * Called by synaptics_rmi4_irq().
 *
 * This function determines the interrupt source(s) from the sensor
 * and calls synaptics_rmi4_report_touch() with the appropriate
 * function handler for each function with valid data inputs.
 */
static void synaptics_rmi4_sensor_report(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char data[MAX_INTR_REGISTERS + 1];
	unsigned char *intr = &data[1];
	struct synaptics_rmi4_f01_device_status status;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fn *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;
//	unsigned char intr_status[MAX_INTR_REGISTERS];
//	int ret = 0;

	rmi = &(rmi4_data->rmi4_mod_info);

	/*
	 * Get interrupt status information from F01 Data1 register to
	 * determine the source(s) that are flagging the interrupt.
	 */
	/*ret = is_charger_plug_in();*/
#if 0
	if((ret != glove) && ret)
	{
		glove = ret;
		ret = synaptics_rmi4_f12_set_enables(rmi4_data, 0);
		printk(KERN_DEBUG "glove close ret = %d\n", ret);
	}
	else if(ret != glove){
		glove = ret;
		synaptics_rmi4_f12_set_enables(rmi4_data, 1);
		printk(KERN_DEBUG "glove open ret = %d\n", ret);
	}
#endif
	//printk("lyh:sensor report\n");
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			data,
			rmi4_data->num_of_intr_regs + 1);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read interrupt status\n",
				__func__);
		return;
	}

	status.data[0] = data[0];
/*	
	if(data[0] == 3)
	{
		SYNAP_DEBUG("esd detected!!,do power reset");
		retval = synaptics_rmi4_power_on(rmi4_data, false);
		if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power on\n");

		}
		mdelay(100);
		retval = synaptics_rmi4_power_on(rmi4_data, true);
		if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power on\n");
		}
		mdelay(200);
		dev_err(&rmi4_data->i2c_client->dev,
				"%s:esd detected,power reset\n",
				__func__);
		return;
	}
*/	
	if (status.unconfigured && !status.flash_prog) {
		pr_notice("%s: spontaneous reset detected\n", __func__);
		retval = synaptics_rmi4_reinit_device(rmi4_data);
		
		retval = synaptics_rmi4_reset_device(rmi4_data, rmi4_data->f01_cmd_base_addr);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to reinit device\n",
					__func__);
		}
#if 0
	/* Clear interrupts first */
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr + 1,
			intr_status,
			rmi4_data->num_of_intr_regs);
	if (retval < 0)
	{
		SYNAP_DEBUG("clear interrupts bit err");
	}
#endif
	return;
	}

	/*
	 * Traverse the function handler list and service the source(s)
	 * of the interrupt accordingly.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				if (fhandler->intr_mask &
						intr[fhandler->intr_reg_num]) {
					synaptics_rmi4_report_touch(rmi4_data,
							fhandler);
				}
			}
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (exp_fhandler->inserted &&
					(exp_fhandler->func_attn != NULL))
				exp_fhandler->func_attn(rmi4_data, intr[0]);
		}
	}
	mutex_unlock(&exp_data.mutex);

	return;
}
static void synap_work_func(struct work_struct *work)
{

	struct synaptics_rmi4_data *rmi4_data = NULL;
	struct delayed_work *dwork = to_delayed_work(work);
	rmi4_data = container_of(dwork, struct synaptics_rmi4_data, tp_work);
//	printk("lyh:work func\n");
	synaptics_rmi4_sensor_report(rmi4_data);
	SYNA_WORK_DONE = 1;
//	synaptics_rmi4_irq_enable(rmi4_data, true);
}
 /**
 * synaptics_rmi4_irq()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 * This function is the ISR thread and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */
static irqreturn_t synaptics_rmi4_irq(int irq, void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;
	SYNA_WORK_DONE = 0;
	//printk("lyh:irq\n");
//	synaptics_rmi4_irq_enable(rmi4_data, false);
#if SYNA_SLIDE_WAKEUP
	if(syna_wakeup_flag && ((doze_status == DOZE_ENABLED) || (doze_status == DOZE_WAKEUP)))	//wakeup system by slide
		queue_delayed_work(rmi4_data->tp_workqueue,&rmi4_data->tp_work,5);
	else
	{
		if (!rmi4_data->touch_stopped)
		{
			//synaptics_rmi4_sensor_report(rmi4_data);
			queue_delayed_work(rmi4_data->tp_workqueue,&rmi4_data->tp_work,0);
		}
		//else
		//	synaptics_rmi4_irq_enable(rmi4_data, true);

	}
#else
	if (!rmi4_data->touch_stopped)
		//synaptics_rmi4_sensor_report(rmi4_data);
		queue_delayed_work(rmi4_data->tp_workqueue,&rmi4_data->tp_work,0);
//	else
//		synaptics_rmi4_irq_enable(rmi4_data, true);

#endif
	return IRQ_HANDLED;
}

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int synaptics_rmi4_regulator_configure(struct synaptics_rmi4_data
						*rmi4_data, bool on)
{
	int retval;

	if (on == false)
		goto hw_shutdown;

	rmi4_data->vdd = regulator_get(&rmi4_data->i2c_client->dev,
					"vdd");
	if (IS_ERR(rmi4_data->vdd)) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to get vdd regulator\n",
				__func__);
		return PTR_ERR(rmi4_data->vdd);
	}

	if (regulator_count_voltages(rmi4_data->vdd) > 0) {
		retval = regulator_set_voltage(rmi4_data->vdd,
			RMI4_VTG_MIN_UV, RMI4_VTG_MAX_UV);
		if (retval) {
			dev_err(&rmi4_data->i2c_client->dev,
				"regulator set_vtg failed retval =%d\n",
				retval);
			goto err_set_vtg_vdd;
		}
	}

	rmi4_data->vdd1 = regulator_get(&rmi4_data->i2c_client->dev,
					"vdd1");
	if (IS_ERR(rmi4_data->vdd1)) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to get vdd1 regulator\n",
				__func__);
		return PTR_ERR(rmi4_data->vdd1);
	}

	if (regulator_count_voltages(rmi4_data->vdd1) > 0) {
		retval = regulator_set_voltage(rmi4_data->vdd1,
			RMI4_VTG_MIN_UV, RMI4_VTG_MAX_UV);
		if (retval) {
			dev_err(&rmi4_data->i2c_client->dev,
				"regulator set_vtg failed retval =%d\n",
				retval);
			goto err_set_vtg_vdd1;
		}
	}
	if (rmi4_data->board->i2c_pull_up) {
		rmi4_data->vcc_i2c = regulator_get(&rmi4_data->i2c_client->dev,
						"vcc_i2c");
		if (IS_ERR(rmi4_data->vcc_i2c)) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to get i2c regulator\n",
					__func__);
			retval = PTR_ERR(rmi4_data->vcc_i2c);
			goto err_get_vtg_i2c;
		}

		if (regulator_count_voltages(rmi4_data->vcc_i2c) > 0) {
			retval = regulator_set_voltage(rmi4_data->vcc_i2c,
				RMI4_I2C_VTG_MIN_UV, RMI4_I2C_VTG_MAX_UV);
			if (retval) {
				dev_err(&rmi4_data->i2c_client->dev,
					"reg set i2c vtg failed retval =%d\n",
					retval);
			goto err_set_vtg_i2c;
			}
		}
	}
	return 0;

err_set_vtg_i2c:
	if (rmi4_data->board->i2c_pull_up)
		regulator_put(rmi4_data->vcc_i2c);
err_get_vtg_i2c:
	if (regulator_count_voltages(rmi4_data->vdd1) > 0)
		regulator_set_voltage(rmi4_data->vdd1, 0,
			RMI4_VTG_MAX_UV);
err_set_vtg_vdd1:
	regulator_put(rmi4_data->vdd1);
	if (regulator_count_voltages(rmi4_data->vdd) > 0)
		regulator_set_voltage(rmi4_data->vdd, 0,
			RMI4_VTG_MAX_UV);
err_set_vtg_vdd:
	regulator_put(rmi4_data->vdd);
	return retval;

hw_shutdown:
	if (regulator_count_voltages(rmi4_data->vdd) > 0)
		regulator_set_voltage(rmi4_data->vdd, 0,
			RMI4_VTG_MAX_UV);
	regulator_put(rmi4_data->vdd);
	if (regulator_count_voltages(rmi4_data->vdd1) > 0)
		regulator_set_voltage(rmi4_data->vdd1, 0,
			RMI4_VTG_MAX_UV);
	regulator_put(rmi4_data->vdd1);
	if (rmi4_data->board->i2c_pull_up) {
		if (regulator_count_voltages(rmi4_data->vcc_i2c) > 0)
			regulator_set_voltage(rmi4_data->vcc_i2c, 0,
					RMI4_I2C_VTG_MAX_UV);
		regulator_put(rmi4_data->vcc_i2c);
	}
	return 0;
};

static int synaptics_rmi4_reset_command(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int page_number;
	unsigned char command = 0x01;
	unsigned short pdt_entry_addr;
	struct synaptics_rmi4_fn_desc rmi_fd;
	bool done = false;

	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
				pdt_entry_addr -= PDT_ENTRY_SIZE) {
			retval = synaptics_rmi4_i2c_read(rmi4_data,
				pdt_entry_addr,
				(unsigned char *)&rmi_fd,
				sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			if (rmi_fd.fn_number == 0)
				break;

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				rmi4_data->f01_cmd_base_addr =
					rmi_fd.cmd_base_addr;
				done = true;
				break;
			}
		}
		if (done) {
			dev_info(&rmi4_data->i2c_client->dev,
				"%s: Find F01 in page description table 0x%x\n",
				__func__, rmi4_data->f01_cmd_base_addr);
			break;
		}
	}

	if (!done) {
		dev_err(&rmi4_data->i2c_client->dev,
			"%s: Cannot find F01 in page description table\n",
			__func__);
		return -EINVAL;
	}

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		return retval;
	}

	mdelay(RESET_DELAY);
	return retval;
};

static int synaptics_rmi4_parse_dt(struct device *dev,
				struct synaptics_rmi4_platform_data *rmi4_pdata)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_NUMBER_OF_BUTTONS];
	int rc, i;

	rmi4_pdata->i2c_pull_up = of_property_read_bool(np,
			"synaptics,i2c-pull-up");
	rmi4_pdata->x_flip = of_property_read_bool(np, "synaptics,x-flip");
	rmi4_pdata->y_flip = of_property_read_bool(np, "synaptics,y-flip");

	rc = of_property_read_u32(np, "synaptics,panel-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read panel X dimension\n");
		return rc;
	} else {
		rmi4_pdata->panel_x = temp_val;
	}

	rc = of_property_read_u32(np, "synaptics,panel-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read panel Y dimension\n");
		return rc;
	} else {
		rmi4_pdata->panel_y = temp_val;
	}

	rc = of_property_read_string(np, "synaptics,fw-image-name",
		&rmi4_pdata->fw_image_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw image name\n");
		return rc;
	}

	/* reset, irq gpio info */
	rmi4_pdata->reset_gpio = of_get_named_gpio_flags(np,
			"synaptics,reset-gpio", 0, &rmi4_pdata->reset_flags);
	rmi4_pdata->irq_gpio = of_get_named_gpio_flags(np,
			"synaptics,irq-gpio", 0, &rmi4_pdata->irq_flags);

	prop = of_find_property(np, "synaptics,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);

		rmi4_pdata->capacitance_button_map = devm_kzalloc(dev,
			sizeof(*rmi4_pdata->capacitance_button_map),
			GFP_KERNEL);
		if (!rmi4_pdata->capacitance_button_map)
			return -ENOMEM;

		rmi4_pdata->capacitance_button_map->map = devm_kzalloc(dev,
			sizeof(*rmi4_pdata->capacitance_button_map->map) *
			MAX_NUMBER_OF_BUTTONS, GFP_KERNEL);
		if (!rmi4_pdata->capacitance_button_map->map)
			return -ENOMEM;

		if (num_buttons <= MAX_NUMBER_OF_BUTTONS) {
			rc = of_property_read_u32_array(np,
				"synaptics,button-map", button_map,
				num_buttons);
			if (rc) {
				dev_err(dev, "Unable to read key codes\n");
				return rc;
			}
			for (i = 0; i < num_buttons; i++)
				rmi4_pdata->capacitance_button_map->map[i] =
					button_map[i];
			rmi4_pdata->capacitance_button_map->nbuttons =
				num_buttons;
		} else {
			return -EINVAL;
		}
	}
	return 0;
}

static int synaptics_rmi4_power_on(struct synaptics_rmi4_data *rmi4_data,
					bool on) {
	int retval;

	if (on == false)
		goto power_off;

	retval = reg_set_optimum_mode_check(rmi4_data->vdd,
		RMI4_ACTIVE_LOAD_UA);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
			"Regulator vdd set_opt failed rc=%d\n",
			retval);
		return retval;
	}

	retval = regulator_enable(rmi4_data->vdd);
	if (retval) {
		dev_err(&rmi4_data->i2c_client->dev,
			"Regulator vdd enable failed rc=%d\n",
			retval);
		goto error_reg_en_vdd;
	}

	retval = reg_set_optimum_mode_check(rmi4_data->vdd1,
		RMI4_ACTIVE_LOAD_UA);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
			"Regulator vdd1 set_opt failed rc=%d\n",
			retval);
		return retval;
	}

	retval = regulator_enable(rmi4_data->vdd1);
	if (retval) {
		dev_err(&rmi4_data->i2c_client->dev,
			"Regulator vdd1 enable failed rc=%d\n",
			retval);
		goto error_reg_en_vdd1;
	}
	if (rmi4_data->board->i2c_pull_up) {
		retval = reg_set_optimum_mode_check(rmi4_data->vcc_i2c,
			RMI4_I2C_LOAD_UA);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
				"Regulator vcc_i2c set_opt failed rc=%d\n",
				retval);
			goto error_reg_opt_i2c;
		}

		retval = regulator_enable(rmi4_data->vcc_i2c);
		if (retval) {
			dev_err(&rmi4_data->i2c_client->dev,
				"Regulator vcc_i2c enable failed rc=%d\n",
				retval);
			goto error_reg_en_vcc_i2c;
		}
	}
	return 0;

error_reg_en_vcc_i2c:
	if (rmi4_data->board->i2c_pull_up)
		reg_set_optimum_mode_check(rmi4_data->vcc_i2c, 0);
error_reg_opt_i2c:
	regulator_disable(rmi4_data->vdd1);
error_reg_en_vdd1:
	reg_set_optimum_mode_check(rmi4_data->vdd1, 0);
	regulator_disable(rmi4_data->vdd);
error_reg_en_vdd:
	reg_set_optimum_mode_check(rmi4_data->vdd, 0);
	return retval;

power_off:
	reg_set_optimum_mode_check(rmi4_data->vdd, 0);
	regulator_disable(rmi4_data->vdd);
	reg_set_optimum_mode_check(rmi4_data->vdd1, 0);
	regulator_disable(rmi4_data->vdd1);
	if (rmi4_data->board->i2c_pull_up) {
		reg_set_optimum_mode_check(rmi4_data->vcc_i2c, 0);
		regulator_disable(rmi4_data->vcc_i2c);
	}
	return 0;
}

 /**
 * synaptics_rmi4_irq_enable()
 *
 * Called by synaptics_rmi4_probe() and the power management functions
 * in this driver and also exported to other expansion Function modules
 * such as rmi_dev.
 *
 * This function handles the enabling and disabling of the attention
 * irq including the setting up of the ISR thread.
 */
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval = 0;
	unsigned char intr_status[MAX_INTR_REGISTERS];
	//SYNAP_DEBUG("%s,irq enable status:%d",__func__,rmi4_data->irq_enabled);
	if (enable) {
		if (rmi4_data->irq_enabled)
			return retval;

		/* Clear interrupts first */
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr + 1,
				intr_status,
				rmi4_data->num_of_intr_regs);
		if (retval < 0)
			return retval;

	    enable_irq(rmi4_data->irq);
		rmi4_data->irq_enabled = true;
//	SYNAP_DEBUG("%s,irq enabled",__func__);
	} else {
		if (rmi4_data->irq_enabled) {
			disable_irq_nosync(rmi4_data->irq);
			rmi4_data->irq_enabled = false;
//	SYNAP_DEBUG("%s,irq disabled",__func__);
		}
	}

	return retval;
}

 /**
 * synaptics_rmi4_f11_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 11 registers
 * and determines the number of fingers supported, x and y data ranges,
 * offset to the associated interrupt status register, interrupt bit
 * mask, and gathers finger data acquisition capabilities from the query
 * registers.
 */
static int synaptics_rmi4_f11_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	unsigned char ii;
	unsigned char intr_offset;
	unsigned char abs_data_size;
	unsigned char abs_data_blk_size;
	unsigned char query[F11_STD_QUERY_LEN];
	unsigned char control[F11_STD_CTRL_LEN];

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base,
			query,
			sizeof(query));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	if ((query[1] & MASK_3BIT) <= 4)
		fhandler->num_of_data_points = (query[1] & MASK_3BIT) + 1;
	else if ((query[1] & MASK_3BIT) == 5)
		fhandler->num_of_data_points = 10;

	rmi4_data->num_of_fingers = fhandler->num_of_data_points;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base,
			control,
			sizeof(control));
	if (retval < 0)
		return retval;

	/* Maximum x and y */
	rmi4_data->sensor_max_x = ((control[6] & MASK_8BIT) << 0) |
			((control[7] & MASK_4BIT) << 8);
	rmi4_data->sensor_max_y = ((control[8] & MASK_8BIT) << 0) |
			((control[9] & MASK_4BIT) << 8);
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);

	rmi4_data->max_touch_width = MAX_F11_TOUCH_WIDTH;

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++)
		fhandler->intr_mask |= 1 << ii;

	abs_data_size = query[5] & MASK_2BIT;
	abs_data_blk_size = 3 + (2 * (abs_data_size == 0 ? 1 : 0));
	fhandler->size_of_data_register_block = abs_data_blk_size;
	fhandler->data = NULL;
	fhandler->extra = NULL;

	return retval;
}

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28)
{
	int retval;
	static unsigned short ctrl_28_address;

	if (ctrl28)
		ctrl_28_address = ctrl28;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			ctrl_28_address,
			&rmi4_data->report_enable,
			sizeof(rmi4_data->report_enable));
	if (retval < 0)
		return retval;

	return retval;
}

 /**
 * synaptics_rmi4_f12_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 12 registers and
 * determines the number of fingers supported, offset to the data1
 * register, x and y data ranges, offset to the associated interrupt
 * status register, interrupt bit mask, and allocates memory resources
 * for finger data acquisition.
 */
static int synaptics_rmi4_f12_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	unsigned char ii;
	unsigned char intr_offset;
	unsigned char size_of_2d_data;
	unsigned char size_of_query8;
	unsigned char ctrl_8_offset;
	unsigned char ctrl_20_offset;
	unsigned char ctrl_23_offset;
	unsigned char ctrl_28_offset;
	unsigned char num_of_fingers;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_query_5 query_5;
	struct synaptics_rmi4_f12_query_8 query_8;
	struct synaptics_rmi4_f12_ctrl_8 ctrl_8;
//	struct synaptics_rmi4_f12_ctrl_9 ctrl_9;
	struct synaptics_rmi4_f12_ctrl_23 ctrl_23;
//	unsigned char sen[2]={0xff,0x04}; 
	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->extra = kmalloc(sizeof(*extra_data), GFP_KERNEL);
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base + 5,
			query_5.data,
			sizeof(query_5.data));
	if (retval < 0)
		return retval;

	ctrl_8_offset = query_5.ctrl0_is_present +
			query_5.ctrl1_is_present +
			query_5.ctrl2_is_present +
			query_5.ctrl3_is_present +
			query_5.ctrl4_is_present +
			query_5.ctrl5_is_present +
			query_5.ctrl6_is_present +
			query_5.ctrl7_is_present;
	
//	ctrl_9_offset = ctrl_8_offset +
//			query_5.ctrl8_is_present;	
		
	ctrl_20_offset = ctrl_8_offset +
			query_5.ctrl8_is_present +
			query_5.ctrl9_is_present +
			query_5.ctrl10_is_present +
			query_5.ctrl11_is_present +
			query_5.ctrl12_is_present +
			query_5.ctrl13_is_present +
			query_5.ctrl14_is_present +
			query_5.ctrl15_is_present +
			query_5.ctrl16_is_present +
			query_5.ctrl17_is_present +
			query_5.ctrl18_is_present +
			query_5.ctrl19_is_present;

	ctrl_23_offset = ctrl_20_offset +
			query_5.ctrl20_is_present +
			query_5.ctrl21_is_present +
			query_5.ctrl22_is_present;

	ctrl_28_offset = ctrl_23_offset +
			query_5.ctrl23_is_present +
			query_5.ctrl24_is_present +
			query_5.ctrl25_is_present +
			query_5.ctrl26_is_present +
			query_5.ctrl27_is_present;

	rmi4_data->f12_ctrl_20_offset = ctrl_20_offset;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_23_offset,
			ctrl_23.data,
			sizeof(ctrl_23.data));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	fhandler->num_of_data_points = min(ctrl_23.max_reported_objects,
			(unsigned char)F12_FINGERS_TO_SUPPORT);

	num_of_fingers = fhandler->num_of_data_points;
	rmi4_data->num_of_fingers = num_of_fingers;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base + 7,
			&size_of_query8,
			sizeof(size_of_query8));
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base + 8,
			query_8.data,
			size_of_query8);
	if (retval < 0)
		return retval;

	/* Determine the presence of the Data0 register */
	extra_data->data1_offset = query_8.data0_is_present;
#if SYNA_SLIDE_WAKEUP
	extra_data->data4_offset = query_8.data0_is_present+
							   query_8.data1_is_present+
							   query_8.data2_is_present+
							   query_8.data3_is_present;
#endif
	if ((size_of_query8 >= 3) && (query_8.data15_is_present)) {
		extra_data->data15_offset = query_8.data0_is_present +
				query_8.data1_is_present +
				query_8.data2_is_present +
				query_8.data3_is_present +
				query_8.data4_is_present +
				query_8.data5_is_present +
				query_8.data6_is_present +
				query_8.data7_is_present +
				query_8.data8_is_present +
				query_8.data9_is_present +
				query_8.data10_is_present +
				query_8.data11_is_present +
				query_8.data12_is_present +
				query_8.data13_is_present +
				query_8.data14_is_present;
		extra_data->data15_size = (num_of_fingers + 7) / 8;
	} else {
		extra_data->data15_size = 0;
	}

	rmi4_data->report_enable = RPT_DEFAULT;
#ifdef REPORT_2D_Z
	rmi4_data->report_enable |= RPT_Z;
#endif
#ifdef REPORT_2D_W
	rmi4_data->report_enable |= (RPT_WX | RPT_WY);
#endif

	retval = synaptics_rmi4_f12_set_enables(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_28_offset);
	if (retval < 0)
		return retval;
	//SYNAP_DEBUG("ctrl addr is 0x%x,ctrl9 offset is %d",fhandler->full_addr.ctrl_base,ctrl_9_offset);
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_8_offset,
			ctrl_8.data,
			sizeof(ctrl_8.data));
	if (retval < 0)
		return retval;
/*
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_9_offset,
			ctrl_9.data,
			sizeof(ctrl_9.data));
	if (retval < 0)
		return retval;
	SYNAP_DEBUG("ctrl9 0-0 is %d,0-1 is %d",ctrl_9.ctrl0_0,ctrl_9.ctrl0_1);
	mdelay(100);
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			0x0015,
			ctrl_9.data,
			sizeof(ctrl_9.data));
	if (retval < 0)
		return retval;
	SYNAP_DEBUG("ctrl9 0-0 is %d,0-1 is %d",ctrl_9.ctrl0_0,ctrl_9.ctrl0_1);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_9_offset,
			sen,
			sizeof(sen));
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_9_offset,
			ctrl_9.data,
			sizeof(ctrl_9.data));
	if (retval < 0)
		return retval;
	SYNAP_DEBUG("ctrl9 0-0 is %d,0-1 is %d",ctrl_9.ctrl0_0,ctrl_9.ctrl0_1);
	mdelay(100);
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			0x0015,
			ctrl_9.data,
			sizeof(ctrl_9.data));
	if (retval < 0)
		return retval;
	SYNAP_DEBUG("ctrl9 0-0 is %d,0-1 is %d",ctrl_9.ctrl0_0,ctrl_9.ctrl0_1);
*/	
	/* Maximum x and y */
	rmi4_data->sensor_max_x =
			((unsigned short)ctrl_8.max_x_coord_lsb << 0) |
			((unsigned short)ctrl_8.max_x_coord_msb << 8);
	rmi4_data->sensor_max_y =
			((unsigned short)ctrl_8.max_y_coord_lsb << 0) |
			((unsigned short)ctrl_8.max_y_coord_msb << 8);
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);

	rmi4_data->num_of_rx = ctrl_8.num_of_rx;
	rmi4_data->num_of_tx = ctrl_8.num_of_tx;
	rmi4_data->max_touch_width = max(rmi4_data->num_of_rx,
			rmi4_data->num_of_tx);

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++)
		fhandler->intr_mask |= 1 << ii;

	/* Allocate memory for finger data storage space */
	fhandler->data_size = num_of_fingers * size_of_2d_data;
	fhandler->data = kmalloc(fhandler->data_size, GFP_KERNEL);

	return retval;
}

static int synaptics_rmi4_f1a_alloc_mem(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	struct synaptics_rmi4_f1a_handle *f1a;

	f1a = kzalloc(sizeof(*f1a), GFP_KERNEL);
	if (!f1a) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for function handle\n",
				__func__);
		return -ENOMEM;
	}

	fhandler->data = (void *)f1a;
	fhandler->extra = NULL;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base,
			f1a->button_query.data,
			sizeof(f1a->button_query.data));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read query registers\n",
				__func__);
		return retval;
	}

	f1a->max_count = f1a->button_query.max_button_count + 1;

	f1a->button_control.txrx_map = kzalloc(f1a->max_count * 2, GFP_KERNEL);
	if (!f1a->button_control.txrx_map) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for tx rx mapping\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_bitmask_size = (f1a->max_count + 7) / 8;

	f1a->button_data_buffer = kcalloc(f1a->button_bitmask_size,
			sizeof(*(f1a->button_data_buffer)), GFP_KERNEL);
	if (!f1a->button_data_buffer) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for data buffer\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_map = kcalloc(f1a->max_count,
			sizeof(*(f1a->button_map)), GFP_KERNEL);
	if (!f1a->button_map) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for button map\n",
				__func__);
		return -ENOMEM;
	}

	return 0;
}

static int synaptics_rmi4_f1a_button_map(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char ii;
	unsigned char mapping_offset = 0;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	const struct synaptics_rmi4_platform_data *pdata = rmi4_data->board;

	mapping_offset = f1a->button_query.has_general_control +
			f1a->button_query.has_interrupt_enable +
			f1a->button_query.has_multibutton_select;

	if (f1a->button_query.has_tx_rx_map) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				fhandler->full_addr.ctrl_base + mapping_offset,
				f1a->button_control.txrx_map,
				sizeof(f1a->button_control.txrx_map));
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to read tx rx mapping\n",
					__func__);
			return retval;
		}

		rmi4_data->button_txrx_mapping = f1a->button_control.txrx_map;
	}

	if (!pdata->capacitance_button_map) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: capacitance_button_map is NULL in board file\n",
				__func__);
		return -ENODEV;
	} else if (!pdata->capacitance_button_map->map) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Button map is missing in board file\n",
				__func__);
		return -ENODEV;
	} else {
		if (pdata->capacitance_button_map->nbuttons != f1a->max_count) {
			f1a->valid_button_count = min(f1a->max_count,
					pdata->capacitance_button_map->nbuttons);
		} else {
			f1a->valid_button_count = f1a->max_count;
		}

		for (ii = 0; ii < f1a->valid_button_count; ii++)
			f1a->button_map[ii] = pdata->capacitance_button_map->map[ii];
	}

	return 0;
}

static void synaptics_rmi4_f1a_kfree(struct synaptics_rmi4_fn *fhandler)
{
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

	if (f1a) {
		kfree(f1a->button_control.txrx_map);
		kfree(f1a->button_data_buffer);
		kfree(f1a->button_map);
		kfree(f1a);
		fhandler->data = NULL;
	}

	return;
}

static int synaptics_rmi4_f1a_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	unsigned char ii;
	unsigned short intr_offset;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++)
		fhandler->intr_mask |= 1 << ii;

	retval = synaptics_rmi4_f1a_alloc_mem(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	retval = synaptics_rmi4_f1a_button_map(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	rmi4_data->button_0d_enabled = 1;

	return 0;

error_exit:
	synaptics_rmi4_f1a_kfree(fhandler);

	return retval;
}

static int synaptics_rmi4_check_status(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int timeout = CHECK_STATUS_TIMEOUT_MS;
	unsigned char command = 0x01;
	unsigned char intr_status;
	struct synaptics_rmi4_f01_device_status status;

	/* Do a device reset first */
	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0)
		return retval;

	mdelay(rmi4_data->board->reset_delay_ms);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			status.data,
			sizeof(status.data));
	if (retval < 0)
		return retval;

	while (status.status_code == STATUS_CRC_IN_PROGRESS) {
		if (timeout > 0)
			mdelay(20);
		else
			return -1;

		retval = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr,
				status.data,
				sizeof(status.data));
		if (retval < 0)
			return retval;

		timeout -= 20;
	}

	if (status.flash_prog == 1) {
		rmi4_data->flash_prog_mode = true;
		pr_notice("%s: In flash prog mode, status = 0x%02x\n",
				__func__,
				status.status_code);
	} else {
		rmi4_data->flash_prog_mode = false;
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr + 1,
			&intr_status,
			sizeof(intr_status));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read interrupt status\n",
				__func__);
		return retval;
	}

	return 0;
}

static void synaptics_rmi4_set_configured(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to set configured\n",
				__func__);
		return;
	}

	rmi4_data->no_sleep_setting = device_ctrl & NO_SLEEP_ON;
	device_ctrl |= CONFIGURED;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to set configured\n",
				__func__);
	}

	return;
}

static int synaptics_rmi4_alloc_fh(struct synaptics_rmi4_fn **fhandler,
		struct synaptics_rmi4_fn_desc *rmi_fd, int page_number)
{
	*fhandler = kmalloc(sizeof(**fhandler), GFP_KERNEL);
	if (!(*fhandler))
		return -ENOMEM;

	(*fhandler)->full_addr.data_base =
			(rmi_fd->data_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.ctrl_base =
			(rmi_fd->ctrl_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.cmd_base =
			(rmi_fd->cmd_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.query_base =
			(rmi_fd->query_base_addr |
			(page_number << 8));

	return 0;
}

 /**
 * synaptics_rmi4_query_device()
 *
 * Called by synaptics_rmi4_probe().
 *
 * This funtion scans the page description table, records the offsets
 * to the register types of Function $01, sets up the function handlers
 * for Function $11 and Function $12, determines the number of interrupt
 * sources from the sensor, adds valid Functions with data inputs to the
 * Function linked list, parses information from the query registers of
 * Function $01, and enables the interrupt sources from the valid Functions
 * with data inputs.
 */
static int synaptics_rmi4_query_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char ii;
	unsigned char page_number;
	unsigned char intr_count = 0;
	unsigned char f01_query[F01_STD_QUERY_LEN];
	unsigned short pdt_entry_addr;
	unsigned short intr_addr;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;
	unsigned char vendor_id = 0;

	rmi = &(rmi4_data->rmi4_mod_info);

	INIT_LIST_HEAD(&rmi->support_fn_list);

	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
				pdt_entry_addr -= PDT_ENTRY_SIZE) {
			pdt_entry_addr |= (page_number << 8);

			retval = synaptics_rmi4_i2c_read(rmi4_data,
					pdt_entry_addr,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0)
				return retval;
			fhandler = NULL;

			if (rmi_fd.fn_number == 0) {
				dev_dbg(&rmi4_data->i2c_client->dev,
						"%s: Reached end of PDT\n",
						__func__);
				break;
			}

			dev_dbg(&rmi4_data->i2c_client->dev,
					"%s: F%02x found (page %d)\n",
					__func__, rmi_fd.fn_number,
					page_number);

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				rmi4_data->f01_query_base_addr =
						rmi_fd.query_base_addr;
				rmi4_data->f01_ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				rmi4_data->f01_data_base_addr =
						rmi_fd.data_base_addr;
				rmi4_data->f01_cmd_base_addr =
						rmi_fd.cmd_base_addr;

				//SYNAP_DEBUG("F01 query base:%x,ctrl base:%x,data base:%x,cmd base %x",rmi4_data->f01_query_base_addr,rmi4_data->f01_ctrl_base_addr,rmi4_data->f01_data_base_addr,rmi4_data->f01_cmd_base_addr);
				retval = synaptics_rmi4_check_status(rmi4_data);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
							"%s: Failed to check status\n",
							__func__);
					return retval;
				}

				if (rmi4_data->flash_prog_mode)
					goto flash_prog_mode;
			retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_query_base_addr + VENDOR_ID_OFFSET,
			&vendor_id,
			sizeof(vendor_id));
			if (retval < 0) {
				dev_dbg(&rmi4_data->i2c_client->dev,
				"%s: Failed to read vendor id from device\n",
				__func__);
						}
				SYNA_VENDOR_ID = vendor_id;
				SYNAP_DEBUG("Vendor id is :%d",vendor_id);

				break;
			case SYNAPTICS_RMI4_F11:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f11_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F12:
				if (rmi_fd.intr_src_count == 0)
					break;
				
				rmi4_data->f12_query_base_addr =
						rmi_fd.query_base_addr;
				rmi4_data->f12_ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				rmi4_data->f12_data_base_addr =
						rmi_fd.data_base_addr;
				rmi4_data->f12_cmd_base_addr =
						rmi_fd.cmd_base_addr;
				//SYNAP_DEBUG("F12 query base:%x,ctrl base:%x,data base:%x,cmd base %x",rmi4_data->f12_query_base_addr,rmi4_data->f12_ctrl_base_addr,rmi4_data->f12_data_base_addr,rmi4_data->f12_cmd_base_addr);
				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f12_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F34:
				rmi4_data->f34_query_base_addr =
						rmi_fd.query_base_addr;
				rmi4_data->f34_ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				rmi4_data->f34_data_base_addr =
						rmi_fd.data_base_addr;
				rmi4_data->f34_cmd_base_addr =
						rmi_fd.cmd_base_addr;
				retval = synaptics_rmi4_i2c_read(rmi4_data,
						rmi_fd.ctrl_base_addr,
						config_id,4);
				if (retval < 0) {
					dev_dbg(&rmi4_data->i2c_client->dev,
							"%s: Failed to read configID from device\n",
							__func__);
				} 
			SYNAP_DEBUG(" device_config_id 1-2-3-4: %x %x %x %x.",config_id[0],
								config_id[1],config_id[2],config_id[3]);
				SYNA_FW_VERSION = config_id[3];
				break;
			case SYNAPTICS_RMI4_F51:
				rmi4_data->f51_data_base_addr =
						page_number << 8 |rmi_fd.data_base_addr;
				//SYNAP_DEBUG("F51 DATA BASE IS :0X%x",rmi4_data->f51_data_base_addr);

				break;
			case SYNAPTICS_RMI4_F54:
				rmi4_data->f54_cmd_base_addr =
						page_number << 8 |rmi_fd.cmd_base_addr;
				break;
			case SYNAPTICS_RMI4_F1A:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f1a_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0) {
#ifdef IGNORE_FN_INIT_FAILURE
					kfree(fhandler);
					fhandler = NULL;
#else
					return retval;
#endif
				}
				break;
			}

			/* Accumulate the interrupt count */
			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);

			if (fhandler && rmi_fd.intr_src_count) {
				list_add_tail(&fhandler->link,
						&rmi->support_fn_list);
			}
		}
	}

flash_prog_mode:
	rmi4_data->num_of_intr_regs = (intr_count + 7) / 8;
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Number of interrupt registers = %d\n",
			__func__, rmi4_data->num_of_intr_regs);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_query_base_addr,
			f01_query,
			sizeof(f01_query));
	if (retval < 0)
		return retval;

	/* RMI Version 4.0 currently supported */
	rmi->version_major = 4;
	rmi->version_minor = 0;

	rmi->manufacturer_id = f01_query[0];
	rmi->product_props = f01_query[1];
	rmi->product_info[0] = f01_query[2] & MASK_7BIT;
	rmi->product_info[1] = f01_query[3] & MASK_7BIT;
	rmi->date_code[0] = f01_query[4] & MASK_5BIT;
	rmi->date_code[1] = f01_query[5] & MASK_4BIT;
	rmi->date_code[2] = f01_query[6] & MASK_5BIT;
	rmi->tester_id = ((f01_query[7] & MASK_7BIT) << 8) |
			(f01_query[8] & MASK_7BIT);
	rmi->serial_number = ((f01_query[9] & MASK_7BIT) << 8) |
			(f01_query[10] & MASK_7BIT);
	memcpy(rmi->product_id_string, &f01_query[11], 10);

	if (rmi->manufacturer_id != 1) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Non-Synaptics device found, manufacturer ID = %d\n",
				__func__, rmi->manufacturer_id);
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_query_base_addr + F01_BUID_ID_OFFSET,
			rmi->build_id,
			sizeof(rmi->build_id));
	if (retval < 0)
		return retval;

	memset(rmi4_data->intr_mask, 0x00, sizeof(rmi4_data->intr_mask));

	/*
	 * Map out the interrupt bit masks for the interrupt sources
	 * from the registered function handlers.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				rmi4_data->intr_mask[fhandler->intr_reg_num] |=
						fhandler->intr_mask;
			}
		}
	}

	/* Enable the interrupt sources */
	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (rmi4_data->intr_mask[ii] != 0x00) {
			dev_dbg(&rmi4_data->i2c_client->dev,
					"%s: Interrupt enable mask %d = 0x%02x\n",
					__func__, ii, rmi4_data->intr_mask[ii]);
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			retval = synaptics_rmi4_i2c_write(rmi4_data,
					intr_addr,
					&(rmi4_data->intr_mask[ii]),
					sizeof(rmi4_data->intr_mask[ii]));
			if (retval < 0)
				return retval;
		}
	}

	synaptics_rmi4_set_configured(rmi4_data);

	return 0;
}

static int synaptics_rmi4_set_input_dev(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int temp;
	unsigned char ii;
	struct synaptics_rmi4_f1a_handle *f1a;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	rmi4_data->input_dev = input_allocate_device();
	if (rmi4_data->input_dev == NULL) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to allocate input device\n",
				__func__);
		retval = -ENOMEM;
		goto err_input_device;
	}
/*
	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to query device\n",
				__func__);
		goto err_query_device;
	}
*/
	rmi4_data->input_dev->name = DRIVER_NAME;
	rmi4_data->input_dev->phys = INPUT_PHYS_NAME;
	rmi4_data->input_dev->id.product = SYNAPTICS_DSX_DRIVER_PRODUCT;
	rmi4_data->input_dev->id.version = SYNAPTICS_DSX_DRIVER_VERSION;
	rmi4_data->input_dev->id.bustype = BUS_I2C;
	rmi4_data->input_dev->dev.parent = &rmi4_data->i2c_client->dev;
	input_set_drvdata(rmi4_data->input_dev, rmi4_data);

	set_bit(EV_SYN, rmi4_data->input_dev->evbit);
	set_bit(EV_KEY, rmi4_data->input_dev->evbit);
	set_bit(EV_ABS, rmi4_data->input_dev->evbit);
	set_bit(BTN_TOUCH, rmi4_data->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, rmi4_data->input_dev->keybit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);
#endif

	if (rmi4_data->board->swap_axes) {
		temp = rmi4_data->sensor_max_x;
		rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
		rmi4_data->sensor_max_y = temp;
	}
if(!rmi4_data->flash_prog_mode)
{
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_X, 0,
			rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, 0,
			rmi4_data->sensor_max_y, 0, 0);
#ifdef REPORT_2D_W
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, 0,
			rmi4_data->max_touch_width, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MINOR, 0,
			rmi4_data->max_touch_width, 0, 0);
#endif

#ifdef TYPE_B_PROTOCOL
	input_mt_init_slots(rmi4_data->input_dev,
			rmi4_data->num_of_fingers);
#endif

	f1a = NULL;
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				f1a = fhandler->data;
		}
	}

	if (f1a) {
		for (ii = 0; ii < f1a->valid_button_count; ii++) {
			set_bit(f1a->button_map[ii],
					rmi4_data->input_dev->keybit);
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, f1a->button_map[ii]);
		}
	}
}
else
{

	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_X, 0,
			SYNAP_MAX_X, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, 0,
			SYNAP_MAX_Y, 0, 0);
#ifdef REPORT_2D_W
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, 0,
			SYNAP_MT_TOUCH, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MINOR, 0,
			SYNAP_MT_TOUCH, 0, 0);
#endif

#ifdef TYPE_B_PROTOCOL
	input_mt_init_slots(rmi4_data->input_dev,
			SYNAP_MAX_FINGER);
#endif
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, KEY_MENU);
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, KEY_HOME);
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, KEY_BACK);
}
#if SYNA_SLIDE_WAKEUP
	//input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_POWER);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_M);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_C);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_LR);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_RL);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_DT);
#endif
	retval = input_register_device(rmi4_data->input_dev);
	if (retval) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to register input device\n",
				__func__);
		goto err_register_input;
	}

	return 0;

err_register_input:
	input_free_device(rmi4_data->input_dev);
/*
err_query_device:
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
				synaptics_rmi4_f1a_kfree(fhandler);
			} else {
				kfree(fhandler->extra);
				kfree(fhandler->data);
			}
			kfree(fhandler);
		}
	}
*/
err_input_device:
	return retval;
}

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char ii;
#ifdef TYPE_B_PROTOCOL
	for (ii = 0; ii < rmi4_data->num_of_fingers; ii++) {
		input_mt_slot(rmi4_data->input_dev, ii);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(rmi4_data->input_dev,
			BTN_TOUCH, 0);
	input_report_key(rmi4_data->input_dev,
			BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(rmi4_data->input_dev);
#endif
	input_sync(rmi4_data->input_dev);

	rmi4_data->fingers_on_2d = false;

	return 0;
}

#ifdef TYPE_B_PROTOCOL
static int synaptics_rmi4_free_abnormall_fingers(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char ii;
//	printk("%s,\n",__func__);
	for (ii = 0; ii < rmi4_data->num_of_fingers; ii++) {
		input_mt_slot(rmi4_data->input_dev, ii);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, 1);	
		input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X,0);
		input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y,0);
		input_sync(rmi4_data->input_dev);
		
		input_mt_slot(rmi4_data->input_dev, ii);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, 0);	
		input_report_key(rmi4_data->input_dev,
			BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,
			BTN_TOOL_FINGER, 0);
		input_sync(rmi4_data->input_dev);

	}
	return 0;
}
#endif
static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char ii;
	unsigned short intr_addr;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	mutex_lock(&(rmi4_data->rmi4_reset_mutex));

	synaptics_rmi4_free_fingers(rmi4_data);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F12) {
				synaptics_rmi4_f12_set_enables(rmi4_data, 0);
				break;
			}
		}
	}

	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (rmi4_data->intr_mask[ii] != 0x00) {
			dev_dbg(&rmi4_data->i2c_client->dev,
					"%s: Interrupt enable mask %d = 0x%02x\n",
					__func__, ii, rmi4_data->intr_mask[ii]);
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			retval = synaptics_rmi4_i2c_write(rmi4_data,
					intr_addr,
					&(rmi4_data->intr_mask[ii]),
					sizeof(rmi4_data->intr_mask[ii]));
			if (retval < 0)
				goto exit;
		}
	}

	synaptics_rmi4_set_configured(rmi4_data);

	retval = 0;

exit:
	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
	return retval;
}

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data,
		unsigned short f01_cmd_base_addr)
{
	int retval;
	int temp;
	unsigned char ii;
	unsigned char command = 0x01;
	struct synaptics_rmi4_f1a_handle *f1a;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	mutex_lock(&(rmi4_data->rmi4_reset_mutex));

	synaptics_rmi4_irq_enable(rmi4_data, false);

	rmi4_data->touch_stopped = true;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
		return retval;
	}

	mdelay(rmi4_data->board->reset_delay_ms);

	synaptics_rmi4_free_fingers(rmi4_data);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
				synaptics_rmi4_f1a_kfree(fhandler);
			} else {
				kfree(fhandler->extra);
				kfree(fhandler->data);
			}
			kfree(fhandler);
		}
	}

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to query device\n",
				__func__);
		mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
		return retval;
	}

	if (rmi4_data->board->swap_axes) {
		temp = rmi4_data->sensor_max_x;
		rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
		rmi4_data->sensor_max_y = temp;
	}

	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_X, 0,
			rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, 0,
			rmi4_data->sensor_max_y, 0, 0);
#ifdef REPORT_2D_W
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, 0,
			rmi4_data->max_touch_width, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MINOR, 0,
			rmi4_data->max_touch_width, 0, 0);
#endif

	f1a = NULL;
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				f1a = fhandler->data;
		}
	}

	if (f1a) {
		for (ii = 0; ii < f1a->valid_button_count; ii++) {
			set_bit(f1a->button_map[ii],
					rmi4_data->input_dev->keybit);
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, f1a->button_map[ii]);
		}
	}

	rmi4_data->touch_stopped = false;
#if SYNA_SLIDE_WAKEUP
	if(syna_wakeup_flag && doze_status != DOZE_DISABLED)
	synaptics_rmi4_sensor_doze(rmi4_data);
#endif

	synaptics_rmi4_irq_enable(rmi4_data, true);

	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));

	return 0;
}

/**
* synaptics_rmi4_exp_fn_work()
*
* Called by the kernel at the scheduled time.
*
* This function is a work thread that checks for the insertion and
* removal of other expansion Function modules such as rmi_dev and calls
* their initialization and removal callback functions accordingly.
*/
static void synaptics_rmi4_exp_fn_work(struct work_struct *work)
{
	struct synaptics_rmi4_exp_fn *exp_fhandler, *next_list_entry;
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry_safe(exp_fhandler,
				next_list_entry,
				&exp_data.list,
				link) {
			if ((exp_fhandler->func_init != NULL) &&
					(exp_fhandler->inserted == false)) {
				exp_fhandler->func_init(rmi4_data);
				exp_fhandler->inserted = true;
			} else if ((exp_fhandler->func_init == NULL) &&
					(exp_fhandler->inserted == true)) {
				exp_fhandler->func_remove(rmi4_data);
				list_del(&exp_fhandler->link);
				kfree(exp_fhandler);
			}
		}
	}
	mutex_unlock(&exp_data.mutex);

	return;
}

/**
* synaptics_rmi4_new_function()
*
* Called by other expansion Function modules in their module init and
* module exit functions.
*
* This function is used by other expansion Function modules such as
* rmi_dev to register themselves with the driver by providing their
* initialization and removal callback function pointers so that they
* can be inserted or removed dynamically at module init and exit times,
* respectively.
*/
void synaptics_rmi4_new_function(enum exp_fn fn_type, bool insert,
		int (*func_init)(struct synaptics_rmi4_data *rmi4_data),
		void (*func_remove)(struct synaptics_rmi4_data *rmi4_data),
		void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask))
{
	struct synaptics_rmi4_exp_fn *exp_fhandler;

	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = true;
	}

	mutex_lock(&exp_data.mutex);
	if (insert) {
		exp_fhandler = kzalloc(sizeof(*exp_fhandler), GFP_KERNEL);
		if (!exp_fhandler) {
			pr_err("%s: Failed to alloc mem for expansion function\n",
					__func__);
			goto exit;
		}
		exp_fhandler->fn_type = fn_type;
		exp_fhandler->func_init = func_init;
		exp_fhandler->func_attn = func_attn;
		exp_fhandler->func_remove = func_remove;
		exp_fhandler->inserted = false;
		list_add_tail(&exp_fhandler->link, &exp_data.list);
	} else if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (exp_fhandler->fn_type == fn_type) {
				exp_fhandler->func_init = NULL;
				exp_fhandler->func_attn = NULL;
				goto exit;
			}
		}
	}

exit:
	mutex_unlock(&exp_data.mutex);

	if (exp_data.queue_work) {
		queue_delayed_work(exp_data.workqueue,
				&exp_data.work,
				msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
	}

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_new_function);
#ifdef CONFIG_FB
static void configure_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	int retval = 0;

	rmi4_data->fb_notif.notifier_call = fb_notifier_callback;

	retval = fb_register_client(&rmi4_data->fb_notif);
	if (retval)
		dev_err(&rmi4_data->i2c_client->dev,
			"Unable to register fb_notifier: %d\n", retval);
	return;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void configure_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	rmi4_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	rmi4_data->early_suspend.suspend = synaptics_rmi4_early_suspend;
	rmi4_data->early_suspend.resume = synaptics_rmi4_late_resume;
	register_early_suspend(&rmi4_data->early_suspend);

	return;
}
#else
static void configure_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	return;
}
#endif
 /**
 * synaptics_rmi4_probe()
 *
 * Called by the kernel when an association with an I2C device of the
 * same name is made (after doing i2c_add_driver).
 *
 * This funtion allocates and initializes the resources for the driver
 * as an input driver, turns on the power to the sensor, queries the
 * sensor for its supported Functions and characteristics, registers
 * the driver to the input subsystem, sets up the interrupt, handles
 * the registration of the early_suspend and late_resume functions,
 * and creates a work queue for detection of other expansion Function
 * modules.
 */
static int __devinit synaptics_rmi4_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;
	unsigned char attr_count;
    struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_device_info *rmi;
	struct synaptics_rmi4_platform_data *platform_data =
			client->dev.platform_data;
	unsigned char intr_status[MAX_INTR_REGISTERS];
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data not supported\n",
				__func__);
		return -EIO;
	}

	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
			sizeof(*platform_data),
			GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		retval = synaptics_rmi4_parse_dt(&client->dev, platform_data);
		if (retval)
			return retval;
		platform_data->reset_delay_ms = 100;
	} else {
		platform_data = client->dev.platform_data;
	}

	if (!platform_data) {
		dev_err(&client->dev,
				"%s: No platform data found\n",
				__func__);
		return -EINVAL;
	}

	rmi4_data = kzalloc(sizeof(*rmi4_data), GFP_KERNEL);
	if (!rmi4_data) {
		dev_err(&client->dev,
				"%s: Failed to alloc mem for rmi4_data\n",
				__func__);
		return -ENOMEM;
	}

	rmi = &(rmi4_data->rmi4_mod_info);
/*
	if (platform_data->regulator_en) {
		rmi4_data->regulator = regulator_get(&client->dev, "vdd");
		if (IS_ERR(rmi4_data->regulator)) {
			dev_err(&client->dev,
					"%s: Failed to get regulator\n",
					__func__);
			retval = PTR_ERR(rmi4_data->regulator);
			goto err_regulator;
		}
		regulator_enable(rmi4_data->regulator);
		mdelay(platform_data->reset_delay_ms);
	}
*/
	rmi4_data->i2c_client = client;
	rmi4_data->current_page = MASK_8BIT;
	rmi4_data->board = platform_data;
	rmi4_data->touch_stopped = false;
	rmi4_data->sensor_sleep = false;
	rmi4_data->irq_enabled = false;
	rmi4_data->fingers_on_2d = false;

	rmi4_data->i2c_read = synaptics_rmi4_i2c_read;
	rmi4_data->i2c_write = synaptics_rmi4_i2c_write;
	rmi4_data->irq_enable = synaptics_rmi4_irq_enable;
	rmi4_data->reset_device = synaptics_rmi4_reset_device;

	mutex_init(&(rmi4_data->rmi4_io_ctrl_mutex));
	mutex_init(&(rmi4_data->rmi4_reset_mutex));

	retval = synaptics_rmi4_regulator_configure(rmi4_data, true);
	if (retval < 0) {
		dev_err(&client->dev, "Failed to configure regulators\n");
		goto err_reg_configure;
	}

	retval = synaptics_rmi4_power_on(rmi4_data, true);
	if (retval < 0) {
		dev_err(&client->dev, "Failed to power on\n");
		goto err_power_device;
	}

	if (gpio_is_valid(platform_data->irq_gpio)) {
		/* configure touchscreen irq gpio */
		retval = gpio_request(platform_data->irq_gpio, "rmi4_irq_gpio");
		if (retval) {
			dev_err(&client->dev, "unable to request gpio [%d]\n",
						platform_data->irq_gpio);
			goto err_irq_gpio_req;
		}
		retval = gpio_direction_input(platform_data->irq_gpio);
		if (retval) {
			dev_err(&client->dev,
				"unable to set direction for gpio [%d]\n",
				platform_data->irq_gpio);
			goto err_irq_gpio_dir;
		}
	} else {
		dev_err(&client->dev, "irq gpio not provided\n");
		goto err_irq_gpio_req;
	}

	if (gpio_is_valid(platform_data->reset_gpio)) {
		/* configure touchscreen reset out gpio */
		retval = gpio_request(platform_data->reset_gpio,
				"rmi4_reset_gpio");
		if (retval) {
			dev_err(&client->dev, "unable to request gpio [%d]\n",
						platform_data->reset_gpio);
			goto err_irq_gpio_dir;
		}

		retval = gpio_direction_output(platform_data->reset_gpio, 1);
		if (retval) {
			dev_err(&client->dev,
				"unable to set direction for gpio [%d]\n",
				platform_data->reset_gpio);
			goto err_reset_gpio_dir;
		}

		gpio_set_value(platform_data->reset_gpio, 0);
		mdelay(RMI4_GPIO_SLEEP_LOW_MS);
		gpio_set_value(platform_data->reset_gpio, 1);
		mdelay(RESET_DELAY);
	} else
		synaptics_rmi4_reset_command(rmi4_data);

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(&client->dev,
				"%s: Failed to query device\n",
				__func__);
		goto err_reset_gpio_dir;
	}

	i2c_set_clientdata(client, rmi4_data);

	retval = synaptics_rmi4_set_input_dev(rmi4_data);
	if (retval < 0) {
		//no_touch = 1;
		dev_err(&client->dev,
				"%s: Failed to set up input device\n",
				__func__);
		goto err_set_input_dev;
	}

	t_dev = &rmi4_data->input_dev->dev;
	configure_sleep(rmi4_data);
	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = true;
	}
	rmi4_data->tp_workqueue = create_singlethread_workqueue("syna_workqueue");
	INIT_DELAYED_WORK(&rmi4_data->tp_work, synap_work_func);
	exp_data.workqueue = create_singlethread_workqueue("dsx_exp_workqueue");
	INIT_DELAYED_WORK(&exp_data.work, synaptics_rmi4_exp_fn_work);
	exp_data.rmi4_data = rmi4_data;
	exp_data.queue_work = true;
	queue_delayed_work(exp_data.workqueue,
			&exp_data.work,
			msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));

	rmi4_data->irq = gpio_to_irq(platform_data->irq_gpio);

	retval = request_threaded_irq(rmi4_data->irq, NULL,
		synaptics_rmi4_irq, platform_data->irq_flags,
		DRIVER_NAME, rmi4_data);
	rmi4_data->irq_enabled = true;
	if (retval < 0) {
		dev_err(&client->dev,
				"%s: Failed to create irq thread\n",
				__func__);
		goto err_enable_irq;
	}

	wake_lock_init(&gesture_wakelock,WAKE_LOCK_SUSPEND,"gesture_wakelock");
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&client->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			goto err_sysfs;
		}
	}
		/* Clear interrupts first */
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr + 1,
				intr_status,
				rmi4_data->num_of_intr_regs);
		if (retval < 0)
		{
			SYNAP_DEBUG("clear interruptes err");
		}
	//add tp class to show tp info
	
		rmi4_data->tp_class = class_create(THIS_MODULE, "touch");
		if (IS_ERR(rmi4_data->tp_class))
		{
			SYNAP_DEBUG("create tp class err!");
			return retval;
		}
		else
		atomic_set(&device_count, 0);
	rmi4_data->index = atomic_inc_return(&device_count);
	rmi4_data->dev = device_create(rmi4_data->tp_class, NULL,
		MKDEV(0, rmi4_data->index), NULL, "tp_dev");
	if (IS_ERR(rmi4_data->dev))
	{
		SYNAP_DEBUG("create device err!");
		return retval;
	}
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->dev->kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&client->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			return retval;
		}
	}
	dev_set_drvdata(rmi4_data->dev,rmi4_data);
	//end tp class to show tp info
	init_done = 1;
	return retval;

err_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

err_enable_irq:
	cancel_delayed_work_sync(&rmi4_data->tp_work);
	flush_workqueue(rmi4_data->tp_workqueue);
	destroy_workqueue(rmi4_data->tp_workqueue);
	cancel_delayed_work_sync(&exp_data.work);
	flush_workqueue(exp_data.workqueue);
	destroy_workqueue(exp_data.workqueue);

	synaptics_rmi4_irq_enable(rmi4_data, false);
	input_free_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;

err_set_input_dev:
	if (platform_data->gpio_config && (platform_data->reset_gpio >= 0))
		platform_data->gpio_config(platform_data->reset_gpio, false);

err_reset_gpio_dir:
	if (gpio_is_valid(platform_data->reset_gpio))
		gpio_free(platform_data->reset_gpio);

err_irq_gpio_dir:
	if (gpio_is_valid(platform_data->irq_gpio))
		gpio_free(platform_data->irq_gpio);

    if (!list_empty(&rmi->support_fn_list)) {
        list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
            if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
                synaptics_rmi4_f1a_kfree(fhandler);
            } else {
                kfree(fhandler->extra);
                kfree(fhandler->data);
            }
            kfree(fhandler);
        }
    }

err_irq_gpio_req:
	synaptics_rmi4_power_on(rmi4_data, false);

err_power_device:
	synaptics_rmi4_regulator_configure(rmi4_data, false);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&rmi4_data->early_suspend);
#endif
err_reg_configure:
	kfree(rmi4_data);

	return retval;
}

 /**
 * synaptics_rmi4_remove()
 *
 * Called by the kernel when the association with an I2C device of the
 * same name is broken (when the driver is unloaded).
 *
 * This funtion terminates the work queue, stops sensor data acquisition,
 * frees the interrupt, unregisters the driver from the input subsystem,
 * turns off the power to the sensor, and frees other allocated resources.
 */
static int __devexit synaptics_rmi4_remove(struct i2c_client *client)
{
	unsigned char attr_count;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = i2c_get_clientdata(client);
	struct synaptics_rmi4_device_info *rmi;
	/*const struct synaptics_rmi4_platform_data *platform_data =*/
			/*rmi4_data->board;*/

	rmi = &(rmi4_data->rmi4_mod_info);

	synaptics_rmi4_irq_enable(rmi4_data, false);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}
	if(!rmi4_data->dev && !rmi4_data->tp_class)
	{
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->dev->kobj,
				&attrs[attr_count].attr);
	}
	dev_set_drvdata(rmi4_data->dev,NULL);
	device_destroy(rmi4_data->tp_class, MKDEV(0, rmi4_data->index));
	}
	if(!rmi4_data->tp_class)
	class_destroy(rmi4_data->tp_class);
	cancel_delayed_work_sync(&rmi4_data->tp_work);
	flush_workqueue(rmi4_data->tp_workqueue);
	destroy_workqueue(rmi4_data->tp_workqueue);
	cancel_delayed_work_sync(&exp_data.work);
	flush_workqueue(exp_data.workqueue);
	destroy_workqueue(exp_data.workqueue);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
				synaptics_rmi4_f1a_kfree(fhandler);
			} else {
				kfree(fhandler->extra);
				kfree(fhandler->data);
			}
			kfree(fhandler);
		}
	}

	if (gpio_is_valid(rmi4_data->board->reset_gpio))
		gpio_free(rmi4_data->board->reset_gpio);
	if (gpio_is_valid(rmi4_data->board->irq_gpio))
		gpio_free(rmi4_data->board->irq_gpio);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&rmi4_data->early_suspend);
#endif
	input_unregister_device(rmi4_data->input_dev);

	synaptics_rmi4_power_on(rmi4_data, false);
	synaptics_rmi4_regulator_configure(rmi4_data, false);

	kfree(rmi4_data);

	return 0;
}

#if SYNA_SLIDE_WAKEUP
static void synaptics_rmi4_sensor_doze(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char slide_on[3]={0x01,0x01,0x02}; 
	int retval = 0;
	unsigned char ctrl_20_offset = rmi4_data->f12_ctrl_20_offset;
	SYNAP_DEBUG("%s,tp enter doze mode",__func__);
	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f12_ctrl_base_addr + ctrl_20_offset,
			slide_on,
			sizeof(slide_on));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
			"%s: Failed to enter sleep mode\n", __func__);
		return;
	}else
			doze_status = DOZE_ENABLED;
			return;
}
#endif
 /**
 * synaptics_rmi4_sensor_sleep()
 *
 * Called by synaptics_rmi4_early_suspend() and synaptics_rmi4_suspend().
 *
 * This function stops finger data acquisition and puts the sensor to sleep.
 */
static void synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to enter sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = false;
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | NO_SLEEP_OFF | SENSOR_SLEEP);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to enter sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = false;
		return;
	} else {
		rmi4_data->sensor_sleep = true;
	}

	return;
}
#if SYNA_SLIDE_WAKEUP
static void synaptics_rmi4_sensor_slidewake(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char slide_off[3]={0x01,0x01,0x00}; 
	unsigned char force_cal;
	int retval = 0;
	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f12_ctrl_base_addr + rmi4_data->f12_ctrl_20_offset,
			slide_off,
			sizeof(slide_off));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to exit sleep mode\n",
				__func__);
		return;
	}
	else
	{	
		doze_status = DOZE_DISABLED;
		//force cal
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f54_cmd_base_addr,
			&force_cal,
			sizeof(force_cal));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to read calibration status\n",
				__func__);
	}
	force_cal |= 2;
	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f54_cmd_base_addr,
			&force_cal,
			sizeof(force_cal));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to write calibration status\n",
				__func__);
	}
	//force update
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f54_cmd_base_addr,
			&force_cal,
			sizeof(force_cal));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to read calibration status\n",
				__func__);
	}
	force_cal |= 4;
	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f54_cmd_base_addr,
			&force_cal,
			sizeof(force_cal));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to write calibration status\n",
				__func__);
	}
		synaptics_rmi4_free_fingers(rmi4_data);
	}
	
	return;
}
#endif
 /**
 * synaptics_rmi4_sensor_wake()
 *
 * Called by synaptics_rmi4_resume() and synaptics_rmi4_late_resume().
 *
 * This function wakes the sensor from sleep.
 */
static void synaptics_rmi4_sensor_wake(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;
	unsigned char no_sleep_setting = rmi4_data->no_sleep_setting;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to wake from sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = true;
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | no_sleep_setting | NORMAL_OPERATION);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to wake from sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = true;
		return;
	} else {
		rmi4_data->sensor_sleep = false;
	}
	return;
}
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct synaptics_rmi4_data *rmi4_data =
		container_of(self, struct synaptics_rmi4_data, fb_notif);
//	printk("liuyh,%s,suspend flag is %d,event is %lu\n",__func__,synaptics_suspend_flag,event);
	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
		rmi4_data && rmi4_data->i2c_client) {
		blank = evdata->data;
//	printk("liuyh,%s,blank is %d\n",__func__,*blank);
		if(synaptics_suspend_flag>1)
		{
		if (*blank == FB_BLANK_UNBLANK)
			synaptics_rmi4_resume(&(rmi4_data->input_dev->dev));
		else if (*blank == FB_BLANK_POWERDOWN)
			synaptics_rmi4_suspend(&(rmi4_data->input_dev->dev));
	}
	}
	if(synaptics_suspend_flag<2)
		synaptics_suspend_flag++;
	return 0;
}
#elif defined CONFIG_HAS_EARLYSUSPEND
 /**
 * synaptics_rmi4_suspend()
 *
 * Called by the kernel during the early suspend phase when the system
 * enters suspend.
 *
 * This function calls synaptics_rmi4_sensor_sleep() to stop finger
 * data acquisition and put the sensor to sleep.
 */
static void synaptics_rmi4_early_suspend(struct early_suspend *h)
{
	struct synaptics_rmi4_data *rmi4_data =
			container_of(h, struct synaptics_rmi4_data,
			early_suspend);

	printk(KERN_ALERT "----------------early suspend");
	if (rmi4_data->stay_awake) {
		rmi4_data->staying_awake = true;
		return;
	} else {
		rmi4_data->staying_awake = false;
	}

	rmi4_data->touch_stopped = true;
	synaptics_rmi4_irq_enable(rmi4_data, false);
	synaptics_rmi4_sensor_sleep(rmi4_data);
	synaptics_rmi4_free_fingers(rmi4_data);

	if (rmi4_data->full_pm_cycle)
		synaptics_rmi4_suspend(&(rmi4_data->input_dev->dev));
	return;
}

 /**
 * synaptics_rmi4_late_resume()
 *
 * Called by the kernel during the late resume phase when the system
 * wakes up from suspend.
 *
 * This function goes through the sensor wake process if the system wakes
 * up from early suspend (without going into suspend).
 */
static void synaptics_rmi4_late_resume(struct early_suspend *h)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(h, struct synaptics_rmi4_data,
			early_suspend);

	printk(KERN_ALERT "----------------late resume");
	if (rmi4_data->staying_awake)
		return;

	if (rmi4_data->full_pm_cycle)
		synaptics_rmi4_resume(&(rmi4_data->input_dev->dev));

	if (rmi4_data->sensor_sleep == true) {
		synaptics_rmi4_sensor_wake(rmi4_data);
		synaptics_rmi4_irq_enable(rmi4_data, true);
		rmi4_data->touch_stopped = false;
		retval = synaptics_rmi4_reinit_device(rmi4_data);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to reinit device\n",
					__func__);
		}
	}
	return;
}
#endif

 /**
 * synaptics_rmi4_suspend()
 *
 * Called by the kernel during the suspend phase when the system
 * enters suspend.
 *
 * This function stops finger data acquisition and puts the sensor to
 * sleep (if not already done so during the early suspend phase),
 * disables the interrupt, and turns off the power to the sensor.
 */
static int synaptics_rmi4_suspend(struct device *dev)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	int retval = 0;
	unsigned char intr_status[MAX_INTR_REGISTERS];
	/*const struct synaptics_rmi4_platform_data *platform_data =*/
			/*rmi4_data->board;*/

#if SYNA_SLIDE_WAKEUP
	if(syna_wakeup_flag)
	{
	/* Clear interrupts first */
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr + 1,
			intr_status,
			rmi4_data->num_of_intr_regs);
	if (retval < 0)
	{
		SYNAP_DEBUG("clear interrupts bit err");
		return retval;
	}
	synaptics_rmi4_sensor_doze(rmi4_data);
//	synaptics_rmi4_free_fingers(rmi4_data);
	}
	else
	{
	SYNAP_DEBUG("%s",__func__);
	if (rmi4_data->staying_awake)
		return 0;

	if (!rmi4_data->sensor_sleep) {
		rmi4_data->touch_stopped = true;
		synaptics_rmi4_irq_enable(rmi4_data, false);
		synaptics_rmi4_sensor_sleep(rmi4_data);
		synaptics_rmi4_free_fingers(rmi4_data);
	}
	}
#else
	SYNAP_DEBUG("%s",__func__);
	if (rmi4_data->staying_awake)
		return 0;

	if (!rmi4_data->sensor_sleep) {
		rmi4_data->touch_stopped = true;
		synaptics_rmi4_irq_enable(rmi4_data, false);
		synaptics_rmi4_sensor_sleep(rmi4_data);
		synaptics_rmi4_free_fingers(rmi4_data);
	}
#endif
	/*if (platform_data->regulator_en)*/
		/*regulator_disable(rmi4_data->regulator);*/

	cancel_delayed_work_sync(&rmi4_data->tp_work);

	return 0;
}

 /**
 * synaptics_rmi4_resume()
 *
 * Called by the kernel during the resume phase when the system
 * wakes up from suspend.
 *
 * This function turns on the power to the sensor, wakes the sensor
 * from sleep, enables the interrupt, and starts finger data
 * acquisition.
 */
static int synaptics_rmi4_resume(struct device *dev)
{
	static int retval = 0;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
//	finger_f = 0;
	/*const struct synaptics_rmi4_platform_data *platform_data =*/
			/*rmi4_data->board;*/

#if SYNA_SLIDE_WAKEUP
	if(syna_wakeup_flag)
	{
	synaptics_rmi4_sensor_slidewake(rmi4_data);
	synaptics_rmi4_irq_enable(rmi4_data, true);
	retval = synaptics_rmi4_reinit_device(rmi4_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to reinit device\n",
				__func__);
		return retval;
	}
	}
	else
	{
	SYNAP_DEBUG("%s",__func__);
	if (rmi4_data->staying_awake)
	{
		SYNAP_DEBUG("staying awake,not do resume!!!");
		return 0;
	}

	/*if (platform_data->regulator_en) {*/
		/*regulator_enable(rmi4_data->regulator);*/
		/*mdelay(platform_data->reset_delay_ms);*/
		/*rmi4_data->current_page = MASK_8BIT;*/
	/*}*/

	synaptics_rmi4_sensor_wake(rmi4_data);
	synaptics_rmi4_irq_enable(rmi4_data, true);
	rmi4_data->touch_stopped = false;
	retval = synaptics_rmi4_reinit_device(rmi4_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to reinit device\n",
				__func__);
		return retval;
	}
	}

#ifdef TYPE_B_PROTOCOL
	synaptics_rmi4_free_abnormall_fingers(rmi4_data);
#endif
#else
	SYNAP_DEBUG("%s",__func__);
	if (rmi4_data->staying_awake)
		return 0;

	/*if (platform_data->regulator_en) {*/
		/*regulator_enable(rmi4_data->regulator);*/
		/*mdelay(platform_data->reset_delay_ms);*/
		/*rmi4_data->current_page = MASK_8BIT;*/
	/*}*/

	synaptics_rmi4_sensor_wake(rmi4_data);
	synaptics_rmi4_irq_enable(rmi4_data, true);
	rmi4_data->touch_stopped = false;
	retval = synaptics_rmi4_reinit_device(rmi4_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to reinit device\n",
				__func__);
		return retval;
	}
#ifdef TYPE_B_PROTOCOL
	synaptics_rmi4_free_abnormall_fingers(rmi4_data);
#endif
#endif
/*
	retval = synaptics_rmi4_power_on(rmi4_data, false);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power off\n");

	}
	mdelay(100);
	retval = synaptics_rmi4_power_on(rmi4_data, true);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev, "Failed to power on\n");
	}
		mdelay(200);
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: resume,power reset\n",
				__func__);
*/	
	return 0;
}

#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
static const struct dev_pm_ops synaptics_rmi4_dev_pm_ops = {
	.suspend = synaptics_rmi4_suspend,
	.resume  = synaptics_rmi4_resume,
};
#else
static const struct dev_pm_ops synaptics_rmi4_dev_pm_ops = {
};
#endif


static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

#ifdef CONFIG_OF
static struct of_device_id rmi4_match_table[] = {
	{ .compatible = "synaptics,s3408",},
	{ },
};
#else
#define rmi4_match_table NULL
#endif

static struct i2c_driver synaptics_rmi4_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = rmi4_match_table,
#ifdef CONFIG_HAS_EARLYSUSPEND
		.pm = &synaptics_rmi4_dev_pm_ops,
#endif
	},
	.probe = synaptics_rmi4_probe,
	.remove = __devexit_p(synaptics_rmi4_remove),
	.id_table = synaptics_rmi4_id_table,
};

 /**
 * synaptics_rmi4_init()
 *
 * Called by the kernel during do_initcalls (if built-in)
 * or when the driver is loaded (if a module).
 *
 * This function registers the driver to the I2C subsystem.
 *
 */
static int __init synaptics_rmi4_init(void)
{
	return i2c_add_driver(&synaptics_rmi4_driver);
}

 /**
 * synaptics_rmi4_exit()
 *
 * Called by the kernel when the driver is unloaded.
 *
 * This funtion unregisters the driver from the I2C subsystem.
 *
 */
static void __exit synaptics_rmi4_exit(void)
{
	i2c_del_driver(&synaptics_rmi4_driver);

	return;
}

module_init(synaptics_rmi4_init);
module_exit(synaptics_rmi4_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Touch Driver");
MODULE_LICENSE("GPL v2");
