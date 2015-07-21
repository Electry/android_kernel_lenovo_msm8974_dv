//
// hw_ver.c
//
// Drivers for hw version detected.
//

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/err.h>

#include <linux/qpnp/qpnp-adc.h>

struct hw_ver_pdata {
	enum qpnp_vadc_channels	adc0_channel;
	enum qpnp_vadc_channels	adc1_channel;
	struct qpnp_vadc_chip		*vadc_dev;
	unsigned int	ver_gpio0;
	unsigned int	ver_gpio1;
};

static void convert_adc_value(unsigned long *adc)
{
	unsigned long val = *adc;

	// mode 0: VER_ADC0 = 0.0V
	// mode 1: VER_ADC0 = 0.6V
	// mode 2: VER_ADC0 = 0.9V
	// mode 3: VER_ADC0 = 1.2V

	val /= 1000;
	if(val < 500)		// mode 0
		val = 0;
	else if((val > 550) && (val < 650))	// mode 1
		val = 1;
	else if((val > 850) && (val < 950))	// mode 2
		val = 2;
	else if((val > 1150) && (val < 1250))	// mode 3
		val = 3;
	else
		val = 0;

	*adc = val;
}

static ssize_t show_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hw_ver_pdata *pdata;
	struct qpnp_vadc_result adc_result;
	unsigned long ver_adc0, ver_adc1;
	unsigned int ver_gpio0, ver_gpio1;
	unsigned int hw_ver = 0;
	int rc;

	pdata = dev_get_drvdata(dev);

	rc = qpnp_vadc_read(pdata->vadc_dev, pdata->adc0_channel, &adc_result);
	if (!rc){
		ver_adc0 = adc_result.physical;
		pr_info("ver_adc0 = %ld\n", ver_adc0);
	}
	else
		pr_err("%s: qpnp_vadc_read(%d) failed, rc=%d\n", __func__, pdata->adc0_channel, rc);

	convert_adc_value(&ver_adc0);

	rc = qpnp_vadc_read(pdata->vadc_dev, pdata->adc1_channel, &adc_result);
	if (!rc){
		ver_adc1 = adc_result.physical;
		pr_info("ver_adc1 = %ld\n", ver_adc1);
	}
	else
		pr_err("%s: qpnp_vadc_read(%d) failed, rc=%d\n", __func__, pdata->adc1_channel, rc);

	convert_adc_value(&ver_adc1);

	ver_gpio0 = gpio_get_value(pdata->ver_gpio0);
	ver_gpio1 = gpio_get_value(pdata->ver_gpio1);
	pr_info("ver_gpio0 = %d, ver_gpio1 = %d\n", ver_gpio0, ver_gpio1);

	hw_ver = (ver_gpio0 & 0x1);
	hw_ver |= (ver_gpio1 & 0x1) << 4;
	hw_ver |= ver_adc0 << 8;
	hw_ver |= ver_adc1 << 12;

	return sprintf(buf, "%04x\n", hw_ver);
}


static DEVICE_ATTR(version, S_IRUGO, show_version, NULL);

static struct attribute *hw_ver_attrs[] = {
	&dev_attr_version.attr,
	NULL,
};

static struct attribute_group hw_ver_attr_group = {
	.attrs = hw_ver_attrs,
};

static int __devinit hw_ver_probe(struct platform_device *pdev)
{
	struct device_node *node;
	struct hw_ver_pdata *pdata;
	int rc = 0;

	node = pdev->dev.of_node;

	pdata = kzalloc(sizeof(struct hw_ver_pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err( "%s: Can't allocate qpnp_tm_chip\n", __func__);
		return -ENOMEM;
	}

	dev_set_drvdata(&pdev->dev, pdata);

	rc = of_property_read_u32(node, "qcom,adc0-channel-num", &pdata->adc0_channel);
	if (!rc) {
		if (pdata->adc0_channel < 0 || pdata->adc0_channel >= ADC_MAX_NUM) {
			pr_err("%s: invalid qcom,adc0-channel-num=%d specified\n",
				__func__, pdata->adc0_channel);
			goto free_data;
		}
	}

	rc = of_property_read_u32(node, "qcom,adc1-channel-num", &pdata->adc1_channel);
	if (!rc) {
		if (pdata->adc1_channel < 0 || pdata->adc1_channel >= ADC_MAX_NUM) {
			pr_err("%s: invalid qcom,adc1-channel-num=%d specified\n",
				__func__, pdata->adc1_channel);
			goto free_data;
		}
	}
	pr_info("hw_ver: adc0_channel = %d, adc1_channel = %d\n", pdata->adc0_channel, pdata->adc1_channel);

	pdata->vadc_dev = qpnp_get_vadc(&pdev->dev, "hw_ver");
	if (IS_ERR(pdata->vadc_dev)) {
		rc = PTR_ERR(pdata->vadc_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("vadc property missing\n");
		goto free_data;
	}

	if (of_gpio_count(node) < 2)
		goto free_data;

	pdata->ver_gpio0 = of_get_gpio(node, 0);
	pdata->ver_gpio1 = of_get_gpio(node, 1);

	if (!gpio_is_valid(pdata->ver_gpio0) || !gpio_is_valid(pdata->ver_gpio1)) {
		pr_err("%s: invalid GPIO pins, ver_gpio0=%d/ver_gpio1=%d\n",
		       node->full_name, pdata->ver_gpio0, pdata->ver_gpio1);
		goto free_data;
	}
	pr_info("hw_ver: ver_gpio0 = %d, ver_gpio1 = %d\n", pdata->ver_gpio0, pdata->ver_gpio1);

	rc = gpio_request(pdata->ver_gpio0, "ver_gpio0");
	if (rc)
		goto err_request_gpio0;
	rc = gpio_request(pdata->ver_gpio1, "ver_gpio1");
	if (rc)
		goto err_request_gpio1;

	gpio_direction_input(pdata->ver_gpio0);
	gpio_direction_input(pdata->ver_gpio1);

	rc = sysfs_create_group(&pdev->dev.kobj, &hw_ver_attr_group);
	if (rc) {
		pr_err("Unable to create sysfs for hw_ver, errors: %d\n", rc);
		goto err_sysfs_create;
	}

	return 0;

err_sysfs_create:
	gpio_free(pdata->ver_gpio1);
err_request_gpio1:
	gpio_free(pdata->ver_gpio0);
err_request_gpio0:
free_data:
	kfree(pdata);
	return rc;
}

static int __devexit hw_ver_remove(struct platform_device *pdev)
{
	struct hw_ver_pdata *pdata;

	pdata = dev_get_drvdata(&pdev->dev);

	gpio_free(pdata->ver_gpio0);
	gpio_free(pdata->ver_gpio1);

	sysfs_remove_group(&pdev->dev.kobj, &hw_ver_attr_group);

	if(pdata != NULL)
		kfree(pdata);

	return 0;
}

static struct of_device_id hw_ver_of_match[] = {
	{ .compatible = "hw-version", },
	{ },
};

static struct platform_driver hw_ver_device_driver = {
	.probe		= hw_ver_probe,
	.remove		= __devexit_p(hw_ver_remove),
	.driver		= {
		.name	= "hw-version",
		.owner	= THIS_MODULE,
		.of_match_table = hw_ver_of_match,
	}
};

static int __init hw_ver_init(void)
{
	return platform_driver_register(&hw_ver_device_driver);
}

static void __exit hw_ver_exit(void)
{
	platform_driver_unregister(&hw_ver_device_driver);
}

late_initcall(hw_ver_init);
module_exit(hw_ver_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Cheng Xuetao <chengxta@lenovo.com>");
MODULE_DESCRIPTION("Drivers for HW Version detected");
MODULE_ALIAS("platform:hw-version");

// end of file
