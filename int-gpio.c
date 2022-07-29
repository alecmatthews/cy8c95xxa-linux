#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

static char *INT_NAME = "P0_line1_INT";

static irqreturn_t gpio_isr(int irq, void *data)
{
	struct device *dev = data;
	dev_info(dev, "INT received. key: %s", INT_NAME);
	return IRQ_HANDLED;
}

static struct miscdevice gpio_int_miscdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gpio-int",
};

static int gpio_int_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct device *dev = &pdev->dev;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "irq not available");
		return -EINVAL;
	}

	ret =
	    devm_request_threaded_irq(dev, irq, NULL, gpio_isr,
				      IRQF_ONESHOT | IRQF_TRIGGER_FALLING |
				      IRQF_TRIGGER_RISING, INT_NAME, dev);

	if (ret) {
		dev_err(dev, "Failed to get interrupt");
		return ret;
	}

	ret = misc_register(&gpio_int_miscdevice);
	if (ret != 0) {
		dev_err(dev, "Could not register misc device");
		return ret;
	}
	return 0;
}

static int gpio_int_remove(struct platform_device *pdev)
{
	misc_deregister(&gpio_int_miscdevice);
	return 0;
}

static const struct of_device_id my_of_ids[] = {
	{.compatible = "htv,gpio_int" },
	{ },
};

MODULE_DEVICE_TABLE(of, my_of_ids);

static struct platform_driver my_platform_driver = {
	.probe = gpio_int_probe,
	.remove = gpio_int_remove,
	.driver = {
		   .name = "int_gpio",
		   .of_match_table = my_of_ids,
		   .owner = THIS_MODULE,
		    }
};

module_platform_driver(my_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alec Matthews <me@alecmatthews.dev");
