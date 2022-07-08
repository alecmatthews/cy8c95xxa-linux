#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/miscdevice.h>

#include <linux/gpio/driver.h>

#define MODULE_NAME "gpio-cy8c95xxa"

enum cypress_ioexpander_type {
    CYP_TYPE_20A,
    CYP_TYPE_40A,
    CYP_TYPE_60A
};

struct ioexp_dev {
    struct i2c_client *c;
    struct miscdevice ioexp_miscdev;
    struct gpio_chip chip;
    char *name;
};


#define REG_OUTPUT_PORT0 0x08

static ssize_t ioexp_read_file (struct file *file, char __user *buf,
                                size_t count, loff_t *ppos)
{
    struct ioexp_dev *ioexp;
    s32 recv_data;
    int size;
    char to_user[32];

    ioexp = container_of(file->private_data, struct ioexp_dev, ioexp_miscdev);
    dev_info(&ioexp->c->dev, "Reading from device...");

    recv_data = i2c_smbus_read_byte_data(ioexp->c, REG_OUTPUT_PORT0);
    if (recv_data < 0)
        return -EIO;

    size = snprintf(to_user, ARRAY_SIZE(to_user), "0x%02X\n", recv_data);

    if (*ppos != 0)
        return 0;

    if(copy_to_user(buf, to_user, size))
        return -EFAULT;

    *ppos += size;
    return size;
}

static ssize_t ioexp_write_file (struct file *file, const char __user *buf,
                                 size_t count, loff_t *ppos)
{
    struct ioexp_dev *ioexp;
    int ret;
    u8 reg_value;

    ioexp = container_of(file->private_data, struct ioexp_dev, ioexp_miscdev);
    dev_info(&ioexp->c->dev, "Writing to device...");

    if (*ppos != 0)
        return -EFAULT;

    ret = kstrtou8_from_user(buf, count, 0, &reg_value);
    if (ret != 0)
        return -EINVAL;

    dev_info(&ioexp->c->dev, "Value to write: 0x%02X", reg_value);

    ret = i2c_smbus_write_byte_data(ioexp->c, REG_OUTPUT_PORT0, reg_value);
    if (ret < 0)
        return -EIO;

    return count;
}

static const struct file_operations ioexp_fops = {
    .owner = THIS_MODULE,
    .read = ioexp_read_file,
    .write = ioexp_write_file,
};

static int gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
    dev_info(gc->parent, "%s", __func__);
    return 0;
}

static int gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
    dev_info(gc->parent, "%s", __func__);
    return 0;
}

static int gpio_direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
    dev_info(gc->parent, "%s", __func__);
    return 0;
}

static int gpio_get(struct gpio_chip *gc, unsigned int offset)
{
    dev_info(gc->parent, "%s", __func__);
    return 0;
}

static void gpio_set(struct gpio_chip *gc, unsigned int offset, int value) {
    dev_info(gc->parent, "%s", __func__);
    return;
}

static int init_gpio_chip(struct ioexp_dev *ioexp)
{
    struct gpio_chip *chip = &ioexp->chip;

    chip->owner = THIS_MODULE;
    chip->label = MODULE_NAME;
    chip->parent = &ioexp->c->dev;
    chip->get_direction = gpio_get_direction;
    chip->direction_input = gpio_direction_input;
    chip->direction_output = gpio_direction_output;
    chip->get = gpio_get;
    chip->set = gpio_set;
    chip->base = -1;
    chip->ngpio = 8;
    chip->can_sleep = true;
    if (IS_ENABLED(CONFIG_OF_GPIO)) {
        chip->of_node = chip->parent->of_node;
        chip->of_gpio_n_cells = 2; // TODO: read from of
    }

    devm_gpiochip_add_data(chip->parent, chip, ioexp);

    return 0;
}

static int cy8c95xxa_probe(struct i2c_client *client)
{
    static u8 dev_count = 0;
    struct ioexp_dev *ioexp;

    dev_info(&client->dev, "Device detected");

    ioexp = devm_kzalloc(&client->dev, sizeof(*ioexp), GFP_KERNEL);
    if (!ioexp)
        return -ENOMEM;

    ioexp->c = client;
    ioexp->name = devm_kasprintf(&client->dev, GFP_KERNEL, "ioexp%02d", dev_count++);
    ioexp->ioexp_miscdev.name = ioexp->name;
    ioexp->ioexp_miscdev.minor = MISC_DYNAMIC_MINOR;
    ioexp->ioexp_miscdev.fops = &ioexp_fops;

    i2c_set_clientdata(client, ioexp);
    dev_info(&client->dev, "Device allocated");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
        dev_err(&client->dev, "I2C bus incompatible");
        return -EIO;
    }

    misc_register(&ioexp->ioexp_miscdev);

    init_gpio_chip(ioexp);

    dev_info(&client->dev, "Name: %s", dev_name(&client->dev));

    dev_info(&client->dev, "Probe finished");
    return 0;
}

static int cy8c95xxa_remove(struct i2c_client *client)
{
    struct ioexp_dev *ioexp;
    ioexp = i2c_get_clientdata(client);
    misc_deregister(&ioexp->ioexp_miscdev);

    dev_info(&client->dev, "Device removed");
    return 0;
}

static const struct of_device_id cy8c95xxa_of_match[] = {
    {
        .compatible = "cypress,cy8c9520a",
        .data = (void*) CYP_TYPE_20A,
    },
    {
        .compatible = "cypress,cy8c9540a",
        .data = (void*) CYP_TYPE_40A,
    },
    {
        .compatible = "cypress,cy8c9560a",
        .data = (void*) CYP_TYPE_60A,
    },
    { },
};
MODULE_DEVICE_TABLE(of, cy8c95xxa_of_match);

static const struct i2c_device_id cy8c95xxa_i2c_match[] = {
    { "cypress,cy8c9520a", CYP_TYPE_20A },
    { "cypress,cy8c9540a", CYP_TYPE_40A },
    { "cypress,cy8c9560a", CYP_TYPE_60A },
    { },
};
MODULE_DEVICE_TABLE(i2c, cy8c95xxa_i2c_match);

static struct i2c_driver cy8c95xxa_driver = {
    .driver = {
        .name = "cy8c95xxa",
        .of_match_table = cy8c95xxa_of_match,
        .owner = THIS_MODULE,
    },
    .probe_new = cy8c95xxa_probe,
    .remove = cy8c95xxa_remove,
    .id_table = cy8c95xxa_i2c_match
};

module_i2c_driver(cy8c95xxa_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alec Matthews <me@alecmatthews.dev>");
MODULE_DESCRIPTION("Kernel driver for the Cypress CY8C95XXA family of i2c gpio \
                    expanders");
