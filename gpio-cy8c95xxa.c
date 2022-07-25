#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/of_device.h>

#include <linux/gpio/driver.h>

#define MODULE_NAME "gpio-cy8c95xxa"


/* Registers are all 8 bits wide. Each base register can be offset by
 * the port number to access each individual port.
 * Each bit in the register controls a single I/O pin on the selected
 * port.
 */
#define REG_IN_BASE         0x00
#define REG_OUT_BASE        0x08
#define REG_INT_STATUS_BASE 0x10

#define IN_REG(p) (REG_IN_BASE + p)
#define OUT_REG(p) (REG_OUT_BASE + p)

/* To modify the port first select it, then change the behavior with
 * the setting register.
 */
#define REG_PORT_SELECT 0x18
#define REG_INT_MASK    0x19
#define REG_PIN_DIR     0x1C

/* Chip properties */
// TODO: this will be different for the other chip models!
#define NGPIO 20

enum cypress_ioexpander_type {
	CYP_TYPE_20A = 20,
	CYP_TYPE_40A = 40,
	CYP_TYPE_60A = 60,
};

#define devm_alloc_chip_name(dev, type) \
	devm_kasprintf(dev, GFP_KERNEL, "cy8c95%da", type)


struct ioexp_dev {
	struct i2c_client *c;
	struct miscdevice ioexp_miscdev;
	struct gpio_chip chip;
	char *misc_name;
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

// TODO: add offsets for larger chips
static const u8 cy8c9520a_port_offset[] = {
	0,
	8,
	16,
};

u8 gpio_to_port(unsigned gpio)
{
	u8 i = 0;
	for (; i < ARRAY_SIZE(cy8c9520a_port_offset) - 1; i++) {
		if (!(gpio / cy8c9520a_port_offset[i + 1]))
			break;
	}
	return i;
}

u8 gpio_to_bit(unsigned gpio, u8 port) {
	return gpio - cy8c9520a_port_offset[port];
}

static int gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	u8 port = gpio_to_port(offset);
	dev_info(gc->parent, "%s: port: %d, bit: %d", __func__, port, gpio_to_bit(offset, port));
	return 0;
}

static int gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	u8 port = gpio_to_port(offset);
	dev_info(gc->parent, "%s: port: %d, bit: %d", __func__, port, gpio_to_bit(offset, port));
	return 0;
}

static int gpio_direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	int ret;
	u8 port, bit, pins;
	struct ioexp_dev *ioexp = gpiochip_get_data(gc);

	port = gpio_to_port(offset);
	bit = gpio_to_bit(offset, port);

	dev_info(gc->parent, "%s: port: %d, bit: %d, value: %d", __func__, port, bit, value);

	ret = i2c_smbus_write_byte_data(ioexp->c, REG_PORT_SELECT, port);
	if (ret < 0) {
		dev_err(gc->parent, "Can not select port %u!\n", port);
		return ret;
	}

	ret = i2c_smbus_read_byte_data(ioexp->c, REG_PIN_DIR);
	if (ret < 0) {
		dev_err(gc->parent, "Could not read pin direction\n");
		return -1;
	}

	pins = (u8)ret;
	pins &= ~BIT(bit);

	ret = i2c_smbus_write_byte_data(ioexp->c, REG_PIN_DIR, pins);
	if (ret < 0) {
		dev_err(gc->parent, "Could not write pin direction!\n");
		return ret;
	}

	ret = i2c_smbus_read_byte_data(ioexp->c, OUT_REG(port));
	if (ret < 0) {
		dev_err(gc->parent, "Count not read output port!\n");
		return ret;
	}

	if (value) {
		pins = (u8)ret | BIT(bit);
	} else {
		pins = (u8)ret & ~BIT(bit);
	}

	ret = i2c_smbus_write_byte_data(ioexp->c, OUT_REG(port), pins);
	if (ret < 0) {
		dev_err(gc->parent, "Could not write output port!\n");
		return ret;
	}

	return 0;
}

static int gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	u8 port = gpio_to_port(offset);
	dev_info(gc->parent, "%s: port: %d, bit: %d", __func__, port, gpio_to_bit(offset, port));
	return 0;
}

static void gpio_set(struct gpio_chip *gc, unsigned int offset, int value) {
	u8 port = gpio_to_port(offset);
	dev_info(gc->parent, "%s: port: %d, bit: %d, value: %d", __func__, port, gpio_to_bit(offset, port), value);
	return;
}

static int init_gpio_chip(struct ioexp_dev *ioexp)
{
	struct gpio_chip *chip = &ioexp->chip;

	chip->owner = THIS_MODULE;
	chip->label = ioexp->name;
	chip->parent = &ioexp->c->dev;
	chip->get_direction = gpio_get_direction;
	chip->direction_input = gpio_direction_input;
	chip->direction_output = gpio_direction_output;
	chip->get = gpio_get;
	chip->set = gpio_set;
	chip->base = -1;
	chip->ngpio = NGPIO;
	chip->can_sleep = true;
	if (IS_ENABLED(CONFIG_OF_GPIO)) {
		chip->of_node = chip->parent->of_node;
		chip->of_gpio_n_cells = 2; // TODO: read from fimware
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
	ioexp->misc_name = devm_kasprintf(&client->dev, GFP_KERNEL, "ioexp%02d", dev_count++);
	ioexp->name = client->name;
	ioexp->ioexp_miscdev.name = ioexp->misc_name;
	ioexp->ioexp_miscdev.minor = MISC_DYNAMIC_MINOR;
	ioexp->ioexp_miscdev.fops = &ioexp_fops;

	i2c_set_clientdata(client, ioexp);
	dev_info(&client->dev, "Device allocated");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "I2C bus incompatible");
		return -EIO;
	}

	dev_info(&client->dev, "I2C client name: %s", client->name);

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
