#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/of_device.h>

#include <linux/gpio/driver.h>

#define MODULE_NAME "gpio-cy8c95xxa"

/*
 * Registers are all 8 bits wide. Each base register can be offset by
 * the port number to access each individual port.
 * Each bit in the register controls a single I/O pin on the selected
 * port.
 */
#define REG_IN_BASE         0x00
#define REG_OUT_BASE        0x08
#define REG_INT_STATUS_BASE 0x10

#define IN_REG(p) (REG_IN_BASE + p)
#define OUT_REG(p) (REG_OUT_BASE + p)

/*
 * To modify the port first select it, then change the behavior with
 * the setting register.
 */
#define REG_PORT_SELECT 0x18
#define REG_INT_MASK    0x19
#define REG_PIN_DIR     0x1C

/* Chip properties */
// TODO: this will be different for the other chip models!
#define NGPIO 20
/* Number of ports in the larges chip. */
#define NPORTS 8

#define SET_BIT(nr, p)		(*p |= BIT(nr))
#define CLEAR_BIT(nr, p)	(*p &= ~BIT(nr))

enum cypress_ioexpander_type {
	CYP_TYPE_20A = 20,
	CYP_TYPE_40A = 40,
	CYP_TYPE_60A = 60,
};

struct cy8c95xxa {
	u8 out_cache[NPORTS];
	u8 dir_cache[NPORTS];

	struct gpio_chip chip;

	struct i2c_client *i2c_client;
	struct device *dev;

	char *name;
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

u8 gpio_to_bit(unsigned gpio, u8 port)
{
	return gpio - cy8c9520a_port_offset[port];
}

static int cyp_read(struct cy8c95xxa *cyp, u8 reg)
{
	return i2c_smbus_read_byte_data(cyp->i2c_client, reg);
}

static int cyp_write(struct cy8c95xxa *cyp, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(cyp->i2c_client, reg, val);
}

static int gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	int ret, status;
	u8 port, bit;
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);

	port = gpio_to_port(offset);
	bit = gpio_to_bit(offset, port);

	ret = cyp_read(cyp, IN_REG(port));
	if (ret < 0)
		status = 0;
	else
		status = !!(ret & BIT(bit));

	return status;
}

static void gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	int ret;
	u8 port, bit;
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);

	port = gpio_to_port(offset);
	bit = gpio_to_bit(offset, port);

	if (value)
		SET_BIT(bit, &cyp->out_cache[port]);
	else
		CLEAR_BIT(bit, &cyp->out_cache[port]);

	ret = cyp_write(cyp, OUT_REG(port), cyp->out_cache[port]);
	if (ret < 0)
		dev_err(cyp->dev, "Could not set pin");
}

static int gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	u8 port, bit;
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);

	port = gpio_to_port(offset);
	bit = gpio_to_bit(offset, port);

	return cyp->dir_cache[port] & BIT(bit) ?
	    GPIO_LINE_DIRECTION_IN : GPIO_LINE_DIRECTION_OUT;
}

static int gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	int ret;
	u8 port, bit;
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);

	port = gpio_to_port(offset);
	bit = gpio_to_bit(offset, port);

	SET_BIT(bit, &cyp->dir_cache[port]);

	ret = cyp_write(cyp, REG_PORT_SELECT, port);
	if (ret < 0) {
		dev_err(gc->parent, "Could not select port");
		return ret;
	}

	ret = cyp_write(cyp, REG_PIN_DIR, cyp->dir_cache[port]);
	if (ret < 0) {
		dev_err(gc->parent, "Could not write pin direction");
		return ret;
	}

	return 0;
}

static int gpio_direction_output(struct gpio_chip *gc, unsigned int offset,
				 int value)
{
	int ret;
	u8 port, bit;
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);

	port = gpio_to_port(offset);
	bit = gpio_to_bit(offset, port);

	CLEAR_BIT(bit, &cyp->dir_cache[port]);

	ret = cyp_write(cyp, REG_PORT_SELECT, port);
	if (ret < 0) {
		dev_err(gc->parent, "Could not select port");
		return ret;
	}

	ret = cyp_write(cyp, REG_PIN_DIR, cyp->dir_cache[port]);
	if (ret < 0) {
		dev_err(gc->parent, "Could not write pin direction");
		return ret;
	}

	gpio_set(gc, offset, value);
	return 0;
}

static int init_gpio_chip(struct cy8c95xxa *cyp)
{
	struct gpio_chip *chip = &cyp->chip;

	chip->owner = THIS_MODULE;
	chip->label = cyp->name;
	chip->parent = cyp->dev;
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
		chip->of_gpio_n_cells = 2;	// TODO: read from fimware
	}

	devm_gpiochip_add_data(chip->parent, chip, cyp);

	return 0;
}

static int cache_output_reg(struct cy8c95xxa *cyp)
{
	int ret;

	ret =
	    i2c_smbus_read_i2c_block_data(cyp->i2c_client, REG_OUT_BASE,
					  ARRAY_SIZE(cyp->out_cache),
					  cyp->out_cache);
	if (ret < 0) {
		dev_err(cyp->dev, "Could not cache output registers");
		return ret;
	}
	return 0;
}

static int cache_port_dir(struct cy8c95xxa *cyp)
{
	int ret, i;

	for (i = 0; i < NPORTS; i++) {
		ret = cyp_write(cyp, REG_PORT_SELECT, i);
		if (ret < 0)
			return ret;

		ret = cyp_read(cyp, REG_PIN_DIR);
		if (ret < 0)
			return ret;

		cyp->dir_cache[i] = ret;
	}

	return 0;
}

static int cy8c95xxa_probe(struct i2c_client *client)
{
	int ret;
	struct cy8c95xxa *cyp;

	dev_info(&client->dev, "Device detected");

	cyp = devm_kzalloc(&client->dev, sizeof(*cyp), GFP_KERNEL);
	if (!cyp)
		return -ENOMEM;

	cyp->i2c_client = client;
	cyp->dev = &client->dev;
	cyp->name = client->name;

	i2c_set_clientdata(client, cyp);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA
				     | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(cyp->dev, "I2C bus incompatible");
		return -EIO;
	}

	ret = cache_output_reg(cyp);
	if (ret < 0)
		return ret;

	ret = cache_port_dir(cyp);
	if (ret < 0)
		return ret;

	init_gpio_chip(cyp);
	return 0;
}

static int cy8c95xxa_remove(struct i2c_client *client)
{
	struct ioexp_dev *ioexp;
	ioexp = i2c_get_clientdata(client);

	dev_info(&client->dev, "Device removed");
	return 0;
}

static const struct of_device_id cy8c95xxa_of_match[] = {
	{
	 .compatible = "cypress,cy8c9520a",
	 .data = (void *)CYP_TYPE_20A,
	  },
	{
	 .compatible = "cypress,cy8c9540a",
	 .data = (void *)CYP_TYPE_40A,
	  },
	{
	 .compatible = "cypress,cy8c9560a",
	 .data = (void *)CYP_TYPE_60A,
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
