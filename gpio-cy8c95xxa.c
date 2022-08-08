#include <linux/i2c.h>
#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>

#include <linux/pwm.h>

#include <asm/div64.h>

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
#define REG_PORT_SELECT 				0x18
#define REG_INT_MASK    				0x19
#define REG_PWM_EAN						0x1A
#define REG_INVERSION					0x1B
#define REG_PIN_DIR     				0x1C
#define REG_DRV_MODE_PULL_UP			0x1D
#define REG_DRV_MODE_PULL_DWN			0x1E
#define REG_DRV_MODE_OPEN_DRAIN_HIGH	0x1F
#define REG_DRV_MODE_OPEN_DRAIN_LOW		0x20
#define REG_DRV_MODE_STRONG				0x21
#define REG_DRV_MODE_HIGH_Z				0x23

#define REG_PWM_SELECT	0x28
#define REG_PWM_PERIOD	0x2A
#define REG_PWM_PULSE_W	0x2B

/* Chip properties */
// TODO: this will be different for the other chip models!
#define NGPIO	20
/* Number of ports in the larges chip. */
#define NPORTS	8
/* Number of PWM channels; TODO: up to 16 for larger chips... */
#define NPWM		4
#define PWM_BASE	0
#define PWM_CLK_NS	31250

#define SET_BIT(nr, p)		(*p |= BIT(nr))
#define CLEAR_BIT(nr, p)	(*p &= ~BIT(nr))

enum cypress_ioexpander_type {
	CYP_TYPE_20A = 20,
	CYP_TYPE_40A = 40,
	CYP_TYPE_60A = 60,
};

/**
 * struct cy8c95xxa - status of the chip under control
 * @out_cache: cache of the output registers
 * @dir_cache: cache of the pin direction registers
 * @chip: struct gpio_chip
 * @pinctrl: struct pinctrl_desc
 * @lock: protects cache modifications
 * @i2c_client: underlying bus
 * @dev: underlying device
 * @name: name of the connected device
 */
struct cy8c95xxa {
	u8 out_cache[NPORTS];
	u8 dir_cache[NPORTS];
	u8 irq_mask[NPORTS];
	u8 irq_mask_cache[NPORTS];

	struct gpio_chip chip;

	struct pwm_chip pwm_chip;

	struct pinctrl_desc pinctrl;
	struct pinctrl_dev *pctldev;
	struct pinctrl *pin_ctrl;

	struct mutex lock;
	struct mutex irq_lock;

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

u8 gpio_to_port(unsigned long gpio)
{
	u8 i = 0;
	for (; i < ARRAY_SIZE(cy8c9520a_port_offset) - 1; i++) {
		if (!(gpio / cy8c9520a_port_offset[i + 1]))
			break;
	}
	return i;
}

u8 gpio_to_bit(unsigned long gpio, u8 port)
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

#define CY8C95XXA_PIN(a)	PINCTRL_PIN(a, "gpio" #a)
struct pinctrl_pin_desc pinctrl_pins_20a[] = {
	CY8C95XXA_PIN(0),
	CY8C95XXA_PIN(1),
	CY8C95XXA_PIN(2),
	CY8C95XXA_PIN(3),
	CY8C95XXA_PIN(4),
	CY8C95XXA_PIN(5),
	CY8C95XXA_PIN(6),
	CY8C95XXA_PIN(7),
	CY8C95XXA_PIN(8),
	CY8C95XXA_PIN(9),
	CY8C95XXA_PIN(10),
	CY8C95XXA_PIN(11),
	CY8C95XXA_PIN(12),
	CY8C95XXA_PIN(13),
	CY8C95XXA_PIN(14),
	CY8C95XXA_PIN(15),
	CY8C95XXA_PIN(16),
	CY8C95XXA_PIN(17),
	CY8C95XXA_PIN(18),
	CY8C95XXA_PIN(19),
};

#define CY8C95XXA_GROUP(a)	"gpio" #a
static const char *const cy8c95xxa_group_names[] = {
	CY8C95XXA_GROUP(0),
	CY8C95XXA_GROUP(1),
	CY8C95XXA_GROUP(2),
	CY8C95XXA_GROUP(3),
	CY8C95XXA_GROUP(4),
	CY8C95XXA_GROUP(5),
	CY8C95XXA_GROUP(6),
	CY8C95XXA_GROUP(7),
	CY8C95XXA_GROUP(8),
	CY8C95XXA_GROUP(9),
	CY8C95XXA_GROUP(10),
	CY8C95XXA_GROUP(11),
	CY8C95XXA_GROUP(12),
	CY8C95XXA_GROUP(13),
	CY8C95XXA_GROUP(14),
	CY8C95XXA_GROUP(15),
	CY8C95XXA_GROUP(16),
	CY8C95XXA_GROUP(17),
	CY8C95XXA_GROUP(18),
	CY8C95XXA_GROUP(19),
};

#define CY8C95XXA_FUNC_SIZE	2
static const char *cy8c95xxa_functions[CY8C95XXA_FUNC_SIZE] = {
	"gpio",
	"pwm",
};

static int cy8c95xxa_get_functions_count(struct pinctrl_dev *pctldev)
{
	return CY8C95XXA_FUNC_SIZE;
}

static const char *cy8c95xxa_get_function_name(struct pinctrl_dev *pctrldev,
		unsigned selector)
{
	return cy8c95xxa_functions[selector];
}

static int cy8c95xxa_get_function_groups(struct pinctrl_dev *pctrldev,
		unsigned selector, const char * const **groups, unsigned *num_groups)
{
	*groups = cy8c95xxa_group_names;
	*num_groups = ARRAY_SIZE(cy8c95xxa_group_names);
	return 0;
}

static int cy8c95xxa_set_mux(struct pinctrl_dev *pctrldev,
		unsigned func_selector, unsigned group_selector)
{
	struct cy8c95xxa *cyp = pinctrl_dev_get_drvdata(pctrldev);
	u8 port, bit;
	int ret;

	dev_info(cyp->dev, "%s called:", __func__);
	dev_info(cyp->dev, "\tfunc_selector: %d", func_selector);
	dev_info(cyp->dev, "\tgroup_selector: %d", group_selector);

	if (func_selector != 1)
		return -ENOTSUPP;

	port = gpio_to_port(group_selector);
	bit = gpio_to_bit(group_selector, port);

	// select port
	ret = cyp_write(cyp, REG_PORT_SELECT, port);
	if (ret < 0) {
		dev_err(cyp->dev, "Could not select port");
		return ret;
	}

	// enable pwm
	ret = cyp_write(cyp, REG_PWM_EAN, BIT(bit));
	if (ret < 0) {
		dev_err(cyp->dev, "Could not enable pwm");
		return ret;
	}

	return 0;
}

static struct pinmux_ops cy8c95xxa_pinmux_ops = {
	.get_functions_count = cy8c95xxa_get_functions_count,
	.get_function_name = cy8c95xxa_get_function_name,
	.get_function_groups = cy8c95xxa_get_function_groups,
	.set_mux = cy8c95xxa_set_mux,
};

static int cy8c95xxa_pin_config_get(struct pinctrl_dev *pctldev,
				    unsigned pin, unsigned long *config)
{
	return -ENOTSUPP;
}

static const u8 pin_config_to_reg[] = {
	[PIN_CONFIG_BIAS_HIGH_IMPEDANCE] = REG_DRV_MODE_HIGH_Z,
	[PIN_CONFIG_BIAS_PULL_DOWN] = REG_DRV_MODE_PULL_DWN,
	[PIN_CONFIG_BIAS_PULL_UP] = REG_DRV_MODE_PULL_UP,
	[PIN_CONFIG_DRIVE_OPEN_DRAIN] = REG_DRV_MODE_OPEN_DRAIN_LOW,
	[PIN_CONFIG_DRIVE_OPEN_SOURCE] = REG_DRV_MODE_OPEN_DRAIN_HIGH,
	[PIN_CONFIG_DRIVE_PUSH_PULL] = REG_DRV_MODE_STRONG,
};

static int cy8c95xxa_pin_config_set(struct pinctrl_dev *pctldev,
				    unsigned pin,
				    unsigned long *configs,
				    unsigned num_configs)
{
	struct cy8c95xxa *cyp = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param;
	u8 port, bit, value;
	int ret;

	port = gpio_to_port(pin);
	bit = gpio_to_bit(pin, port);

	dev_info(cyp->dev, "For pin %d the parameters are:", pin);

	/* only bother to apply the LAST config, since that's the
	 * one that sicks anyway. */
	param = pinconf_to_config_param(configs[num_configs - 1]);
	dev_info(cyp->dev, "\tlast parameter: %d, register: 0x%x",
		 param, pin_config_to_reg[param]);

	value = BIT(bit);

	ret = cyp_write(cyp, REG_PORT_SELECT, port);
	if (ret < 0)
		return ret;

	ret = cyp_write(cyp, pin_config_to_reg[param], value);
	if (ret < 0)
		return ret;

	ret = cyp_read(cyp, pin_config_to_reg[param]);
	if (ret < 0)
		return ret;

	dev_info(cyp->dev, "READ: 0x%x", ret);

	return 0;
}

static const struct pinconf_ops cy8_pinconf_ops = {
	.pin_config_get = cy8c95xxa_pin_config_get,
	.pin_config_set = cy8c95xxa_pin_config_set,
	.is_generic = true,
};

static int cy8c95xxa_get_groups_count(struct pinctrl_dev *pctrldev)
{
	return ARRAY_SIZE(cy8c95xxa_group_names);
}

static const char *cy8c95xxa_get_group_name(struct pinctrl_dev *pctldev,
					    unsigned selector)
{
	return cy8c95xxa_group_names[selector];
}

static const struct pinctrl_ops cy8_pinctrl_ops = {
	.get_groups_count = cy8c95xxa_get_groups_count,
	.get_group_name = cy8c95xxa_get_group_name,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static int cy8c95xxa_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
		const struct pwm_state *state)
{
	struct cy8c95xxa *cyp = container_of(chip, struct cy8c95xxa, pwm_chip);
	u64 period = state->period;
	u64 duty = state->duty_cycle;
	int ret;

	// TODO: apply enable...

	// TODO: for now, lock the clock frequency to the default 32kHz.
	period = __div64_const32(period, PWM_CLK_NS);
	duty = __div64_const32(duty, PWM_CLK_NS);

	// TODO: check the bounds of period!

	dev_info(cyp->dev, "%s called!", __func__);
	dev_info(cyp->dev, "\t For pwm: %d", pwm->pwm);
	dev_info(cyp->dev, "\t Period: %lld", period);
	dev_info(cyp->dev, "\t Duty: %lld", duty);

	ret = cyp_write(cyp, REG_PWM_SELECT, pwm->pwm);
	if (ret < 0) {
		dev_err(cyp->dev, "Could not select pwm %d", pwm->pwm);
		return ret;
	}

	ret = cyp_write(cyp, REG_PWM_PERIOD, period);
	if (ret < 0) {
		dev_err(cyp->dev, "Could not set period");
		return ret;
	}

	ret = cyp_write(cyp, REG_PWM_PULSE_W, duty);
	if (ret < 0) {
		dev_err(cyp->dev, "Could not set pulse width");
		return ret;
	}

	return 0;
}

static void cy8c95xxa_pwm_get_state(struct pwm_chip *chip,
		struct pwm_device *pwm, struct pwm_state *state)
{
	struct cy8c95xxa *cyp = container_of(chip, struct cy8c95xxa, pwm_chip);

	dev_info(cyp->dev, "%s called!", __func__);
	dev_info(cyp->dev, "\t For pwm: %d", pwm->pwm);
}

static const struct pwm_ops cy8c95xxa_pwm_ops = {
	.apply = cy8c95xxa_pwm_apply,
	.get_state = cy8c95xxa_pwm_get_state,
};

/**
 * cy8c9520a_irq_mask - mask (disable) an irq
 */
static void cy8c9520a_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);
	unsigned long offset = data->hwirq;
	u8 port, bit;

	dev_info(cyp->dev, "%s: disable irq %lu", __func__, offset);

	port = gpio_to_port(offset);
	bit = gpio_to_bit(offset, port);
	SET_BIT(bit, &cyp->irq_mask[port]);
}

/**
 * cy8c9520a_irq_unmask - unmask (enable) and irq
 */
static void cy8c9520a_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);
	unsigned long offset = data->hwirq;
	u8 port, bit;

	dev_info(cyp->dev, "%s: enable irq %lu", __func__, offset);

	port = gpio_to_port(offset);
	bit = gpio_to_bit(offset, port);
	CLEAR_BIT(bit, &cyp->irq_mask[port]);
}

static int cy8c9520a_irq_set_type(struct irq_data *data, unsigned int flow_type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);

	if (!(flow_type == IRQ_TYPE_EDGE_BOTH
	      || flow_type == IRQ_TYPE_EDGE_FALLING)) {
		dev_err(cyp->dev, "irq: %d; unsupported type: %d", data->irq,
			flow_type);
		return -EINVAL;
	}
	return 0;
}

static void cy8c9520a_bus_lock(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);

	dev_info(cyp->dev, "%s: Lock bus", __func__);
	mutex_lock(&cyp->irq_lock);
}

static void cy8c9520a_bus_sync_unlock(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);
	int i, ret;

	dev_info(cyp->dev, "%s: sync irq mask", __func__);

	for (i = 0; i < NPORTS; i++) {
		if (cyp->irq_mask_cache[i] ^ cyp->irq_mask[i]) {
			cyp->irq_mask_cache[i] = cyp->irq_mask[i];

			ret = cyp_write(cyp, REG_PORT_SELECT, i);
			if (ret < 0)
				goto unlock;

			ret =
			    cyp_write(cyp, REG_INT_MASK,
				      cyp->irq_mask_cache[i]);
			if (ret < 0)
				goto unlock;
		}
	}

unlock:
	mutex_unlock(&cyp->irq_lock);
}

static irqreturn_t cy8c9520a_irq(int irq, void *data)
{
	struct cy8c95xxa *cyp = data;
	struct gpio_chip *gc = &cyp->chip;
	u8 status[NPORTS], pending;
	int ret, port, gpio, gpio_irq;

	dev_info(cyp->dev, "%s irq entered", __func__);

	ret =
	    i2c_smbus_read_i2c_block_data(cyp->i2c_client, REG_INT_STATUS_BASE,
					  NPORTS, status);
	if (ret < 0)
		memset(status, 0, ARRAY_SIZE(status));

	ret = IRQ_NONE;
	for (port = 0; port < NPORTS; port++) {
		mutex_lock(&cyp->irq_lock);
		pending = status[port] & (~cyp->irq_mask_cache[port]);
		mutex_unlock(&cyp->irq_lock);

		while (pending) {
			ret = IRQ_HANDLED;
			gpio = __ffs(pending);
			pending &= ~BIT(gpio);
			gpio_irq = cy8c9520a_port_offset[port] + gpio;

			handle_nested_irq(irq_find_mapping
					  (gc->irq.domain, gpio_irq));
		}
	}

	return ret;
}

static struct irq_chip cy8c9520a_irq_chip = {
	.name = "cy8c9520a-irq",
	.irq_startup = NULL,	// TODO: lock gpio
	.irq_shutdown = NULL,	// TODO: unlock gpio
	.irq_mask = cy8c9520a_irq_mask,
	.irq_unmask = cy8c9520a_irq_unmask,
	.irq_set_type = cy8c9520a_irq_set_type,
	.irq_bus_lock = cy8c9520a_bus_lock,
	.irq_bus_sync_unlock = cy8c9520a_bus_sync_unlock,
};

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
	u8 port, bit, data;
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);

	port = gpio_to_port(offset);
	bit = gpio_to_bit(offset, port);

	mutex_lock(&cyp->lock);
	assign_bit(bit, (unsigned long *)&cyp->out_cache[port], value);
	data = cyp->out_cache[port];
	mutex_unlock(&cyp->lock);

	ret = cyp_write(cyp, OUT_REG(port), data);
	if (ret < 0)
		dev_err(cyp->dev, "Could not set pin");
}

static int gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	u8 port, bit;
	int line_direction;
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);

	port = gpio_to_port(offset);
	bit = gpio_to_bit(offset, port);

	mutex_lock(&cyp->lock);

	line_direction = cyp->dir_cache[port] & BIT(bit) ?
	    GPIO_LINE_DIRECTION_IN : GPIO_LINE_DIRECTION_OUT;

	mutex_unlock(&cyp->lock);

	return line_direction;
}

static int gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	int ret;
	u8 port, bit, data;
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);

	port = gpio_to_port(offset);
	bit = gpio_to_bit(offset, port);
	mutex_lock(&cyp->lock);
	SET_BIT(bit, &cyp->dir_cache[port]);
	data = cyp->dir_cache[port];
	mutex_unlock(&cyp->lock);

	ret = cyp_write(cyp, REG_PORT_SELECT, port);
	if (ret < 0) {
		dev_err(gc->parent, "Could not select port");
		return ret;
	}

	ret = cyp_write(cyp, REG_PIN_DIR, data);
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
	u8 port, bit, data;
	struct cy8c95xxa *cyp = gpiochip_get_data(gc);

	port = gpio_to_port(offset);
	bit = gpio_to_bit(offset, port);
	mutex_lock(&cyp->lock);
	CLEAR_BIT(bit, &cyp->dir_cache[port]);
	data = cyp->dir_cache[port];
	mutex_unlock(&cyp->lock);

	ret = cyp_write(cyp, REG_PORT_SELECT, port);
	if (ret < 0) {
		dev_err(gc->parent, "Could not select port");
		return ret;
	}

	ret = cyp_write(cyp, REG_PIN_DIR, data);
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

static int init_gpio_irq(struct cy8c95xxa *cyp)
{
	struct gpio_chip *gc = &cyp->chip;
	struct gpio_irq_chip *girq = &gc->irq;
	u8 dummy[NPORTS];
	int ret, i;

	// clear interrupts
	ret =
	    i2c_smbus_read_i2c_block_data(cyp->i2c_client, REG_INT_STATUS_BASE,
					  NPORTS, dummy);
	if (ret < 0)
		goto err;

	// set defaults
	memset(cyp->irq_mask, 0xFF, ARRAY_SIZE(cyp->irq_mask));
	memset(cyp->irq_mask_cache, 0xFF, ARRAY_SIZE(cyp->irq_mask_cache));

	// write defaults
	for (i = 0; i < NPORTS; i++) {
		ret = cyp_write(cyp, REG_PORT_SELECT, i);
		if (ret < 0) {
			dev_err(cyp->dev, "Couldn't select port");
			goto err;
		}

		ret = cyp_write(cyp, REG_INT_MASK, cyp->irq_mask_cache[i]);
		if (ret < 0) {
			dev_err(cyp->dev, "Couldn't set mask");
			goto err;
		}
	}

	girq->chip = &cy8c9520a_irq_chip;
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_bad_irq;
	girq->threaded = true;

	ret =
	    devm_request_threaded_irq(cyp->dev, cyp->i2c_client->irq, NULL,
				      cy8c9520a_irq,
				      IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				      dev_name(cyp->dev), cyp);

err:
	return ret;
}

static int cache_output_reg(struct cy8c95xxa *cyp)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(cyp->i2c_client,
					    REG_OUT_BASE,
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
	mutex_init(&cyp->lock);
	mutex_init(&cyp->irq_lock);

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

	init_gpio_irq(cyp);

	init_gpio_chip(cyp);

	cyp->pwm_chip.dev = cyp->dev;
	cyp->pwm_chip.ops = &cy8c95xxa_pwm_ops;
	cyp->pwm_chip.base = PWM_BASE;
	cyp->pwm_chip.npwm = NPWM;

	ret = devm_pwmchip_add(cyp->dev, &cyp->pwm_chip);
	if (ret < 0) {
		dev_err(cyp->dev, "Could not add pwm chip");
		return ret;
	}

	cyp->pinctrl.name = "cyp8c95xxa-pinctrl";
	cyp->pinctrl.pctlops = &cy8_pinctrl_ops;
	cyp->pinctrl.confops = &cy8_pinconf_ops;
	cyp->pinctrl.pmxops = &cy8c95xxa_pinmux_ops;
	cyp->pinctrl.pins = pinctrl_pins_20a;
	cyp->pinctrl.npins = ARRAY_SIZE(pinctrl_pins_20a);
	cyp->pinctrl.owner = THIS_MODULE;

	ret =
	    devm_pinctrl_register_and_init(cyp->dev, &cyp->pinctrl, cyp,
					   &cyp->pctldev);
	if (ret != 0) {
		dev_err(cyp->dev, "Can't register pin controller");
		return ret;
	}

	ret = pinctrl_enable(cyp->pctldev);
	if (ret != 0) {
		dev_err(cyp->dev, "Could not enable pin controller");
		return ret;
	}

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
