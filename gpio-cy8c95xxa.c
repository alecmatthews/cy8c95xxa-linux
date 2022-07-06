#include <linux/i2c.h>
#include <linux/module.h>

enum cypress_ioexpander_type {
    CYP_TYPE_20A,
    CYP_TYPE_40A,
    CYP_TYPE_60A
};

static int cy8c95xxa_probe(struct i2c_client *client)
{
    dev_info(&client->dev, "Device detected");
    return 0;
}

static int cy8c95xxa_remove(struct i2c_client *client)
{
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
