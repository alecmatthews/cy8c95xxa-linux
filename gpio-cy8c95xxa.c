#include <linux/module.h>

static int __init helloworld_init(void) {
    pr_info("Hello, World: init\n");
    pr_info("small change");
    return 0;
}

static void __exit helloworld_exit(void) {
    pr_info("Hello, World: exit\n");
}

module_init(helloworld_init);
module_exit(helloworld_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alec Matthews <me@alecmatthews.dev>");
MODULE_DESCRIPTION("Kernel driver for the Cypress CY8C95XXA family of i2c gpio expanders");