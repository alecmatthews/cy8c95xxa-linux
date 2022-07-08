# Cypress CY8C9520A Device driver makefile
#
# Author: Alec Matthews <me@alecmatthews.dev>

obj-m += gpio-cy8c95xxa.o # TODO: add pinctrl driver

KERNEL_SRC ?= $(HOME)/rpi/linux

all default: modules
install: modules_install

modules modules_install help clean:
	@$(MAKE) -C $(KERNEL_SRC) \
	        ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
			M=$(PWD) $@

.PHONY: deploy
deploy: all ;-scp *.ko overlays/cypress-cy8c95xxa.dts alec@192.168.16.203:
