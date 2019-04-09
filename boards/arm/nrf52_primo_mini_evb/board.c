/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 *
 * Linino.org is a dog hunter sponsored community project
 *
 * nrf52 primo mini evb, board initialization
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <init.h>
#include <gpio.h>

#define GPIO_DEV_NAME "GPIO_0"
#define GPIO_OUT_PIN 23

/* Turn on P0.23 */
static int board_nrf52_primo_mini_evb_init(struct device *dev)
{
	struct device *gpio_dev;
	int ret;

	ARG_UNUSED(dev);

	gpio_dev = device_get_binding(GPIO_DEV_NAME);
	if (!gpio_dev) {
		printk("%s: cannot find %s\n", __func__, GPIO_DEV_NAME);
		return -1;
	}
	ret = gpio_pin_configure(gpio_dev, GPIO_OUT_PIN, GPIO_DIR_OUT);
	if (ret < 0) {
		printk("%s: cannot configure gpio pin\n", __func__);
		return -1;
	}
	ret = gpio_pin_write(gpio_dev, GPIO_OUT_PIN, 1);
	if (ret < 0) {
		printk("%s: could not turn on peripherals\n", __func__);
		return -1;
	}
	return 0;
}


SYS_INIT(board_nrf52_primo_mini_evb_init, PRE_KERNEL_1,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
