/*
 * Copyright (c) DogHunter LLC and the Linino organization 2019
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * Implementation of common utility functions for displays
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <spi.h>
#include <gpio.h>
#include "display-util.h"

#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

#define LOG_LEVEL CONFIG_DISPLAY_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(display_util);

int display_spi_4w_write_cmd(struct display_common_data *cd,
			     const struct display_cmd *cmd)
{
	int i, err;
	struct spi_buf buf = {.buf = &cmd->cmd, .len = 1, };
	struct spi_buf_set buf_set = {.buffers = &buf, .count = 1};
	const u8_t *ptr = cmd->data_len > sizeof(cmd->data.b) ?
		cmd->data.ptr : cmd->data.b;

	gpio_pin_write(cd->cd_gpio.dev, cd->cd_gpio.pin, 0);
	if (cd->cd_gpio.delay)
		k_sleep(cd->cd_gpio.delay);
	err = spi_write(cd->comm_dev, cd->spi_config, &buf_set);

	if ((cmd->data_len > 8 && !cmd->data.ptr) || err < 0)
		goto no_data;

	gpio_pin_write(cd->cd_gpio.dev, cd->cd_gpio.pin, 1);
	for (i = 0; i < cmd->data_len; i++) {
		u8_t c = ptr[i];
		buf.buf = &c;
		buf.len = 1;
		err = spi_write(cd->comm_dev, cd->spi_config, &buf_set);
	}

no_data:
	if ((!cmd->delay1 && !cmd->check_bsy_wait_after) || err < 0) {
		return err;
	}
	if (cmd->delay1)
		k_sleep(cmd->delay1);
	err = display_busy_wait(cd, cmd->check_bsy_wait_timeout);

	if (!cmd->delay2 || err < 0) {
		return err;
	}
	k_sleep(cmd->delay2);
	return err;
}

int display_busy_wait(struct display_common_data *cd, int timeout)
{
	u32_t v;

	LOG_DBG("BSY WAIT ENTERED");
	do {
		gpio_pin_read(cd->bsy_gpio.dev, cd->bsy_gpio.pin, &v);
		if (!v) {
			LOG_DBG("BSY WAIT OK");
			return 0;
		}
		k_sleep(min(timeout / 100, 1));
	} while(1);
	LOG_DBG("BSY WAIT TIMEOUT");
	return -ETIMEDOUT;
}

int display_reset(struct display_common_data *cd)
{
	gpio_pin_write(cd->rst_gpio.dev, cd->rst_gpio.pin, 0);
	k_sleep(cd->rst_gpio.delay);
	gpio_pin_write(cd->rst_gpio.dev, cd->rst_gpio.pin, 1);
	k_sleep(cd->rst_gpio.delay);
	gpio_pin_write(cd->rst_gpio.dev, cd->rst_gpio.pin, 0);
	return 0;
}
