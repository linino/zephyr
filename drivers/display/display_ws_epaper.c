/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * This file comes from display_ssd1673.c and sample code for the waveshare
 * e-paper display.
 * Author Davide CIminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display-util.h"
#include <display.h>

#define LOG_LEVEL CONFIG_DISPLAY_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(display_ws_epaper);

#include <gpio.h>
#include <misc/byteorder.h>
#include <spi.h>
#include <string.h>

#define WHITE 0x00
#define BLACK 0xff

struct ws_epaper_data {
	int sleeping;
	int inverted;
	struct display_common_data common;
	struct spi_config spi_config;
};

#define WS_EPAPER_CMD_DATA_PIN_COMMAND 0
#define WS_EPAPER_CMD_DATA_PIN_DATA 1

#ifdef DT_WS_E_PAPER_0_CS_GPIO_CONTROLLER

static inline int setup_cs_gpio(struct display_common_data *cd)
{
	const char *label = DT_WS_E_PAPER_0_CS_GPIO_CONTROLLER;

	cd->cs_ctrl.gpio_dev = device_get_binding(label);
	if (!cd->cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get spi gpio cd device %s\n", label);
		return -EIO;
	}
	cd->cs_ctrl.gpio_pin = DT_WS_E_PAPER_0_CS_GPIO_PIN;
	cd->cs_ctrl.delay = 0U;
	return 0;
}

#else /* !defined DT_WS_E_PAPER_0_CS_GPIO_CONTROLLER */

static inline int setup_cs_gpio(struct display_common_data *cd)
{
	return 0;
}

#endif /* DT_WS_E_PAPER_0_CS_GPIO_CONTROLLER */


static int ws_epaper_send_init_cmds(struct display_common_data *cd)
{
	const struct display_cmd init_cmds[] = {
		/* Power setting */
		{
			.cmd = 0x01,
			.data = {
				.b = { 0x03, 0x00, 0x2b, 0x2b, 0x03, },
			},
			.data_len = 5,
		},
		/* Booster soft start */
		{
			.cmd = 0x06,
			.data = {
				.b = { 0x17, 0x17, 0x17 },
			},
			.data_len = 3,
		},
		/* Power on */
		{
			.cmd = 0x04,
			.check_bsy_wait_after = 1,
			.check_bsy_wait_timeout = 1000,
		},
		/* Panel setting */
		{
			.cmd = 0x00,
			.data = {
				.b = {
					/* LUT from register */
					0xbf,
					/* VCOM to 0V fast */
					0x0d,
				}
			},
			.data_len = 2,
		},
		/* PLL setting */
		{
			.cmd = 0x30,
			.data = {
				.b = {
					/* 100Hz */
					0x3a,
				},
			},
			.data_len = 1,
		},
		/* Resolution setting */
		{
			.cmd = 0x61,
			.data = {
				.b = {
					DT_WS_E_PAPER_0_YRES,
					DT_WS_E_PAPER_0_XRES >> 8,
					DT_WS_E_PAPER_0_XRES & 0xff,
				},
			},
			.data_len = 3,
		},
		/* VCOM_DC setting */
		{
			.cmd = 0x82,
			.data = { .b = { 0x08, }, },
			.data_len = 1,
		},
		/* VCOM and DATA interval setting */
		{
			.cmd = 0x50,
			.data = { .b = { 0x97, }, },
			.data_len = 1,
		},
	};

	return display_spi_4w_write_cmds(cd, init_cmds, ARRAY_SIZE(init_cmds));
}

static int ws_epaper_do_resume(struct ws_epaper_data *data)
{
	struct display_common_data *cd = &data->common;
	int stat;

	LOG_DBG("%s entered\n", __func__);
	display_reset(cd);
	stat = ws_epaper_send_init_cmds(cd);
	if (stat < 0) {
		LOG_ERR("%s: init cmds error\n", __func__);
		return stat;
	}
	data->sleeping = 0;
	LOG_DBG("%s done\n", __func__);
	return stat;
}


static int ws_epaper_init(struct device *dev)
{
	struct ws_epaper_data *data = (struct ws_epaper_data *)dev->driver_data;
	struct display_common_data *cd = &data->common;

	LOG_DBG("Initializing display driver");

	cd->comm_dev = device_get_binding(DT_WS_E_PAPER_0_BUS_NAME);
	if (cd->comm_dev == NULL) {
		LOG_ERR("Could not get SPI device for WAVESHARE E-PAPER");
		return -EPERM;
	}
	cd->spi_config = &data->spi_config;

	data->spi_config.frequency = DT_WS_E_PAPER_0_SPI_MAX_FREQUENCY;
	data->spi_config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
	data->spi_config.slave = DT_WS_E_PAPER_0_BASE_ADDRESS;
	data->spi_config.cs = &(cd->cs_ctrl);

	cd->rst_gpio.dev =
		device_get_binding(DT_WS_E_PAPER_0_RESET_GPIOS_CONTROLLER);
	if (cd->rst_gpio.dev == NULL) {
		LOG_ERR("Could not get GPIO port for e-paper reset");
		return -EPERM;
	}
	cd->rst_gpio.pin = DT_WS_E_PAPER_0_RESET_GPIOS_PIN;
	gpio_pin_configure(cd->rst_gpio.dev, DT_WS_E_PAPER_0_RESET_GPIOS_PIN,
			   GPIO_DIR_OUT|DT_WS_E_PAPER_0_RESET_GPIOS_FLAGS);
	cd->rst_gpio.delay = 100;

	cd->cd_gpio.dev =
		device_get_binding(DT_WS_E_PAPER_0_DC_GPIOS_CONTROLLER);
	if (cd->cd_gpio.dev == NULL) {
		LOG_ERR("Could not get GPIO port for e-paper command/data");
		return -EPERM;
	}
	cd->cd_gpio.pin = DT_WS_E_PAPER_0_DC_GPIOS_PIN;
	gpio_pin_configure(cd->cd_gpio.dev,
			   cd->cd_gpio.pin,
			   GPIO_DIR_OUT|DT_WS_E_PAPER_0_DC_GPIOS_FLAGS);

	cd->bsy_gpio.dev =
		device_get_binding(DT_WS_E_PAPER_0_BUSY_GPIOS_CONTROLLER);
	if (cd->bsy_gpio.dev == NULL) {
		LOG_ERR("Could not get GPIO port for e-paper busy");
		return -EPERM;
	}
	cd->bsy_gpio.pin = DT_WS_E_PAPER_0_BUSY_GPIOS_PIN;
	gpio_pin_configure(cd->bsy_gpio.dev,
			   cd->bsy_gpio.pin,
			   GPIO_DIR_IN|DT_WS_E_PAPER_0_BUSY_GPIOS_FLAGS);

	setup_cs_gpio(cd);

	data->sleeping = 1;

	return 0;
}

static int ws_epaper_partial_prepare(struct ws_epaper_data *data)
{
	const u8_t lut_vcom_dc_cmd_data[] = {
		0x00, 0x19, 0x01, 0x00, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00,
	};
	const u8_t lut_ww1_cmd_data[] = {
		0x00, 0x19, 0x01, 0x00, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};
	const u8_t lut_bw1_cmd_data[] = {
		0x80, 0x19, 0x01, 0x00, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};
	const u8_t lut_wb1_cmd_data[] = {
		0x40, 0x19, 0x01, 0x00, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};
	const u8_t lut_bb1_cmd_data[] = {
		0x00, 0x19, 0x01, 0x00, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};
	struct display_cmd partial_prepare_cmds[] = {
		{
			/* VCOM_DC setting */
			.cmd = 0x82,
			.data = {
				.b = { 0x08, },
			},
			.data_len = 1,
		},
		{
			/* VCOM and DATA interval setting */
			.cmd= 0x50,
			.data = {
				.b = { 0x17, },
			},
			.data_len = 1,
		},
		/* LUTs */
		{
			.cmd = 0x20,
			.data = {
				.ptr = lut_vcom_dc_cmd_data,
			},
			.data_len = sizeof(lut_vcom_dc_cmd_data),
		},
		{
			.cmd = 0x21,
			.data = {
				.ptr = lut_ww1_cmd_data,
			},
			.data_len = sizeof(lut_ww1_cmd_data),
		},
		{
			.cmd = 0x22,
			.data = {
				.ptr = lut_bw1_cmd_data,
			},
			.data_len = sizeof(lut_bw1_cmd_data),
		},
		{
			.cmd = 0x23,
			.data = {
				.ptr = lut_wb1_cmd_data,
			},
			.data_len = sizeof(lut_wb1_cmd_data),
		},
		{
			.cmd = 0x24,
			.data = {
				.ptr = lut_bb1_cmd_data,
			},
			.data_len = sizeof(lut_bb1_cmd_data),
		},
	};
	struct display_common_data *cd = &data->common;
	int stat;

	LOG_DBG("%s entered\n", __func__);
	stat = display_spi_4w_write_cmds(cd, partial_prepare_cmds,
					 ARRAY_SIZE(partial_prepare_cmds));
	if (stat < 0) {
		LOG_ERR("%s error writing commands\n", __func__);
		return stat;
	}
	LOG_DBG("%s done OK\n", __func__);
	return stat;
}

static int ws_epaper_set_mem_area(struct ws_epaper_data *data,
				  u16_t x_start, u16_t y_start,
				  u16_t x_end, u16_t y_end)
{
	static struct display_cmd set_mem_area_cmds[] = {
		{
			/* Enter partial mode */
			.cmd = 0x91,
		},
		{
			/* Set partial window */
			.cmd = 0x90,
			.data_len = 7,
		},
	};
	struct display_common_data *cd = &data->common;
	u8_t *ptr = set_mem_area_cmds[1].data.b;

	*ptr++ = x_start & 0xff;
	*ptr++ = x_end & 0xff;
	*ptr++ = y_start >> 8;
	*ptr++ = y_start & 0xff;
	*ptr++ = y_end >> 8;
	*ptr++ = y_end & 0xff;
	*ptr++ = 0x01;
	LOG_DBG("%s: args = 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
		__func__,
		set_mem_area_cmds[1].data.b[0], set_mem_area_cmds[1].data.b[1],
		set_mem_area_cmds[1].data.b[2], set_mem_area_cmds[1].data.b[3],
		set_mem_area_cmds[1].data.b[4], set_mem_area_cmds[1].data.b[5]);
	return display_spi_4w_write_cmds(cd, set_mem_area_cmds,
					 ARRAY_SIZE(set_mem_area_cmds));
}

static int ws_epaper_write_old_data(struct ws_epaper_data *data)
{
	struct display_cmd write_old_data_cmds[] = {
		{
			.cmd = 0x10,
		},
	};
	struct display_common_data *cd = &data->common;
	int stat;

	LOG_DBG("%s entered\n", __func__);
	stat = display_spi_4w_write_cmds(cd, write_old_data_cmds,
					 ARRAY_SIZE(write_old_data_cmds));
	if (stat < 0) {
		LOG_ERR("%s: error sending data\n", __func__);
		return stat;
	}
	LOG_DBG("%s done OK\n", __func__);
	return stat;
}

static int ws_epaper_write_new_data(struct ws_epaper_data *data)
{
	struct display_cmd write_new_data_cmds[] = {
		{
			.cmd = 0x13,
		},
	};
	struct display_common_data *cd = &data->common;
	int stat;

	LOG_DBG("%s entered\n", __func__);
	stat = display_spi_4w_write_cmds(cd, write_new_data_cmds,
					 ARRAY_SIZE(write_new_data_cmds));
	if (stat < 0) {
		LOG_ERR("%s: error sending data\n", __func__);
		return stat;
	}
	LOG_DBG("%s done OK\n", __func__);
	return stat;
}

static int ws_epaper_refresh(struct ws_epaper_data *data)
{
	struct display_cmd refresh_cmds[] = {
		{
			.cmd = 0x12,
			.check_bsy_wait_after = 1,
			.check_bsy_wait_timeout = 10000,
		},
	};
	struct display_common_data *cd = &data->common;
	int stat;

	LOG_DBG("%s entered\n", __func__);
	stat = display_spi_4w_write_cmds(cd, refresh_cmds,
					 ARRAY_SIZE(refresh_cmds));
	if (stat < 0) {
		LOG_ERR("%s: error in cmd\n", __func__);
		return stat;
	}
	LOG_DBG("%s done OK\n", __func__);
	return stat;
}

static int ws_epaper_leave_partial_mode(struct ws_epaper_data *data)
{
	struct display_cmd refresh_cmds[] = {
		{
			.cmd = 0x92,
		},
	};
	struct display_common_data *cd = &data->common;
	int stat;

	LOG_DBG("%s entered\n", __func__);
	stat = display_spi_4w_write_cmds(cd, refresh_cmds,
					 ARRAY_SIZE(refresh_cmds));
	if (stat < 0) {
		LOG_ERR("%s: error in cmd\n", __func__);
		return stat;
	}
	LOG_DBG("%s done OK\n", __func__);
	return stat;
}

static int ws_epaper_send_data(struct ws_epaper_data *data,
			       const u8_t *buf, size_t size, size_t actual_size)
{
	struct spi_buf tx_buf;
	struct spi_buf_set tx_bufs;
	int i, stat;
	struct display_common_data *cd = &data->common;

	gpio_pin_write(cd->cd_gpio.dev, cd->cd_gpio.pin, 1);
	tx_bufs.buffers = &tx_buf;
	tx_bufs.count = 1;
	for (i = 0; i < size; i++) {
		/* Works even with buf in flash ! */
		u8_t c;

		if (i < actual_size)
			c = data->inverted ? buf[i] : ~buf[i];
		else
			c = data->inverted ? BLACK : WHITE;
		tx_buf.buf = &c;
		tx_buf.len = 1;
		stat = spi_write(cd->comm_dev, cd->spi_config, &tx_bufs);
		if (stat < 0)
			return stat;
	}
	return 0;
}

static const u8_t white_buf[3000] = {
	[ 0 ... 12 ] = BLACK,
	[ 13 ... 2999 ] = BLACK,
};

static int ws_epaper_clear_screen(struct ws_epaper_data *data)
{
	int stat;

	if (data->sleeping) {
		/* Wakeup if sleeping */
		ws_epaper_do_resume(data);
	}
	stat = ws_epaper_write_old_data(data);
	if (stat < 0)
		return stat;
	stat = ws_epaper_send_data(data, white_buf, 212*104/8, 212*104/8);
	if (stat < 0)
		return stat;
	stat = ws_epaper_write_new_data(data);
	if (stat < 0)
		return stat;
	stat = ws_epaper_send_data(data, white_buf, 212*104/8, 212*104/8);
	if (stat < 0)
		return stat;
	stat = ws_epaper_refresh(data);
	if (stat < 0)
		return stat;
	k_sleep(100);
	return 0;
}

static int ws_epaper_do_write(struct ws_epaper_data *data, u16_t x_start,
			      u16_t x_end, u16_t y_start, u16_t y_end,
			      const void *buf, size_t buf_size)
{
	u16_t nbytes;
	int stat, i;

	/*
	 * Every byte contains 8 pixels, go back with x_start to bit 0 of
	 * relevant byte
	 */
	x_start &= (u16_t)~0x7;
	/* As above, extend x_end to bit 7 */
	x_end = (x_end - 1) | 0x07;
	/* Required buffer size */
	nbytes = ((x_end - x_start + 1) * (y_end - y_start + 1)) >> 3;

	LOG_DBG("%s: x_start = %u, x_end = %u, y_start = %u, y_end = %u, nbytes = %u\n", __func__, x_start, x_end, y_start, y_end, nbytes);

	if (data->sleeping) {
		/* Wakeup if sleeping */
		ws_epaper_do_resume(data);
	}
	/* LUTs */
	stat = ws_epaper_partial_prepare(data);
	if (stat < 0)
		/* FIXME: go to sleep again */
		return stat;
	for (i = 0; i < 2; i++) {
		stat = ws_epaper_set_mem_area(data, x_start, y_start,
					      x_end, y_end);
		if (stat < 0)
			break;
		stat = ws_epaper_write_new_data(data);
		if (stat < 0)
		    break;
		stat = ws_epaper_send_data(data, buf, nbytes, buf_size);
		if (stat < 0)
			break;
		stat = ws_epaper_refresh(data);
		if (stat < 0)
			break;
		stat = ws_epaper_leave_partial_mode(data);
		if (stat < 0)
			break;
	}
	/* Hopefully avoid damaging the display */
	k_sleep(100);
	LOG_DBG("%s done (%d)\n", __func__, stat);
	return stat;
}

static int ws_epaper_write(const struct device *dev, const u16_t X_start,
			   const u16_t Y_start,
			   const struct display_buffer_descriptor *desc,
			   const void *buf)
{
	struct ws_epaper_data *data = (struct ws_epaper_data *)dev->driver_data;
	struct display_common_data *cd = &data->common;
	int i, stat;
	u16_t x_start, y_start, x_end, y_end, X_end, Y_end, nbytes;

	/*             X
	 *    +------------------>
	 *    |          y
	 *    |+------------------>       x = display x coord
	 *    ||  +-----------------+        y = display y coord
	 *    ||  |      W h        |        h = display height (212)
	 *    ||  |              H w|        w = display width (104)
	 *    ||  |                 |
	 *  Y ||x +-----------------+        X = o.s. X coord
	 *    VV                          Y = o.s. Y coord
	 *                                W = o.s. display width (212)
	 *                                H = o.s. display height (104)
	 *         X = y
	 *         Y = x
	 *
	 */
	X_end = X_start + desc->width - 1;
	Y_end = Y_start + desc->height - 1;

	x_start = Y_start;
	x_end = Y_end;
	y_start = X_start;
	y_end = X_end;

	LOG_DBG("%s, X = %u, Y = %u, width = %u, height = %u\n", __func__,
		X_start, Y_start, desc->width, desc->height);
	LOG_DBG("%s, x = %u, y = %u, width = %u, height = %u\n", __func__,
		x_start, y_start, desc->height, desc->width);
	if (desc->pitch > desc->width) {
		LOG_ERR("Unsupported mode");
		return -ENOTSUP;
	}
	if (Y_end > DT_WS_E_PAPER_0_YRES) {
		LOG_ERR("Buffer out of bounds (height)");
		return -EINVAL;
	}
	if (X_end > DT_WS_E_PAPER_0_XRES) {
		LOG_ERR("Buffer out of bounds (width)");
		return -EINVAL;
	}
	LOG_DBG("Writing %dx%d (w,h) @ %dx%d (x,y)",
		desc->height, desc->width, x_start, y_start);

	return ws_epaper_do_write(data, x_start, x_end, y_start, y_end, buf,
				  desc->buf_size);
}

static int ws_epaper_read(const struct device *dev, const u16_t x,
			const u16_t y,
			const struct display_buffer_descriptor *desc,
			void *buf)
{
	LOG_ERR("Reading not supported");
	return -ENOTSUP;
}

static void *ws_epaper_get_framebuffer(const struct device *dev)
{
	LOG_ERR("Direct framebuffer access not supported");
	return NULL;
}

static int ws_epaper_display_suspend(const struct device *dev)
{
	int stat;
	struct ws_epaper_data *data = (struct ws_epaper_data *)dev->driver_data;
	struct display_common_data *cd = &data->common;
	const struct display_cmd off_cmds[] = {
		/* Power off */
		{
			.cmd = 0x02,
			.check_bsy_wait_after = 1,
			.check_bsy_wait_timeout = 1000,
		},
		/* Deep sleep */
		{
			.cmd = 0x07,
			.data = {
				.b = { 0xa5, },
			},
			.data_len = 1,
		},
	};

	if (data->sleeping)
		return 0;

	ws_epaper_clear_screen(data);

	stat = display_spi_4w_write_cmds(cd, off_cmds, ARRAY_SIZE(off_cmds));
	if (stat < 0)
		return stat;

	data->sleeping = 1;
	return stat;
}

static int ws_epaper_display_resume(const struct device *dev)
{
	struct ws_epaper_data *data = (struct ws_epaper_data *)dev->driver_data;
	int ret;

	ret = ws_epaper_do_resume(data);
	if (ret < 0)
		return ret;
	return ws_epaper_clear_screen(data);
}

static int ws_epaper_set_brightness(const struct device *dev,
				  const u8_t brightness)
{
	LOG_WRN("Set brightness not supported");
	return -ENOTSUP;
}

static int ws_epaper_set_contrast(const struct device *dev, const u8_t contrast)
{
	LOG_ERR("Set contrast not supported");
	return -ENOTSUP;
}

static int ws_epaper_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format
				    pixel_format)
{
	struct ws_epaper_data *data = (struct ws_epaper_data *)dev->driver_data;

	switch (pixel_format) {
	case PIXEL_FORMAT_MONO01:
		data->inverted = 0;
		return 0;
	case PIXEL_FORMAT_MONO10:
		data->inverted = 1;
		return 0;
	default:
		LOG_ERR("Unsupported pixel %d", pixel_format);
		return -ENOTSUP;
	}
	/* NEVER REACHED */
	return -ENOTSUP;
}

static int ws_epaper_set_orientation(const struct device *dev,
				   const enum display_orientation orientation)
{
	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		return 0;
	}
	LOG_ERR("Changing display orientation not supported");
	return -ENOTSUP;
}

static void
ws_epaper_get_capabilities(const struct device *dev,
			   struct display_capabilities *capabilities)
{
	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = DT_WS_E_PAPER_0_XRES;
	capabilities->y_resolution = DT_WS_E_PAPER_0_YRES;
	capabilities->supported_pixel_formats = PIXEL_FORMAT_MONO01|
		PIXEL_FORMAT_MONO10;
	capabilities->current_pixel_format = PIXEL_FORMAT_MONO01;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
	capabilities->screen_info = SCREEN_INFO_MONO_VTILED |
		SCREEN_INFO_MONO_MSB_FIRST |
		SCREEN_INFO_EPD |
		SCREEN_INFO_DOUBLE_BUFFER;
}

static const struct display_driver_api ws_epaper_api = {
	.blanking_on = ws_epaper_display_resume,
	.blanking_off = ws_epaper_display_suspend,
	.write = ws_epaper_write,
	.read = ws_epaper_read,
	.get_framebuffer = ws_epaper_get_framebuffer,
	.set_brightness = ws_epaper_set_brightness,
	.set_contrast = ws_epaper_set_contrast,
	.get_capabilities = ws_epaper_get_capabilities,
	.set_pixel_format = ws_epaper_set_pixel_format,
	.set_orientation = ws_epaper_set_orientation,
};

static struct ws_epaper_data ws_epaper_data;

DEVICE_AND_API_INIT(ws_epaper, DT_WS_E_PAPER_0_LABEL, &ws_epaper_init,
		    &ws_epaper_data, NULL, POST_KERNEL,
		    CONFIG_APPLICATION_INIT_PRIORITY, &ws_epaper_api);
