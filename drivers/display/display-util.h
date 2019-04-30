/*
 * Copyright (c) DogHunter LLC and the Linino organization 2019
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * Common utility functions for displays
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spi.h>

/*
 * A display gpio, just like a struct spi_cs_control
 */
struct display_gpio {
	struct device *dev;
	int pin;
	int delay;
};

/*
 * Common display data
 */
struct display_common_data {
	/* SPI or I2C device */
	struct device *comm_dev;
	/* Spi config (if spi) */
	struct spi_config *spi_config;
	/* Reset gpio device */
	struct display_gpio rst_gpio;
	/* Command/data gpio device */
	struct display_gpio cd_gpio;
	/* Spi cs gpio */
	struct spi_cs_control cs_ctrl;
	/* Display busy gpio */
	struct display_gpio bsy_gpio;
};

/*
 * A display command
 */
struct display_cmd {
	u8_t cmd;
	union {
		const u8_t *ptr;
		u8_t b[8];
	} data;
	/*
	 * If data_len > 8, data contains a pointer, otherwise it contains
	 * actual data
	 */
	size_t data_len;
	/* pre busy wait check */
	int delay1;
	int check_bsy_wait_after;
	int check_bsy_wait_timeout;
	/* post busy wait check */
	int delay2;
};

/*
 * 4 wires spi write command (regular spi + gpio operated command/data line)
 *
 * @display_data: pointer to display common data
 * @cmd: pointer to command structure
 */
int display_spi_4w_write_cmd(struct display_common_data *display_data,
			     const struct display_cmd *cmd);

/*
 * 4 wires spi write commands
 *
 * @display_data: pointer to display common data
 * @cmd: pointer to array of command structures
 * @ncmds: number of commands
 */
static inline
int display_spi_4w_write_cmds(struct display_common_data *display_data,
			      const struct display_cmd *cmds, int ncmds)
{
	int i, stat = 0;

	for (i = 0; i < ncmds && !stat; i++)
		stat = display_spi_4w_write_cmd(display_data, &cmds[i]);

	return stat;
}

/*
 * Busy wait function
 *
 * @display_data: pointer to display common data struct
 * @timeout_ms: wait timeout in msecs (-1 for no timeout)
 */
int display_busy_wait(struct display_common_data *, int timeout_ms);

/*
 * Reset function
 *
 * @display_data: pointer to display common data struct
 */
int display_reset(struct display_common_data *);
