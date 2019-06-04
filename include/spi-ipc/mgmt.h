/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * Generic inter-processor communication via spi link
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SPI_IPC_MGMT_H__
#define __SPI_IPC_MGMT_H__

#include <kernel.h>

extern int init_spi_ipc_mgmt(struct device *spi_ipc_dev,
			     struct spi_ipc_data *instance_data);

#endif /* __SPI_IPC_MGMT_H__ */
