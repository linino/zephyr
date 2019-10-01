/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * SPI-IPC private header
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SPI_IPC_PRIVATE_H__
#define __SPI_IPC_PRIVATE_H__

struct net_buf;

void spi_ipc_net_buf_destroy(struct net_buf *nb);


#endif /* __SPI_IPC_PRIVATE_H__ */
