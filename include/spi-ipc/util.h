/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * Inter-processor communication via spi link, utility functions
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SPI_IPC_UTIL_H__
#define __SPI_IPC_UTIL_H__
#include <spi-ipc/spi-ipc.h>
#include <net/buf.h>

#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

/*
 * Do a simple transaction (header only request, reply with or without data)
 *
 * @spi_ipc_dev: pointer to spi ipc device
 * @pool: pointer to net buffer pool from which buffers must be allocated
 * @request_hdr: pointer to request header
 * @reply_data: pointer to reply data (can be NULL)
 * @len: pointer to reply data length (must be initialized to expected max len,
 *       it is set to actual length by function on success).
 */
extern int spi_ipc_simple_trans(struct device *spi_ipc_dev,
				struct net_buf_pool *pool,
				const union spi_thb *request_hdr,
				void *reply_data,
				size_t *len);


#endif /* __SPI_IPC_UTIL_H__ */
