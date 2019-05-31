/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * Inter-processor communication via spi link, utility functions
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <device.h>
#include <spi-ipc/util.h>

#include "spi-ipc-log.h"

LOG_MODULE_DECLARE(LOG_MODULE_NAME);

int spi_ipc_simple_trans(struct device *spi_ipc_dev,
			 struct net_buf_pool *pool,
			 const union spi_thb *request_hdr,
			 void *reply_data,
			 size_t *len)
{
	const struct spi_ipc_driver_api *api = spi_ipc_dev->driver_api;
	struct net_buf *buf =
		net_buf_alloc_len(pool, sizeof(union spi_thb), 0), *reply;
	struct net_buf **preply;
	union spi_thb b;
	int ret;

	if (!buf) {
		LOG_ERR("%s: no memory\n", __func__);
		return -ENOMEM;
	}
	net_buf_add_mem(buf, request_hdr, sizeof(*request_hdr));
	preply = reply_data && len && *len ? &reply : NULL;
	ret = api->submit_buf(spi_ipc_dev, buf, preply);
	if (ret < 0 || !preply) {
		goto end0;
	}
	net_buf_linearize(&b.hdr, sizeof(b.hdr), reply, 0, sizeof(b.hdr));
	if (spi_ipc_error(&b)) {
		LOG_ERR("%s: error from remote (%d)\n", __func__,
			spi_ipc_error(&b));
		ret = spi_ipc_error(&b);
		goto end1;
	}
	if (reply_data && len && *len) {
		size_t l = min(*len, spi_ipc_data_len(&b));

		net_buf_linearize(reply_data, l, reply, sizeof(b.hdr), l);
		*len = l;
	}
end1:
	net_buf_unref(reply);
end0:
	net_buf_unref(buf);
	return ret;
}
