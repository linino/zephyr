/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * Inter-processor communication via spi link, low level management messages
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <kernel.h>
#include <net/ethernet.h>
#include <spi-ipc/spi-ipc.h>
#include <spi-ipc/proto.h>
#include <spi-ipc/mgmt.h>
#include <spi-ipc/util.h>
#include "spi-ipc-log.h"

LOG_MODULE_DECLARE(LOG_MODULE_NAME);

/* Tx buffer pool */
NET_BUF_POOL_DEFINE(spi_ipc_mgmt_pool, 2,
		    sizeof(union spi_thb),
		    sizeof(struct spi_msg *), NULL);

static void spi_ipc_mgmt_alive(struct k_timer *timer)
{
	struct device *spi_ipc_dev = k_timer_user_data_get(timer);
	/* Send an alive message to the other end */
	DECLARE_SPI_IPC_BUF(b, SPI_IPC_PROTO_MGMT, ALIVE, 0, 0, 0, 0);

	if (spi_ipc_simple_trans(spi_ipc_dev, &spi_ipc_mgmt_pool, &b,
				 NULL, NULL) < 0) {
		LOG_ERR("%s: spi_ipc_simple_trans() returns error\n", __func__);
	}
}

static void spi_ipc_mgmt_rx_cb(const struct spi_ipc_proto *proto,
			       struct spi_ipc_data *instance_data,
			       struct net_buf *buf, void *proto_data)
{
	union spi_thb hdr;
	int err = 0;
	size_t stat;
	struct spi_ipc_mgmt_data *md = spi_ipc_get_mgmt_data(instance_data);

	stat = net_buf_linearize(&hdr, sizeof(hdr), buf, 0, sizeof(hdr));
	if (stat != sizeof(hdr)) {
		LOG_ERR("ERROR READING MGMT MESSAGE\n");
	}
	if (spi_ipc_code(&hdr) != ALIVE) {
		LOG_ERR("UNKNOWN MGMT MESSAGE\n");
		err = -EINVAL;
	} else
		err = spi_ipc_error(&hdr);
	k_poll_signal_raise(&md->diag_signal, err);
}

static const struct spi_ipc_proto spi_ipc_mgmt_proto = {
	.name = "spi-ipc-mgmt",
	.proto_id = SPI_IPC_PROTO_MGMT,
	.rx_cb = spi_ipc_mgmt_rx_cb,
};

int init_spi_ipc_mgmt(struct device *spi_ipc_dev,
		      struct spi_ipc_data *instance_data)
{
	const struct spi_ipc_driver_api *api = spi_ipc_dev->driver_api;
	int ret;
	struct spi_ipc_mgmt_data *md = spi_ipc_get_mgmt_data(instance_data);

	k_poll_signal_init(&md->diag_signal);
	k_timer_init(&md->alive_timer, spi_ipc_mgmt_alive, NULL);
	if (!api || !api->open)
		return -EINVAL;
	ret = api->open(spi_ipc_dev, &spi_ipc_mgmt_proto, NULL);
	if (ret < 0)
		return ret;
	k_timer_user_data_set(&md->alive_timer, spi_ipc_dev);
	k_timer_start(&md->alive_timer, 1000, 1000);
	return ret;
}
