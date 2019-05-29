/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * Inter-processor communication via spi link, ethernet protocol
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <net/ethernet.h>
#include <spi-ipc/spi-ipc.h>
#include <spi-ipc/proto.h>
#include <spi-ipc/wifi-mgmt.h>

int spi_ipc_wifi_mgmt_scan(struct device *dev, scan_result_cb_t cb)
{
	return -1;
}

int spi_ipc_wifi_mgmt_connect(struct device *dev,
			      struct wifi_connect_req_params *params)
{
	return -1;
}

int spi_ipc_wifi_mgmt_disconnect(struct device *dev)
{
	return -1;
}

static void spi_ipc_wifi_mgmt_rx_cb(const struct spi_ipc_proto *proto,
				    struct net_buf *buf, void *proto_data)
{
	/* does nothing for the moment */
}

static const struct spi_ipc_proto spi_ipc_wifi_mgmt_proto = {
	.name = "spi-ipc-wifi-mgmt",
	.proto_id = SPI_IPC_PROTO_WIFI_MGMT,
	.rx_cb = spi_ipc_wifi_mgmt_rx_cb,
};

extern int init_spi_ipc_wifi_mgmt(struct device *spi_ipc_dev,
				  struct net_if *iface)
{
	const struct spi_ipc_driver_api *api = spi_ipc_dev->driver_api;

	if (!api || !api->open)
		return -EINVAL;
	spi_ipc_dev->driver_data = iface;
	return api->open(spi_ipc_dev, &spi_ipc_wifi_mgmt_proto, iface);
}
