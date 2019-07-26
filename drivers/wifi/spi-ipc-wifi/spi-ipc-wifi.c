/**
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * spi-ipc based wifi driver
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "spi-ipc-wifi-log.h"
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <gpio.h>
#include <net/net_if.h>
#include <net/ethernet.h>
#include <net/net_l2.h>
#include <net/wifi_mgmt.h>
#include <net/ethernet.h>
#include <net/net_pkt.h>
#include <spi-ipc/spi-ipc.h>
#include <spi-ipc/ethernet.h>
#include <spi-ipc/wifi-mgmt.h>
#include <spi-ipc/proto.h>


static void _spi_ipc_wifi_iface_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct device *spi_ipc_dev = dev->driver_data;
	int stat = init_spi_ipc_ethernet(spi_ipc_dev, iface);

	if (stat < 0) {
		LOG_ERR("%s: error initializing ethernet via spi-ipc\n",
			__func__);
		return;
	}
	stat = init_spi_ipc_wifi_mgmt(spi_ipc_dev, iface);
	if (stat < 0) {
		LOG_ERR("%s: error initializing wifi mgmt via spi-ipc\n",
			__func__);
		return;
	}
	spi_ipc_iface_init(spi_ipc_dev, iface);
}

static int _spi_ipc_wifi_send(struct device *dev, struct net_pkt *pkt)
{
	struct device *spi_ipc_dev = dev->driver_data;

	return spi_ipc_ether_send(spi_ipc_dev, pkt);
}

static int _spi_ipc_wifi_mgmt_scan(struct device *dev, scan_result_cb_t cb)
{
	struct device *spi_ipc_dev = dev->driver_data;

	return spi_ipc_wifi_mgmt_scan(spi_ipc_dev, cb);
}

static int _spi_ipc_wifi_mgmt_connect(struct device *dev,
				      struct wifi_connect_req_params *params)
{
	struct device *spi_ipc_dev = dev->driver_data;

	return spi_ipc_wifi_mgmt_connect(spi_ipc_dev, params);
}

static int _spi_ipc_wifi_mgmt_disconnect(struct device *dev)
{
	struct device *spi_ipc_dev = dev->driver_data;

	return spi_ipc_wifi_mgmt_disconnect(spi_ipc_dev);
}

static const struct ethernet_api spi_ipc_wifi_api = {
	.iface_api.init		= _spi_ipc_wifi_iface_init,
	.send			= _spi_ipc_wifi_send,
	.wifi_scan		= _spi_ipc_wifi_mgmt_scan,
	.wifi_connect		= _spi_ipc_wifi_mgmt_connect,
	.wifi_disconnect	= _spi_ipc_wifi_mgmt_disconnect,
};

#ifdef DT_SPI_IPC_0_LABEL
static int spi_ipc_wifi_init0(struct device *dev)
{
	dev->driver_data = device_get_binding(DT_SPI_IPC_0_LABEL);

	if (!dev->driver_data) {
		LOG_ERR("%s: no spi ipc device\n", __func__);
		return -ENODEV;
	}

	LOG_DBG("spi-ipc driver initialized");

	return 0;
}

NET_DEVICE_INIT(spi_ipc_wifi, DT_SPI_IPC_0_LABEL,
		spi_ipc_wifi_init0, NULL,
		NULL, CONFIG_WIFI_INIT_PRIORITY, &spi_ipc_wifi_api,
		ETHERNET_L2, NET_L2_GET_CTX_TYPE(ETHERNET_L2),
		NET_ETH_MTU);
#endif
