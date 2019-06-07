/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * Generic inter-processor communication via spi link
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SPI_IPC_WIFI_MGMT_H__
#define __SPI_IPC_WIFI_MGMT_H__

#include <net/ethernet.h>
#include <net/wifi_mgmt.h>
#include <device.h>
#include <net/net_pkt.h>

enum spi_ipc_wifi_mgmt_sec {
	SEC_OPEN = 0,
	SEC_WEP,
	SEC_WPA_PSK,
	SEC_WPA2_PSK,
	SEC_WPA_WPA2_PSK,
};

extern int spi_ipc_wifi_mgmt_scan(struct device *dev, scan_result_cb_t cb);
extern int spi_ipc_wifi_mgmt_connect(struct device *dev,
				     struct wifi_connect_req_params *params);
extern int spi_ipc_wifi_mgmt_disconnect(struct device *dev);

extern int init_spi_ipc_wifi_mgmt(struct device *spi_ipc_dev,
				  struct net_if *iface);

#endif /* __SPI_IPC_WIFI_MGMT_H__ */

