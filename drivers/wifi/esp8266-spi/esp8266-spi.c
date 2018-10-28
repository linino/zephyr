/**
 * Copyright (c) 2018 WhatsNext gmbh
 * Author: Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp8266-spi-log.h"
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <net/net_if.h>
#include <net/wifi_mgmt.h>
#include <net/net_offload.h>

#include "esp8266-spi.h"

#define SCAN_RETRY_DELAY 2000  /* ms */

static struct esp8266_spi_data esp8266_spi_data;

static int esp8266_spi_get_mac(struct esp8266_spi_data *data)
{
	return -1;
}

static int esp8266_spi_iface_send(struct net_if *iface, struct net_pkt *pkt)
{
	return -1;
}

static int esp8266_spi_mgmt_scan(struct device *dev, scan_result_cb_t cb)
{
	return -1;
}

static int esp8266_spi_mgmt_connect(struct device *dev,
				    struct wifi_connect_req_params *params)
{
	return -1;
}

static int esp8266_spi_mgmt_disconnect(struct device *dev)
{
	return -1;
}

static void esp8266_spi_iface_init(struct net_if *iface)
{
	esp8266_spi_data.iface = iface;

	/* Grab our MAC address: */
	if (esp8266_spi_get_mac(&esp8266_spi_data) < 0) {
		LOG_ERR("could not read mac address");
		return;
	}

	LOG_DBG("MAC Address %02X:%02X:%02X:%02X:%02X:%02X",
		esp8266_spi_data.mac[0], esp8266_spi_data.mac[1],
		esp8266_spi_data.mac[2],
		esp8266_spi_data.mac[3], esp8266_spi_data.mac[4],
		esp8266_spi_data.mac[5]);

	net_if_set_link_addr(iface, esp8266_spi_data.mac,
			     sizeof(esp8266_spi_data.mac),
			     NET_LINK_ETHERNET);
}

static const struct net_wifi_mgmt_offload esp8266_spi_api = {
	.iface_api.init = esp8266_spi_iface_init,
	.iface_api.send = esp8266_spi_iface_send,
	.scan		= esp8266_spi_mgmt_scan,
	.connect	= esp8266_spi_mgmt_connect,
	.disconnect	= esp8266_spi_mgmt_disconnect,
};

static void esp8266_spi_scan_work_handler(struct k_work *work)
{
}

static int esp8266_spi_init(struct device *dev)
{
	ARG_UNUSED(dev);

	/* We use system workqueue to deal with scan retries: */
	k_delayed_work_init(&esp8266_spi_data.work,
			    esp8266_spi_scan_work_handler);

	LOG_DBG("SimpleLink driver Initialized");

	return 0;
}

NET_DEVICE_OFFLOAD_INIT(esp8266_spi, CONFIG_WIFI_ESP8266_SPI_NAME,
			esp8266_spi_init, &esp8266_spi_data, NULL,
			CONFIG_WIFI_INIT_PRIORITY, &esp8266_spi_api,
			CONFIG_WIFI_ESP8266_SPI_MAX_PACKET_SIZE);
