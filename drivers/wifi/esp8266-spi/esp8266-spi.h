/**
 * Copyright (c) 2018 WhatsNext gmbh
 * Author: Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __ESP8266_SPI_H__
#define __ESP8266_SPI_H__

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <net/net_if.h>
#include <net/wifi_mgmt.h>
#include <net/net_offload.h>

struct esp8266_spi_data {
	struct net_if *iface;
	unsigned char mac[6];

	struct k_delayed_work work;
	scan_result_cb_t scan_cb;
	int scan_result;
	int scan_retries;
};


#endif /* __ESP8266_SPI_H__ */
