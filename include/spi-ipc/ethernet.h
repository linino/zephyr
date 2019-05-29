/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * Generic inter-processor communication via spi link
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SPI_IPC_NET_H__
#define __SPI_IPC_NET_H__

#include <net/ethernet.h>
#include <device.h>
#include <net/net_pkt.h>

union spi_thb;
struct net_buf;

extern void spi_ipc_iface_init(struct net_if *iface);
extern int spi_ipc_ether_start(struct device *dev);
extern int spi_ipc_ether_stop(struct device *dev);
extern int spi_ipc_ether_send(struct device *dev, struct net_pkt *pkt);
extern int spi_ipc_ether_get_mac(struct device *dev, u8_t mac[6]);
extern int init_spi_ipc_ethernet(struct device *spi_ipc_dev,
				 struct net_if *iface);

#endif /* __SPI_IPC_NET_H__ */

