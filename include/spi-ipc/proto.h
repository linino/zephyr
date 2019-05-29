/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * Generic inter-processor communication via spi link
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SPI_IPC_PROTO_H__
#define __SPI_IPC_PROTO_H__

/* Well known protcol ids */

/* Spi ipc management */
#define SPI_IPC_PROTO_MGMT		0x0001
# define ALIVE				0x0001
/* Wifi management */
#define SPI_IPC_PROTO_WIFI_MGMT		0x0002
# define SCAN_NETWORK			0x0001
# define CONNECT			0x0002
# define DISCONNECT			0x0003
/* Network interface */
#define SPI_IPC_PROTO_ETHERNET		0x0003
# define MAC_ADDR			0x0001
# define NET_PACKET			0x0002
# define START				0x0003
# define STOP				0x0004

#endif /* __SPI_IPC_PROTO_H__ */

