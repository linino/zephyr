/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * Inter-processor communication via spi link, ethernet protocol
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "spi-ipc-log.h"
#include <net/ethernet.h>
#include <spi-ipc/spi-ipc.h>
#include <spi-ipc/proto.h>
#include <spi-ipc/ethernet.h>
#include <spi-ipc/util.h>

LOG_MODULE_DECLARE(LOG_MODULE_NAME);

/* Tx buffer pool */
NET_BUF_POOL_DEFINE(eth_spi_ipc_pool, 2,
		    sizeof(union spi_thb),
		    sizeof(struct spi_msg *), NULL);

static int spi_ipc_ether_get_mac(struct device *spi_ipc_dev, u8_t mac[6])
{
	DECLARE_SPI_IPC_REQUEST_BUF(b, SPI_IPC_PROTO_ETHERNET, MAC_ADDR,
				    0, 0, 0);
	int ret;
	size_t len = 6;

	ret = spi_ipc_simple_trans(spi_ipc_dev, &eth_spi_ipc_pool, &b,
				   mac, &len);
	if (ret < 0)
		return ret;
	if (len != 6) {
		LOG_ERR("%s: invalid reply length %d\n", __func__, len);
		return -EINVAL;
	}
	return ret;
}

void spi_ipc_iface_init(struct device *spi_ipc_dev, struct net_if *iface)
{
	u8_t mac[6];

	if (spi_ipc_ether_get_mac(spi_ipc_dev, mac) < 0) {
		LOG_ERR("could not read mac address");
		return;
	}

	LOG_DBG("MAC Address %02X:%02X:%02X:%02X:%02X:%02X",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	net_if_set_link_addr(iface, mac, sizeof(mac), NET_LINK_ETHERNET);
}


int spi_ipc_ether_start(struct device *spi_ipc_dev, struct device *eth_dev)
{
	DECLARE_SPI_IPC_REQUEST_BUF(b, SPI_IPC_PROTO_ETHERNET, START, 0, 0, 0);

	return spi_ipc_simple_trans(spi_ipc_dev, &eth_spi_ipc_pool, &b,
				    NULL, NULL);
}

int spi_ipc_ether_stop(struct device *spi_ipc_dev, struct device *eth_dev)
{
	DECLARE_SPI_IPC_REQUEST_BUF(b, SPI_IPC_PROTO_ETHERNET, STOP, 0, 0, 0);

	return spi_ipc_simple_trans(spi_ipc_dev, &eth_spi_ipc_pool, &b,
				    NULL, NULL);
}

int spi_ipc_ether_send(struct device *dev, struct net_pkt *pkt)
{
	int ret;
	DECLARE_SPI_IPC_BUF(thb, SPI_IPC_PROTO_ETHERNET, NET_PACKET, 0, 0, 0,
			    0);
	size_t len = net_pkt_get_len(pkt);
	struct net_buf *hdr, *buf;
	const struct spi_ipc_driver_api *api = dev->driver_api;

	if (!api || !api->submit_buf)
		return -EINVAL;

	spi_ipc_set_data_len(&thb, len);
	hdr = net_pkt_get_frag(pkt, 1000);
	if (!hdr) {
		LOG_ERR("%s: cannot allocate header fragment", __func__);
		return -ENOMEM;
	}
	net_buf_add_mem(hdr, &thb, sizeof(thb));
	/* Insert spi ipc header */
	net_pkt_frag_insert(pkt, hdr);

	/* Throw away network packet, but save its network buffer */
	buf = pkt->buffer;
	net_buf_ref(buf);
	net_pkt_unref(pkt);
	/* Submit buffer, no reply expected */
	ret = api->submit_buf(dev, buf, NULL);
	return ret;
}

static void spi_ipc_rx_cb(const struct spi_ipc_proto *proto,
			  struct spi_ipc_data *instance_data,
			  struct net_buf *buf, void *proto_data)
{
	/* Allocate network packet with no buffer */
	struct net_pkt *pkt = net_pkt_rx_alloc(0);
	struct net_if *iface = proto_data;

	if (!pkt) {
		LOG_ERR("%s: cannot allocate net packet\n", __func__);
		return;
	}
	/* Remove spi ipc header in front of packet */
	net_buf_pull(buf, sizeof(union spi_thb));
	/* Keep buffer referenced (will be unreferenced by our caller) */
	net_buf_ref(buf);
	/* Assign buffer to network packet */
	net_pkt_append_buffer(pkt, buf);
	/* Pass network packet on to upper layers */
	if (net_recv_data(iface, pkt) < 0) {
		net_pkt_unref(pkt);
	}
}

static const struct spi_ipc_proto spi_ipc_eth_proto = {
	.name = "spi-ipc-ether",
	.proto_id = SPI_IPC_PROTO_ETHERNET,
	.rx_cb = spi_ipc_rx_cb,
};

/* Registers ethernet protocol */
int init_spi_ipc_ethernet(struct device *spi_ipc_dev, struct net_if *iface)
{
	const struct spi_ipc_driver_api *api = spi_ipc_dev->driver_api;

	if (!api || !api->open)
		return -EINVAL;
	return api->open(spi_ipc_dev, &spi_ipc_eth_proto, iface);
}
