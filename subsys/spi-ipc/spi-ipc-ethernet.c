/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * Inter-processor communication via spi link, ethernet protocol
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "spi-ipc-log.h"
#include "spi-ipc-private.h"
#include <net/ethernet.h>
#include <spi-ipc/spi-ipc.h>
#include <spi-ipc/proto.h>
#include <spi-ipc/ethernet.h>
#include <spi-ipc/util.h>

LOG_MODULE_DECLARE(LOG_MODULE_NAME);

/* Tx buffer pool */
NET_BUF_POOL_DEFINE(eth_spi_ipc_pool, 64,
		    sizeof(union spi_thb),
		    sizeof(struct spi_msg *), spi_ipc_net_buf_destroy);

static int spi_ipc_ether_get_mac(struct device *spi_ipc_dev, u8_t mac[6])
{
	DECLARE_SPI_IPC_REQUEST_BUF(b, SPI_IPC_PROTO_ETHERNET, MAC_ADDR,
				    0, 0, 0);
	int ret;
	size_t len = 6;

	k_sleep(1000);

	ret = spi_ipc_simple_trans(spi_ipc_dev, &eth_spi_ipc_pool, &b,
				   mac, &len, 4000);
	if (ret < 0)
		return ret;
	if (len != 6) {
		printk("%s: invalid reply length %d\n", __func__, len);
		return -EINVAL;
	}
	printk("%s: got mac = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
	       __func__, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	return ret;
}

void spi_ipc_iface_init(struct device *spi_ipc_dev, struct net_if *iface)
{
	static u8_t mac[6];

	if (spi_ipc_ether_get_mac(spi_ipc_dev, mac) < 0) {
		printk("could not read mac address\n");
		return;
	}

	K_DEBUG("MAC Address %02X:%02X:%02X:%02X:%02X:%02X\n",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	net_if_set_link_addr(iface, mac, sizeof(mac), NET_LINK_ETHERNET);
	ethernet_init(iface);
}


int spi_ipc_ether_start(struct device *spi_ipc_dev, struct device *eth_dev)
{
	DECLARE_SPI_IPC_REQUEST_BUF(b, SPI_IPC_PROTO_ETHERNET, START, 0, 0, 0);

	return spi_ipc_simple_trans(spi_ipc_dev, &eth_spi_ipc_pool, &b,
				    NULL, NULL, 2000);
}

int spi_ipc_ether_stop(struct device *spi_ipc_dev, struct device *eth_dev)
{
	DECLARE_SPI_IPC_REQUEST_BUF(b, SPI_IPC_PROTO_ETHERNET, STOP, 0, 0, 0);

	return spi_ipc_simple_trans(spi_ipc_dev, &eth_spi_ipc_pool, &b,
				    NULL, NULL, 2000);
}

int spi_ipc_ether_send(struct device *dev, struct net_pkt *pkt)
{
	int ret;
	DECLARE_SPI_IPC_BUF(thb, SPI_IPC_PROTO_ETHERNET, NET_PACKET, 0, 0, 0,
			    0);
	size_t len = net_pkt_get_len(pkt);
	struct net_buf *hdr_frag;
	struct net_pkt *cloned_pkt;
	const struct spi_ipc_driver_api *api = dev->driver_api;

	if (!api || !api->submit_buf)
		return -EINVAL;

	spi_ipc_set_data_len(&thb, len);
	spi_ipc_set_transaction(&thb, spi_ipc_new_transaction());
	cloned_pkt = net_pkt_clone(pkt, K_FOREVER);
	if (!cloned_pkt) {
		printk("%s: CANNOT CLONE packet\n", __func__);
		return -ENOMEM;
	}
	hdr_frag = net_buf_alloc_len(&eth_spi_ipc_pool, 32, K_FOREVER);
	if (!hdr_frag) {
		net_pkt_unref(cloned_pkt);
		return -ENOMEM;
	}
	net_buf_add_mem(hdr_frag, &thb, sizeof(thb));
	net_pkt_frag_insert(cloned_pkt, hdr_frag);
	net_pkt_cursor_init(cloned_pkt);
	{
		struct net_buf *frag = cloned_pkt->frags;
		int i = 0;
		while (frag) {
			/*
			 * References first fragment
			 * Refs should be: pkt -> 1, frag1 -> 2, frag2 -> 1
			 */
			if (i++ == 0)
				net_pkt_frag_ref(frag);
			frag = frag->frags;
		}
	}
	/* Submit buffer, no reply expected */
	ret = api->submit_buf(dev, cloned_pkt->buffer, NULL, NULL, 0);
	/*
	 * Unreference packet
	 * references should be pkt -> 0 (freed), frag1 -> 1, frag2 -> 1
	 */
	net_pkt_unref(cloned_pkt);
	return ret;
}

static void spi_ipc_rx_cb(const struct spi_ipc_proto *proto,
			  struct spi_ipc_data *instance_data,
			  struct net_buf *buf, void *proto_data)
{
	/* Allocate network packet with no buffer */
	struct net_if *iface = proto_data;
	struct net_pkt *pkt = net_pkt_rx_alloc_on_iface(iface, K_FOREVER);

	if (!pkt) {
		printk("%s: cannot allocate net packet\n", __func__);
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
