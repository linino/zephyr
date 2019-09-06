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
#include <net/wifi_mgmt.h>
#include <spi-ipc/spi-ipc.h>
#include <spi-ipc/proto.h>
#include <spi-ipc/wifi-mgmt.h>

#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

LOG_MODULE_DECLARE(LOG_MODULE_NAME);

struct scan_msg_cb_data {
	scan_result_cb_t cb;
	struct net_if *iface;
	struct k_sem completion_sem;
};

struct spi_ipc_wifi_net_data {
	uint32_t ssl_ch_sec_psl_rssi;
	uint32_t bss_low;
	uint32_t bss_hi;
	uint32_t rsvd;
};

struct spi_ipc_wifi_connect_request_hdr {
	struct spi_header_thb p1;
	struct spi_ipc_wifi_net_data p2;
};

/* Tx buffer pool */
NET_BUF_POOL_DEFINE(wifi_mgmt_pool, 4,
		    sizeof(union spi_thb),
		    sizeof(struct spi_msg *), NULL);

static void _scan_msg_cb(struct net_buf *reply, void *cb_arg)
{
	struct scan_msg_cb_data *scbd = cb_arg;
	struct spi_ipc_wifi_net_data nd;
	union spi_thb thb;
	struct wifi_scan_result sr;
	int error;
	u8_t rssi, sec, ch, ssl;

	K_DEBUG("%s %d\n", __func__, __LINE__);

	if (!reply) {
		K_DEBUG("NO REPLY, TIMEOUT !\n");
		k_sem_give(&scbd->completion_sem);
		return;
	}

	net_buf_linearize(&thb.hdr, sizeof(thb.hdr), reply, 0, sizeof(thb.hdr));
	error = spi_ipc_error(&thb);
	if (error) {
		/* Signal scan done with error */
		scbd->cb(scbd->iface, error, NULL);
		return;
	}

	/*
	 * Read third subframe, which contains network data (rssi, sec,
	 * ch, ssl, bssid
	 */
	net_buf_linearize(&nd, sizeof(nd), reply, 2 * sizeof(thb.data),
			  sizeof(nd));
	ssl = nd.ssl_ch_sec_psl_rssi & 0xffUL;
	ch = (nd.ssl_ch_sec_psl_rssi >> 8) & 0xffUL;
	sec = (nd.ssl_ch_sec_psl_rssi >> 16) & 0xffUL;
	rssi = (nd.ssl_ch_sec_psl_rssi >> 24) & 0xffUL;

	K_DEBUG("ssl = %u, ch = %u, sec = %u, rssi = %u\n", ssl, ch, sec, rssi);

	memset(sr.ssid, 0, sizeof(sr.ssid));
	sr.ssid_length = ssl;
	sr.channel = ch;
	sr.security = sec > 0 ?
		WIFI_SECURITY_TYPE_PSK : WIFI_SECURITY_TYPE_NONE;
	sr.rssi = rssi;

	net_buf_linearize(sr.ssid, ssl, reply, sizeof(thb.data), ssl);

	K_DEBUG("%s, found network %s\n", __func__, sr.ssid);

	/* Signal scan result */
	scbd->cb(scbd->iface, error, &sr);

	if (spi_ipc_is_last_reply(&thb)) {
		K_DEBUG("SCAN LAST REPLY\n");
		/* Signal scan done */
		scbd->cb(scbd->iface, error, NULL);
		/* Make spi_ipc_wifi_mgmt_scan finish */
		k_sem_give(&scbd->completion_sem);
	}
}

int spi_ipc_wifi_mgmt_scan(struct device *dev, scan_result_cb_t cb)
{
	struct net_if *iface = dev->driver_data;
	struct net_buf *buf =
	    net_buf_alloc_len(&wifi_mgmt_pool, sizeof(union spi_thb), 1000);
	DECLARE_SPI_IPC_BUF(b, SPI_IPC_PROTO_WIFI_MGMT,
			    SCAN_NETWORK | SPI_IPC_REQUEST,
			    0, 0, 0, 0);
	struct scan_msg_cb_data static scbd;
	const struct spi_ipc_driver_api *api = dev->driver_api;
	int ret;

	scbd.cb = cb;
	scbd.iface = iface;
	spi_ipc_set_transaction(&b, spi_ipc_new_transaction());
	k_sem_init(&scbd.completion_sem, 0, 1);
	net_buf_add_mem(buf, &b, sizeof(b));
	ret = api->submit_buf(dev, buf, _scan_msg_cb, &scbd, 10000);
	if (ret)
		return ret;
	if (k_sem_take(&scbd.completion_sem, 10000)) {
		printk("%s: timeout\n", __func__);
		/* FIXME: INVOKE CALLBACK ? */
		/* FIXME: CANCEL BUFFER ? */
	}
	return 0;
}

static void _connect_msg_cb(struct net_buf *reply, void *cb_arg)
{
	struct net_if *iface = cb_arg;
	union spi_thb b;

	net_buf_linearize(&b, sizeof(b.hdr), reply, 0, sizeof(b.hdr));
	wifi_mgmt_raise_connect_result_event(iface, spi_ipc_error(&b));
}

int spi_ipc_wifi_mgmt_connect(struct device *dev,
			      struct wifi_connect_req_params *params)
{
	struct net_if *iface = dev->driver_data;
	DECLARE_SPI_IPC_BUF(b, SPI_IPC_PROTO_WIFI_MGMT,
			    CONNECT | SPI_IPC_REQUEST,
			    0, 0, 0, 0);
	struct spi_ipc_wifi_connect_request_hdr *hdr =
		(struct spi_ipc_wifi_connect_request_hdr *)&b;
	/* Header + ssid + psk (if psk shorter than 33 bytes) */
	int len = 3 * sizeof(union spi_thb), ret;
	struct net_buf *buf;
	const struct spi_ipc_driver_api *api = dev->driver_api;
	u8_t sec;

	if (params->psk_length > sizeof(union spi_thb)) {
		/* More than 32 bytes required for psk */
		len += sizeof(union spi_thb);
	}
	spi_ipc_set_data_len(&b, len - sizeof(union spi_thb));
	spi_ipc_set_transaction(&b, spi_ipc_new_transaction());
	buf = net_buf_alloc_len(&wifi_mgmt_pool, len, 1000);
	if (!buf) {
		printk("%s: NO MEMORY FOR BUFFER\n", __func__);
		return -ENOMEM;
	}
	/* Set up protocol specific part of header */
	memset(&hdr->p2, 0, sizeof(hdr->p2));
	sec = params->security == WIFI_SECURITY_TYPE_NONE ?
		SEC_OPEN : SEC_WPA_WPA2_PSK;
	hdr->p2.ssl_ch_sec_psl_rssi = params->ssid_length |
		(params->channel << 8) |
		(sec << 16) |
		(params->psk_length << 24);
	net_buf_add_mem(buf, &b, sizeof(b));
	/* Add ssid */
	memset(&b, 0, sizeof(b));
	memcpy(&b, params->ssid, min(params->ssid_length, sizeof(b)));
	K_DEBUG("ssid = %s\n", params->ssid);
	net_buf_add_mem(buf, &b, sizeof(b));
	/* Add psk */
	memset(&b, 0, sizeof(b));
	len = min(params->psk_length, sizeof(b));
	memcpy(&b, params->psk, len);
	net_buf_add_mem(buf, &b, sizeof(b));
	len -= sizeof(b);
	if (len > 0) {
		memset(&b, 0, sizeof(b));
		memcpy(&b, &params->psk[sizeof(b)], len);
		net_buf_add_mem(buf, &b, sizeof(b));
	}
	ret = api->submit_buf(dev, buf, _connect_msg_cb, iface, 5000);
	if (ret) {
		net_buf_unref(buf);
	}
	return ret;
}

static void _disconnect_msg_cb(struct net_buf *reply, void *cb_arg)
{
	struct net_if *iface = cb_arg;
	union spi_thb b;

	net_buf_linearize(&b, sizeof(b.hdr), reply, 0, sizeof(b.hdr));
	wifi_mgmt_raise_disconnect_result_event(iface, spi_ipc_error(&b));
}

int spi_ipc_wifi_mgmt_disconnect(struct device *dev)
{
	struct net_if *iface = dev->driver_data;
	DECLARE_SPI_IPC_BUF(b, SPI_IPC_PROTO_WIFI_MGMT, DISCONNECT,
			    0, 0, 0, 0);
	int ret;
	struct net_buf *buf;
	const struct spi_ipc_driver_api *api = dev->driver_api;

	buf = net_buf_alloc_len(&wifi_mgmt_pool, sizeof(b), 1000);
	if (!buf) {
		printk("%s: NO MEMORY FOR BUFFER\n", __func__);
		return -ENOMEM;
	}
	spi_ipc_set_transaction(&b, spi_ipc_new_transaction());
	net_buf_add_mem(buf, &b, sizeof(b));
	ret = api->submit_buf(dev, buf, _disconnect_msg_cb, iface, 2000);
	if (ret) {
		net_buf_unref(buf);
	}
	return ret;
}

static void spi_ipc_wifi_mgmt_rx_cb(const struct spi_ipc_proto *proto,
				    struct spi_ipc_data *instance_data,
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
	return api->open(spi_ipc_dev, &spi_ipc_wifi_mgmt_proto, iface);
}
