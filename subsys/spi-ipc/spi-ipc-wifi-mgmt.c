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
	net_buf_linearize(&nd, sizeof(nd), reply, 2 * sizeof(thb.hdr),
			  sizeof(nd));
	ssl = nd.ssl_ch_sec_psl_rssi & 0xffUL;
	ch = (nd.ssl_ch_sec_psl_rssi >> 8) & 0xffUL;
	sec = (nd.ssl_ch_sec_psl_rssi >> 16) & 0xffUL;
	rssi = (nd.ssl_ch_sec_psl_rssi >> 24) & 0xffUL;

	memset(sr.ssid, 0, sizeof(sr.ssid));
	sr.ssid_length = ssl;
	sr.channel = ch;
	sr.security = sec > 0 ?
		WIFI_SECURITY_TYPE_PSK : WIFI_SECURITY_TYPE_NONE;
	sr.rssi = rssi;

	net_buf_linearize(sr.ssid, ssl, reply, sizeof(thb.hdr), ssl);

	/* Signal scan result */
	scbd->cb(scbd->iface, error, &sr);

	if (spi_ipc_is_last_reply(&thb)) {
		LOG_DBG("SCAN LAST REPLY");
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
	DECLARE_SPI_IPC_BUF(b, SPI_IPC_PROTO_WIFI_MGMT, SCAN_NETWORK,
			    0, 0, 0, 0);
	struct scan_msg_cb_data scbd;
	const struct spi_ipc_driver_api *api = dev->driver_api;
	int ret;

	scbd.cb = cb;
	scbd.iface = iface;
	k_sem_init(&scbd.completion_sem, 0, 1);
	net_buf_add_mem(buf, &b, sizeof(b));
	ret = api->submit_buf(dev, buf, _scan_msg_cb, &scbd);
	if (ret)
		return ret;
	if (k_sem_take(&scbd.completion_sem, 10000)) {
		LOG_ERR("%s: timeout", __func__);
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
	DECLARE_SPI_IPC_BUF(b, SPI_IPC_PROTO_WIFI_MGMT, CONNECT,
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
	buf = net_buf_alloc_len(&wifi_mgmt_pool, len, 1000);
	if (!buf) {
		LOG_ERR("%s: NO MEMORY FOR BUFFER \n", __func__);
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
	ret = api->submit_buf(dev, buf, _connect_msg_cb, iface);
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
		LOG_ERR("%s: NO MEMORY FOR BUFFER \n", __func__);
		return -ENOMEM;
	}
	net_buf_add_mem(buf, &b, sizeof(b));
	ret = api->submit_buf(dev, buf, _disconnect_msg_cb, iface);
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
	spi_ipc_dev->driver_data = iface;
	return api->open(spi_ipc_dev, &spi_ipc_wifi_mgmt_proto, iface);
}
