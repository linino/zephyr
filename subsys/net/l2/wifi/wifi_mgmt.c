/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(net_wifi_mgmt, CONFIG_NET_L2_WIFI_MGMT_LOG_LEVEL);

#include <errno.h>

#include <net/net_core.h>
#include <net/net_if.h>
#include <net/wifi_mgmt.h>
#ifdef CONFIG_NET_L2_WIFI
#include <net/ethernet.h>
#endif /* CONFIG_NET_L2_WIFI */

#ifndef CONFIG_NET_L2_WIFI
static int do_wifi_connect(u32_t mgmt_request, struct device *dev,
			   struct wifi_connect_req_params *params)
{
	struct net_wifi_mgmt_offload *off_api =
		(struct net_wifi_mgmt_offload *) dev->driver_api;

	if (off_api == NULL || off_api->connect == NULL) {
		return -ENOTSUP;
	}

	return off_api->connect(dev, params);
}
#else /* CONFIG_NET_L2_WIFI */
static int do_wifi_connect(u32_t mgmt_request, struct device *dev,
			   struct wifi_connect_req_params *params)
{
	const struct ethernet_api *eth_api = dev->driver_api;

	if (eth_api == NULL || eth_api->wifi_connect == NULL) {
		return -ENOTSUP;
	}

	return eth_api->wifi_connect(dev, params);
}
#endif

static int wifi_connect(u32_t mgmt_request, struct net_if *iface,
			void *data, size_t len)
{
	struct wifi_connect_req_params *params =
		(struct wifi_connect_req_params *)data;
	struct device *dev = net_if_get_device(iface);

	NET_DBG("%s %u %u %u %s %u",
		params->ssid, params->ssid_length,
		params->channel, params->security,
		params->psk, params->psk_length);

	if ((params->security > WIFI_SECURITY_TYPE_PSK) ||
	    (params->ssid_length > WIFI_SSID_MAX_LEN) ||
	    (params->ssid_length == 0U) ||
	    ((params->security == WIFI_SECURITY_TYPE_PSK) &&
	     ((params->psk_length < 8) || (params->psk_length > 64) ||
	      (params->psk_length == 0U) || !params->psk)) ||
	    ((params->channel != WIFI_CHANNEL_ANY) &&
	     (params->channel > WIFI_CHANNEL_MAX)) ||
	    !params->ssid) {
		return -EINVAL;
	}

	return do_wifi_connect(mgmt_request, dev, params);
}

NET_MGMT_REGISTER_REQUEST_HANDLER(NET_REQUEST_WIFI_CONNECT, wifi_connect);

static void scan_result_cb(struct net_if *iface, int status,
			    struct wifi_scan_result *entry)
{
	if (!iface) {
		return;
	}

	if (!entry) {
		struct wifi_status scan_status = {
			.status = status,
		};

		net_mgmt_event_notify_with_info(NET_EVENT_WIFI_SCAN_DONE,
						iface, &scan_status,
						sizeof(struct wifi_status));
		return;
	}

	net_mgmt_event_notify_with_info(NET_EVENT_WIFI_SCAN_RESULT, iface,
					entry, sizeof(struct wifi_scan_result));
}

#ifndef CONFIG_NET_L2_WIFI
static int wifi_scan(u32_t mgmt_request, struct net_if *iface,
		     void *data, size_t len)
{
	struct device *dev = net_if_get_device(iface);
	struct net_wifi_mgmt_offload *off_api =
		(struct net_wifi_mgmt_offload *) dev->driver_api;

	if (off_api == NULL || off_api->scan == NULL) {
		return -ENOTSUP;
	}

	return off_api->scan(dev, scan_result_cb);
}
#else /* CONFIG_NET_L2_WIFI */
static int wifi_scan(u32_t mgmt_request, struct net_if *iface,
		     void *data, size_t len)
{
	struct device *dev = net_if_get_device(iface);
	const struct ethernet_api *eth_api = dev->driver_api;

	if (eth_api == NULL || eth_api->wifi_scan == NULL) {
		return -ENOTSUP;
	}

	return eth_api->wifi_scan(dev, scan_result_cb);
}
#endif

NET_MGMT_REGISTER_REQUEST_HANDLER(NET_REQUEST_WIFI_SCAN, wifi_scan);

#ifndef CONFIG_NET_L2_WIFI
static int wifi_disconnect(u32_t mgmt_request, struct net_if *iface,
			   void *data, size_t len)
{
	struct device *dev = net_if_get_device(iface);
	struct net_wifi_mgmt_offload *off_api =
		(struct net_wifi_mgmt_offload *) dev->driver_api;

	if (off_api == NULL || off_api->disconnect == NULL) {
		return -ENOTSUP;
	}

	return off_api->disconnect(dev);
}
#else /* CONFIG_NET_L2_WIFI */
static int wifi_disconnect(u32_t mgmt_request, struct net_if *iface,
			   void *data, size_t len)
{
	struct device *dev = net_if_get_device(iface);
	const struct ethernet_api *eth_api = dev->driver_api;

	if (eth_api == NULL || eth_api->wifi_disconnect == NULL) {
		return -ENOTSUP;
	}

	return eth_api->wifi_disconnect(dev);
}
#endif

NET_MGMT_REGISTER_REQUEST_HANDLER(NET_REQUEST_WIFI_DISCONNECT, wifi_disconnect);

void wifi_mgmt_raise_connect_result_event(struct net_if *iface, int status)
{
	struct wifi_status cnx_status = {
		.status = status,
	};

	net_mgmt_event_notify_with_info(NET_EVENT_WIFI_CONNECT_RESULT,
					iface, &cnx_status,
					sizeof(struct wifi_status));
}

void wifi_mgmt_raise_disconnect_result_event(struct net_if *iface, int status)
{
	struct wifi_status cnx_status = {
		.status = status,
	};

	net_mgmt_event_notify_with_info(NET_EVENT_WIFI_DISCONNECT_RESULT,
					iface, &cnx_status,
					sizeof(struct wifi_status));
}

#ifndef CONFIG_NET_L2_WIFI
static int wifi_ap_enable(u32_t mgmt_request, struct net_if *iface,
			  void *data, size_t len)
{
	struct wifi_connect_req_params *params =
		(struct wifi_connect_req_params *)data;
	struct device *dev = net_if_get_device(iface);
	struct net_wifi_mgmt_offload *off_api =
		(struct net_wifi_mgmt_offload *) dev->driver_api;

	if (off_api == NULL || off_api->ap_enable == NULL) {
		return -ENOTSUP;
	}

	return off_api->ap_enable(dev, params);
}
#else /* CONFIG_NET_L2_WIFI */
static int wifi_ap_enable(u32_t mgmt_request, struct net_if *iface,
			  void *data, size_t len)
{
	struct wifi_connect_req_params *params =
		(struct wifi_connect_req_params *)data;
	struct device *dev = net_if_get_device(iface);
	const struct ethernet_api *eth_api = dev->driver_api;

	if (eth_api == NULL || eth_api->wifi_ap_enable == NULL) {
		return -ENOTSUP;
	}

	return eth_api->wifi_ap_enable(dev, params);
}
#endif /* CONFIG_NET_L2_WIFI */

NET_MGMT_REGISTER_REQUEST_HANDLER(NET_REQUEST_WIFI_AP_ENABLE, wifi_ap_enable);

#ifndef CONFIG_NET_L2_WIFI
static int wifi_ap_disable(u32_t mgmt_request, struct net_if *iface,
			  void *data, size_t len)
{
	struct device *dev = net_if_get_device(iface);
	struct net_wifi_mgmt_offload *off_api =
		(struct net_wifi_mgmt_offload *) dev->driver_api;

	if (off_api == NULL || off_api->ap_enable == NULL) {
		return -ENOTSUP;
	}

	return off_api->ap_disable(dev);
}
#else /* CONFIG_NET_L2_WIFI */
static int wifi_ap_disable(u32_t mgmt_request, struct net_if *iface,
			  void *data, size_t len)
{
	struct device *dev = net_if_get_device(iface);
	const struct ethernet_api *eth_api = dev->driver_api;

	if (eth_api == NULL || eth_api->wifi_ap_enable == NULL) {
		return -ENOTSUP;
	}

	return eth_api->wifi_ap_disable(dev);
}
#endif /* CONFIG_NET_L2_WIFI */

NET_MGMT_REGISTER_REQUEST_HANDLER(NET_REQUEST_WIFI_AP_DISABLE, wifi_ap_disable);
