/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * Generic inter-processor communication via spi link
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "spi-ipc-log.h"

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <spi.h>
#include <gpio.h>
#include <init.h>
#include <net/net_if.h>
#include <net/ethernet.h>
#include <net/net_l2.h>
#include <net/wifi_mgmt.h>
#include <net/ethernet.h>
#include <net/net_pkt.h>
#include <spi-ipc/spi-ipc.h>
#include <spi-ipc/mgmt.h>

LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define N_SPI_MSGS 8
/* Max number of supported protocols (CONFIG ?) */
#define N_SPI_IPC_PROTOS 4
/* Max number of spi devices */
#ifdef DT_SPI_IPC_0_LABEL
#define N_SPI_IPC_DEVS 1
#elif defined DT_SPI_IPC_0_LABEL && defined DT_SPI_IPC_1_LABEL;
#define N_SPI_IPC_DEVS 2
#endif

#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

/*
 *
 * struct spi_msg : describes a message made of multiple transaction halves
 * (32 bytes each). Contains a completion semaphore which is signalled on
 * reply reception. Request data is stored in a net_buf, whose user_data
 * pointer points to the relevant struct spi_msg:
 *
 * struct spi_msg         struct net_buf       struct net_buf     struct spi_msg
 * +-------------+<-+ +->+--------------+   +--------------+<--+ +->+---------+
 * |             |  | |  |       A      |   |      B       |   | |  |         |
 * |             | +--+  |              |   |              | +---+  |         |
 * |             | ||    |              |   |              | | |    |         |
 * |             | |+----|-- user_data  |   | user_data ---|-o |    |         |
 * |    netbuf --|-+     +--------------+   +--------------+ | +----|--netbuf |
 * |    reply ---|-------------------------------------------+   +--|--reply  |
 * +-------------+                                               |  +---------+
 *                                                              NULL
 *
 */
struct spi_msg {
	/* Error status of message + flags (last reply) */
	u32_t flags_error;
	/* Transaction number */
	u16_t trans;
	/* Total message length in bytes, excluding the header */
	unsigned int data_len;
	/* Protocol + code */
	u32_t proto_code;
	/* Pointer to related actual message */
	struct net_buf *netbuf;
	/* Pointer to relevant reply (if any) */
	struct spi_msg *reply;
	/* Completion semaphore */
	struct k_sem completion_sem;
	/* Used to link to outstanding messages (requests) */
	sys_dnode_t list;
};

struct spi_ipc_config_data {
	const char *spi_dev_label;
	/* Master attention request gpio data */
	const char *gpio_dev_label;
	int gpio_pin_number;
	int gpio_pin_flags;
	/* Device minor number (index starting from 0) */
	int minor;
};

struct spi_ipc_proto_instance {
	const struct spi_ipc_proto *proto;
	void *data;
};

struct spi_ipc_data {
	struct device *spi_dev;
	struct device *gpio_dev;
	struct k_fifo fifo;
	/* List of outstanding request messages */
	sys_dlist_t outstanding;
	struct spi_ipc_proto_instance protos[N_SPI_IPC_PROTOS];
	int input_index;
	int output_index;
	struct k_sem sem;
	struct k_poll_signal spi_signal;
	union spi_thb spi_input[2];
	union spi_thb spi_output[2];
	struct spi_buf spi_input_buf;
	struct spi_buf_set spi_input_bs;
	struct spi_buf spi_output_buf;
	struct spi_buf_set spi_output_bs;
	struct spi_buf_set *tx_bs;
	struct net_buf *curr_tx_net_buf;
	struct spi_msg *curr_tx_spi_msg;
	struct net_buf *curr_rx_net_buf;
	struct spi_msg *curr_rx_spi_msg;
	const struct spi_ipc_proto *curr_rx_proto;
	void *curr_rx_proto_data;
	/* Used in case of rx errors (no memory) */
	int n_discard_subframes;
	struct spi_config spi_config;
#ifdef CONFIG_SPI_IPC_MGMT
	struct spi_ipc_mgmt_data mgmt_data;
#endif
};

static struct k_mem_slab spi_ipc_msg_slab;
static struct spi_msg __aligned(4) spi_msg_slab_buf[N_SPI_MSGS];
/* spi-ipc rx pool: get room for 2 eth frames */
#define N_BUFFERS_PER_ETH_FRAME \
    (NET_ETH_MAX_FRAME_SIZE/sizeof(union spi_thb) + \
     (NET_ETH_MAX_FRAME_SIZE%sizeof(union spi_thb) ? 1 : 0))

NET_BUF_POOL_DEFINE(spi_ipc_pool, 2 * N_BUFFERS_PER_ETH_FRAME,
		    sizeof(union spi_thb),
		    sizeof(struct spi_msg *), NULL);
static struct k_thread spi_ipc_thread;

#ifdef CONFIG_SPI_IPC_MGMT
/* spi done signal, fifo ready, mgmt event */
#define EVTS_PER_DEV 3
#else
/* spi done signal, fifo ready */
#define EVTS_PER_DEV 2
#endif

struct k_poll_event events[EVTS_PER_DEV * N_SPI_IPC_DEVS];

#define SPI_IPC_THREAD_STACK_SIZE 1024
K_THREAD_STACK_MEMBER(spi_ipc_thread_stack, SPI_IPC_THREAD_STACK_SIZE);

struct device *active_devices[N_SPI_IPC_DEVS];

#ifdef CONFIG_SPI_IPC_MGMT
static int init_mgmt(struct device *dev, struct spi_ipc_data *data)
{
	return init_spi_ipc_mgmt(dev, data);
}

static void init_mgmt_event(struct spi_ipc_data *data, int minor)
{
	k_poll_event_init(&events[minor + 2],
			  K_POLL_TYPE_IGNORE,
			  K_POLL_MODE_NOTIFY_ONLY,
			  &data->mgmt_data.diag_signal);
}

/* Call this with data->sem taken */
static inline int _check_mgmt_evt(struct spi_ipc_data *data, int *result)
{
	int got_signal;

	k_poll_signal_check(&data->mgmt_data.diag_signal, &got_signal, result);
	if (got_signal)
		k_poll_signal_reset(&data->spi_signal);
	return got_signal;
}
#else /* !CONFIG_SPI_IPC_MGMT */
static inline int init_mgmt(struct device *dev, struct spi_ipc_data *data)
{
	return 0;
}

static inline void init_mgmt_event(struct spi_ipc_data *data, int minor)
{
}

static inline int _check_mgmt_evt(struct spi_ipc_data *data, int *result)
{
	return 0;
}
#endif /* !CONFIG_SPI_IPC_MGMT */

static inline
void spi_ipc_request_spi_xfer(struct spi_ipc_data *data,
				     const struct spi_ipc_config_data *cfg)
{
	gpio_pin_write(data->gpio_dev, cfg->gpio_pin_number, 1);
}

static inline
void spi_ipc_reset_spi_xfer_request(struct spi_ipc_data *data,
				    const struct spi_ipc_config_data *cfg)
{
	gpio_pin_write(data->gpio_dev, cfg->gpio_pin_number, 0);
}


static int _new_proto(struct spi_ipc_data *data,
		      const struct spi_ipc_proto *p, void *proto_data)
{
	/* FIXME: SEMAPHORE */
	int i;

	for (i = 0; i < ARRAY_SIZE(data->protos); i++) {
		if (!data->protos[i].proto) {
			data->protos[i].proto = p;
			data->protos[i].data = proto_data;
			return i;
		}
	}
	return -ENOMEM;
}

static const struct spi_ipc_proto *_find_proto(struct spi_ipc_data *data,
					       u16_t id, void **proto_data)
{
	int i;
	const struct spi_ipc_proto *out;

	if (proto_data)
		*proto_data = NULL;
	for (i = 0; i < ARRAY_SIZE(data->protos); i++) {
		if (data->protos[i].proto &&
		    (data->protos[i].proto->proto_id == id)) {
			out = data->protos[i].proto;
			if (proto_data)
				*proto_data = data->protos[i].data;
			return out;
		}
	}
	return NULL;
}

static struct spi_msg *_find_relevant_request(struct spi_ipc_data *data,
					      union spi_thb *in)
{
	struct spi_msg *r;

	if (!spi_ipc_is_reply(in))
		/* Not a reply at all */
		return NULL;

	SYS_DLIST_FOR_EACH_CONTAINER(&data->outstanding, r, list) {
		if (spi_ipc_reply_matches(in, r->proto_code, r->trans))
			return r;
	}
	return NULL;
}

static void setup_dev(struct device *dev)
{
	struct spi_ipc_data *data = dev->driver_data;
	const struct spi_ipc_config_data *cfg = dev->config->config_info;
	int stat;

	data->tx_bs = NULL;

	k_sem_take(&data->sem, K_FOREVER);
	if (!data->curr_tx_net_buf) {
		data->curr_tx_net_buf = net_buf_get(&data->fifo, 0);
	}
	if (data->curr_tx_net_buf) {
		size_t l;

		if (!data->curr_tx_spi_msg) {
			memcpy(&data->curr_tx_spi_msg,
			       data->curr_tx_net_buf->user_data,
			       sizeof(data->curr_tx_spi_msg));
		}
		l = min(net_buf_frags_len(data->curr_tx_net_buf), 32);
		memset(data->spi_output[data->output_index].data, 0,
		       sizeof(data->spi_output[0]));
		memcpy(data->spi_output[data->output_index].data,
		       net_buf_pull_mem(data->curr_tx_net_buf, l), l);
		data->output_index = (data->output_index + 1) & 0x1;
		data->tx_bs = &data->spi_output_bs;
		if ((l - sizeof(union spi_thb)) >=
		    data->curr_tx_spi_msg->data_len ) {
			/* Transmitting header */
			union spi_thb *thb = data->spi_output;

			if (spi_ipc_is_request(thb)) {
				/*
				 * If message is a request, append it
				 * to the list of outstanding requests
				 */
				sys_dlist_append(&data->outstanding,
						 &data->curr_tx_spi_msg->list);
				/*
				 * Buffer will be unreferenced after tx, keep
				 * a reference to it
				 */
				net_buf_ref(data->curr_tx_net_buf);
			}
		}
		if (l <= sizeof(union spi_thb)) {
			/* Last tx for this message All done */
			net_buf_unref(data->curr_tx_net_buf);
			k_mem_slab_free(&spi_ipc_msg_slab,
					(void **)&data->curr_rx_spi_msg);
			data->curr_tx_net_buf = NULL;
			data->curr_tx_spi_msg = NULL;
		}
	}

	/* Ping pong input buffer */
	data->spi_input_buf.buf = data->spi_input[data->input_index].data;
	data->spi_input_buf.len = sizeof(union spi_thb);
	data->input_index = (data->input_index + 1) & 0x1;
	k_sem_give(&data->sem);

	stat = spi_transceive_async(data->spi_dev, &data->spi_config,
				    data->tx_bs, &data->spi_input_bs,
				    &data->spi_signal);

	if (data->tx_bs) {
		/* Request a transfer via gpio (we're slave) */
		spi_ipc_request_spi_xfer(data, cfg);
	}
}

#ifdef CONFIG_SPI_IPC_MGMT
struct spi_ipc_mgmt_data *spi_ipc_get_mgmt_data(struct spi_ipc_data *data)
{
	return &data->mgmt_data;
}
#endif

static void spi_ipc_handle_input(struct spi_ipc_data *data, u8_t *buf)
{
	union spi_thb *in = (union spi_thb *)buf;
	int stat;
	size_t l, tot_msg_len;

	if (!data->curr_rx_net_buf) {
		/* First sub frame received */
		data->curr_rx_proto = _find_proto(data, spi_ipc_proto(in),
						  &data->curr_rx_proto_data);
		if (!data->curr_rx_proto) {
			LOG_DBG("Unsupported protocol, discarding frame\n");
			data->n_discard_subframes = spi_ipc_data_subframes(in);
			return;
		}
		data->curr_rx_net_buf =
			net_buf_alloc_len(&spi_ipc_pool,
					  32 + spi_ipc_data_len(in), 0);
		if (!data->curr_rx_net_buf) {
			LOG_ERR("cannot allocate rx net buffer");
		}
		stat = k_mem_slab_alloc(&spi_ipc_msg_slab,
					(void **)&data->curr_rx_spi_msg, 0);
		if (stat < 0) {
			LOG_ERR("cannot allocate rx spi message");
			data->n_discard_subframes = spi_ipc_data_subframes(in);
			net_buf_unref(data->curr_rx_net_buf);
			return;
		}
		data->n_discard_subframes = 0;
		data->curr_rx_spi_msg->flags_error = in->hdr.flags_error;
		data->curr_rx_spi_msg->proto_code = in->hdr.proto_code;
		data->curr_rx_spi_msg->netbuf = data->curr_rx_net_buf;
		data->curr_rx_spi_msg->reply = NULL;
		data->curr_rx_spi_msg->data_len = spi_ipc_data_len(in);
	}
	if (data->n_discard_subframes) {
		/* Discarding input subframes (error or unsupported proto) */
		if (data->n_discard_subframes == data->input_index) {
			return;
		}
		data->n_discard_subframes = 0;
	}
	tot_msg_len = data->curr_rx_spi_msg->data_len + sizeof(union spi_thb);
	l =  tot_msg_len - net_buf_frags_len(data->curr_rx_net_buf);
	if (l > sizeof(union spi_thb))
		l = sizeof(union spi_thb);
	net_buf_add_mem(data->curr_rx_net_buf, in, l);
	if (net_buf_frags_len(data->curr_rx_net_buf) >= tot_msg_len) {
		/* Last subframe in a frame */
		struct spi_msg *request = _find_relevant_request(data, in);

		if (request) {
			/* Input message is a reply */
			request->reply = data->curr_rx_spi_msg;
			/* Remove related request from outstanding list */
			sys_dlist_remove(&request->list);
			k_sem_give(&request->completion_sem);
		} else
			data->curr_rx_proto->rx_cb(data->curr_rx_proto,
						   data,
						   data->curr_rx_net_buf,
						   data->curr_rx_proto_data);
		/*
		 * Rx cb should have referenced the net buffer if
		 * interested
		 */
		net_buf_unref(data->curr_rx_net_buf);
		data->curr_rx_net_buf = NULL;
		k_mem_slab_free(&spi_ipc_msg_slab,
				(void **)&data->curr_rx_spi_msg);
		data->curr_tx_spi_msg = NULL;
	}
}

static void check_dev(struct device *dev)
{
	const struct spi_ipc_config_data *cfg = dev->config->config_info;
	struct spi_ipc_data *data = dev->driver_data;
	int spi_done, result;

	k_sem_take(&data->sem, K_FOREVER);
	k_poll_signal_check(&data->spi_signal, &spi_done, &result);
	if (spi_done) {
		/* Spi slave transaction done */
		k_poll_signal_reset(&data->spi_signal);
		/* Reset transfer request */
		spi_ipc_reset_spi_xfer_request(data, cfg);
		/* Handle input buffer */
		spi_ipc_handle_input(data, data->spi_input_buf.buf);
	}
	if (events[cfg->minor + 1].state == K_POLL_STATE_FIFO_DATA_AVAILABLE) {
		/* Something available in output queue */
		if (!spi_done)
			spi_release(data->spi_dev, &data->spi_config);
		/* ANYTHING MORE TO DO HERE ? */
	}
	if (_check_mgmt_evt(data, &result)) {
		if (result) {
			LOG_ERR("ERROR MESSAGE FROM OTHER END (0x%04x)\n",
				(unsigned short)result);
			/* FIXME: ANYTHING MORE TO DO HERE ? */
		}
	}
	k_sem_give(&data->sem);
}

/* SPI IPC thread */
static void spi_ipc_main(void *arg)
{
	int i, stat;

	while (1) {
		for (i = 0; i < ARRAY_SIZE(active_devices); i++) {
			if (active_devices[i]) {
				setup_dev(active_devices[i]);
			}
		}

		stat = k_poll(events, ARRAY_SIZE(active_devices), K_FOREVER);

		if (stat == -EINTR) {
			/* FIXME: DO SOMETHING HERE ?? */
		}

		for (i = 0; i < ARRAY_SIZE(active_devices); i++) {
			if (active_devices[i]) {
				check_dev(active_devices[i]);
			}
		}
	}
}

static int spi_ipc_drv_open(struct device *dev, const struct spi_ipc_proto *p,
			    void *proto_data)
{
	const struct spi_ipc_config_data *cfg = dev->config->config_info;
	struct spi_ipc_data *data = dev->driver_data;
	int stat;

	k_sem_take(&data->sem, K_FOREVER);
	if (_find_proto(data, p->proto_id, NULL)) {
		k_sem_give(&data->sem);
		LOG_ERR("%s: protocol already registered\n", __func__);
		return -EBUSY;
	}
	stat = _new_proto(data, p, proto_data);
	if (stat < 0) {
		k_sem_give(&data->sem);
		LOG_ERR("%s failed (%d)\n", __func__, stat);
		return stat;
	}
	k_poll_event_init(&events[cfg->minor],
			  K_POLL_TYPE_IGNORE,
			  K_POLL_MODE_NOTIFY_ONLY, &data->spi_signal);
	k_poll_event_init(&events[cfg->minor + 1],
			  K_POLL_TYPE_IGNORE,
			  K_POLL_MODE_NOTIFY_ONLY, &data->fifo);
	init_mgmt_event(data, cfg->minor);
	k_sem_give(&data->sem);
	return 0;
}

static int spi_ipc_submit_buf(struct device *dev,
			      struct net_buf *outgoing,
			      struct net_buf **incoming)
{
	int stat, ret = 0;
	struct spi_ipc_data *data = dev->driver_data;
	union spi_thb header;
	struct spi_msg *msg;

	stat = k_mem_slab_alloc(&spi_ipc_msg_slab, (void **)&msg, 1000);
	if (stat < 0) {
		return stat;
	}
	net_buf_linearize(&header, sizeof(header), outgoing, 0, sizeof(header));
	msg->flags_error = (u16_t)(-ETIMEDOUT);
	msg->data_len = spi_ipc_data_len(&header);
	msg->reply = NULL;
	msg->netbuf = outgoing;
	msg->proto_code = header.hdr.proto_code;
	k_sem_init(&msg->completion_sem, 0, 1);
	/* net buf user data array contains pointer to relevant spi message */
	BUILD_ASSERT(sizeof(msg) <= CONFIG_NET_BUF_USER_DATA_SIZE);

	/* Enqueue buffer */
	net_buf_put(&data->fifo, outgoing);

	if (!spi_ipc_is_request(&header) || !incoming)
		/* Discard any incoming */
		goto end;

	/* Block until incoming is available */
	k_sem_take(&msg->completion_sem, 1000);

	/* Read incoming header */
	*incoming = msg->reply ? msg->reply->netbuf : NULL;
end:
	return ret;
}

/* Initializations common to all spi_ipc devices */
static int spi_ipc_common_init(struct device *dev)
{
	ARG_UNUSED(dev);

	LOG_DBG("%s invoked\n", __func__);

	/* Initialize spi messages memory slab */
	k_mem_slab_init(&spi_ipc_msg_slab, spi_msg_slab_buf,
			sizeof(spi_msg_slab_buf[0]), N_SPI_MSGS);
	/* Low level spi thread */
	k_thread_create(&spi_ipc_thread, spi_ipc_thread_stack,
			SPI_IPC_THREAD_STACK_SIZE,
			(k_thread_entry_t)spi_ipc_main,
			NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_SPI_IPC_THREAD_PRIO),
			0, K_NO_WAIT);
	return 0;
}
SYS_INIT(spi_ipc_common_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static int spi_ipc_init(struct device *dev)
{
	const struct spi_ipc_config_data *cfg = dev->config->config_info;
	struct spi_ipc_data *data = dev->driver_data;

	if (!cfg) {
		LOG_ERR("%s: no configuration info available\n", __func__);
		return -EINVAL;
	}
	if (!data) {
		LOG_ERR("%s: pointer to driver data is NULL\n", __func__);
		return -EINVAL;
	}

	memset(data, 0, sizeof(*data));

	data->spi_input_bs.buffers = &data->spi_input_buf;
	data->spi_input_bs.count = 1;
	data->spi_output_bs.buffers = &data->spi_output_buf;
	data->spi_output_bs.count = 1;

	k_sem_init(&data->sem, 1, 1);

	data->spi_config.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8) |
		SPI_LOCK_ON;
	data->spi_input_buf.len = 32;
	data->spi_output_buf.len = 32;
	data->spi_input_bs.buffers = &data->spi_input_buf;
	data->spi_input_bs.count = 1;
	data->spi_output_bs.buffers = &data->spi_output_buf;
	data->spi_output_bs.count = 1;

	sys_dlist_init(&data->outstanding);

	data->spi_dev = device_get_binding(cfg->spi_dev_label);
	if (!data->spi_dev) {
		LOG_ERR("%s: cannot find spi controller\n", __func__);
		return -ENODEV;
	}

	data->gpio_dev = device_get_binding(cfg->gpio_dev_label);
	if (!data->gpio_dev) {
		LOG_ERR("%s: cannot find poll request gpio controller\n",
			__func__);
		return -ENODEV;
	}
	gpio_pin_configure(data->gpio_dev, cfg->gpio_pin_number,
			   GPIO_DIR_OUT|cfg->gpio_pin_flags);

	/* Init outgoing requests queue */
	k_fifo_init(&data->fifo);

	/* Ignore events till device opened and active */
	k_poll_event_init(&events[cfg->minor], K_POLL_TYPE_IGNORE,
			  K_POLL_MODE_NOTIFY_ONLY, &data->spi_signal);
	k_poll_event_init(&events[cfg->minor + 1],
			  K_POLL_TYPE_IGNORE,
			  K_POLL_MODE_NOTIFY_ONLY, &data->fifo);

	if (init_mgmt(dev, data) < 0)
		LOG_ERR("spi-ipc (%s): low level mgmt init error\n",
			dev->config->name);

	LOG_DBG("spi-ipc driver initialized (%s)", dev->config->name);

	return 0;
}

static struct spi_ipc_driver_api spi_ipc_api_funcs = {
	.open = spi_ipc_drv_open,
	.submit_buf = spi_ipc_submit_buf,
};

#ifdef DT_SPI_IPC_0_LABEL

struct spi_ipc_data spi_ipc_data0;
struct spi_ipc_config_data spi_ipc_config0 = {
	.spi_dev_label = DT_SPI_IPC_0_LABEL,
	.gpio_dev_label = DT_SPI_IPC_0_POLL_REQUEST_GPIOS_CONTROLLER,
	.gpio_pin_number = DT_SPI_IPC_0_POLL_REQUEST_GPIOS_PIN,
	.gpio_pin_flags = DT_SPI_IPC_0_POLL_REQUEST_GPIOS_FLAGS,
	.minor = 0,
};

DEVICE_AND_API_INIT(spi_ipc, DT_SPI_IPC_0_LABEL, spi_ipc_init,
		    &spi_ipc_data0, &spi_ipc_config0,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &spi_ipc_api_funcs);

#endif
