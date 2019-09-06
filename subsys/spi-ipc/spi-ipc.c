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
#include <spi-ipc/proto.h>

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
	/* Callback pointer and relevant arg */
	buf_reply_cb reply_cb;
	void *cb_arg;
	/* Expiry time (requests, K_FOREVER means forever) */
	s32_t expiry;
	/* Used to link to outstanding messages (requests) */
	sys_dnode_t list;
	/* Delayed work for new timeout management */
	struct k_delayed_work to_work;
	/* Signal used for request removal */
	struct k_poll_signal *removal_signal;
	/* Atomic flags */
#define REQ_RUNNING		0
#define REQ_DONE_OK		1
#define REQ_DONE_TIMEDOUT	2
#define REQ_DONE_REMOVING	3
	atomic_val_t status;
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
	struct k_poll_signal removal_signal;
	union spi_thb spi_input[2];
	union spi_thb spi_output[2];
	struct spi_buf spi_input_buf;
	struct spi_buf_set spi_input_bs;
	struct spi_buf spi_output_buf;
	struct spi_buf_set spi_output_bs;
	struct spi_buf_set *tx_bs;
	struct net_buf *curr_tx_net_buf;
	size_t curr_tx_net_buf_offset;
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
	int nopen;
};

static struct k_mem_slab spi_ipc_msg_slab;
static struct spi_msg __aligned(4) spi_msg_slab_buf[N_SPI_MSGS];
/* spi-ipc rx pool: get room for 2 eth frames */
#define N_BUFFERS_PER_ETH_FRAME \
    (NET_ETH_MAX_FRAME_SIZE/sizeof(union spi_thb) + \
     (NET_ETH_MAX_FRAME_SIZE%sizeof(union spi_thb) ? 1 : 0))

static void _net_buf_destroy(struct net_buf *nb);

NET_BUF_POOL_DEFINE(spi_ipc_pool, 2 * N_BUFFERS_PER_ETH_FRAME,
		    sizeof(union spi_thb),
		    sizeof(struct spi_msg *), _net_buf_destroy);
static struct k_thread spi_ipc_thread;

#ifdef CONFIG_SPI_IPC_MGMT
/* spi done signal, fifo ready, mgmt event */
#define EVTS_PER_DEV 4
#else
/* spi done signal, fifo ready */
#define EVTS_PER_DEV 3
#endif

struct k_poll_event events[EVTS_PER_DEV * N_SPI_IPC_DEVS];

#define SPI_IPC_THREAD_STACK_SIZE 512
K_THREAD_STACK_MEMBER(spi_ipc_thread_stack, SPI_IPC_THREAD_STACK_SIZE);

struct device *active_devices[N_SPI_IPC_DEVS];

#ifdef CONFIG_SPI_IPC_MGMT
static int init_mgmt(struct device *dev, struct spi_ipc_data *data)
{
	return init_spi_ipc_mgmt(dev, data);
}

static void init_mgmt_event(struct spi_ipc_data *data, int minor)
{
	k_poll_event_init(&events[minor + 3],
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
		k_poll_signal_reset(&data->mgmt_data.diag_signal);
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

static inline void _net_buf_ref(struct net_buf *nb)
{
	net_buf_ref(nb);
}

static void _net_buf_destroy(struct net_buf *nb)
{
	struct spi_msg *m = NULL;

	memcpy(&m, nb->user_data, sizeof(m));

	net_buf_destroy(nb);

	if (m) {
		k_mem_slab_free(&spi_ipc_msg_slab, (void **)&m);
	}
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

static const char _dummy_v[32] = "NOTHING_TO_SAY, REALLY          ";

static struct spi_msg *_find_matching_request(struct spi_ipc_data *data)
{
	struct spi_msg *m = data->curr_rx_spi_msg;
	struct spi_msg *r;

	K_DEBUG("%s entered, proto/code = 0x%08x, trans = %u\n", __func__,
		m->proto_code, m->trans);

	if (m->proto_code & SPI_IPC_REQUEST) {
		/* Not a reply at all */
		K_DEBUG("%s, message is not a reply", __func__);
		return NULL;
	}

	SYS_DLIST_FOR_EACH_CONTAINER(&data->outstanding, r, list) {
		K_DEBUG("%s: trying request %p, pr/code = 0x%08x, trans = %u\n",
			__func__, r, r->proto_code, r->trans);
		if ((r->proto_code & ~SPI_IPC_REQUEST) ==
		    (m->proto_code & ~SPI_IPC_REQUEST) &&
		    r->trans == m->trans) {
			K_DEBUG("%s: found request %p\n", __func__, r);
			return r;
		}
	}
	K_DEBUG("%s: no request found\n", __func__);
	return NULL;
}

static void setup_dev(struct device *dev)
{
	struct spi_ipc_data *data = dev->driver_data;
	const struct spi_ipc_config_data *cfg = dev->config->config_info;
	int stat, request_master_attention = 0;

	data->tx_bs = NULL;

	k_sem_take(&data->sem, K_FOREVER);
	if (!data->curr_tx_net_buf) {
		data->curr_tx_net_buf = net_buf_get(&data->fifo, 0);
		data->curr_tx_net_buf_offset = 0;
		K_DEBUG("%s: got buffer %p\n", __func__, data->curr_tx_net_buf);
	}
	K_DEBUG("%s %d, curr_tx_net_buf  = %p\n", __func__, __LINE__,
		data->curr_tx_net_buf);
	if (data->curr_tx_net_buf) {
		size_t l, _l;
		struct spi_msg *m;

		if (!data->curr_tx_spi_msg) {
			memcpy(&data->curr_tx_spi_msg,
			       data->curr_tx_net_buf->user_data,
			       sizeof(data->curr_tx_spi_msg));
		}
		m = data->curr_tx_spi_msg;
		_l = net_buf_frags_len(data->curr_tx_net_buf);
		l = min(_l, sizeof(union spi_thb));
		memset(data->spi_output[data->output_index].data, 0,
		       sizeof(data->spi_output[0]));
		net_buf_linearize(data->spi_output[data->output_index].data,
				  l, data->curr_tx_net_buf,
				  data->curr_tx_net_buf_offset, l);
		data->curr_tx_net_buf_offset += l;
		data->tx_bs = &data->spi_output_bs;
		data->spi_output_buf.buf =
		    &data->spi_output[data->output_index];
		data->output_index = (data->output_index + 1) & 0x1;
		if (data->curr_tx_net_buf_offset >= _l) {
			/* Last tx for this message All done */
			net_buf_unref(data->curr_tx_net_buf);
			data->curr_tx_net_buf = NULL;
			data->curr_tx_spi_msg = NULL;
		}
		request_master_attention = 1;
	} else {
		const char *v = _dummy_v;

		memcpy(data->spi_output[data->output_index].data, v,
		       sizeof(_dummy_v));
		data->tx_bs = &data->spi_output_bs;
		data->spi_output_buf.buf =
		    &data->spi_output[data->output_index];
		data->output_index = (data->output_index + 1) & 0x1;
	}

	/* Ping pong input buffer */
	data->spi_input_buf.buf = data->spi_input[data->input_index].data;
	data->spi_input_buf.len = sizeof(union spi_thb);
	data->input_index = (data->input_index + 1) & 0x1;
	k_sem_give(&data->sem);

	K_DEBUG("Transceiving: tx_bs = %p, buffers[0] = %p, buf = %p\n",
		data->tx_bs, &data->tx_bs->buffers[0],
		data->tx_bs->buffers[0].buf);
	stat = spi_transceive_async(data->spi_dev, &data->spi_config,
				    data->tx_bs, &data->spi_input_bs,
				    &data->spi_signal);
	K_DEBUG("spi_transceive_async() returned %d\n", stat);
	if (request_master_attention) {
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

static int remove_outstanding_request(struct spi_msg **_request)
{
	struct spi_msg *request = *_request;
	int ret = 0;

	sys_dlist_remove(&request->list);
	net_buf_unref(request->netbuf);
	return ret;
}

static void spi_ipc_handle_input(struct spi_ipc_data *data, u8_t *buf)
{
	union spi_thb *in = (union spi_thb *)buf;
	struct spi_msg *request;
	int stat;
	size_t l, tot_msg_len;

	if (data->n_discard_subframes) {
		K_DEBUG("%s: discarding frame, counter = %d\n",
			__func__, data->n_discard_subframes);
		/* Discarding input subframes (error or unsupported proto) */
		data->n_discard_subframes--;
		return;
	}
	if (!data->curr_rx_net_buf) {
		struct net_buf *nb;
		struct spi_msg *m;

		if (in->hdr.magic != SPI_IPC_MAGIC) {
			K_DEBUG("%s: no spi magic (0x%08x)\n", __func__,
				in->hdr.magic);
			return;
		}
		/* First sub frame received */
		data->curr_rx_proto = _find_proto(data, spi_ipc_proto(in),
						  &data->curr_rx_proto_data);
		if (!data->curr_rx_proto) {
			K_DEBUG("Unsupported protocol 0x%04x, discarding frame\n", spi_ipc_proto(in));
			data->n_discard_subframes = spi_ipc_data_subframes(in);
			return;
		}
		K_DEBUG("msg started, rx proto = %s\n",
			data->curr_rx_proto->name);
		nb = net_buf_alloc_len(&spi_ipc_pool,
				       32 + spi_ipc_data_len(in), 0);
		data->curr_rx_net_buf = nb;
		memset(nb->user_data, 0, sizeof(m));
		if (!nb) {
			printk("cannot allocate rx net buffer\n");
			return;
		}
		stat = k_mem_slab_alloc(&spi_ipc_msg_slab, (void **)&m, 0);
		if (stat < 0) {
			printk("cannot allocate rx spi message\n");
			data->n_discard_subframes = spi_ipc_data_subframes(in);
			net_buf_unref(data->curr_rx_net_buf);
			return;
		}
		data->n_discard_subframes = 0;
		memcpy(nb->user_data, &m, sizeof(m));
		data->curr_rx_spi_msg = m;
		m->flags_error = in->hdr.flags_error;
		m->proto_code = in->hdr.proto_code;
		m->netbuf = data->curr_rx_net_buf;
		m->data_len = spi_ipc_data_len(in);
		m->trans = spi_ipc_transaction(in);
		K_DEBUG("transaction = %u, data_len = %u\n", m->trans,
			m->data_len);
	}
	tot_msg_len = data->curr_rx_spi_msg->data_len + sizeof(union spi_thb);
	l =  tot_msg_len - net_buf_frags_len(data->curr_rx_net_buf);
	if (l > sizeof(union spi_thb))
		l = sizeof(union spi_thb);
	K_DEBUG("tot msg len = %d, adding %d\n", tot_msg_len, l);
	K_DEBUG("in = 0x%08x\n", *(uint32_t *)in);
	net_buf_add_mem(data->curr_rx_net_buf, in, l);
	K_DEBUG("received length = %d\n",
		net_buf_frags_len(data->curr_rx_net_buf));
	if (net_buf_frags_len(data->curr_rx_net_buf) >= tot_msg_len) {
		request = _find_matching_request(data);
		K_DEBUG("%s: frame received, matching request = %p\n", __func__,
		       request);
		if (request) {
			int last_reply = data->curr_rx_spi_msg->flags_error &
				LAST_REPLY;
			int next_state = last_reply ? REQ_DONE_OK : REQ_RUNNING;

			/* Input message is a reply */
			K_DEBUG("frame is a reply, invoking reply cb %p\n",
			       request->reply_cb);
			/*
			 * If request is not running, a timeout has occurred
			 * Removal should already have been scheduled
			 */
			if (!atomic_cas(&request->status, REQ_RUNNING,
					next_state))
				return;
			/* Request is running, invoke its callback */
			if (request->reply_cb)
				request->reply_cb(data->curr_rx_net_buf,
						  request->cb_arg);
			if (last_reply) {
				/*
				 * Request complete: cancel timeout
				 * and schedule request for removal
				 */
				K_DEBUG("last reply for request %p\n",
					request);
				k_delayed_work_cancel(&request->to_work);
				k_poll_signal_raise(request->removal_signal,
						    (int)request);
			}
		} else {
			K_DEBUG("request/notification received\n");
			data->curr_rx_proto->rx_cb(data->curr_rx_proto,
						   data,
						   data->curr_rx_net_buf,
						   data->curr_rx_proto_data);
		}
		/*
		 * Rx cb should have referenced the net buffer if
		 * interested
		 */
		net_buf_unref(data->curr_rx_net_buf);
		data->curr_rx_net_buf = NULL;
		data->curr_rx_spi_msg = NULL;
	}
}

static int check_dev(struct device *dev)
{
	const struct spi_ipc_config_data *cfg = dev->config->config_info;
	struct spi_ipc_data *data = dev->driver_data;
	int spi_done, result;
	int do_remove;
	union {
		int dummy;
		struct spi_msg *request;
	} to_result;
	int ret = 0;

	k_sem_take(&data->sem, K_FOREVER);
	k_poll_signal_check(&data->spi_signal, &spi_done, &result);
	K_DEBUG("k_poll_signal_check(): spi_done = %u, result = %d\n",
		spi_done, result);
	if (spi_done) {
		/* Spi slave transaction done */
		k_poll_signal_reset(&data->spi_signal);
		/* Reset transfer request */
		spi_ipc_reset_spi_xfer_request(data, cfg);
		/* Handle input buffer */
		K_DEBUG("%s %d, input buf = %p\n", __func__, __LINE__,
			data->spi_input_buf.buf);
		spi_ipc_handle_input(data, data->spi_input_buf.buf);
		ret = 1;
	}
	k_poll_signal_check(&data->removal_signal, &do_remove,
			    &to_result.dummy);
	K_DEBUG("k_poll_signal_check(): removal = %u, result = %d\n",
		do_remove, result);
	if (do_remove) {
		/* Timeout on request */
		struct spi_msg *req = to_result.request;

		K_DEBUG("Removing request %p\n", req);
		k_poll_signal_reset(&data->removal_signal);
		if (req->reply_cb &&
		    atomic_cas(&req->status,
			       REQ_DONE_TIMEDOUT, REQ_DONE_REMOVING))
			req->reply_cb(NULL, req->cb_arg);
		remove_outstanding_request(&req);
	}
	if (events[cfg->minor + 1].state == K_POLL_STATE_FIFO_DATA_AVAILABLE) {
		/* Something available in output queue */
		if (!spi_done)
			spi_release(data->spi_dev, &data->spi_config);
		/* ANYTHING MORE TO DO HERE ? */
		ret = 1;
	}
	if (_check_mgmt_evt(data, &result)) {
		if (result) {
			printk("ERROR MESSAGE FROM OTHER END (0x%04x)\n",
			       (unsigned short)result);
			/* FIXME: ANYTHING MORE TO DO HERE ? */
		}
	}
	k_sem_give(&data->sem);
	return ret;
}

/* SPI IPC thread */
static void spi_ipc_main(void *arg)
{
	int i, stat, ndevs, do_setup = 1;

	while (1) {
		for (i = 0, ndevs = 0; i < ARRAY_SIZE(active_devices); i++) {
			struct device *d = active_devices[i];
			
			if (d) {
				if (do_setup)
					setup_dev(d);
				ndevs++;
			}
		}
		if (!ndevs) {
			K_DEBUG("NO ACTIVE DEVICES\n");
			k_sleep(K_MSEC(1000));
			continue;
		}

		K_DEBUG("%s: entering K_POLL\n", __func__);
		stat = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
		K_DEBUG("%s: k_poll() returned %d\n", __func__, stat);

		if (stat == -EINTR) {
			/* FIXME: DO SOMETHING HERE ?? */
			continue;
		}

		if (stat == -EAGAIN) {
			/* Request timeout, should never happen */
			continue;
		}
		for (i = 0; i < ARRAY_SIZE(active_devices); i++) {
			if (active_devices[i]) {
				K_DEBUG("Invoking check_dev() (%p)\n",
					active_devices[i]);
				do_setup = check_dev(active_devices[i]);
				K_DEBUG("check_dev() done\n");
			}
		}
	}
}

static int spi_ipc_drv_open(struct device *dev, const struct spi_ipc_proto *p,
			    void *proto_data)
{
	const struct spi_ipc_config_data *cfg;
	struct spi_ipc_data *data;
	int stat;

	cfg = dev->config->config_info;
	data = dev->driver_data;
	k_sem_take(&data->sem, K_FOREVER);
	if (_find_proto(data, p->proto_id, NULL)) {
		k_sem_give(&data->sem);
		printk("%s: protocol already registered\n", __func__);
		return -EBUSY;
	}
	stat = _new_proto(data, p, proto_data);
	if (stat < 0) {
		k_sem_give(&data->sem);
		printk("%s failed (%d)\n", __func__, stat);
		return stat;
	}
	if (!data->nopen++) {
		k_poll_event_init(&events[cfg->minor],
				  K_POLL_TYPE_SIGNAL,
				  K_POLL_MODE_NOTIFY_ONLY, &data->spi_signal);
		k_poll_event_init(&events[cfg->minor + 1],
				  K_POLL_TYPE_FIFO_DATA_AVAILABLE,
				  K_POLL_MODE_NOTIFY_ONLY, &data->fifo);
		k_poll_event_init(&events[cfg->minor + 2],
				  K_POLL_TYPE_SIGNAL,
				  K_POLL_MODE_NOTIFY_ONLY,
				  &data->removal_signal);
		init_mgmt_event(data, cfg->minor);
	}
	k_sem_give(&data->sem);
	return 0;
}

static void request_timeout(struct k_work *work)
{
	struct spi_msg *request = CONTAINER_OF(work, struct spi_msg, to_work);

	if (!(atomic_cas(&request->status, REQ_RUNNING, REQ_DONE_TIMEDOUT)))
		return;
	/* Otherwise just remove it */
	K_DEBUG("%s: raising removal signal\n", __func__);
	k_poll_signal_raise(request->removal_signal, (int)request);
}

static int spi_ipc_submit_buf(struct device *dev,
			      struct net_buf *outgoing,
			      buf_reply_cb reply_cb, void *cb_arg, s32_t expiry)
{
	int stat, ret = 0;
	struct spi_ipc_data *data = dev->driver_data;
	union spi_thb header;
	struct spi_msg *msg;

	if (!expiry && reply_cb) {
		printk("%s: warning, expiry is 0, invalid\n", __func__);
		return -EINVAL;
	}

	stat = k_mem_slab_alloc(&spi_ipc_msg_slab, (void **)&msg, 1000);
	if (stat < 0) {
		return stat;
	}
	net_buf_linearize(&header, sizeof(header), outgoing, 0, sizeof(header));
	msg->flags_error = (u16_t)(-ETIMEDOUT);
	msg->data_len = spi_ipc_data_len(&header);
	msg->trans = spi_ipc_transaction(&header);
	msg->netbuf = outgoing;
	msg->proto_code = header.hdr.proto_code;
	msg->reply_cb = reply_cb;
	msg->cb_arg = cb_arg;
	msg->expiry = expiry;
	msg->removal_signal = &data->removal_signal;
	atomic_set(&msg->status, REQ_RUNNING);
	K_DEBUG("new msg (0x%08x) = %p\n", msg->proto_code, msg);

	if (msg->proto_code & SPI_IPC_REQUEST) {
		K_DEBUG("%s %d, appending request %p (0x%08x)\n",
		       __func__, __LINE__, msg, msg->proto_code);
		K_DEBUG("message %p is a request, appending to list of outstanding requests\n", msg);
		k_sem_take(&data->sem, K_FOREVER);
		/*
		 * If message is a request, append it
		 * to the list of outstanding requests
		 */
		sys_dlist_append(&data->outstanding,
				 &msg->list);
		k_sem_give(&data->sem);
		/*
		 * Buffer will be unreferenced after tx, keep
		 * a reference to it
		 */
		K_DEBUG("OUTGOING NETBUF %p, request %p\n", outgoing, msg);
		_net_buf_ref(outgoing);

		if (expiry > 0) {
			/* Setup timeout, 0 timeout not accepted */
			k_delayed_work_init(&msg->to_work, request_timeout);
			k_delayed_work_submit(&msg->to_work, expiry);
		}
	}

	/* net buf user data array contains pointer to relevant spi message */
	BUILD_ASSERT(sizeof(msg) <= CONFIG_NET_BUF_USER_DATA_SIZE);

	memcpy(outgoing->user_data, &msg, sizeof(msg));

	/* Enqueue buffer */
	net_buf_put(&data->fifo, outgoing);

	return ret;
}

/* Initializations common to all spi_ipc devices */
static int spi_ipc_common_init(struct device *dev)
{
	ARG_UNUSED(dev);

	K_DEBUG("%s invoked\n", __func__);

#ifdef CONFIG_SOC_SERIES_NRF52X
	*(volatile uint32_t *)0x4006EC00 = 0x00009375;
	*(volatile uint32_t *)0x4006ED08 = 0x00000003;
	*(volatile uint32_t *)0x4006EC00 = 0x00009375;
#endif

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
		printk("%s: no configuration info available\n", __func__);
		return -EINVAL;
	}
	if (!data) {
		printk("%s: pointer to driver data is NULL\n", __func__);
		return -EINVAL;
	}

	memset(data, 0, sizeof(*data));

	data->spi_input_bs.buffers = &data->spi_input_buf;
	data->spi_input_bs.count = 1;
	data->spi_output_bs.buffers = &data->spi_output_buf;
	data->spi_output_bs.count = 1;

	k_sem_init(&data->sem, 1, 1);
	k_poll_signal_init(&data->spi_signal);
	k_poll_signal_init(&data->removal_signal);

	data->spi_config.frequency = 1000000;
	data->spi_config.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8);
	data->spi_input_buf.len = 32;
	data->spi_output_buf.len = 32;
	data->spi_input_bs.buffers = &data->spi_input_buf;
	data->spi_input_bs.count = 1;
	data->spi_output_bs.buffers = &data->spi_output_buf;
	data->spi_output_bs.count = 1;

	sys_dlist_init(&data->outstanding);

	data->spi_dev = device_get_binding(cfg->spi_dev_label);
	if (!data->spi_dev) {
		printk("%s: cannot find spi controller\n", __func__);
		return -ENODEV;
	}

	data->gpio_dev = device_get_binding(cfg->gpio_dev_label);
	if (!data->gpio_dev) {
		printk("%s: cannot find poll request gpio controller\n",
		       __func__);
		return -ENODEV;
	}
	gpio_pin_configure(data->gpio_dev, cfg->gpio_pin_number,
			   GPIO_DIR_OUT|cfg->gpio_pin_flags);
	data->spi_config.cs = NULL;

	/* Init outgoing requests queue */
	k_fifo_init(&data->fifo);

	/* Ignore events till device opened and active */
	k_poll_event_init(&events[cfg->minor], K_POLL_TYPE_IGNORE,
			  K_POLL_MODE_NOTIFY_ONLY, &data->spi_signal);
	k_poll_event_init(&events[cfg->minor + 1],
			  K_POLL_TYPE_IGNORE,
			  K_POLL_MODE_NOTIFY_ONLY, &data->fifo);
	k_poll_event_init(&events[cfg->minor + 2], K_POLL_TYPE_IGNORE,
			  K_POLL_MODE_NOTIFY_ONLY, &data->removal_signal);

	active_devices[cfg->minor] = dev;

	if (init_mgmt(dev, data) < 0)
		printk("spi-ipc (%s): low level mgmt init error\n",
		       dev->config->name);

	K_DEBUG("spi-ipc driver initialized (%s)", dev->config->name);

	return 0;
}

static struct spi_ipc_driver_api spi_ipc_api_funcs = {
	.open = spi_ipc_drv_open,
	.submit_buf = spi_ipc_submit_buf,
};

/* Slave trans # must have bit 15 set */
static atomic_t _transaction_num = ATOMIC_INIT(0UL);

u16_t spi_ipc_new_transaction(void)
{
	return (atomic_add(&_transaction_num, 1) & 0xffffUL) | BIT(15);
}

#ifdef DT_SPI_IPC_0_LABEL

struct spi_ipc_data spi_ipc_data0;
struct spi_ipc_config_data spi_ipc_config0 = {
	.spi_dev_label = DT_NORDIC_NRF_SPIS_0_LABEL,
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
