/*
 * Copyright (c) 2019 dog hunter LLC and the Linino community
 * Generic inter-processor communication via spi link
 *
 * Author Davide Ciminaghi <davide@linino.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SPI_IPC_H__
#define __SPI_IPC_H__

#include <stdint.h>


union spi_thb {
	/*
	 * An spi header (32 bytes)
	 * thb stands for transaction half buffer
	 */
	struct spi_header_thb {
		/* Magic number */
#define SPI_IPC_MAGIC 0xdeadbeef
		uint32_t magic;
		/*
		 * enum spi_msg_type bitwise or'ed with SPI_MSG_REPLY flag
		 */
		uint32_t proto_code;
		/* data length (from next "half transaction" on) */
		uint32_t trans_data_len;
		/* Error status */
		uint32_t error;
	} hdr;
	uint32_t data[8];
};

static inline uint16_t spi_ipc_proto(union spi_thb *thb)
{
	return thb->hdr.proto_code >> 16;
}

static inline void spi_ipc_set_proto(union spi_thb *thb, uint16_t proto)
{
	thb->hdr.proto_code = (thb->hdr.proto_code & 0xffffUL) |
		(proto << 16UL);
}

static inline uint16_t spi_ipc_code(union spi_thb *thb)
{
	return thb->hdr.proto_code & 0x7fff;
}

static inline void spi_ipc_set_code(union spi_thb *thb, uint16_t code)
{
	thb->hdr.proto_code = (thb->hdr.proto_code & ~0x7fffUL) | code;
}

static inline void spi_ipc_set_request_code(union spi_thb *thb, uint16_t code)
{
	thb->hdr.proto_code = (thb->hdr.proto_code & ~0x7fffUL) |
		(code | 0x8000);
}

static inline int spi_ipc_is_request(union spi_thb *thb)
{
	return thb->hdr.proto_code & 0x8000;
}

static inline int spi_ipc_is_reply(union spi_thb *thb)
{
	return !spi_ipc_is_request(thb);
}

static inline int spi_ipc_reply_matches(union spi_thb *thb, u32_t proto_code)
{
	return thb->hdr.proto_code == proto_code;
}

static inline uint16_t spi_ipc_transaction(union spi_thb *thb)
{
	return thb->hdr.trans_data_len >> 16;
}

static inline uint16_t spi_ipc_data_len(union spi_thb *thb)
{
	return thb->hdr.trans_data_len & 0xffff;
}

static inline void spi_ipc_set_data_len(union spi_thb *thb, uint16_t l)
{
	thb->hdr.trans_data_len &= ~0xffffUL;
	thb->hdr.trans_data_len |= l;
}

static inline int spi_ipc_data_subframes(union spi_thb *thb)
{
	return (spi_ipc_data_len(thb) >> 5) +
		(spi_ipc_data_len(thb) & 0x1f) ? 1 : 0;
}

static inline uint32_t spi_ipc_error(union spi_thb *thb)
{
	return thb->hdr.error;
}

static inline void spi_ipc_set_error(union spi_thb *thb, uint32_t error)
{
	thb->hdr.error = error;
}


/*
 * High level protocol descriptor
 */
struct spi_ipc_proto {
	const char *name;
#define HAVE_SPLIT_TRANSACTIONS (1 << 0)
	int flags;
	uint16_t proto_id;
	/* Rx notifications callback (unsolicited messaeges) */
	void (*rx_cb)(const struct spi_ipc_proto *, struct net_buf *in,
		      void *proto_data);
	/* Pointer to protocol private data */
	void *priv;
};

/* Driver API */
struct spi_ipc_driver_api {
	/*
	 * Does initialization for a given protocol.
	 */
	int (*open)(struct device *, const struct spi_ipc_proto *,
		    void *proto_data);
	/*
	 * Sends a message, blocks until relevant reply has been received, if
	 * applicable
	 */
	int (*submit_buf)(struct device *dev, struct net_buf *request,
			  struct net_buf **reply);
#ifdef CONFIG_SPI_IPC_ASYNC
	/* FIXME: NON BLOCKING VARIANT, RECEIVES A CALLBACK POINTER */
#endif
};

#endif /* __SPI_IPC_H__ */
