/*
 * Copyright (c) 2018, Nordic Semiconductor ASA
 * Copyright (c) DogHunter LLC and the Linino organization 2019
 *
 * Patched by Davide Ciminaghi <davide@linino.org> 2019 to avoid ext/hal/ stuff
 * SPDX-License-Identifier: Apache-2.0
 */


#include <i2c.h>
#include <dt-bindings/i2c/i2c.h>

#define LOG_DOMAIN "i2c_nrfx_twim"
#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_nrfx_twim);

struct i2c_nrfx_twim_data {
	struct k_sem transfer_sync;
	struct k_sem completion_sync;
	volatile u32_t last_int;
};

/* Interrupt enable flags */
#define INT_STOPPED (1UL << 1)
#define INT_ERROR   (1UL << 9)
#define INT_SUSPENDED (1UL << 18)
#define INT_LASTRX (1UL << 23)
#define INT_LASTTX (1UL << 24)

/* Shorts */
#define LASTTX_STARTRX (1UL << 7)
#define LASTTX_SUSPEND (1UL << 8)
#define LASTTX_STOP    (1UL << 9)
#define LASTRX_STARTTX (1UL << 10)
#define LASTRX_STOP    (1UL << 12)

enum i2c_nrfx_twim_registers {
	NRFX_TWIM_TASKS_STARTRX = 0x000,
	NRFX_TWIM_TASKS_STARTTX = 0x008,
	NRFX_TWIM_TASKS_STOP = 0x014,
	NRFX_TWIM_TASKS_SUSPEND = 0x01c,
	NRFX_TWIM_TASKS_RESUME = 0x020,
	NRFX_TWIM_EVENTS_STOPPED = 0x104,
	NRFX_TWIM_EVENTS_ERROR = 0x124,
	NRFX_TWIM_EVENTS_SUSPENDED = 0x148,
	NRFX_TWIM_EVENTS_RXSTARTED = 0x14c,
	NRFX_TWIM_EVENTS_TXSTARTED = 0x150,
	NRFX_TWIM_EVENTS_LASTRX = 0x15c,
	NRFX_TWIM_EVENTS_LASTTX = 0x160,
	NRFX_TWIM_SHORTS = 0x200,
	NRFX_TWIM_INTEN = 0x300,
	NRFX_TWIM_INTENSET = 0x304,
	NRFX_TWIM_INTENCLR = 0x308,
	NRFX_TWIM_ERRORSRC = 0x4c4,
	NRFX_TWIM_ENABLE = 0x500,
	NRFX_TWIM_PSEL_SCL = 0x508,
	NRFX_TWIM_PSEL_SDA = 0x50c,
	NRFX_TWIM_FREQUENCY = 0x524,
	NRFX_TWIM_RXD_PTR = 0x534,
	NRFX_TWIM_RXD_MAXCNT = 0x538,
	NRFX_TWIM_RXD_AMOUNT = 0x53c,
	NRFX_TWIM_TXD_PTR = 0x544,
	NRFX_TWIM_TXD_MAXCNT = 0x548,
	NRFX_TWIM_TXD_AMOUNT = 0x54c,
	NRFX_TWIM_ADDRESS = 0x588,
};

struct i2c_nrfx_twim_config {
	uint32_t *registers;
	uint32_t frequency;
	int scl;
	int sda;
	int irq;
	int irq_prio;
};

static inline void
i2c_nrfx_twim_write(const struct i2c_nrfx_twim_config *config,
		    enum i2c_nrfx_twim_registers reg,
		    uint32_t v)
{
	config->registers[reg / sizeof(uint32_t)] = v;
}

static inline u32_t
i2c_nrfx_twim_read(const struct i2c_nrfx_twim_config *config,
		   enum i2c_nrfx_twim_registers reg)
{
	return config->registers[reg / sizeof(uint32_t)];
}

static inline struct i2c_nrfx_twim_data *get_dev_data(struct device *dev)
{
	return dev->driver_data;
}

static inline
const struct i2c_nrfx_twim_config *get_dev_config(struct device *dev)
{
	return dev->config->config_info;
}

static int i2c_nrfx_twim_transfer(struct device *dev, struct i2c_msg *msgs,
				  u8_t num_msgs, u16_t addr)
{
	int ret = 0, suspended = 0, i;
	const struct i2c_nrfx_twim_config *config = dev->config->config_info;
	u32_t inten;
	enum i2c_nrfx_twim_registers start_reg;

	k_sem_take(&(get_dev_data(dev)->transfer_sync), K_FOREVER);
	/* Enable device */
	i2c_nrfx_twim_write(config, NRFX_TWIM_ENABLE, 6);

	for (i = 0; i < num_msgs; i++) {
		if (I2C_MSG_ADDR_10_BITS & msgs[i].flags) {
			ret = -ENOTSUP;
			break;
		}
		inten = 0;
		start_reg = 0;
		if (msgs[i].flags & I2C_MSG_READ) {
			i2c_nrfx_twim_write(config, NRFX_TWIM_RXD_MAXCNT,
					    msgs[i].len);
			i2c_nrfx_twim_write(config, NRFX_TWIM_RXD_PTR,
					    (u32_t)msgs[i].buf);
			if (msgs[i].flags & I2C_MSG_STOP) {
				i2c_nrfx_twim_write(config, NRFX_TWIM_SHORTS,
						    LASTRX_STOP);
				inten = INT_STOPPED;
			}
			if (num_msgs > 1 && i < (num_msgs - 1)) {
				/* Unsupported at the moment */
				return -ENOTSUP;
			}
			start_reg = NRFX_TWIM_TASKS_STARTRX;
		} else {
			i2c_nrfx_twim_write(config, NRFX_TWIM_TXD_MAXCNT,
					    msgs[i].len);
			i2c_nrfx_twim_write(config, NRFX_TWIM_TXD_PTR,
					    (u32_t)msgs[i].buf);
			if (msgs[i].flags & I2C_MSG_STOP) {
				i2c_nrfx_twim_write(config, NRFX_TWIM_SHORTS,
						    LASTTX_STOP);
				inten = INT_STOPPED;
			}
			if (num_msgs > 1 && i < (num_msgs - 1)) {
				i2c_nrfx_twim_write(config, NRFX_TWIM_SHORTS,
						    LASTTX_SUSPEND);
				inten = INT_SUSPENDED;
			}
			start_reg = NRFX_TWIM_TASKS_STARTTX;
		}

		/* Set address */
		i2c_nrfx_twim_write(config, NRFX_TWIM_ADDRESS, addr);
		/* Enable interrupts */
		inten |= INT_ERROR;
		i2c_nrfx_twim_write(config, NRFX_TWIM_INTENSET, inten);
		/* Start transaction */
		i2c_nrfx_twim_write(config, start_reg, 1);
		/* Resume if needed */
		if (suspended) {
			suspended = 0;
			i2c_nrfx_twim_write(config, NRFX_TWIM_TASKS_RESUME, 1);
		}
		/* Wait for transaction end */
		k_sem_take(&(get_dev_data(dev)->completion_sync), K_FOREVER);
		/* An event has happened */
		if (get_dev_data(dev)->last_int & INT_ERROR) {
			/* Error occurred */
			ret = -EIO;
			break;
		}
		if (get_dev_data(dev)->last_int & INT_SUSPENDED) {
			/* Suspended */
			suspended = 1;
		}
	}
	/* Disable device */
	i2c_nrfx_twim_write(config, NRFX_TWIM_ENABLE, 0);
	k_sem_give(&(get_dev_data(dev)->transfer_sync));
	return ret;
}

static int i2c_nrfx_twim_set_speed(struct device *dev, u32_t freq)
{
	const struct i2c_nrfx_twim_config *config = dev->config->config_info;
	u32_t v;

	switch (freq) {
	case 100000:
		/* 100KHz */
		v = 0x01980000UL;
		break;
	case 400000:
		/* 400KHz */
		v = 0x06400000UL;
		break;
	case 250000:
		/* 250KHz: non-standard bit rate */
		v = 0x04000000UL;
		break;
	default:
		LOG_ERR("unsupported speed");
		return -EINVAL;
	}
	i2c_nrfx_twim_write(config, NRFX_TWIM_FREQUENCY, v);
	return 0;
}

static int i2c_nrfx_twim_configure(struct device *dev, u32_t dev_config)
{
	if (I2C_ADDR_10_BITS & dev_config) {
		return -EINVAL;
	}
	return i2c_nrfx_twim_set_speed(dev, I2C_SPEED_GET(dev_config));
}

static const struct i2c_driver_api i2c_nrfx_twim_driver_api = {
	.configure = i2c_nrfx_twim_configure,
	.transfer  = i2c_nrfx_twim_transfer,
};

static void twim_isr(void *arg)
{
	struct device *dev = arg;
	const struct i2c_nrfx_twim_config *config = dev->config->config_info;
	struct i2c_nrfx_twim_data *dev_data = get_dev_data(dev);

	dev_data->last_int = 0;
	/* Reset event(s) */
	if (i2c_nrfx_twim_read(config, NRFX_TWIM_EVENTS_SUSPENDED)) {
		i2c_nrfx_twim_write(config, NRFX_TWIM_EVENTS_SUSPENDED, 0);
		dev_data->last_int |= INT_STOPPED;
	}
	if (i2c_nrfx_twim_read(config, NRFX_TWIM_EVENTS_STOPPED)) {
		i2c_nrfx_twim_write(config, NRFX_TWIM_EVENTS_STOPPED, 0);
		dev_data->last_int |= INT_STOPPED;
	}
	if (i2c_nrfx_twim_read(config, NRFX_TWIM_EVENTS_ERROR)) {
		i2c_nrfx_twim_write(config, NRFX_TWIM_EVENTS_ERROR, 0);
		dev_data->last_int |= INT_ERROR;
	}
	if (i2c_nrfx_twim_read(config, NRFX_TWIM_EVENTS_LASTRX)) {
		i2c_nrfx_twim_write(config, NRFX_TWIM_EVENTS_LASTRX, 0);
		dev_data->last_int |= INT_LASTRX;
	}
	if (i2c_nrfx_twim_read(config, NRFX_TWIM_EVENTS_LASTTX)) {
		i2c_nrfx_twim_write(config, NRFX_TWIM_EVENTS_LASTTX, 0);
		dev_data->last_int |= INT_LASTTX;
	}
	k_sem_give(&dev_data->completion_sync);
}

static int i2c_nrfx_twim_init(struct device *dev)
{
	const struct i2c_nrfx_twim_config *config = dev->config->config_info;
	int ret;

	irq_connect_dynamic(config->irq, config->irq_prio, twim_isr, dev, 0);
	irq_enable(config->irq);

	/* Configure sda/scl */
	i2c_nrfx_twim_write(config, NRFX_TWIM_PSEL_SCL, config->scl);
	i2c_nrfx_twim_write(config, NRFX_TWIM_PSEL_SDA, config->sda);
	/* Configure speed */
	ret = i2c_nrfx_twim_set_speed(dev, config->frequency);
	if (ret < 0) {
		return -EINVAL;
	}
	return 0;
}

#define I2C_NRFX_TWIM_INVALID_FREQUENCY	 ((u32_t)-1)
#define I2C_NRFX_TWIM_FREQUENCY(bitrate)				       \
	(bitrate == I2C_BITRATE_STANDARD ? 100000			       \
	 : bitrate == 250000		  ? 250000			       \
	 : bitrate == I2C_BITRATE_FAST	  ? 400000			       \
					  : I2C_NRFX_TWIM_INVALID_FREQUENCY)

#define I2C_NRFX_TWIM_DEVICE(idx)					       \
	BUILD_ASSERT_MSG(						       \
		I2C_NRFX_TWIM_FREQUENCY(				       \
			DT_NORDIC_NRF_I2C_I2C_##idx##_CLOCK_FREQUENCY)	       \
		!= I2C_NRFX_TWIM_INVALID_FREQUENCY,			       \
		"Wrong I2C " #idx " frequency setting in dts");		       \
	const struct i2c_nrfx_twim_config twim_config_##idx = {		       \
		.registers = (u32_t *)					       \
		    DT_NORDIC_NRF_I2C_I2C_##idx##_BASE_ADDRESS,		       \
		.frequency = DT_NORDIC_NRF_I2C_I2C_##idx##_CLOCK_FREQUENCY,    \
		.scl	   = DT_NORDIC_NRF_I2C_I2C_##idx##_SCL_PIN,	       \
		.sda	   = DT_NORDIC_NRF_I2C_I2C_##idx##_SDA_PIN,	       \
		.irq = DT_NORDIC_NRF_I2C_I2C_##idx##_IRQ,		       \
		.irq_prio = DT_NORDIC_NRF_I2C_I2C_##idx##_IRQ_PRIORITY	       \
	};								       \
	static struct i2c_nrfx_twim_data twim_##idx##_data = {		       \
		.transfer_sync = Z_SEM_INITIALIZER(			       \
			twim_##idx##_data.transfer_sync, 1, 1),		       \
		.completion_sync = Z_SEM_INITIALIZER(			       \
			twim_##idx##_data.completion_sync, 0, 1)	       \
	};								       \
	static int i2c_nrfx_twim_init_##idx(struct device *dev)		       \
	{								       \
		return i2c_nrfx_twim_init(dev);				       \
	}								       \
	DEVICE_AND_API_INIT(twim_##idx,					       \
			    DT_NORDIC_NRF_I2C_I2C_##idx##_LABEL,	       \
			    i2c_nrfx_twim_init_##idx,			       \
			    &twim_##idx##_data,				       \
			    &twim_config_##idx,				       \
			    POST_KERNEL,				       \
			    CONFIG_I2C_INIT_PRIORITY,			       \
			    &i2c_nrfx_twim_driver_api)

#ifdef CONFIG_I2C_0_NRF_TWIM
I2C_NRFX_TWIM_DEVICE(0);
#endif

#ifdef CONFIG_I2C_1_NRF_TWIM
I2C_NRFX_TWIM_DEVICE(1);
#endif

#ifdef CONFIG_I2C_2_NRF_TWIM
I2C_NRFX_TWIM_DEVICE(2);
#endif

#ifdef CONFIG_I2C_3_NRF_TWIM
I2C_NRFX_TWIM_DEVICE(3);
#endif
