/* Copyright 2022 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief I2C driver for Infineon CAT1 MCU family.
 *
 * Note:
 * - I2C Slave functionality is not implemented in current
 *   version of I2C CAT1 driver.
 */

#define DT_DRV_COMPAT infineon_mtbhal_i2c

#include <zephyr/kernel.h>
#include <soc.h>
#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include "cyhal_i2c.h"

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_infineon_cat1);

/* Map I2C pins in Pinctrl array */
#define I2C_SCL_PIN   (CAT1_PIN_SIGNAL_SCB_I2C_SCL)     /* SCL pin */
#define I2C_SDA_PIN   (CAT1_PIN_SIGNAL_SCB_I2C_SDA)     /* SDA pin */

/* Default I2C interrupt priority */
#define I2C_CAT1_INTR_PRIORITY          (3U)

/* Default I2C semaphore timeout */
#define I2C_CAT1_SEM_TIMEOUT_MS         (5U)

#define I2C_CAT1_EVENTS_MASK            (CYHAL_I2C_MASTER_WR_CMPLT_EVENT | \
					 CYHAL_I2C_MASTER_RD_CMPLT_EVENT | \
					 CYHAL_I2C_MASTER_ERR_EVENT)

#if !defined(CONFIG_CAT1_I2C_DO_NOT_USE_ASYNC)
#define CONFIG_CAT1_I2C_ASYNC
#endif /* defined(CONFIG_CAT1_I2C_DO_NOT_USE_ASYNC) */

/* States for ASYNC operations */
#define CAT1_I2C_PENDING_NONE           (0U)
#define CAT1_I2C_PENDING_RX             (1U)
#define CAT1_I2C_PENDING_TX             (2U)
#define CAT1_I2C_PENDING_TX_RX          (3U)

/* I2C speed */
#define CAT1_I2C_SPEED_STANDARD_HZ      (100000UL)
#define CAT1_I2C_SPEED_FAST_HZ          (400000UL)
#define CAT1_I2C_SPEED_FAST_PLUS_HZ     (1000000UL)

/* Data structure */
struct i2c_cat1_data {
	cyhal_i2c_t i2c_master_obj;
	cyhal_i2c_cfg_t i2c_master_config;
	struct k_sem i2c_operation_sem;
	struct k_sem i2c_transfer_sem;
	uint32_t i2c_error_status;
	uint32_t i2c_async_pending;
};

/* Device config structure */
struct i2c_cat1_config {
	uint32_t i2c_master_frequency;
	const struct pinctrl_dev_config *pcfg;
};

#define DEV_DATA(dev) \
	((struct i2c_cat1_data *const)(dev)->data)

#define DEV_CFG(dev) \
	((struct i2c_cat1_config *const)(dev)->config)

/* Internal function to get cyhal_gpio_t from pinctrl array by defined index. */
extern cyhal_gpio_t cat1_pinctrl_get_cyhal_gpio(const struct pinctrl_dev_config *config,
						uint8_t signal_name);

#ifdef CONFIG_CAT1_I2C_ASYNC
static void master_event_handler(void *callback_arg, cyhal_i2c_event_t event)
{
	const struct device *dev = (const struct device *) callback_arg;
	struct i2c_cat1_data *data = DEV_DATA(dev);

	if ((CYHAL_I2C_MASTER_ERR_EVENT & event) != 0U) {
		/* In case of error abort transfer */
		(void)cyhal_i2c_abort_async(&data->i2c_master_obj);
		data->i2c_error_status = 1;
	}

	/* Release semaphore if operation complete.
	 * When we have pending TX, RX operations, the semaphore will be released
	 * after TX, RX complete.
	 */
	if (((data->i2c_async_pending == CAT1_I2C_PENDING_TX_RX) &&
	     ((CYHAL_I2C_MASTER_RD_CMPLT_EVENT & event) != 0U)) ||
	    (data->i2c_async_pending != CAT1_I2C_PENDING_TX_RX)) {

		/* Release semaphore (After I2C async transfer is complete) */
		k_sem_give(&data->i2c_transfer_sem);
	}
}
#endif


static int i2c_cat1_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_cat1_data *data = DEV_DATA(dev);
	cy_rslt_t rslt;
	int ret;

	if (dev_config != 0) {
		switch (I2C_SPEED_GET(dev_config)) {
		case I2C_SPEED_STANDARD:
			data->i2c_master_config.frequencyhal_hz = CAT1_I2C_SPEED_STANDARD_HZ;
			break;
		case I2C_SPEED_FAST:
			data->i2c_master_config.frequencyhal_hz = CAT1_I2C_SPEED_FAST_HZ;
			break;
		case I2C_SPEED_FAST_PLUS:
			data->i2c_master_config.frequencyhal_hz = CAT1_I2C_SPEED_FAST_PLUS_HZ;
			break;
		default:
			LOG_ERR("Unsupported speed");
			return -ERANGE;
		}

		/* Support for slave mode has not been implemented */
		if (!(dev_config & I2C_MODE_CONTROLLER)) {
			LOG_ERR("Slave mode is not supported");
			return -EIO;
		}

		/* This is deprecated and could be ignored in the future */
		if (dev_config & I2C_ADDR_10_BITS) {
			LOG_ERR("10-bit addressing mode is not supported");
			return -EIO;
		}
	}

	/* Acquire semaphore (block I2C operation for another thread) */
	ret = k_sem_take(&data->i2c_operation_sem, K_MSEC(I2C_CAT1_SEM_TIMEOUT_MS));
	if (ret) {
		return -EIO;
	}

	/* Configure the I2C resource to be master */
	rslt = cyhal_i2c_configure(&data->i2c_master_obj, &data->i2c_master_config);
	if (rslt != CY_RSLT_SUCCESS) {
		k_sem_give(&data->i2c_operation_sem);
		return -EIO;
	}

#ifdef CONFIG_CAT1_I2C_ASYNC
	/* Register an I2C event callback handler */
	cyhal_i2c_register_callback(&data->i2c_master_obj, master_event_handler,
				    (void *) dev);
#endif
	/* Release semaphore */
	k_sem_give(&data->i2c_operation_sem);
	return 0;
}

static int i2c_cat1_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_cat1_data *data = DEV_DATA(dev);
	uint32_t config;

	switch (data->i2c_master_config.frequencyhal_hz) {
	case CAT1_I2C_SPEED_STANDARD_HZ:
		config = I2C_SPEED_SET(I2C_SPEED_STANDARD);
		break;
	case CAT1_I2C_SPEED_FAST_HZ:
		config = I2C_SPEED_SET(I2C_SPEED_FAST);
		break;
	case CAT1_I2C_SPEED_FAST_PLUS_HZ:
		config = I2C_SPEED_SET(I2C_SPEED_FAST_PLUS);
		break;
	default:
		LOG_ERR("Unsupported speed");
		return -ERANGE;
	}

	/* Return current configuration */
	*dev_config = config | I2C_MODE_CONTROLLER;

	return 0;
}

static int i2c_infineon_cat1_msg_validate(struct i2c_msg *msg, uint8_t num_msgs)
{
	for (uint32_t i = 0u; i < num_msgs; i++) {
		if ((I2C_MSG_ADDR_10_BITS & msg[i].flags) || (msg[i].buf == NULL)) {
			return -EINVAL;
		}
	}
	return 0;
}

static int i2c_cat1_transfer(const struct device *dev, struct i2c_msg *msg,
			     uint8_t num_msgs, uint16_t addr)
{
	struct i2c_cat1_data *data = DEV_DATA(dev);
	cy_rslt_t rslt = CY_RSLT_SUCCESS;
	int ret;

	if (!num_msgs) {
		return 0;
	}

	/* Acquire semaphore (block I2C transfer for another thread) */
	ret = k_sem_take(&data->i2c_operation_sem, K_MSEC(I2C_CAT1_SEM_TIMEOUT_MS));
	if (ret) {
		return -EIO;
	}

	/* This function checks if msg.buf is not NULL and if
	 * slave address is not 10 bit.
	 */
	if (i2c_infineon_cat1_msg_validate(msg, num_msgs) != 0) {
		k_sem_give(&data->i2c_operation_sem);
		return -EINVAL;
	}

#ifdef CONFIG_CAT1_I2C_ASYNC
	struct i2c_msg *tx_msg;
	struct i2c_msg *rx_msg;

	data->i2c_error_status = 0;
	data->i2c_async_pending = CAT1_I2C_PENDING_NONE;

	/* Enable I2C Interrupt */
	cyhal_i2c_enable_event(&data->i2c_master_obj,
			       (cyhal_i2c_event_t)I2C_CAT1_EVENTS_MASK,
			       I2C_CAT1_INTR_PRIORITY, true);

	for (uint32_t i = 0u; i < num_msgs; i++) {
		tx_msg = NULL;
		rx_msg = NULL;

		if ((msg[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			tx_msg = &msg[i];
			data->i2c_async_pending = CAT1_I2C_PENDING_TX;
		}

		if ((msg[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			rx_msg = &msg[i];
			data->i2c_async_pending = CAT1_I2C_PENDING_TX;
		}

		if ((tx_msg != NULL) && ((i + 1U) < num_msgs) &&
		    ((msg[i + 1U].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ)) {
			rx_msg = &msg[i + 1U];
			i++;
			data->i2c_async_pending = CAT1_I2C_PENDING_TX_RX;
		}

		/* Initiate master write and read transfer
		 * using tx_buff and rx_buff respectively
		 */
		rslt = cyhal_i2c_master_transfer_async(&data->i2c_master_obj, addr,
						       (tx_msg == NULL) ? NULL : tx_msg->buf,
						       (tx_msg == NULL) ? 0u : tx_msg->len,
						       (rx_msg == NULL) ? NULL : rx_msg->buf,
						       (rx_msg == NULL) ? 0u : rx_msg->len);

		if (rslt != CY_RSLT_SUCCESS) {
			k_sem_give(&data->i2c_operation_sem);
			return -EIO;
		}

		/* Acquire semaphore (block I2C async transfer for another thread) */
		ret = k_sem_take(&data->i2c_transfer_sem, K_MSEC(I2C_CAT1_SEM_TIMEOUT_MS));
		if (ret) {
			k_sem_give(&data->i2c_operation_sem);
			return -EIO;
		}

		/* If i2c_error_status != 1 we have error during transfer async.
		 * i2c_error_status is handling in master_event_handler function.
		 */
		if (data->i2c_error_status != 0) {
			/* Release semaphore */
			k_sem_give(&data->i2c_operation_sem);
			return -EIO;
		}
	}

	/* Disable I2C Interrupt */
	cyhal_i2c_enable_event(&data->i2c_master_obj, (cyhal_i2c_event_t)
			       I2C_CAT1_EVENTS_MASK, I2C_CAT1_INTR_PRIORITY, false);
#else
	for (uint32_t i = 0u; i < num_msgs; i++) {
		bool stop_flag = ((msg[i].flags & I2C_MSG_STOP) != 0u) ? true : false;

		if ((msg[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			rslt = cyhal_i2c_master_write(&data->i2c_master_obj,
						      addr, msg[i].buf, msg[i].len, 0, stop_flag);
		}
		if ((msg[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			rslt = cyhal_i2c_master_read(&DEV_DATA(dev)->i2c_master_obj,
						     addr, msg[i].buf, msg[i].len, 0, stop_flag);
		}

		if (rslt != CY_RSLT_SUCCESS) {
			/* Release semaphore */
			k_sem_give(&data->i2c_operation_sem);
			return -EIO;
		}
	}
#endif
	/* Release semaphore (After I2C transfer is complete) */
	k_sem_give(&data->i2c_operation_sem);
	return 0;
}

static int i2c_cat1_init(const struct device *dev)
{
	struct i2c_cat1_data *data = DEV_DATA(dev);
	const struct i2c_cat1_config *config = DEV_CFG(dev);
	cy_rslt_t rslt;
	int ret;

	/* Configure semaphores */
	ret = k_sem_init(&data->i2c_transfer_sem, 0, 1);
	if (ret) {
		return ret;
	}

	ret = k_sem_init(&data->i2c_operation_sem, 1, 1);
	if (ret) {
		return ret;
	}

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Initialize the I2C peripheral */
	rslt = cyhal_i2c_init(
		&data->i2c_master_obj,
		cat1_pinctrl_get_cyhal_gpio(config->pcfg, I2C_SDA_PIN),
		cat1_pinctrl_get_cyhal_gpio(config->pcfg, I2C_SCL_PIN),
		NULL);
	if (rslt != CY_RSLT_SUCCESS) {
		return -EIO;
	}

	/* Store Master initial configuration */
	data->i2c_master_config.is_slave = false;
	data->i2c_master_config.address = 0;
	data->i2c_master_config.frequencyhal_hz = config->i2c_master_frequency;

	if (i2c_cat1_configure(dev, 0) != 0) {
		/* Free I2C resource */
		cyhal_i2c_free(&data->i2c_master_obj);
	}
	return 0;
}

/* I2C API structure */
static const struct i2c_driver_api i2c_cat1_driver_api = {
	.configure = i2c_cat1_configure,
	.transfer = i2c_cat1_transfer,
	.get_config = i2c_cat1_get_config
};

/* Macros for I2C instance declaration */
#define INFINEON_CAT1_I2C_INIT(n)						   \
	PINCTRL_DT_INST_DEFINE(n);						   \
										   \
	static struct i2c_cat1_data i2c_cat1_data##n = { 0 };			   \
										   \
	static const struct i2c_cat1_config i2c_cat1_cfg_##n = {		   \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			   \
		.i2c_master_frequency = DT_INST_PROP(n, clock_frequency)	   \
	};									   \
										   \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_cat1_init,				   \
				  NULL, &i2c_cat1_data##n,			   \
				  &i2c_cat1_cfg_##n,				   \
				  POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
				  &i2c_cat1_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INFINEON_CAT1_I2C_INIT)
