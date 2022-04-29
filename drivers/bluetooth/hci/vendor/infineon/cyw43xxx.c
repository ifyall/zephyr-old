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
 * @brief CYW43xxx HCI extension driver.
 */

#include <errno.h>
#include <stddef.h>

#include <zephyr.h>
#include <device.h>
#include <bluetooth/bluetooth.h>
#include <drivers/bluetooth/hci_driver.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_HCI_DRIVER)
#define LOG_MODULE_NAME cyw43xxx_driver
#include "common/log.h"
#include <stdint.h>

#define HCI_UART_NODE                     DT_COMPAT_GET_ANY_STATUS_OKAY(infineon_cyw43xxx_bt_hci)

/* BT settling time after power on */
#define BT_POWER_ON_SETTLING_TIME         (500u)

/* Stabilization delay after FW loading */
#define BT_STABILIZATION_DELAY            (250)

/* HCI Command packet from Host to Controller */
#define HCI_COMMAND_PACKET                (0x01)

/* Length of UPDATE BAUD RATE command */
#define HCI_VSC_UPDATE_BAUD_RATE_LENGTH   (6u)

/* Default BAUDRATE */
#define HCI_UART_DEFAULT_BAUDRATE         (115200)

/* Externs for CY43xxx controller FW */
extern const uint8_t brcm_patchram_buf[];
extern const int brcm_patch_ram_length;

enum {
	BT_HCI_VND_OP_DOWNLOAD_MINIDRIVER       = 0xFC2E,
	BT_HCI_VND_OP_WRITE_RAM                 = 0xFC4C,
	BT_HCI_VND_OP_LAUNCH_RAM                = 0xFC4E,
	BT_HCI_VND_OP_UPDATE_BAUDRATE           = 0xFC18,
};

/*  bt_h4_vnd_setup function.
 * This function executes vendor-specific commands sequence to
 * initialize BT Controller before BT Host executes Reset sequence.
 * bt_h4_vnd_setup function must be implemented in vendor-specific HCI
 * extansion module if CONFIG_BT_HCI_SETUP is enabled.
 */
int bt_h4_vnd_setup(const struct device *dev);

static int bt_hci_uart_set_baudrate(const struct device *bt_uart_dev, uint32_t baudrate)
{
	struct uart_config uart_cfg;
	int err = 0;

	/* Get current UART configuration */
	err = uart_config_get(bt_uart_dev, &uart_cfg);
	if (err) {
		return err;
	}

	if (uart_cfg.baudrate != baudrate) {
		/* Re-configure UART */
		uart_cfg.baudrate = baudrate;
		err = uart_configure(bt_uart_dev, &uart_cfg);
		if (err) {
			return err;
		}

		/* Revert Interrupt options */
		uart_irq_rx_enable(bt_uart_dev);
	}
	return 0;
}

static int bt_update_controller_baudrate(const struct device *bt_uart_dev, uint32_t baudrate)
{
	/*
	 *  NOTE from datasheet for update baudrate:
	 *  - To speed up application downloading, the MCU host commands the CYWxxx device
	 *    to communicate at a new, higher rate by issuing the following Vendor Specific
	 *    UPDATE_BAUDRATE command:
	 *      01 18 FC 06 00 00 xx xx xx xx
	 *    In the above command, the xx xx xx xx bytes specify the 32-bit little-endian
	 *    value of the new rate in bits per second. For example,
	 *    115200 is represented as 00 C2 01 00.
	 *  The following response to the UPDATE_BAUDRATE command is expected within 100 ms:
	 *  04 0E 04 01 18 FC 00
	 *  - The host switches to the new baud rate after receiving the response at the old
	 *  baud rate.
	 */
	struct net_buf *buf;
	struct net_buf *rsp;
	int err;
	uint8_t hci_data[HCI_VSC_UPDATE_BAUD_RATE_LENGTH];

	/* Baudrate is loaded LittleEndian */
	hci_data[0] = 0;
	hci_data[1] = 0;
	hci_data[2] = (uint8_t)(baudrate & 0xFFUL);
	hci_data[3] = (uint8_t)((baudrate >> 8) & 0xFFUL);
	hci_data[4] = (uint8_t)((baudrate >> 16) & 0xFFUL);
	hci_data[5] = (uint8_t)((baudrate >> 24) & 0xFFUL);

	/* Allocate buffer for update uart baudrate command.
	 * It will be BT_HCI_OP_RESET with extra parameters.
	 */
	buf = bt_hci_cmd_create(BT_HCI_VND_OP_UPDATE_BAUDRATE,
				HCI_VSC_UPDATE_BAUD_RATE_LENGTH);
	if (buf == NULL) {
		printk("Unable to allocate command buffer\n");
		return -1;
	}

	/* Add data part of packet */
	void *data_buf = net_buf_add(buf, HCI_VSC_UPDATE_BAUD_RATE_LENGTH);

	/* Copy data */
	(void)memcpy(data_buf, (void *)hci_data, HCI_VSC_UPDATE_BAUD_RATE_LENGTH);

	/* Send update uart baudrate command. */
	err = bt_hci_cmd_send_sync(BT_HCI_VND_OP_UPDATE_BAUDRATE, buf, &rsp);
	if (err) {
		return err;
	}

	/* Re-configure Uart baudrate */
	err = bt_hci_uart_set_baudrate(bt_uart_dev, baudrate);
	if (err) {
		return err;
	}

	return 0;
}

static int bt_firmware_download(const uint8_t *firmware_image, uint32_t size)
{
	uint8_t *data = (uint8_t *) firmware_image;
	volatile uint32_t remaining_length = size;
	struct net_buf *buf;
	struct net_buf *rsp;
	int err;

	BT_DBG("Executing Fw downloading for CYW43xx device");

	/* Send hci_download_minidriver command */
	err = bt_hci_cmd_send_sync(BT_HCI_VND_OP_DOWNLOAD_MINIDRIVER, NULL, &rsp);
	if (err) {
		net_buf_unref(rsp);
		return err;
	}
	net_buf_unref(rsp);

	/* The firmware image (.hcd format) contains a collection of hci_write_ram
	 * command + a block of the image, followed by a hci_write_ram image at the end.
	 * Parse and send each individual command and wait for the response. This is to
	 * ensure the integrity of the firmware image sent to the bluetooth chip.
	 */
	while (remaining_length) {
		size_t data_length = data[2]; /* data length from firmware image block */
		uint16_t op_code = *(uint16_t *) data;

		/* Allocate buffer for hci_write_ram/hci_launch_ram command. */
		buf = bt_hci_cmd_create(op_code, data_length);
		if (buf == NULL) {
			printk("Unable to allocate command buffer\n");
			return err;
		}

		/* Add data part of packet */
		void *data_buf = net_buf_add(buf, data_length);

		(void)memcpy(data_buf, (void *)&data[3], data_length);

		/* Send hci_write_ram command. */
		err = bt_hci_cmd_send_sync(op_code, buf, &rsp);
		if (err) {
			return err;
		}

		switch (op_code) {
		case BT_HCI_VND_OP_WRITE_RAM:
			/* Update remaining length and data pointer:
			 * content of data length + 2 bytes of opcode and 1 byte of data length.
			 */
			data += data_length + 3;
			remaining_length -= data_length + 3;
			break;

		case BT_HCI_VND_OP_LAUNCH_RAM:
			remaining_length = 0;
			break;

		default:
			return -1;
		}
		net_buf_unref(rsp);
		net_buf_unref(buf);
	}

	BT_DBG("Fw downloading complete");
	return 0;
}

int bt_h4_vnd_setup(const struct device *dev)
{
	uint32_t baudrate_for_fw_download = DT_PROP(HCI_UART_NODE, baudrate_for_fw_download);
	uint32_t baudrate_for_operation = DT_PROP(DT_CHOSEN(zephyr_bt_uart), current_speed);
	struct gpio_dt_spec bt_reg_on = GPIO_DT_SPEC_GET(HCI_UART_NODE, bt_reg_on_gpios);

	struct net_buf *rsp;
	int err;

	/* Check BT Uart instance */
	if (!device_is_ready(dev)) {
		return -EINVAL;
	}

	/* Check BT REG_ON gpio instance */
	if (!device_is_ready(bt_reg_on.port)) {
		printk("Error: failed to configure bt_reg_on %s pin %d\n",
		       bt_reg_on.port->name, bt_reg_on.pin);
		return -EIO;
	}

	/* Configure bt_reg_on as output  */
	err = gpio_pin_configure_dt(&bt_reg_on, GPIO_OUTPUT | GPIO_PULL_UP);
	if (err) {
		printk("Error %d: failed to configure bt_reg_on %s pin %d\n",
		       err, bt_reg_on.port->name, bt_reg_on.pin);
		return err;
	}
	err = gpio_pin_set_dt(&bt_reg_on, 1);
	if (err) {
		return err;
	}

	/* BT settling time after power on */
	(void)k_msleep(BT_POWER_ON_SETTLING_TIME);

	/* Set default baudrate */
	err = bt_hci_uart_set_baudrate(dev, HCI_UART_DEFAULT_BAUDRATE);
	if (err) {
		return err;
	}

	/* Send HCI_RESET */
	err = bt_hci_cmd_send_sync(BT_HCI_OP_RESET, NULL, &rsp);
	if (err) {
		return err;
	}
	net_buf_unref(rsp);

	/* Re-configure baudrate for BT Controller */
	if (baudrate_for_fw_download != HCI_UART_DEFAULT_BAUDRATE) {
		err = bt_update_controller_baudrate(dev, baudrate_for_fw_download);
		if (err) {
			return err;
		}
	}

	/* BT firmware download */
	err = bt_firmware_download(brcm_patchram_buf, (uint32_t) brcm_patch_ram_length);
	if (err) {
		return err;
	}

	/* Stabilization delay */
	(void)k_msleep(BT_STABILIZATION_DELAY);

	/* When FW launched, HCI UART baudrate should be configured to default */
	if (baudrate_for_fw_download != HCI_UART_DEFAULT_BAUDRATE) {
		err = bt_hci_uart_set_baudrate(dev, HCI_UART_DEFAULT_BAUDRATE);
		if (err) {
			return err;
		}
	}

	/* Send HCI_RESET */
	err = bt_hci_cmd_send_sync(BT_HCI_OP_RESET, NULL, &rsp);
	if (err) {
		net_buf_unref(rsp);
		return err;
	}
	net_buf_unref(rsp);

	/* Set host controller functionality to user defined baudrate
	 * after fw downloading.
	 */
	if (baudrate_for_operation != HCI_UART_DEFAULT_BAUDRATE) {
		err = bt_update_controller_baudrate(dev, baudrate_for_operation);
		if (err) {
			return err;
		}
	}

	return 0;
}
