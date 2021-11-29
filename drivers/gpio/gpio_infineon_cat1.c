/* Copyright (c) 2021 Cypress Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT  cypress_psoc6_gpio

#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <drivers/gpio.h>

#include "gpio_utils.h"
#include "cy_gpio.h"
#include "cy_sysint.h"

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(gpio_cat1);

typedef void (*config_func_t)(const struct device *dev);

struct gpio_cat1_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	GPIO_PRT_Type *regs;
	config_func_t config_func;

	uint32_t port
};

struct gpio_cat1_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

#define DEV_CFG(dev) \
	((const struct gpio_cat1_config * const)(dev)->config)
#define DEV_DATA(dev) \
	((struct gpio_cat1_data * const)(dev)->data)

static int gpio_cat1_config(const struct device *dev,
			               gpio_pin_t pin,
			               gpio_flags_t flags)
{
	const struct gpio_cat1_config * const cfg = DEV_CFG(dev);


//	LOG_DBG("GPIO_CAT1: 0x%08x, Pin: %d, Mode: 0x%08x, Val: 0x%02x",
//			(unsigned int) port, pin, drv_mode, pin_val);

	return 0;
}

static int gpio_cat1_port_get_raw(const struct device *dev,
				   uint32_t *value)
{
	const struct gpio_cat1_config * const cfg = DEV_CFG(dev);
	GPIO_PRT_Type * const port = cfg->regs;
	return 0;
}

static int gpio_cat1_port_set_masked_raw(const struct device *dev,
					  uint32_t mask,
					  uint32_t value)
{
	const struct gpio_cat1_config * const cfg = DEV_CFG(dev);
	return 0;
}

static int gpio_cat1_port_set_bits_raw(const struct device *dev,
					uint32_t mask)
{
	const struct gpio_psoc6_config * const cfg = DEV_CFG(dev);
	return 0;
}

static int gpio_cat1_port_clear_bits_raw(const struct device *dev,
					  uint32_t mask)
{
	const struct gpio_cat1_config * const cfg = DEV_CFG(dev);
	return 0;
}

static int gpio_cat1_port_toggle_bits(const struct device *dev,
				       uint32_t mask)
{
	const struct gpio_cat1_config * const cfg = DEV_CFG(dev);
	return 0;
}

static int gpio_cat1_pin_interrupt_configure(const struct device *dev,
					     gpio_pin_t pin,
					     enum gpio_int_mode mode,
					     enum gpio_int_trig trig)
{
	const struct gpio_cat1_config * const cfg = DEV_CFG(dev);
	return 0;
}


static int gpio_cat1_manage_callback(const struct device *port,
				     struct gpio_callback *callback,
				     bool set)
{
	struct gpio_cat1_data *context = port->data;

	return gpio_manage_callback(&context->callbacks, callback, set);
}

static uint32_t gpio_cat1_get_pending_int(const struct device *dev)
{
	const struct gpio_cat1_config * const cfg = DEV_CFG(dev);
	return 0;
}

static const struct gpio_driver_api gpio_cat1_api = {
	.pin_configure           = gpio_cat1_config,
	.port_get_raw            = gpio_cat1_port_get_raw,
	.port_set_masked_raw     = gpio_cat1_port_set_masked_raw,
	.port_set_bits_raw       = gpio_cat1_port_set_bits_raw,
	.port_clear_bits_raw     = gpio_cat1_port_clear_bits_raw,
	.port_toggle_bits        = gpio_cat1_port_toggle_bits,
	.pin_interrupt_configure = gpio_cat1_pin_interrupt_configure,
	.manage_callback         = gpio_cat1_manage_callback,
	.get_pending_int         = gpio_cat1_get_pending_int,
};

int gpio_cat1_init(const struct device *dev)
{
	//const struct gpio_cat1_config * const cfg = DEV_CFG(dev);
	//cfg->config_func(dev);
	return 0;
}


#define GPIO_CAT1_INIT(n)						\
	static const struct gpio_cat1_config port_##n##_cat1_config = { \
		.common = {						\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),\
		},							\
		.regs = (GPIO_PRT_Type *)DT_INST_REG_ADDR(n)		\
	};								\
									\
	static struct gpio_cat1_data port_##n##_cat1_data = { 0 }; \
									\
	DEVICE_DT_INST_DEFINE(n, gpio_cat1_init, NULL,			\
			    &port_##n##_cat1_data,			\
			    &port_##n##_cat1_config, POST_KERNEL,	\
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			    &gpio_cat1_api); \	

DT_INST_FOREACH_STATUS_OKAY(GPIO_CAT1_INIT)
