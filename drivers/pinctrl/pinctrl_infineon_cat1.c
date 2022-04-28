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
 * @brief Pin control driver for Infineon CAT1 MCU family.
 */

#include <drivers/pinctrl.h>
#include <cyhal_gpio.h>
#include "cy_gpio.h"

#if defined(GPIO_PRT0)
#define GPIO0_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt0))
#else
#define GPIO0_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT1)
#define GPIO1_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt1))
#else
#define GPIO1_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT2)
#define GPIO2_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt2))
#else
#define GPIO2_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT3)
#define GPIO3_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt3))
#else
#define GPIO3_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT4)
#define GPIO4_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt4))
#else
#define GPIO4_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT5)
#define GPIO5_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt5))
#else
#define GPIO5_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT6)
#define GPIO6_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt6))
#else
#define GPIO6_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT7)
#define GPIO7_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt7))
#else
#define GPIO7_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT8)
#define GPIO8_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt8))
#else
#define GPIO8_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT9)
#define GPIO9_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt9))
#else
#define GPIO9_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT10)
#define GPIO10_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt10))
#else
#define GPIO10_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT11)
#define GPIO11_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt11))
#else
#define GPIO11_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT12)
#define GPIO12_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt12))
#else
#define GPIO12_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT12)
#define GPIO12_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt12))
#else
#define GPIO12_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT13)
#define GPIO13_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt13))
#else
#define GPIO13_REG_ADDR  NULL
#endif

#if defined(GPIO_PRT14)
#define GPIO14_REG_ADDR  DT_REG_ADDR(DT_NODELABEL(gpio_prt14))
#else
#define GPIO14_REG_ADDR  NULL
#endif

/* @brief Array containing pointers to each GPIO port.
 *
 * Entries will be NULL if the GPIO port is not enabled.
 */
static GPIO_PRT_Type *const gpio_ports[] = {
	(GPIO_PRT_Type *) GPIO0_REG_ADDR,
	(GPIO_PRT_Type *) GPIO1_REG_ADDR,
	(GPIO_PRT_Type *) GPIO2_REG_ADDR,
	(GPIO_PRT_Type *) GPIO3_REG_ADDR,
	(GPIO_PRT_Type *) GPIO4_REG_ADDR,
	(GPIO_PRT_Type *) GPIO5_REG_ADDR,
	(GPIO_PRT_Type *) GPIO6_REG_ADDR,
	(GPIO_PRT_Type *) GPIO7_REG_ADDR,
	(GPIO_PRT_Type *) GPIO8_REG_ADDR,
	(GPIO_PRT_Type *) GPIO9_REG_ADDR,
	(GPIO_PRT_Type *) GPIO10_REG_ADDR,
	(GPIO_PRT_Type *) GPIO11_REG_ADDR,
	(GPIO_PRT_Type *) GPIO12_REG_ADDR,
	(GPIO_PRT_Type *) GPIO13_REG_ADDR,
	(GPIO_PRT_Type *) GPIO14_REG_ADDR
};

/* @brief This function returns gpio drive mode, according to.
 * bias and drive mode params defined in pinctrl node.
 *
 * @param flags - bias and drive mode flags from pinctrl node.
 */
static uint32_t soc_gpio_get_drv_mode(uint32_t flags)
{
	uint32_t drv_mode = CY_GPIO_DM_ANALOG;
	uint32_t _flags;

	_flags = ((flags & SOC_GPIO_FLAGS_MASK) >> SOC_GPIO_FLAGS_POS);

	if (_flags & SOC_GPIO_OPENDRAIN) {
		/* drive_open_drain */
		drv_mode = CY_GPIO_DM_OD_DRIVESLOW_IN_OFF;

	} else if (_flags & SOC_GPIO_OPENSOURCE) {
		/* drive_open_source */
		drv_mode = CY_GPIO_DM_OD_DRIVESHIGH_IN_OFF;

	} else if (_flags & SOC_GPIO_PUSHPULL) {
		/* drive_push_pull */
		drv_mode = CY_GPIO_DM_STRONG_IN_OFF;

	} else if ((_flags & SOC_GPIO_PULLUP) && (_flags & SOC_GPIO_PULLDOWN)) {
		/* bias_pull_up and bias_pull_down */
		drv_mode = CY_GPIO_DM_PULLUP_DOWN_IN_OFF;

	} else if (_flags & SOC_GPIO_PULLUP) {
		/* bias_pull_up */
		drv_mode = CY_GPIO_DM_PULLUP_IN_OFF;

	} else if (_flags & SOC_GPIO_PULLDOWN) {
		/* bias_pull_down */
		drv_mode = CY_GPIO_DM_PULLDOWN_IN_OFF;
	} else {
		/* nothing do here */
	}

	if (_flags & SOC_GPIO_INPUTENABLE) {
		/* input_enable */
		drv_mode |= CY_GPIO_DM_HIGHZ;
	}

	return drv_mode;
}

cyhal_gpio_t cat1_pinctrl_get_cyhal_gpio(const struct pinctrl_dev_config *config,
					 uint8_t signal_name)
{
	int ret;
	const struct pinctrl_state *state;

	ret = pinctrl_lookup_state(config, PINCTRL_STATE_DEFAULT, &state);
	if (ret != 0) {
		return NC;
	}

	for (uint32_t i = 0u; i < state->pin_cnt; i++) {
		if (CAT1_PINMUX_GET_SIGNAL(state->pins[i].pinmux) == (uint32_t)signal_name) {
			uint32_t port = CAT1_PINMUX_GET_PORT_NUM(state->pins[i].pinmux);
			uint32_t pin = CAT1_PINMUX_GET_PIN_NUM(state->pins[i].pinmux);

			return CYHAL_GET_GPIO(port, pin);
		}
	}

	return NC;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		uint32_t drv_mode = soc_gpio_get_drv_mode(pins[i].pincfg);
		uint32_t hsiom = CAT1_PINMUX_GET_HSIOM_FUNC(pins[i].pinmux);
		uint32_t port_num = CAT1_PINMUX_GET_PORT_NUM(pins[i].pinmux);
		uint32_t pin_num = CAT1_PINMUX_GET_PIN_NUM(pins[i].pinmux);

		/* Configures the HSIOM connection to the pin */
		Cy_GPIO_SetHSIOM(gpio_ports[port_num], pin_num, hsiom);

		/* Configures the pin output buffer drive mode and input buffer enable */
		Cy_GPIO_SetDrivemode(gpio_ports[port_num], pin_num, drv_mode);
	}

	return 0;
}
