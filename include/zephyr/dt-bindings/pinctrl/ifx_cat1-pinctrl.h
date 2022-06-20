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
 * @brief Pin control binding helper.
 */

/**
 * Bit definition in PINMUX field
 */
#define SOC_PINMUX_PORT_POS                                (0)
#define SOC_PINMUX_PORT_MASK                               (0xFFul << SOC_PINMUX_PORT_POS)
#define SOC_PINMUX_PIN_POS                                 (8)
#define SOC_PINMUX_PIN_MASK                                (0xFFul << SOC_PINMUX_PIN_POS)
#define SOC_PINMUX_HSIOM_FUNC_POS                          (16)
#define SOC_PINMUX_HSIOM_MASK                              (0xFFul << SOC_PINMUX_HSIOM_FUNC_POS)
#define SOC_PINMUX_SIGNAL_POS                              (24)
#define SOC_PINMUX_SIGNAL_MASK                             (0xFFul << SOC_PINMUX_SIGNAL_POS)

/**
 * Functions are defined using HSIOM SEL
 */
#define HSIOM_SEL_GPIO                                     (0)
#define HSIOM_SEL_GPIO_DSI                                 (1)
#define HSIOM_SEL_DSI_DSI                                  (2)
#define HSIOM_SEL_DSI_GPIO                                 (3)
#define HSIOM_SEL_AMUXA                                    (4)
#define HSIOM_SEL_AMUXB                                    (5)
#define HSIOM_SEL_AMUXA_DSI                                (6)
#define HSIOM_SEL_AMUXB_DSI                                (7)
#define HSIOM_SEL_ACT_0                                    (8)
#define HSIOM_SEL_ACT_1                                    (9)
#define HSIOM_SEL_ACT_2                                    (10)
#define HSIOM_SEL_ACT_3                                    (11)
#define HSIOM_SEL_DS_0                                     (12)
#define HSIOM_SEL_DS_1                                     (13)
#define HSIOM_SEL_DS_2                                     (14)
#define HSIOM_SEL_DS_3                                     (15)
#define HSIOM_SEL_ACT_4                                    (16)
#define HSIOM_SEL_ACT_5                                    (17)
#define HSIOM_SEL_ACT_6                                    (18)
#define HSIOM_SEL_ACT_7                                    (19)
#define HSIOM_SEL_ACT_8                                    (20)
#define HSIOM_SEL_ACT_9                                    (21)
#define HSIOM_SEL_ACT_10                                   (22)
#define HSIOM_SEL_ACT_11                                   (23)
#define HSIOM_SEL_ACT_12                                   (24)
#define HSIOM_SEL_ACT_13                                   (25)
#define HSIOM_SEL_ACT_14                                   (26)
#define HSIOM_SEL_ACT_15                                   (27)
#define HSIOM_SEL_DS_4                                     (28)
#define HSIOM_SEL_DS_5                                     (29)
#define HSIOM_SEL_DS_6                                     (30)
#define HSIOM_SEL_DS_7                                     (31)
/**
 * Drive mode definition for Peripheral pins
 */
/* *SUSPEND-FORMATTING* */
/* I2C */
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_I2C_SCL                drive-open-drain; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_I2C_SDA                drive-open-drain; input-enable
/* SPI (Slave) */
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_M_CLK              drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_M_MISO             input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_M_MOSI             drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_M_SELECT0          drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_M_SELECT1          drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_M_SELECT2          drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_M_SELECT3          drive-push-pull
/* SPI (Master) */
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_S_CLK              input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_S_MISO             drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_S_MOSI             input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_S_SELECT0          input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_S_SELECT1          input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_S_SELECT2          input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_SPI_S_SELECT3          input-enable
/* UART */
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_UART_RX                input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_UART_TX                drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_UART_CTS               input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SCB_UART_RTS               drive-push-pull
/* SDHC */
#define CAT1_PIN_MAP_DRIVE_MODE_SDHC_CARD_CMD              drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SDHC_CARD_DAT_3TO0         drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SDHC_CARD_DAT_7TO4         drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SDHC_CARD_DETECT_N         input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SDHC_CARD_EMMC_RESET_N     drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SDHC_CARD_IF_PWR_EN        drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SDHC_CARD_MECH_WRITE_PROT  input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SDHC_CLK_CARD              drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SDHC_IO_VOLT_SEL           drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SDHC_LED_CTRL              drive-push-pull
/* SMIF */
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_CLK               drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_DATA0             drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_DATA1             drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_DATA2             drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_DATA3             drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_DATA4             drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_DATA5             drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_DATA6             drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_DATA7             drive-push-pull; input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_SELECT0           drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_SELECT1           drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_SELECT2           drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_SMIF_SPI_SELECT3           drive-push-pull
/* TCPWM */
#define CAT1_PIN_MAP_DRIVE_MODE_TCPWM_LINE                 drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_TCPWM_LINE_COMPL           drive-push-pull
/* AUDIOSS */
#define CAT1_PIN_MAP_DRIVE_MODE_AUDIOSS_CLK_I2S_IF         input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_AUDIOSS_PDM_CLK            drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_AUDIOSS_PDM_DATA           input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_AUDIOSS_RX_SCK             drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_AUDIOSS_RX_SDI             input-enable
#define CAT1_PIN_MAP_DRIVE_MODE_AUDIOSS_RX_WS              drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_AUDIOSS_TX_SCK             drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_AUDIOSS_TX_SDO             drive-push-pull
#define CAT1_PIN_MAP_DRIVE_MODE_AUDIOSS_TX_WS              drive-push-pull
/* *RESUME-FORMATTING* */


/**
 * Peripheral signal definitions
 */
#define CAT1_PIN_SIGNAL_NC                                 (0)
/* I2C */
#define CAT1_PIN_SIGNAL_SCB_I2C_SCL                        (1)
#define CAT1_PIN_SIGNAL_SCB_I2C_SDA                        (2)
/* SPI (Slave) */
#define CAT1_PIN_SIGNAL_SCB_SPI_M_CLK                      (3)
#define CAT1_PIN_SIGNAL_SCB_SPI_M_MISO                     (4)
#define CAT1_PIN_SIGNAL_SCB_SPI_M_MOSI                     (5)
#define CAT1_PIN_SIGNAL_SCB_SPI_M_SELECT0                  (6)
#define CAT1_PIN_SIGNAL_SCB_SPI_M_SELECT1                  (7)
#define CAT1_PIN_SIGNAL_SCB_SPI_M_SELECT2                  (8)
#define CAT1_PIN_SIGNAL_SCB_SPI_M_SELECT3                  (9)
/* SPI (Master) */
#define CAT1_PIN_SIGNAL_SCB_SPI_S_CLK                      (10)
#define CAT1_PIN_SIGNAL_SCB_SPI_S_MISO                     (11)
#define CAT1_PIN_SIGNAL_SCB_SPI_S_MOSI                     (12)
#define CAT1_PIN_SIGNAL_SCB_SPI_S_SELECT0                  (13)
#define CAT1_PIN_SIGNAL_SCB_SPI_S_SELECT1                  (14)
#define CAT1_PIN_SIGNAL_SCB_SPI_S_SELECT2                  (15)
#define CAT1_PIN_SIGNAL_SCB_SPI_S_SELECT3                  (16)
/* UART */
#define CAT1_PIN_SIGNAL_SCB_UART_RX                        (17)
#define CAT1_PIN_SIGNAL_SCB_UART_TX                        (18)
#define CAT1_PIN_SIGNAL_SCB_UART_CTS                       (19)
#define CAT1_PIN_SIGNAL_SCB_UART_RTS                       (20)
/* SDHC */
#define CAT1_PIN_SIGNAL_SDHC_CARD_CMD                      (21)
#define CAT1_PIN_SIGNAL_SDHC_CARD_DAT_3TO0                 (22)
#define CAT1_PIN_SIGNAL_SDHC_CARD_DAT_7TO4                 (23)
#define CAT1_PIN_SIGNAL_SDHC_CARD_DETECT_N                 (24)
#define CAT1_PIN_SIGNAL_SDHC_CARD_EMMC_RESET_N             (25)
#define CAT1_PIN_SIGNAL_SDHC_CARD_IF_PWR_EN                (26)
#define CAT1_PIN_SIGNAL_SDHC_CARD_MECH_WRITE_PROT          (27)
#define CAT1_PIN_SIGNAL_SDHC_CLK_CARD                      (28)
#define CAT1_PIN_SIGNAL_SDHC_IO_VOLT_SEL                   (29)
#define CAT1_PIN_SIGNAL_SDHC_LED_CTRL                      (30)
/* SMIF */
#define CAT1_PIN_SIGNAL_SMIF_SPI_CLK                       (31)
#define CAT1_PIN_SIGNAL_SMIF_SPI_DATA0                     (32)
#define CAT1_PIN_SIGNAL_SMIF_SPI_DATA1                     (33)
#define CAT1_PIN_SIGNAL_SMIF_SPI_DATA2                     (34)
#define CAT1_PIN_SIGNAL_SMIF_SPI_DATA3                     (35)
#define CAT1_PIN_SIGNAL_SMIF_SPI_DATA4                     (36)
#define CAT1_PIN_SIGNAL_SMIF_SPI_DATA5                     (37)
#define CAT1_PIN_SIGNAL_SMIF_SPI_DATA6                     (38)
#define CAT1_PIN_SIGNAL_SMIF_SPI_DATA7                     (39)
#define CAT1_PIN_SIGNAL_SMIF_SPI_SELECT0                   (40)
#define CAT1_PIN_SIGNAL_SMIF_SPI_SELECT1                   (41)
#define CAT1_PIN_SIGNAL_SMIF_SPI_SELECT2                   (42)
#define CAT1_PIN_SIGNAL_SMIF_SPI_SELECT3                   (43)
/* TCPWM */
#define CAT1_PIN_SIGNAL_TCPWM_LINE                         (44)
#define CAT1_PIN_SIGNAL_TCPWM_LINE_COMPL                   (45)
/* AUDIOSS */
#define CAT1_PIN_SIGNAL_AUDIOSS_CLK_I2S_IF                 (46)
#define CAT1_PIN_SIGNAL_AUDIOSS_PDM_CLK                    (47)
#define CAT1_PIN_SIGNAL_AUDIOSS_PDM_DATA                   (48)
#define CAT1_PIN_SIGNAL_AUDIOSS_RX_SCK                     (49)
#define CAT1_PIN_SIGNAL_AUDIOSS_RX_SDI                     (50)
#define CAT1_PIN_SIGNAL_AUDIOSS_RX_WS                      (51)
#define CAT1_PIN_SIGNAL_AUDIOSS_TX_SCK                     (52)
#define CAT1_PIN_SIGNAL_AUDIOSS_TX_SDO                     (53)
#define CAT1_PIN_SIGNAL_AUDIOSS_TX_WS                      (54)

/**
 * Macro to set pinmux
 */
#define DT_CAT1_PINMUX_INFO(port, pin, hsiom, signal)	   \
	pinmux = < ((port << SOC_PINMUX_PORT_POS) |	   \
		    (pin << SOC_PINMUX_PIN_POS) |	   \
		    (hsiom << SOC_PINMUX_HSIOM_FUNC_POS) | \
		    (CAT1_PIN_SIGNAL_##signal << SOC_PINMUX_SIGNAL_POS)) >
/**
 * Macro to set drive mode
 */
#define DT_CAT1_DRIVE_MODE_INFO(peripheral_signal) \
	CAT1_PIN_MAP_DRIVE_MODE_##peripheral_signal

/**
 * Macro to set pin control information (from pinctrl node)
 */
#define DT_CAT1_PINCTRL_INFO(port, pin, hsiom, signal) \
	DT_CAT1_PINMUX_INFO(port, pin, hsiom, signal); \
	DT_CAT1_DRIVE_MODE_INFO(signal)

/* Redefine DT GPIO label (Px) to CYHAL port macros (CYHAL_PORT_x) */
#define P0  CYHAL_PORT_0
#define P1  CYHAL_PORT_1
#define P2  CYHAL_PORT_2
#define P3  CYHAL_PORT_3
#define P4  CYHAL_PORT_4
#define P5  CYHAL_PORT_5
#define P6  CYHAL_PORT_6
#define P7  CYHAL_PORT_7
#define P8  CYHAL_PORT_8
#define P9  CYHAL_PORT_9
#define P10 CYHAL_PORT_10
#define P11 CYHAL_PORT_11
#define P12 CYHAL_PORT_12
#define P13 CYHAL_PORT_13
#define P14 CYHAL_PORT_14
#define P15 CYHAL_PORT_15
#define P16 CYHAL_PORT_16
#define P17 CYHAL_PORT_17
#define P18 CYHAL_PORT_18
#define P19 CYHAL_PORT_19
#define P20 CYHAL_PORT_20

/* Returns CYHAL GPIO from Board device tree GPIO configuration */
#define DT_GET_CYHAL_GPIO_FROM_DT_GPIOS(node, gpios_prop)				 \
	CYHAL_GET_GPIO(DT_STRING_TOKEN(DT_GPIO_CTLR_BY_IDX(node, gpios_prop, 0), label), \
		       DT_PHA_BY_IDX(node, gpios_prop, 0, pin))
