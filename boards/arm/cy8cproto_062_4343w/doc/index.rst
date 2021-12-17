.. _cy8cproto_062_4343w:

CY8CPROTO-062-4343W
#########################

Overview
********

The CY8CPROTO-062-4343W PSoC 6 Wi-Fi BT Prototyping Kit is a low-cost hardware
platform that enables design and debug of PSoC 6 MCUs. It comes with a Murata
LBEE5KL1DX module, based on the CYW4343W combo device, industry-leading CAPSENSE
for touch buttons and slider, on-board debugger/programmer with KitProg3, microSD
card interface, 512-Mb Quad-SPI NOR flash, PDM-PCM microphone, and a thermistor.

This kit is designed with a snap-away form-factor, allowing the user to separate
the different components and features that come with this kit and use independently.
In addition, support for Digilent's Pmod interface is also provided with this kit.

.. image:: img/board.png
     :width: 887px
     :align: center
     :alt: CY8CPROTO-062-4343W

Hardware
********

For more information about the PSoC 62 MCU SoC and CY8CPROTO-062-4343W board:

- `PSoC 62 MCU SoC Website`_
- `PSoC 62 MCU Datasheet`_
- `PSoC 62 MCU Architecture Reference Manual`_
- `PSoC 62 MCU Register Reference Manual`_
- `CY8CKIT-062-WiFi-BT Website`_
- `CY8CKIT-062-WiFi-BT User Guide`_
- `CY8CKIT-062-WiFi-BT Schematics`_

Kit Features:
==================
- Support of up to 2MB Flash and 1MB SRAM
- Dedicated SDHC to interface with WICED wireless devices.
- Delivers dual-cores, with a 150-MHz Arm Cortex-M4 as the primary a
  pplication processor and a 100-MHz Arm Cortex-M0+ as the secondary 
  processor for low-power operations.
- Supports Full-Speed USB, capacitive-sensing with CAPSENSE, a PDM-PCM 
  digital microphone interface, a Quad-SPI interface, 13 serial communication
  blocks, 7 programmable analog blocks, and 56 programmable digital blocks.

Kit Contents:
==================
- PSoC 6 Wi-Fi BT Prototyping Board
- USB Type-A to Micro-B cable
- Quick Start Guide

Supported Features
==================

The board configuration supports the following hardware features:

+-----------+------------+-----------------------+
| Interface | Controller | Driver/Component      |
+===========+============+=======================+
| NVIC      | on-chip    | nested vectored       |
|           |            | interrupt controller  |
+-----------+------------+-----------------------+
| SYSTICK   | on-chip    | system clock          |
+-----------+------------+-----------------------+
| GPIO      | on-chip    | GPIO                  |
+-----------+------------+-----------------------+
| UART      | on-chip    | serial port-polling;  |
|           |            | serial port-interrupt |
+-----------+------------+-----------------------+


The default configuration can be found in the Kconfig
:zephyr_file:`boards/arm/cy8cproto_062_4343w/cy8cproto_062_4343w_defconfig`.


System Clock
============

The PSoC 62 MCU SoC is configured to use the internal IMO+FLL as a source for
the system clock. CM0+ works at 50MHz, CM4 - at 100MHz. Other sources for the
system clock are provided in the SOC, depending on your system requirements.


Programming and Debugging
*************************

The CY8CPROTO-062-4343W includes an onboard programmer/debugger (KitProg2) with
mass storage programming to provide debugging, flash programming, and serial
communication over USB. 


References
**********

.. _PSoC 62 MCU SoC Website:
	http://www.cypress.com/products/32-bit-arm-cortex-m4-psoc-6

.. _PSoC 62 MCU Datasheet:
	http://www.cypress.com/documentation/datasheets/psoc-6-mcu-psoc-62-datasheet-programmable-system-chip-psoc-preliminary

.. _PSoC 62 MCU Architecture Reference Manual:
	http://www.cypress.com/documentation/technical-reference-manuals/psoc-6-mcu-psoc-62-architecture-technical-reference-manual

.. _PSoC 62 MCU Register Reference Manual:
	http://www.cypress.com/documentation/technical-reference-manuals/psoc-6-mcu-psoc-62-register-technical-reference-manual-trm

.. _CY8CKIT-062-WiFi-BT Website:
    https://www.infineon.com/cms/en/product/evaluation-boards/cy8cproto-062-4343w/

.. _CY8CKIT-062-WiFi-BT User Guide:
    https://www.infineon.com/cms/en/product/evaluation-boards/cy8cproto-062-4343w/#!?fileId=8ac78c8c7d0d8da4017d0f0118571844

.. _CY8CKIT-062-WiFi-BT Schematics:
    https://www.infineon.com/cms/en/product/evaluation-boards/cy8cproto-062-4343w/#!?fileId=8ac78c8c7d0d8da4017d0f01126b183f
