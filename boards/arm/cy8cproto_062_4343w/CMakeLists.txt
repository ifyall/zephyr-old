#
# Copyright (c) 2021 Cypress Semiconductor Corporation.
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_include_directories(bsp/)
zephyr_include_directories(bsp/COMPONENT_BSP_DESIGN_MODUS/GeneratedSource/)

zephyr_library()
zephyr_library_sources(bsp/cybsp.c)
zephyr_library_sources(bsp/system_psoc6_cm4.c)
zephyr_library_sources(bsp/startup_psoc6_02_cm4.S)

zephyr_library_sources(bsp/COMPONENT_BSP_DESIGN_MODUS/GeneratedSource/cycfg.c)
zephyr_library_sources(bsp/COMPONENT_BSP_DESIGN_MODUS/GeneratedSource/cycfg_clocks.c)
zephyr_library_sources(bsp/COMPONENT_BSP_DESIGN_MODUS/GeneratedSource/cycfg_peripherals.c)
zephyr_library_sources(bsp/COMPONENT_BSP_DESIGN_MODUS/GeneratedSource/cycfg_pins.c)
zephyr_library_sources(bsp/COMPONENT_BSP_DESIGN_MODUS/GeneratedSource/cycfg_qspi_memslot.c)
zephyr_library_sources(bsp/COMPONENT_BSP_DESIGN_MODUS/GeneratedSource/cycfg_routing.c)
zephyr_library_sources(bsp/COMPONENT_BSP_DESIGN_MODUS/GeneratedSource/cycfg_system.c)

