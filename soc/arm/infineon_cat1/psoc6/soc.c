/* Copyright (c) 2021 Cypress Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <init.h>
#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>

#include "cy_syslib.h"
#include "cy_gpio.h"
#include "cy_scb_uart.h"
#include "cy_syslib.h"
#include "cy_syspm.h"
#include "cy_sysclk.h"
#include <string.h>
#include <kernel.h>
#include <kernel_internal.h>
#include <linker/linker-defs.h>

uint32_t __Vectors;

void relocate_vector_table(void)
{
    extern uint32_t _ram_vector_start[];
    size_t vector_size = (size_t)_vector_end - (size_t)_vector_start;
    
    /* Copy interrupt vectors from Flash to RAM */
    memcpy(_ram_vector_start, _vector_start, vector_size);

    /* Update VTOR to use RAM vector table */
    SCB->VTOR = (uint32_t)_ram_vector_start;
    __DSB();
    __ISB();
}


static int init_cycfg_platform_wraper(const struct device *arg)
{
    ARG_UNUSED(arg);

    /* Relocate interrupt vectors from Flash to RAM */
    relocate_vector_table();
    
    /* Initializes the system */
    SystemInit();
    return 0;
}

SYS_INIT(init_cycfg_platform_wraper, PRE_KERNEL_1, 0);