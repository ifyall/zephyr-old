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

void Cy_SystemInit(void)
{
}


void _init(void) {}
void _fini(void) { while(1); }


static int init_cycfg_platform_wraper(const struct device *arg)
{
    ARG_UNUSED(arg);
    /* Enable the FPU if used */
    Cy_SystemInitFpuEnable(); //??

    SystemInit();
    return 0;
}

SYS_INIT(init_cycfg_platform_wraper, PRE_KERNEL_1, 0);
