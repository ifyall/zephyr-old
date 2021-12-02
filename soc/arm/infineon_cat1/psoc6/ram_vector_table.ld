/*
 * Copyright (c) 2021 Cypress Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 */

SECTION_PROLOGUE(.ram_vector_table,(NOLOAD),)
{

/* When setting TBLOFF in VTOR we must align the offset to the number of
 * exception entries in the vector table. The minimum alignment of 32 words
 * is sufficient for the 16 ARM Core exceptions and up to 16 HW interrupts.
 * For more than 16 HW interrupts, we adjust the alignment by rounding up
 * to the next power of two; this restriction guarantees a functional VTOR
 * setting in any Cortex-M implementation (might not be required in every
 * Cortex-M processor).
 */
. = ALIGN( 1 << LOG2CEIL(4 * (16 + CONFIG_NUM_IRQS)) );
_ram_vector_start = .;
. += _vector_end - _vector_start;
_ram_vector_end = .;
__ramVectors =  _ram_vector_start;
} GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
