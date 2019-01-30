// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2018, STMicroelectronics - All Rights Reserved
 */

#include <drivers/stm32mp1_rcc.h>
#include <drivers/stm32_reset.h>
#include <io.h>
#include <kernel/delay.h>
#include <kernel/panic.h>
#include <util.h>

#define RST_CLR_OFFSET		4U
#define RESET_TIMEOUT_US	1000

static size_t id2reg_offset(unsigned int reset_id)
{
	return ((reset_id & GENMASK_32(31, 5)) >> 5) * sizeof(uint32_t);
}

static uint8_t id2reg_bit_pos(unsigned int reset_id)
{
	return (uint8_t)(reset_id & GENMASK_32(4,0));
}

void stm32_reset_assert(unsigned int reset_id)
{
	size_t offset = id2reg_offset(reset_id);
	uint32_t bitmsk = BIT(id2reg_bit_pos(reset_id));
	uint64_t timeout_ref;
	uintptr_t rcc_base = stm32_rcc_base();

	write32(bitmsk, rcc_base + offset);

	timeout_ref = utimeout_init(RESET_TIMEOUT_US);

	while (!(read32(rcc_base + offset) & bitmsk)) {
		if (utimeout_elapsed(RESET_TIMEOUT_US, timeout_ref)) {
			panic("Reset timeout");
		}
	}
}

void stm32_reset_deassert(unsigned int reset_id)
{
	size_t offset = id2reg_offset(reset_id) + RST_CLR_OFFSET;
	uint32_t bitmsk = BIT(id2reg_bit_pos(reset_id));
	uint64_t timeout_ref;
	uintptr_t rcc_base = stm32_rcc_base();

	write32(bitmsk, rcc_base + offset);

	timeout_ref = utimeout_init(RESET_TIMEOUT_US);

	while (read32(rcc_base + offset) & bitmsk) {
		if (utimeout_elapsed(RESET_TIMEOUT_US, timeout_ref)) {
			panic("Reset timeout");
		}
	}
}
