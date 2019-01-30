// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2017-2018, STMicroelectronics
 */

#include <io.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <drivers/stm32mp1_rcc.h>
#include <stm32_util.h>

uintptr_t stm32_rcc_base(void)
{
	static void *va;

	if (!cpu_mmu_enabled()) {
		return RCC_BASE;
	}

	if (!va) {
		va = phys_to_virt(RCC_BASE, MEM_AREA_IO_SEC);
	}

	return (uintptr_t)va;
}

void stm32_rcc_secure(int enable)
{
	uintptr_t base = stm32_rcc_base();

	if (enable != 0) {
		io_mask32_stm32shregs(base + RCC_TZCR, RCC_TZCR_TZEN,
				      RCC_TZCR_TZEN);
	} else {
		io_mask32_stm32shregs(base + RCC_TZCR, 0, RCC_TZCR_TZEN);
	}
}

bool stm32_rcc_is_secure(void)
{
	uintptr_t base = stm32_rcc_base();

	return read32(base + RCC_TZCR) & RCC_TZCR_TZEN;
}

bool stm32_rcc_is_mckprot(void)
{
	uintptr_t base = stm32_rcc_base();

	return read32(base + RCC_TZCR) & RCC_TZCR_MCKPROT;
}
