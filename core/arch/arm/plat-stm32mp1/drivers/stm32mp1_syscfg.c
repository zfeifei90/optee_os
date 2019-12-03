// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2019, STMicroelectronics - All Rights Reserved
 */

#include <dt-bindings/clock/stm32mp1-clks.h>
#include <kernel/delay.h>
#include <stm32_util.h>
#include <trace.h>

/*
 * SYSCFG register offsets (base relative)
 */
#define SYSCFG_CMPCR				0x20U
#define SYSCFG_CMPENSETR			0x24U

/*
 * SYSCFG_CMPCR Register
 */
#define SYSCFG_CMPCR_SW_CTRL			BIT(1)
#define SYSCFG_CMPCR_READY			BIT(8)
#define SYSCFG_CMPCR_RANSRC			GENMASK_32(19, 16)
#define SYSCFG_CMPCR_RANSRC_SHIFT		16
#define SYSCFG_CMPCR_RAPSRC			GENMASK_32(23, 20)
#define SYSCFG_CMPCR_ANSRC_SHIFT		24

#define SYSCFG_CMPCR_READY_TIMEOUT_US		1000U

/*
 * SYSCFG_CMPENSETR Register
 */
#define SYSCFG_CMPENSETR_MPU_EN			BIT(0)

void stm32mp1_syscfg_enable_io_compensation(void)
{
	uintptr_t syscfg_base = stm32_get_syscfg_base();
	uint64_t start;

	/*
	 * Activate automatic I/O compensation.
	 * Warning: need to ensure CSI enabled and ready in clock driver.
	 * Enable non-secure clock, we assume non-secure is suspended.
	 */
	stm32mp1_clk_enable_non_secure(SYSCFG);

	mmio_setbits_32(syscfg_base + SYSCFG_CMPENSETR,
			SYSCFG_CMPENSETR_MPU_EN);

	start = utimeout_init(SYSCFG_CMPCR_READY_TIMEOUT_US);

	while ((mmio_read_32(syscfg_base + SYSCFG_CMPCR) &
		SYSCFG_CMPCR_READY) == 0U) {
		if (utimeout_elapsed(SYSCFG_CMPCR_READY_TIMEOUT_US, start)) {
			EMSG("IO compensation cell not ready");
			break;
		}
	}

	mmio_clrbits_32(syscfg_base + SYSCFG_CMPCR, SYSCFG_CMPCR_SW_CTRL);

	DMSG("[0x"PRIxPTR"] SYSCFG.cmpcr = 0x"PRIx32,
	     syscfg_base + SYSCFG_CMPCR,
	     mmio_read_32(syscfg_base + SYSCFG_CMPCR));
}

void stm32mp1_syscfg_disable_io_compensation(void)
{
	uintptr_t syscfg_base = stm32_get_syscfg_base();
	uint32_t value;

	/*
	 * Deactivate automatic I/O compensation.
	 * Warning: CSI is disabled automatically in STOP if not
	 * requested for other usages and always OFF in STANDBY.
	 * Disable non-secure SYSCFG clock, we assume non-secure is suspended.
	 */
	value = mmio_read_32(syscfg_base + SYSCFG_CMPCR) >>
		SYSCFG_CMPCR_ANSRC_SHIFT;

	mmio_clrbits_32(syscfg_base + SYSCFG_CMPCR,
			SYSCFG_CMPCR_RANSRC | SYSCFG_CMPCR_RAPSRC);

	value = mmio_read_32(syscfg_base + SYSCFG_CMPCR) |
		(value << SYSCFG_CMPCR_RANSRC_SHIFT);

	mmio_write_32(syscfg_base + SYSCFG_CMPCR, value | SYSCFG_CMPCR_SW_CTRL);

	DMSG("[0x"PRIxPTR"] SYSCFG.cmpcr = 0x"PRIx32,
	     syscfg_base + SYSCFG_CMPCR,
	     mmio_read_32(syscfg_base + SYSCFG_CMPCR));

	mmio_clrbits_32(syscfg_base + SYSCFG_CMPENSETR,
			SYSCFG_CMPENSETR_MPU_EN);

	stm32mp1_clk_disable_non_secure(SYSCFG);
}
