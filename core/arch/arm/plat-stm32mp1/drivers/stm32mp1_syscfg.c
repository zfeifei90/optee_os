// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2019-2021, STMicroelectronics
 */

#include <drivers/clk.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <config.h>
#include <drivers/clk.h>
#include <initcall.h>
#include <kernel/delay.h>
#include <mm/core_memprot.h>
#include <stm32_util.h>
#include <io.h>
#include <trace.h>
#include <types_ext.h>

/*
 * SYSCFG register offsets (base relative)
 */
#define SYSCFG_CMPCR				0x20U
#define SYSCFG_CMPENSETR			0x24U
#define SYSCFG_CMPENCLRR			0x28U
#define SYSCFG_IDC				0x380U

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

/*
 * SYSCFG_IDC Register
 */
#define SYSCFG_IDC_DEV_ID_MASK			GENMASK_32(11, 0)
#define SYSCFG_IDC_REV_ID_MASK			GENMASK_32(31, 16)
#define SYSCFG_IDC_REV_ID_SHIFT			U(16)

static vaddr_t get_syscfg_base(void)
{
	static struct io_pa_va base = { .pa = SYSCFG_BASE };

	return io_pa_or_va(&base, SYSCFG_IDC);
}

uint32_t stm32mp_syscfg_get_chip_dev_id(void)
{
	if (IS_ENABLED(CFG_STM32MP13))
		return io_read32(get_syscfg_base() + SYSCFG_IDC) &
			SYSCFG_IDC_DEV_ID_MASK;

	return 0;
}

void stm32mp_syscfg_enable_io_compensation(void)
{
	vaddr_t syscfg_base = get_syscfg_base();
	struct clk *csi_clk = stm32mp_rcc_clock_id_to_clk(CK_CSI);
	struct clk *syscfg_clk = stm32mp_rcc_clock_id_to_clk(SYSCFG);
	uint64_t timeout_ref = 0;

	clk_enable(csi_clk);
	clk_enable(syscfg_clk);

	io_setbits32(syscfg_base + SYSCFG_CMPENSETR, SYSCFG_CMPENSETR_MPU_EN);

	timeout_ref = timeout_init_us(SYSCFG_CMPCR_READY_TIMEOUT_US);

	while (!(io_read32(syscfg_base + SYSCFG_CMPCR) & SYSCFG_CMPCR_READY))
		if (timeout_elapsed(timeout_ref)) {
			EMSG("IO compensation cell not ready");
			/* Allow an almost silent failure here */
			break;
		}

	io_clrbits32(syscfg_base + SYSCFG_CMPCR, SYSCFG_CMPCR_SW_CTRL);

	DMSG("SYSCFG.cmpcr = %#"PRIx32, io_read32(syscfg_base + SYSCFG_CMPCR));
}

void stm32mp_syscfg_disable_io_compensation(void)
{
	vaddr_t syscfg_base = get_syscfg_base();
	struct clk *csi_clk = stm32mp_rcc_clock_id_to_clk(CK_CSI);
	struct clk *syscfg_clk = stm32mp_rcc_clock_id_to_clk(SYSCFG);
	uint32_t value = 0;

	assert(csi_clk && syscfg_clk);

	/* No refcount balance needed on non-secure SYSCFG clock */
	clk_enable(syscfg_clk);


	value = io_read32(syscfg_base + SYSCFG_CMPCR) >>
		SYSCFG_CMPCR_ANSRC_SHIFT;

	io_clrbits32(syscfg_base + SYSCFG_CMPCR,
		     SYSCFG_CMPCR_RANSRC | SYSCFG_CMPCR_RAPSRC);

	value = io_read32(syscfg_base + SYSCFG_CMPCR) |
		(value << SYSCFG_CMPCR_RANSRC_SHIFT);

	io_write32(syscfg_base + SYSCFG_CMPCR, value | SYSCFG_CMPCR_SW_CTRL);

	DMSG("SYSCFG.cmpcr = %#"PRIx32, io_read32(syscfg_base + SYSCFG_CMPCR));

	io_setbits32(syscfg_base + SYSCFG_CMPENCLRR, SYSCFG_CMPENSETR_MPU_EN);

	clk_disable(syscfg_clk);
	clk_disable(csi_clk);
}

static TEE_Result stm32mp1_iocomp(void)
{
	stm32mp_syscfg_enable_io_compensation();

	return TEE_SUCCESS;
}
driver_init(stm32mp1_iocomp);
