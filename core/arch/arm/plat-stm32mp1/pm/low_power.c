// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 */

#include <arm.h>
#include <assert.h>
#include <boot_api.h>
#include <console.h>
#include <drivers/gic.h>
#include <drivers/stm32_iwdg.h>
#include <drivers/stm32_reset.h>
#include <drivers/stm32_rtc.h>
#include <drivers/stm32mp1_ddrc.h>
#include <drivers/stm32mp1_pmic.h>
#include <drivers/stm32mp1_pwr.h>
#include <drivers/stm32mp1_rcc.h>
#include <drivers/stpmic1.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <dt-bindings/etzpc/stm32-etzpc.h>
#include <dt-bindings/power/stm32mp1-power.h>
#include <dt-bindings/reset/stm32mp1-resets.h>
#include <initcall.h>
#include <io.h>
#include <keep.h>
#include <kernel/cache_helpers.h>
#include <kernel/delay.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <sm/psci.h>
#include <stdbool.h>
#include <stm32mp_pm.h>
#include <stm32_util.h>
#include <trace.h>

#include "context.h"
#include "power.h"

static uint8_t gicd_rcc_wakeup;
static uint8_t gicc_pmr;
static bool ddr_in_selfrefresh;

struct pwr_lp_config {
	uint32_t pwr_cr1;
	uint32_t pwr_mpucr;
	const char *regul_suspend_node_name;
};

#define PWR_CR1_MASK	(PWR_CR1_LPDS | PWR_CR1_LPCFG | PWR_CR1_LVDS)
#define PWR_MPUCR_MASK	(PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF | PWR_MPUCR_PDDS)

static const struct pwr_lp_config config_pwr[STM32_PM_MAX_SOC_MODE] = {
	[STM32_PM_CSLEEP_RUN] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = 0U,
		.regul_suspend_node_name = NULL,
	},
	[STM32_PM_CSTOP_ALLOW_STOP] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF,
		.regul_suspend_node_name = NULL,
	},
	[STM32_PM_CSTOP_ALLOW_LP_STOP] = {
		.pwr_cr1 = PWR_CR1_LPDS,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF,
		.regul_suspend_node_name = "lp-stop",
	},
	[STM32_PM_CSTOP_ALLOW_LPLV_STOP] = {
		.pwr_cr1 = PWR_CR1_LVDS | PWR_CR1_LPDS | PWR_CR1_LPCFG,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF,
		.regul_suspend_node_name = "lplv-stop",
	},
	[STM32_PM_CSTOP_ALLOW_STANDBY_DDR_SR] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF |
			PWR_MPUCR_PDDS,
		.regul_suspend_node_name = "standby-ddr-sr",
	},
	[STM32_PM_CSTOP_ALLOW_STANDBY_DDR_OFF] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = PWR_MPUCR_CSTDBYDIS | PWR_MPUCR_CSSF |
			PWR_MPUCR_PDDS,
		.regul_suspend_node_name = "standby-ddr-off",
	},
	[STM32_PM_SHUTDOWN] = {
		.pwr_cr1 = 0U,
		.pwr_mpucr = 0U,
		.regul_suspend_node_name = "standby-ddr-off",
	},
};

static void set_rcc_it_priority(uint8_t *it_prio, uint8_t *pmr)
{
	*it_prio = itr_set_ipriority(RCC_WAKEUP_IT, GIC_HIGHEST_SEC_PRIORITY);
	*pmr = itr_set_pmr(STM32MP_GIC_PRIORITY_CSTOP);
}

static void restore_rcc_it_priority(uint8_t it_prio, uint8_t pmr)
{
	(void)itr_set_ipriority(RCC_WAKEUP_IT, it_prio);
	(void)itr_set_pmr(pmr);
}

static void stm32_apply_pmic_suspend_config(uint32_t mode)
{
	if (stm32mp_with_pmic()) {
		const char *name = config_pwr[mode].regul_suspend_node_name;

		assert(mode < ARRAY_SIZE(config_pwr));
		stm32mp_get_pmic();
		stm32mp_pmic_apply_lp_config(name);
		stm32mp_pmic_apply_boot_on_config(); // ??? should be done at wakeup only?
		stm32mp_put_pmic();
	}
}

#define CONSOLE_FLUSH_DELAY_MS		10

#if TRACE_LEVEL >= TRACE_DEBUG
static void wait_console_flushed(void)
{
	console_flush();
	mdelay(CONSOLE_FLUSH_DELAY_MS);
}
#else
static void wait_console_flushed(void)
{
}
#endif

static void cpu_wfi(void)
{
	dsb();
	isb();
	wfi();
}

void stm32_pm_cpu_wfi(void)
{
	wait_console_flushed();
	cpu_wfi();
}

/* If IWDG is not supported, provide a stubbed weak watchdog kicker */
void __weak stm32_iwdg_refresh(uint32_t __unused instance)
{
}

#define ARM_CNTXCTL_IMASK	BIT(1)

static void stm32mp_mask_timer(void)
{
	/* Mask timer interrupts */
	write_cntp_ctl(read_cntp_ctl() | ARM_CNTXCTL_IMASK);
	write_cntv_ctl(read_cntv_ctl() | ARM_CNTXCTL_IMASK);
}

/*
 * stm32_enter_cstop - Prepare CSTOP mode
 *
 * @mode - Target low power mode
 * Return 0 if succeed to suspend, non 0 else.
 */
int stm32_enter_cstop(uint32_t mode)
{
	uint32_t pwr_cr1 = config_pwr[mode].pwr_cr1;
	uintptr_t pwr_base = stm32_pwr_base();
	uintptr_t rcc_base = stm32_rcc_base();
	int rc;

	stm32_apply_pmic_suspend_config(mode);

	if (stm32mp_with_pmic() && (mode == STM32_PM_CSTOP_ALLOW_LP_STOP)) {
		pwr_cr1 |= PWR_CR1_LPCFG;
	}

	/* Workaround for non secure cache issue: this should not be needed */
	dcache_op_all(DCACHE_OP_CLEAN_INV);

	/* Clear RCC interrupt before enabling it */
	mmio_setbits_32(rcc_base + RCC_MP_CIFR, RCC_MP_CIFR_WKUPF);

	/* Enable RCC Wake-up */
	mmio_setbits_32(rcc_base + RCC_MP_CIER, RCC_MP_CIFR_WKUPF);

	/* Configure low power mode */
	mmio_clrsetbits_32(pwr_base + PWR_MPUCR_OFF, PWR_MPUCR_MASK,
			   config_pwr[mode].pwr_mpucr);
	mmio_clrsetbits_32(pwr_base + PWR_CR1_OFF, PWR_CR1_MASK, pwr_cr1);

	/* Clear RCC pending interrupt flags */
	mmio_write_32(rcc_base + RCC_MP_CIFR, RCC_MP_CIFR_MASK);

	/* Request CSTOP mode to RCC */
	mmio_setbits_32(rcc_base + RCC_MP_SREQSETR,
			RCC_MP_SREQSETR_STPREQ_P0 | RCC_MP_SREQSETR_STPREQ_P1);

	stm32_iwdg_refresh(IWDG2_INST);

	set_rcc_it_priority(&gicd_rcc_wakeup, &gicc_pmr);

	rc = ddr_standby_sr_entry(NULL);

	ddr_in_selfrefresh = (rc == 0);

	if (mode == STM32_PM_CSTOP_ALLOW_STANDBY_DDR_SR) {
		/* Keep retention and backup RAM content in standby */
		mmio_setbits_32(pwr_base + PWR_CR2_OFF, PWR_CR2_BREN |
				PWR_CR2_RREN);
		while ((mmio_read_32(pwr_base + PWR_CR2_OFF) &
			(PWR_CR2_BRRDY | PWR_CR2_RRRDY)) == 0U) {
			;
		}
	}

	return rc;
}

/*
 * stm32_exit_cstop - Exit from CSTOP mode
 */
void stm32_exit_cstop(void)
{
	uintptr_t pwr_base = stm32_pwr_base();
	uintptr_t rcc_base = stm32_rcc_base();

	if (ddr_in_selfrefresh) {
		if (ddr_sw_self_refresh_exit() != 0) {
			panic();
		}
		ddr_in_selfrefresh = false;
	}

	restore_rcc_it_priority(gicd_rcc_wakeup, gicc_pmr);

	/* Disable STOP request */
	mmio_setbits_32(rcc_base + RCC_MP_SREQCLRR,
			RCC_MP_SREQSETR_STPREQ_P0 | RCC_MP_SREQSETR_STPREQ_P1);

	/* Disable RCC Wake-up */
	mmio_clrbits_32(rcc_base + RCC_MP_CIER, RCC_MP_CIFR_WKUPF);

	dsb();
	isb();

	/* Disable retention and backup RAM content after stop */
	mmio_clrbits_32(pwr_base + PWR_CR2_OFF, PWR_CR2_BREN | PWR_CR2_RREN);
}

/*
 * GIC support required in low power sequences and reset sequences
 */
#define GICC_IAR			0x00C
#define GICC_IT_ID_MASK			0x3ff
#define GICC_EOIR			0x010
#define GICC_HPPIR			0x018
#define GICC_AHPPIR			0x028
#define GIC_PENDING_G1_INTID		1022U
#define GIC_SPURIOUS_INTERRUPT		1023U
#define GIC_NUM_INTS_PER_REG		32
#define GIC_MAX_SPI_ID			1020
#define GICD_ICENABLER(n)		(0x180 + (n) * 4)

static void clear_pending_interrupts(void)
{
	uint32_t id;
	uintptr_t gicc_base = get_gicc_base();
	uintptr_t gicd_base = get_gicd_base();

	do {
		id = read32(gicc_base + GICC_HPPIR) & GICC_IT_ID_MASK;

		/*
		 * Find out which interrupt it is under the
		 * assumption that the GICC_CTLR.AckCtl bit is 0.
		 */
		if (id == GIC_PENDING_G1_INTID)
			id = read32(gicc_base + GICC_AHPPIR) & GICC_IT_ID_MASK;

		if (id < GIC_MAX_SPI_ID) {
			size_t idx = id / GIC_NUM_INTS_PER_REG;
			uint32_t mask = 1 << (id % GIC_NUM_INTS_PER_REG);

			write32(id, gicc_base + GICC_EOIR);

			write32(mask, gicd_base + GICD_ICENABLER(idx));

			dsb_ishst();
		}
	} while (id < GIC_MAX_SPI_ID);
}

void stm32mp_gic_set_end_of_interrupt(uint32_t it)
{
	uintptr_t gicc_base = get_gicc_base();

	write32(it, gicc_base + GICC_EOIR);
}

static void __noreturn wait_cpu_reset(void)
{
	dcache_op_all(DCACHE_OP_CLEAN_INV);
	write_sctlr(read_sctlr() & ~SCTLR_C);
	dcache_op_all(DCACHE_OP_CLEAN_INV);
	__asm__("clrex");

	dsb();
	isb();

	for ( ; ; ) {
		clear_pending_interrupts();
		wfi();
	}
}

/*
 * tzc_source_ip contains the TZC transaction source IPs that need to be reset
 * before a C-A7 subsystem is reset (i.e. independent reset):
 * - C-A7 subsystem is reset separately later in the sequence,
 * - C-M4 subsystem is not concerned here,
 * - DAP is excluded for debug purpose,
 * - IPs are stored with their ETZPC IDs (STM32MP1_ETZPC_MAX_ID if not
 *   applicable) because some of them need to be reset only if they are not
 *   configured in MCU isolation mode inside ETZPC device tree.
 */
struct tzc_source_ip {
	uint32_t reset_id;
	uint32_t clock_id;
	uint32_t decprot_id;
};

#define _TZC_FIXED(res, clk)				\
	{						\
		.reset_id = (res),			\
		.clock_id = (clk),			\
		.decprot_id = STM32MP1_ETZPC_MAX_ID,	\
	}

#define _TZC_COND(res, clk, decprot)			\
	{						\
		.reset_id = (res),			\
		.clock_id = (clk),			\
		.decprot_id = (decprot),		\
	}

static const struct tzc_source_ip tzc_source_ip[] = {
	_TZC_FIXED(LTDC_R, LTDC_PX),
	_TZC_FIXED(GPU_R, GPU),
	_TZC_FIXED(USBH_R, USBH),
	_TZC_FIXED(SDMMC1_R, SDMMC1_K),
	_TZC_FIXED(SDMMC2_R, SDMMC2_K),
	_TZC_FIXED(MDMA_R, MDMA),
	_TZC_COND(USBO_R, USBO_K, STM32MP1_ETZPC_OTG_ID),
	_TZC_COND(SDMMC3_R, SDMMC3_K, STM32MP1_ETZPC_SDMMC3_ID),
	_TZC_COND(ETHMAC_R, ETHMAC, STM32MP1_ETZPC_ETH_ID),
	_TZC_COND(DMA1_R, DMA1, STM32MP1_ETZPC_DMA1_ID),
	_TZC_COND(DMA2_R, DMA2, STM32MP1_ETZPC_DMA2_ID),
};

#define RCC_AHB6RSTSETR_GPURST	BIT(5)

static void __noreturn reset_cores(void)
{
	uintptr_t rcc_base = stm32_rcc_base();
	uint32_t reset_mask = RCC_MP_GRSTCSETR_MPUP0RST |
			      RCC_MP_GRSTCSETR_MPUP1RST;
	uint32_t target_mask;
	uint32_t id;

	/* Mask timer interrupts */
	stm32mp_mask_timer();

	for (id = 0U; id < ARRAY_SIZE(tzc_source_ip); id++) {
		if ((!stm32mp1_clk_is_enabled(tzc_source_ip[id].clock_id)) ||
		    ((tzc_source_ip[id].decprot_id != STM32MP1_ETZPC_MAX_ID) &&
		     (etzpc_get_decprot(tzc_source_ip[id].decprot_id) ==
		      TZPC_DECPROT_MCU_ISOLATION))) {
			continue;
		}

		if (tzc_source_ip[id].reset_id != GPU_R) {
			stm32_reset_assert(tzc_source_ip[id].reset_id);
			stm32_reset_deassert(tzc_source_ip[id].reset_id);
		} else {
			/* GPU reset automatically cleared by hardware */
			mmio_setbits_32(rcc_base + RCC_AHB6RSTSETR,
					RCC_AHB6RSTSETR_GPURST);
		}
	}

	if (get_core_pos() == 0) {
		target_mask = TARGET_CPU1_GIC_MASK;
	} else {
		target_mask = TARGET_CPU0_GIC_MASK;
	}

	itr_raise_sgi(GIC_SEC_SGI_1, target_mask);

	clear_pending_interrupts();

	write32(reset_mask, rcc_base + RCC_MP_GRSTCSETR);

	wait_cpu_reset();
}

/*
 * stm32_pm_cpus_reset - Reset only cpus
 */
void __noreturn stm32_cores_reset(void)
{
	reset_cores();
}
KEEP_PAGER(stm32_cores_reset);

static void reset_other_core(void)
{
	uintptr_t rcc_base = stm32_rcc_base();
	uint32_t reset_mask;
	uint32_t target_mask;

	if (get_core_pos() == 0) {
		reset_mask = RCC_MP_GRSTCSETR_MPUP1RST;
		target_mask = TARGET_CPU1_GIC_MASK;
	} else {
		reset_mask = RCC_MP_GRSTCSETR_MPUP0RST;
		target_mask = TARGET_CPU0_GIC_MASK;
	}

	itr_raise_sgi(GIC_SEC_SGI_1, target_mask);

	write32(reset_mask, rcc_base + RCC_MP_GRSTCSETR);
}

/*
 * stm32_enter_cstop_shutdown - Shutdown CPUs to target low power mode
 * @mode - Target low power mode
 */
void __noreturn stm32_enter_cstop_shutdown(uint32_t mode)
{
	switch (mode) {
	case STM32_PM_SHUTDOWN:
		if (stm32mp_with_pmic()) {
			wait_console_flushed();
			stm32mp_get_pmic();
			stpmic1_switch_off();
			udelay(100);
		}
		break;
	case STM32_PM_CSTOP_ALLOW_STANDBY_DDR_SR:
	case STM32_PM_CSTOP_ALLOW_STANDBY_DDR_OFF:
#ifdef STM32MP1_USE_MPU0_RESET
		stm32mp_pm_shutdown_context();
		reset_other_core();
		stm32_enter_cstop(mode);
		dsb();
		isb();
		for ( ; ; ) {
			wfi();
		}
#else
		if (stm32mp_with_pmic()) {
			wait_console_flushed();
			stm32mp_get_pmic();
			stpmic1_switch_off();
			udelay(100);
		}
#endif
		break;
	default:
		break;
	}

	panic();
}

/*
 * stm32_enter_cstop_reset - Reset CPUs to target low power mode
 * @mode - Target low power mode
 */
void __noreturn stm32_enter_cstop_reset(uint32_t mode)
{
	uintptr_t rcc_base = stm32_rcc_base();

	switch (mode) {
	case STM32_PM_SHUTDOWN:
		write32(RCC_MP_GRSTCSETR_MPSYSRST, rcc_base + RCC_MP_GRSTCSETR);
		udelay(100);
		break;
	default:
		IMSG("Forced system reset");
		wait_console_flushed();
		write32(RCC_MP_GRSTCSETR_MPSYSRST, rcc_base + RCC_MP_GRSTCSETR);
		udelay(100);
		break;
	}

	panic();
}

/*
 * stm32_enter_csleep - enter CSLEEP state while WFI and exit in CRUN
 *
 * Configure PWR for CSLEEP state. CPU shall execute a WFI and return
 * once a interrupt is pending.
 */
void stm32_enter_csleep(void)
{
	uintptr_t pwr_base = stm32_pwr_base();

	mmio_clrsetbits_32(pwr_base + PWR_MPUCR_OFF, PWR_MPUCR_MASK,
			   config_pwr[STM32_PM_CSLEEP_RUN].pwr_mpucr);
	mmio_clrsetbits_32(pwr_base + PWR_CR1_OFF, PWR_CR1_MASK,
			   config_pwr[STM32_PM_CSLEEP_RUN].pwr_cr1);

	stm32_pm_cpu_wfi();
}

/* RCC Wakeup interrupt is used to wake from suspeneded mode */
static enum itr_return rcc_wakeup_it_handler(struct itr_handler *hdl __unused)
{
	/* This interrupt is not expected to be handled */
	panic("RCC  wakeup interrupt");
	return ITRR_HANDLED;
}

static struct itr_handler rcc_wakeup_handler = {
	.it = RCC_WAKEUP_IT,
	.handler = rcc_wakeup_it_handler,
};
KEEP_PAGER(rcc_wakeup_handler);

/* SGI9 (secure SGI 1) informs targeted CPU it shall reset */
static enum itr_return sgi9_it_handler(struct itr_handler *handler)
{
	stm32mp_mask_timer();

	stm32mp_gic_set_end_of_interrupt(handler->it);

	clear_pending_interrupts();

	wait_cpu_reset();

	panic("Core reset");

	return ITRR_HANDLED;
}

static struct itr_handler sgi9_reset_handler = {
	.it = GIC_SEC_SGI_1,
	.handler = sgi9_it_handler,
};
KEEP_PAGER(sgi9_reset_handler);

static TEE_Result init_low_power(void)
{
	itr_add(&rcc_wakeup_handler);
	itr_enable(rcc_wakeup_handler.it);

	itr_add(&sgi9_reset_handler);
	itr_enable(sgi9_reset_handler.it);

	return TEE_SUCCESS;
}
service_init(init_low_power);

/*
 * CPU low power sequences
 */
void __noreturn stm32_pm_cpu_power_down_wfi(void)
{
	if (get_core_pos() == 0) {
		void (*reset_ep)(void) = stm32mp_sysram_resume;

		stm32_pm_cpu_wfi();

		/* STANDBY not reached: resume from retained SYSRAM */
		stm32_exit_cstop();
		stm32mp_cpu_reset_state();
		reset_ep();
		panic();
	}

	dcache_op_level1(DCACHE_OP_CLEAN);
	write32(RCC_MP_GRSTCSETR_MPUP1RST, stm32_rcc_base() + RCC_MP_GRSTCSETR);
	cpu_wfi();
	panic();
}
