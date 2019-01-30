// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2017-2018, STMicroelectronics - All Rights Reserved
 */

#include <arm.h>
#include <assert.h>
#include <boot_api.h>
#include <console.h>
#include <drivers/gic.h>
#include <drivers/stm32_iwdg.h>
#include <drivers/stm32_rtc.h>
#include <drivers/stm32mp1_ddrc.h>
#include <drivers/stm32mp1_pmic.h>
#include <drivers/stm32mp1_pwr.h>
#include <drivers/stm32mp1_rcc.h>
#include <drivers/stpmic1.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <dt-bindings/power/stm32mp1-power.h>
#include <initcall.h>
#include <io.h>
#include <keep.h>
#include <kernel/cache_helpers.h>
#include <kernel/delay.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
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

/*If IWDG is not supported, provide a stubbed weak watchdog kicker */
void __weak stm32_iwdg_refresh(uint32_t __unused instance)
{
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

	return rc;
}

/*
 * stm32_exit_cstop - Exit from CSTOP mode
 */
void stm32_exit_cstop(void)
{
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
}

static void __noreturn reset_cores(void)
{
	uintptr_t rcc_base = stm32_rcc_base();
	uint32_t reset_mask;
	uint32_t target_mask;

	if (get_core_pos() == 0) {
		reset_mask = RCC_MP_GRSTCSETR_MPUP0RST;
		target_mask = TARGET_CPU1_GIC_MASK;
	} else {
		reset_mask = RCC_MP_GRSTCSETR_MPUP1RST;
		target_mask = TARGET_CPU0_GIC_MASK;
	}

	itr_raise_sgi(GIC_SEC_SGI_1, target_mask);
	dcache_op_all(DCACHE_OP_CLEAN_INV);
	write32(reset_mask, rcc_base + RCC_MP_GRSTCSETR);
	cpu_wfi();
	panic("Cores reset");
}

/*
 * stm32_pm_cpus_reset - Reset only cpus
 */
void __noreturn stm32_cores_reset(void)
{
	reset_cores();
}
KEEP_PAGER(stm32_cores_reset);

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
		stm32_enter_cstop(mode);
		cpu_wfi();
		reset_cores();
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
#ifdef STM32MP1_USE_MPU0_RESET
		reset_cores();
#else
		IMSG("Forced system reset");
		wait_console_flushed();
		write32(RCC_MP_GRSTCSETR_MPSYSRST, rcc_base + RCC_MP_GRSTCSETR);
		udelay(100);
#endif
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

/*
 * Secure interrupts used in the low power sequences
 */
#define GICC_IAR		0x00C
#define GICC_IAR_IT_ID_MASK	0x3ff
#define GICC_EOIR		0x010

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
	uintptr_t rcc_base = stm32_rcc_base();
	uint32_t reset_mask;
	uintptr_t gicc_base = get_gicc_base();

	write32(handler->it, gicc_base + GICC_EOIR);

	if (get_core_pos() == 0) {
		reset_mask = RCC_MP_GRSTCSETR_MPUP0RST;
	} else {
		reset_mask = RCC_MP_GRSTCSETR_MPUP1RST;
	}

	dcache_op_all(DCACHE_OP_CLEAN_INV);
	write32(reset_mask, rcc_base + RCC_MP_GRSTCSETR);
	cpu_wfi();
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
	uintptr_t pwr_base = stm32_pwr_base();

	itr_add(&rcc_wakeup_handler);
	itr_enable(rcc_wakeup_handler.it);

	itr_add(&sgi9_reset_handler);
	itr_enable(sgi9_reset_handler.it);

	/* Enable retention for BKPSRAM and BKPREG */
	io_mask32(pwr_base + PWR_CR2_OFF,
		  PWR_CR2_BREN | PWR_CR2_RREN, PWR_CR2_BREN | PWR_CR2_RREN);

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

		wait_console_flushed();

		dsb();
		isb();
		wfi();
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
