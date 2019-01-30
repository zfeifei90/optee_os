// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2017, STMicroelectronics - All Rights Reserved
 * Copyright (c) 2017, ARM Limited and Contributors. All rights reserved.
 */

#include <arm32.h>
#include <boot_api.h>
#include <drivers/stm32_rtc.h>
#include <drivers/stm32mp1_clk.h>
#include <drivers/stm32mp1_ddrc.h>
#include <drivers/stm32mp1_pwr.h>
#include <drivers/stm32mp1_rcc.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <kernel/delay.h>
#include <kernel/panic.h>
#include <io.h>
#include <mm/core_mmu.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <stm32_util.h>
#include <string.h>
#include <trace.h>

#define TIMEOUT_500US		(500 * 1000)

static uintptr_t get_ddrctrl_base(void)
{
	static void *va;

	if (!cpu_mmu_enabled()) {
		return DDRCTRL_BASE;
	}

	if (!va) {
		va = phys_to_virt(DDRCTRL_BASE, MEM_AREA_IO_SEC);
	}

	return (uintptr_t)va;
}

static uintptr_t get_ddrphy_base(void)
{
	static void *va;

	if (!cpu_mmu_enabled()) {
		return DDRPHYC_BASE;
	}

	if (!va) {
		va = phys_to_virt(DDRPHYC_BASE, MEM_AREA_IO_SEC);
	}

	return (uintptr_t)va;
}

static void ddr_disable_clock(void)
{
	uintptr_t rcc_base = stm32_rcc_base();

	/* Disable all clocks */
	mmio_clrbits_32(rcc_base + RCC_DDRITFCR,
			RCC_DDRITFCR_DDRC1EN |
			RCC_DDRITFCR_DDRC2EN |
			RCC_DDRITFCR_DDRPHYCAPBEN |
			RCC_DDRITFCR_DDRCAPBEN);
}

static void ddr_enable_clock(void)
{
	uintptr_t rcc_base = stm32_rcc_base();

	/* Enable all clocks */
	mmio_setbits_32(rcc_base + RCC_DDRITFCR,
			RCC_DDRITFCR_DDRC1EN |
			RCC_DDRITFCR_DDRC2EN |
			RCC_DDRITFCR_DDRPHYCEN |
			RCC_DDRITFCR_DDRPHYCAPBEN |
			RCC_DDRITFCR_DDRCAPBEN);
}

static void do_sw_handshake(void)
{
	uintptr_t ddrctrl_base = get_ddrctrl_base();

	mmio_clrbits_32(ddrctrl_base + DDRCTRL_SWCTL, DDRCTRL_SWCTL_SW_DONE);
}

static void do_sw_ack(void)
{
	uint64_t to_ref;
	uintptr_t ddrctrl_base = get_ddrctrl_base();

	mmio_setbits_32(ddrctrl_base + DDRCTRL_SWCTL, DDRCTRL_SWCTL_SW_DONE);

	to_ref = utimeout_init(TIMEOUT_500US);
	while ((mmio_read_32(ddrctrl_base + DDRCTRL_SWSTAT) &
		DDRCTRL_SWSTAT_SW_DONE_ACK) == 0U) {
		if (utimeout_elapsed(TIMEOUT_500US, to_ref)) {
			panic();
		}
	}
}

static int ddr_sw_self_refresh_in(void)
{
	uint64_t to_ref;
	uint32_t operating_mode;
	uint32_t selref_type;
	uint8_t op_mode_changed = 0;
	uintptr_t pwr_base = stm32_pwr_base();
	uintptr_t rcc_base = stm32_rcc_base();
	uintptr_t ddrctrl_base = get_ddrctrl_base();
	uintptr_t ddrphy_base = get_ddrphy_base();

	mmio_clrbits_32(rcc_base + RCC_DDRITFCR, RCC_DDRITFCR_AXIDCGEN);

	/* Blocks AXI ports from taking anymore transactions */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_PCTRL_0,
			DDRCTRL_PCTRL_N_PORT_EN);
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_PCTRL_1,
			DDRCTRL_PCTRL_N_PORT_EN);

	/*
	 * Waits unit all AXI ports are idle
	 * Poll PSTAT.rd_port_busy_n = 0
	 * Poll PSTAT.wr_port_busy_n = 0
	 */
	to_ref = utimeout_init(TIMEOUT_500US);
	while (mmio_read_32(ddrctrl_base + DDRCTRL_PSTAT)) {
		if (utimeout_elapsed(TIMEOUT_500US, to_ref)) {
			goto pstat_failed;
		}
	}

	/* SW Self-Refresh entry */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_SELFREF_SW);

	/*
	 * Wait operating mode change in self-refresh mode
	 * with STAT.operating_mode[1:0]==11.
	 * Ensure transition to self-refresh was due to software
	 * by checking also that STAT.selfref_type[1:0]=2.
	 */
	to_ref = utimeout_init(TIMEOUT_500US);
	while (!utimeout_elapsed(TIMEOUT_500US, to_ref)) {
		uint32_t stat = mmio_read_32(ddrctrl_base + DDRCTRL_STAT);

		operating_mode = stat & DDRCTRL_STAT_OPERATING_MODE_MASK;
		selref_type = stat & DDRCTRL_STAT_SELFREF_TYPE_MASK;

		if ((operating_mode == DDRCTRL_STAT_OPERATING_MODE_SR) &&
		    (selref_type == DDRCTRL_STAT_SELFREF_TYPE_SR)) {
			op_mode_changed = 1;
			break;
		}
	}

	if (op_mode_changed == 0U)
		goto selfref_sw_failed;

	/* IOs powering down (PUBL registers) */
	mmio_setbits_32(ddrphy_base + DDRPHYC_ACIOCR, DDRPHYC_ACIOCR_ACPDD);

	mmio_setbits_32(ddrphy_base + DDRPHYC_ACIOCR, DDRPHYC_ACIOCR_ACPDR);

	mmio_clrsetbits_32(ddrphy_base + DDRPHYC_ACIOCR,
			   DDRPHYC_ACIOCR_CKPDD_MASK,
			   DDRPHYC_ACIOCR_CKPDD_0);

	mmio_clrsetbits_32(ddrphy_base + DDRPHYC_ACIOCR,
			   DDRPHYC_ACIOCR_CKPDR_MASK,
			   DDRPHYC_ACIOCR_CKPDR_0);

	mmio_clrsetbits_32(ddrphy_base + DDRPHYC_ACIOCR,
			   DDRPHYC_ACIOCR_CSPDD_MASK,
			   DDRPHYC_ACIOCR_CSPDD_0);

	mmio_setbits_32(ddrphy_base + DDRPHYC_DXCCR, DDRPHYC_DXCCR_DXPDD);

	mmio_setbits_32(ddrphy_base + DDRPHYC_DXCCR, DDRPHYC_DXCCR_DXPDR);

	mmio_clrsetbits_32(ddrphy_base + DDRPHYC_DSGCR,
			   DDRPHYC_DSGCR_ODTPDD_MASK,
			   DDRPHYC_DSGCR_ODTPDD_0);

	mmio_setbits_32(ddrphy_base + DDRPHYC_DSGCR, DDRPHYC_DSGCR_NL2PD);

	mmio_clrsetbits_32(ddrphy_base + DDRPHYC_DSGCR,
			   DDRPHYC_DSGCR_CKEPDD_MASK,
			   DDRPHYC_DSGCR_CKEPDD_0);

	/* Disable PZQ cell (PUBL register) */
	mmio_setbits_32(ddrphy_base + DDRPHYC_ZQ0CR0, DDRPHYC_ZQ0CRN_ZQPD);

	/* Activate sw retention in PWRCTRL */
	mmio_setbits_32(pwr_base + PWR_CR3_OFF, PWR_CR3_DDRRETEN);

	/* Switch controller clocks (uMCTL2/PUBL) to DLL ref clock */
	mmio_setbits_32(rcc_base + RCC_DDRITFCR, RCC_DDRITFCR_GSKPCTRL);

	/* Disable all DLLs: GLITCH window */
	mmio_setbits_32(ddrphy_base + DDRPHYC_ACDLLCR,
			DDRPHYC_ACDLLCR_DLLDIS);

	mmio_setbits_32(ddrphy_base + DDRPHYC_DX0DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	mmio_setbits_32(ddrphy_base + DDRPHYC_DX1DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	mmio_setbits_32(ddrphy_base + DDRPHYC_DX2DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	mmio_setbits_32(ddrphy_base + DDRPHYC_DX3DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	/* Switch controller clocks (uMCTL2/PUBL) to DLL output clock */
	mmio_clrbits_32(rcc_base + RCC_DDRITFCR, RCC_DDRITFCR_GSKPCTRL);

	/* Disable all clocks */
	ddr_disable_clock();

	return 0;

selfref_sw_failed:
	/* This bit should be cleared to restore DDR in its previous state */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_SELFREF_SW);

pstat_failed:
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PCTRL_0,
			DDRCTRL_PCTRL_N_PORT_EN);
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PCTRL_1,
			DDRCTRL_PCTRL_N_PORT_EN);

	return -1;
}

int ddr_sw_self_refresh_exit(void)
{
	uint64_t to_ref;
	uintptr_t rcc_base = stm32_rcc_base();
	uintptr_t pwr_base = stm32_pwr_base();
	uintptr_t ddrctrl_base = get_ddrctrl_base();
	uintptr_t ddrphy_base = get_ddrphy_base();

	/* Enable all clocks */
	ddr_enable_clock();

	do_sw_handshake();

	/* Mask dfi_init_complete_en */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_DFIMISC,
			DDRCTRL_DFIMISC_DFI_INIT_COMPLETE_EN);

	do_sw_ack();

	/* Switch controller clocks (uMCTL2/PUBL) to DLL ref clock */
	mmio_setbits_32(rcc_base + RCC_DDRITFCR, RCC_DDRITFCR_GSKPCTRL);

	/* Enable all DLLs: GLITCH window */
	mmio_clrbits_32(ddrphy_base + DDRPHYC_ACDLLCR,
			DDRPHYC_ACDLLCR_DLLDIS);

	mmio_clrbits_32(ddrphy_base + DDRPHYC_DX0DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	mmio_clrbits_32(ddrphy_base + DDRPHYC_DX1DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	mmio_clrbits_32(ddrphy_base + DDRPHYC_DX2DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	mmio_clrbits_32(ddrphy_base + DDRPHYC_DX3DLLCR,
			DDRPHYC_DXNDLLCR_DLLDIS);

	/* Additional delay to avoid early DLL clock switch */
	udelay(10);

	/* Switch controller clocks (uMCTL2/PUBL) to DLL ref clock */
	mmio_clrbits_32(rcc_base + RCC_DDRITFCR, RCC_DDRITFCR_GSKPCTRL);
	mmio_clrbits_32(ddrphy_base + DDRPHYC_ACDLLCR, DDRPHYC_ACDLLCR_DLLSRST);

	udelay(10);

	mmio_setbits_32(ddrphy_base + DDRPHYC_ACDLLCR,
			DDRPHYC_ACDLLCR_DLLSRST);

	/* PHY partial init: (DLL lock and ITM reset) */
	mmio_write_32(ddrphy_base + DDRPHYC_PIR,
		      DDRPHYC_PIR_DLLSRST | DDRPHYC_PIR_DLLLOCK |
		      DDRPHYC_PIR_ITMSRST | DDRPHYC_PIR_INIT);

	/* Need to wait at least 10 clock cycles before accessing PGSR */
	udelay(1);

	to_ref = utimeout_init(TIMEOUT_500US);
	while ((mmio_read_32(ddrphy_base + DDRPHYC_PGSR) &
		DDRPHYC_PGSR_IDONE) == 0U) {
		if (utimeout_elapsed(TIMEOUT_500US, to_ref)) {
			return -1;
		}
	}

	do_sw_handshake();

	/* Unmask dfi_init_complete_en to uMCTL2 */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_DFIMISC,
			DDRCTRL_DFIMISC_DFI_INIT_COMPLETE_EN);

	do_sw_ack();

	/* Deactivate sw retention in PWR */
	mmio_clrbits_32(pwr_base + PWR_CR3_OFF, PWR_CR3_DDRRETEN);

	/* Enable PZQ cell (PUBL register) */
	mmio_clrbits_32(ddrphy_base + DDRPHYC_ZQ0CR0, DDRPHYC_ZQ0CRN_ZQPD);

	/* Enable pad drivers */
	mmio_clrbits_32(ddrphy_base + DDRPHYC_ACIOCR, DDRPHYC_ACIOCR_ACPDD);

	mmio_clrbits_32(ddrphy_base + DDRPHYC_ACIOCR,
			DDRPHYC_ACIOCR_CKPDD_MASK);

	mmio_clrbits_32(ddrphy_base + DDRPHYC_ACIOCR,
			DDRPHYC_ACIOCR_CSPDD_MASK);

	mmio_clrbits_32(ddrphy_base + DDRPHYC_DXCCR, DDRPHYC_DXCCR_DXPDD);

	mmio_clrbits_32(ddrphy_base + DDRPHYC_DXCCR, DDRPHYC_DXCCR_DXPDR);

	mmio_clrbits_32(ddrphy_base + DDRPHYC_DSGCR, DDRPHYC_DSGCR_ODTPDD_MASK);

	mmio_clrbits_32(ddrphy_base + DDRPHYC_DSGCR, DDRPHYC_DSGCR_NL2PD);

	mmio_clrbits_32(ddrphy_base + DDRPHYC_DSGCR, DDRPHYC_DSGCR_CKEPDD_MASK);

	/* Remove selfrefresh */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_SELFREF_SW);

	/* Wait operating_mode == normal */
	to_ref = utimeout_init(TIMEOUT_500US);

	while ((mmio_read_32(ddrctrl_base + DDRCTRL_STAT) &
		DDRCTRL_STAT_OPERATING_MODE_MASK) !=
	       DDRCTRL_STAT_OPERATING_MODE_NORMAL) {
		if (utimeout_elapsed(TIMEOUT_500US, to_ref)) {
			return -1;
		}
	}

	/* AXI ports are no longer blocked from taking transactions */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PCTRL_0,
			DDRCTRL_PCTRL_N_PORT_EN);
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PCTRL_1,
			DDRCTRL_PCTRL_N_PORT_EN);

	mmio_setbits_32(rcc_base + RCC_DDRITFCR, RCC_DDRITFCR_AXIDCGEN);

	return 0;
}

uint32_t get_ddrphy_calibration(void)
{
	uintptr_t ddrphy_base = get_ddrphy_base();
	uint32_t zcal = mmio_read_32(ddrphy_base + DDRPHYC_ZQ0CR0);

	return (zcal & DDRPHYC_ZQ0CRN_ZDATA_MASK) >> DDRPHYC_ZQ0CRN_ZDATA_SHIFT;
}

int ddr_standby_sr_entry(uint32_t *zq0cr0_zdata)
{
	uintptr_t pwr_base = stm32_pwr_base();
	uintptr_t ddrphy_base = get_ddrphy_base();

	/* Save IOs calibration values */
	if (zq0cr0_zdata != NULL) {
		*zq0cr0_zdata = mmio_read_32(ddrphy_base + DDRPHYC_ZQ0CR0) &
				DDRPHYC_ZQ0CRN_ZDATA_MASK;
	}

	/* Put DDR in Self-Refresh */
	if (ddr_sw_self_refresh_in() != 0) {
		return -1;
	}

	/* Enable I/O retention mode in standby */
	mmio_setbits_32(pwr_base + PWR_CR3_OFF, PWR_CR3_DDRSREN);

	return 0;
}

void ddr_sr_mode_ssr(void)
{
	uintptr_t rcc_ddritfcr = stm32_rcc_base() + RCC_DDRITFCR;
	uintptr_t ddrctrl_base = get_ddrctrl_base();

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC1LPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC2LPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC1EN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC2EN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRCAPBLPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRPHYCAPBLPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRCAPBEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRPHYCAPBEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRPHYCEN);

	mmio_clrbits_32(rcc_ddritfcr, RCC_DDRITFCR_AXIDCGEN);

	mmio_clrbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRCKMOD_MASK);

	/* Disable HW LP interface of uMCTL2 */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_HWLPCTL,
			DDRCTRL_HWLPCTL_HW_LP_EN);

	/* Configure Automatic LP modes of uMCTL2 */
	mmio_clrsetbits_32(ddrctrl_base + DDRCTRL_PWRTMG,
			   DDRCTRL_PWRTMG_SELFREF_TO_X32_MASK,
			   DDRCTRL_PWRTMG_SELFREF_TO_X32_0);

	/*
	 * Disable Clock disable with LP modes
	 * (used in RUN mode for LPDDR2 with specific timing).
	 */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_EN_DFI_DRAM_CLK_DISABLE);

	/* Disable automatic Self-Refresh mode */
	mmio_clrbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_SELFREF_EN);
}

void ddr_sr_mode_asr(void)
{
	uintptr_t rcc_ddritfcr = stm32_rcc_base() + RCC_DDRITFCR;
	uintptr_t ddrctrl_base = get_ddrctrl_base();

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_AXIDCGEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC1LPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC2LPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRPHYCLPEN);

	mmio_clrsetbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRCKMOD_MASK,
			   RCC_DDRITFCR_DDRCKMOD_ASR1);

	/* Enable HW LP interface of uMCTL2 */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_HWLPCTL,
			DDRCTRL_HWLPCTL_HW_LP_EN);

	/* Configure Automatic LP modes of uMCTL2 */
	mmio_clrsetbits_32(ddrctrl_base + DDRCTRL_PWRTMG,
			   DDRCTRL_PWRTMG_SELFREF_TO_X32_MASK,
			   DDRCTRL_PWRTMG_SELFREF_TO_X32_0);

	/*
	 * Enable Clock disable with LP modes
	 * (used in RUN mode for LPDDR2 with specific timing).
	 */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_EN_DFI_DRAM_CLK_DISABLE);

	/* Enable automatic Self-Refresh for ASR mode */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_SELFREF_EN);
}

void ddr_sr_mode_hsr(void)
{
	uintptr_t rcc_ddritfcr = stm32_rcc_base() + RCC_DDRITFCR;
	uintptr_t ddrctrl_base = get_ddrctrl_base();

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_AXIDCGEN);

	mmio_clrbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC1LPEN);

	mmio_clrbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRC2LPEN);

	mmio_setbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRPHYCLPEN);

	mmio_clrsetbits_32(rcc_ddritfcr, RCC_DDRITFCR_DDRCKMOD_MASK,
			   RCC_DDRITFCR_DDRCKMOD_HSR1);

	/* Enable HW LP interface of uMCTL2 */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_HWLPCTL,
			DDRCTRL_HWLPCTL_HW_LP_EN);

	/* Configure Automatic LP modes of uMCTL2 */
	mmio_clrsetbits_32(ddrctrl_base + DDRCTRL_PWRTMG,
			   DDRCTRL_PWRTMG_SELFREF_TO_X32_MASK,
			   DDRCTRL_PWRTMG_SELFREF_TO_X32_0);

	/*
	 * Enable Clock disable with LP modes
	 * (used in RUN mode for LPDDR2 with specific timing).
	 */
	mmio_setbits_32(ddrctrl_base + DDRCTRL_PWRCTL,
			DDRCTRL_PWRCTL_EN_DFI_DRAM_CLK_DISABLE);
}

