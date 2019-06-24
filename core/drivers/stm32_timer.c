// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2018, STMicroelectronics
 */

#include <arm.h>
#include <drivers/serial.h>
#include <drivers/stm32_timer.h>
#include <initcall.h>
#include <keep.h>
#include <kernel/delay.h>
#include <kernel/dt.h>
#include <kernel/generic_boot.h>
#include <kernel/panic.h>
#include <libfdt.h>
#include <mm/core_memprot.h>
#include <stm32_util.h>
#include <stm32mp_dt.h>

#define TIM_CR1			0x00U		/* Control Register 1      */
#define TIM_CR2			0x04U		/* Control Register 2      */
#define TIM_SMCR		0x08U		/* Slave mode control reg  */
#define TIM_DIER		0x0CU		/* DMA/interrupt register  */
#define TIM_SR			0x10U		/* Status register	   */
#define TIM_EGR			0x14U		/* Event Generation Reg    */
#define TIM_CCMR1		0x18U		/* Capt/Comp 1 Mode Reg    */
#define TIM_CCMR2		0x1CU		/* Capt/Comp 2 Mode Reg    */
#define TIM_CCER		0x20U		/* Capt/Comp Enable Reg    */
#define TIM_CNT			0x24U		/* Counter		   */
#define TIM_PSC			0x28U		/* Prescaler               */
#define TIM_ARR			0x2CU		/* Auto-Reload Register    */
#define TIM_CCR1		0x34U		/* Capt/Comp Register 1    */
#define TIM_CCR2		0x38U		/* Capt/Comp Register 2    */
#define TIM_CCR3		0x3CU		/* Capt/Comp Register 3    */
#define TIM_CCR4		0x40U		/* Capt/Comp Register 4    */
#define TIM_BDTR		0x44U		/* Break and Dead-Time Reg */
#define TIM_DCR			0x48U		/* DMA control register    */
#define TIM_DMAR		0x4CU		/* DMA transfer register   */
#define TIM_AF1			0x60U		/* Alt Function Reg 1      */
#define TIM_AF2			0x64U		/* Alt Function Reg 2      */
#define TIM_TISEL		0x68U		/* Input Selection         */

#define TIM_CR1_CEN		BIT(0)
#define TIM_SMCR_SMS		GENMASK_32(2, 0) /* Slave mode selection */
#define TIM_SMCR_TS		GENMASK_32(6, 4) /* Trigger selection */
#define TIM_CCMR_CC1S_TI1	BIT(0)		/* IC1/IC3 selects TI1/TI3 */
#define TIM_CCMR_CC1S_TI2	BIT(1)		/* IC1/IC3 selects TI2/TI4 */
#define TIM_CCMR_CC2S_TI2	BIT(8)		/* IC2/IC4 selects TI2/TI4 */
#define TIM_CCMR_CC2S_TI1	BIT(9)		/* IC2/IC4 selects TI1/TI3 */
#define TIM_CCER_CC1E		BIT(0)		/* Capt/Comp 1  out Ena    */
#define TIM_CCER_CC1P		BIT(1)		/* Capt/Comp 1  Polarity   */
#define TIM_CCER_CC1NP		BIT(3)		/* Capt/Comp 1N Polarity   */
#define TIM_SR_UIF		BIT(0)		/* UIF interrupt flag      */
#define TIM_SR_CC1IF		BIT(1)		/* CC1 interrupt flag      */
#define TIM_TISEL_TI1SEL_MASK	GENMASK_32(3, 0)
#define TIM_SMCR_SMS_RESET	0x4U
#define TIM_SMCR_TS_SHIFT	4U
#define TIM_SMCR_TS_TI1FP1	0x5U

#define TIM_COMPAT		"st,stm32-timers"
#define TIM_TIMEOUT_US		100000
#define TIM_TIMEOUT_STEP_US	10
#define TIM_PRESCAL_HSI		10U
#define TIM_PRESCAL_CSI		7U
#define TIM_MIN_FREQ_CALIB	50000000U


struct stm32_timer_instance {
	struct io_pa_va base;
	unsigned long clk;
	unsigned long freq;
	uint8_t cal_input;
};

static uintptr_t timer_base(struct stm32_timer_instance *timer)
{
	return (uintptr_t)io_pa_or_va(&timer->base);
}

/* Currently support HSI and CSI calibratrion */
#define TIM_MAX_INSTANCE	2

static struct stm32_timer_instance stm32_timer[TIM_MAX_INSTANCE];

static int timer_get_dt_node(void *fdt, struct dt_node_info *info, int offset)
{
	int node;

	node = fdt_get_node(fdt, info, offset, TIM_COMPAT);
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	return node;
}

static int timer_config(struct stm32_timer_instance *timer)
{
	uintptr_t base = timer_base(timer);

	stm32_clock_enable(timer->clk);

	timer->freq = stm32_clock_get_rate(timer->clk);
	if (timer->freq < TIM_MIN_FREQ_CALIB) {
		EMSG("Timer is not accurate enough for calibration");
		stm32_clock_disable(timer->clk);
		return -1;
	}

	if ((mmio_read_32(base + TIM_TISEL) & TIM_TISEL_TI1SEL_MASK) !=
	    timer->cal_input) {
		mmio_clrsetbits_32(base + TIM_CCMR1,
				   TIM_CCMR_CC1S_TI1 | TIM_CCMR_CC1S_TI2,
				   TIM_CCMR_CC1S_TI1);

		mmio_clrbits_32(base + TIM_CCER,
				TIM_CCER_CC1P | TIM_CCER_CC1NP);

		mmio_clrsetbits_32(base + TIM_SMCR,
				   TIM_SMCR_TS | TIM_SMCR_SMS,
				   (TIM_SMCR_TS_TI1FP1 << TIM_SMCR_TS_SHIFT) |
				   TIM_SMCR_SMS_RESET);

		mmio_write_32(base + TIM_TISEL, timer->cal_input);
		mmio_setbits_32(base + TIM_CR1, TIM_CR1_CEN);
		mmio_setbits_32(base + TIM_CCER, TIM_CCER_CC1E);
	}

	stm32_clock_disable(timer->clk);
	return 0;
}

static uint32_t timer_start_capture(struct stm32_timer_instance *timer)
{
	uint64_t timeout_ref;
	uint32_t counter = 0U;
	uint32_t old_counter = 0U;
	int twice;
	uintptr_t base = timer_base(timer);

	if (timer_config(timer) < 0)
		return 0U;

	stm32_clock_enable(timer->clk);

	mmio_write_32(base + TIM_SR, 0U);

	timeout_ref = utimeout_init(TIM_TIMEOUT_US);

	while ((mmio_read_32(base + TIM_SR) & TIM_SR_UIF) == 0U) {
		if (utimeout_elapsed(TIM_TIMEOUT_US, timeout_ref)) {
			goto bail;
		}
	}

	mmio_write_32(base + TIM_SR, 0U);

	for (twice = 0; (twice < 2) || (counter != old_counter); twice++) {
		timeout_ref = utimeout_init(TIM_TIMEOUT_US);

		while ((mmio_read_32(base + TIM_SR) & TIM_SR_CC1IF) == 0U) {
			if (utimeout_elapsed(TIM_TIMEOUT_US, timeout_ref)) {
				counter = 0U;
				goto bail;
			}
		}

		old_counter = counter;
		counter = mmio_read_32(base + TIM_CCR1);
	}

bail:
	stm32_clock_disable(timer->clk);

	return counter;
}

unsigned long stm32_timer_hsi_freq(void)
{
	struct stm32_timer_instance *timer = &stm32_timer[HSI_CAL];
	uint32_t counter = 0;

	if (timer->base.pa != 0) {
		counter = timer_start_capture(timer);
	}

	return (counter == 0) ? 0 : (timer->freq / counter) << TIM_PRESCAL_HSI;
}
KEEP_PAGER(stm32_timer_hsi_freq);

unsigned long stm32_timer_csi_freq(void)
{
	struct stm32_timer_instance *timer = &stm32_timer[CSI_CAL];
	uint32_t counter = 0;

	if (timer->base.pa != 0) {
		counter = timer_start_capture(timer);
	}

	return (counter == 0U) ? 0 : (timer->freq / counter) << TIM_PRESCAL_CSI;
}
KEEP_PAGER(stm32_timer_csi_freq);

static void _init_stm32_timer(void)
{
	void *fdt = get_dt_blob();
	struct dt_node_info dt_timer;
	int node = -1;
	static bool inited;

	if (inited)
		return;
	inited = true;

	if (!fdt) {
		panic();
	}

	for (node = timer_get_dt_node(fdt, &dt_timer, node);
	     node != -FDT_ERR_NOTFOUND;
	     node = timer_get_dt_node(fdt, &dt_timer, node)) {
		struct stm32_timer_instance *timer;
		const uint32_t *cuint;

		if (!(dt_timer.status & DT_STATUS_OK_SEC)) {
			continue;
		}

		cuint = fdt_getprop(fdt, node, "st,hsi-cal-input", NULL);
		if (cuint != NULL) {
			timer = &stm32_timer[HSI_CAL];
			timer->base.pa = dt_timer.base;
			timer->clk = dt_timer.clock;
			timer->freq = stm32_clock_get_rate(timer->clk);
			timer->cal_input = (uint8_t)fdt32_to_cpu(*cuint);
			if (timer_config(timer) < 0) {
				timer->base.pa = 0;
				continue;
			}
		}

		cuint = fdt_getprop(fdt, node, "st,csi_cal-input", NULL);
		if (cuint != NULL) {
			timer = &stm32_timer[CSI_CAL];
			timer->base.pa = dt_timer.base;
			timer->clk = dt_timer.clock;
			timer->freq = stm32_clock_get_rate(timer->clk);
			timer->cal_input = (uint8_t)fdt32_to_cpu(*cuint);
			if (timer_config(timer) < 0) {
				timer->base.pa = 0;
				continue;
			}
		}
	}
}

void stm32_timer_freq_func(unsigned long (**timer_freq_cb)(void),
			   enum timer_cal type)
{
	_init_stm32_timer();

	*timer_freq_cb = NULL;

	switch (type) {
	case HSI_CAL:
		if (stm32_timer[HSI_CAL].base.pa != 0) {
			*timer_freq_cb = stm32_timer_hsi_freq;
		}
		break;

	case CSI_CAL:
		if (stm32_timer[CSI_CAL].base.pa != 0) {
			*timer_freq_cb = stm32_timer_csi_freq;
		}
		break;
	default:
		panic();
	}
}

static TEE_Result init_stm32_timer(void)
{
	_init_stm32_timer();

	return TEE_SUCCESS;
}
driver_init(init_stm32_timer);
