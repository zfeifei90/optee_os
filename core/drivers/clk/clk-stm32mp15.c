// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0+)
/*
 * Copyright (C) 2018-2019, STMicroelectronics
 */

#include <assert.h>
#include <config.h>
#include <drivers/stm32mp1_rcc.h>
#include <drivers/clk.h>
#include <drivers/clk_dt.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <dt-bindings/clock/stm32mp1-clksrc.h>
#include <initcall.h>
#include <io.h>
#include <keep.h>
#include <kernel/delay.h>
#include <kernel/dt.h>
#include <kernel/boot.h>
#include <kernel/panic.h>
#include <kernel/spinlock.h>
#include <libfdt.h>
#include <platform_config.h>
#include <stdio.h>
#include <stm32_util.h>
#include <trace.h>
#include <util.h>

#define DT_OPP_COMPAT		"operating-points-v2"

/* PLL settings computation related definitions */
#define POST_DIVM_MIN	8000000
#define POST_DIVM_MAX	16000000
#define DIVM_MIN	0
#define DIVM_MAX	63
#define DIVN_MIN	24
#define DIVN_MAX	99
#define DIVP_MIN	0
#define DIVP_MAX	127
#define FRAC_MAX	8192
#define VCO_MIN		800000000
#define VCO_MAX		1600000000

#define PLL1_SETTINGS_VALID_ID		0x504C4C31 /* "PLL1" */

/* Identifiers for root oscillators */
enum stm32mp_osc_id {
	OSC_HSI,
	OSC_HSE,
	OSC_CSI,
	OSC_LSI,
	OSC_LSE,
	OSC_I2S_CKIN,
	OSC_USB_PHY_48,
	NB_OSC,
	_UNKNOWN_OSC_ID = 0xffU
};

/* Identifiers for parent clocks */
enum stm32mp1_parent_id {
	_HSI,
	_HSE,
	_CSI,
	_LSI,
	_LSE,
	_I2S_CKIN,
	_USB_PHY_48,
	_HSI_KER,
	_HSE_KER,
	_HSE_KER_DIV2,
	_HSE_RTC,
	_CSI_KER,
	_PLL1_P,
	_PLL1_Q,
	_PLL1_R,
	_PLL2_P,
	_PLL2_Q,
	_PLL2_R,
	_PLL3_P,
	_PLL3_Q,
	_PLL3_R,
	_PLL4_P,
	_PLL4_Q,
	_PLL4_R,
	_ACLK,
	_PCLK1,
	_PCLK2,
	_PCLK3,
	_PCLK4,
	_PCLK5,
	_HCLK5,
	_HCLK6,
	_HCLK2,
	_CK_PER,
	_CK_MPU,
	_CK_MCU,
	_PARENT_NB,
	_UNKNOWN_ID = 0xff,
};

/*
 * Identifiers for parent clock selectors.
 * This enum lists only the parent clocks we are interested in.
 */
enum stm32mp1_parent_sel {
	_STGEN_SEL,
	_I2C46_SEL,
	_SPI6_SEL,
	_USART1_SEL,
	_RNG1_SEL,
	_UART6_SEL,
	_UART24_SEL,
	_UART35_SEL,
	_UART78_SEL,
	_AXISS_SEL,
	_MCUSS_SEL,
	_USBPHY_SEL,
	_USBO_SEL,
	_RTC_SEL,
	_MPU_SEL,
	_PARENT_SEL_NB,
	_UNKNOWN_SEL = 0xff,
};

static const uint8_t parent_id_clock_id[_PARENT_NB] = {
	[_HSE] = CK_HSE,
	[_HSI] = CK_HSI,
	[_CSI] = CK_CSI,
	[_LSE] = CK_LSE,
	[_LSI] = CK_LSI,
	[_I2S_CKIN] = _UNKNOWN_ID,
	[_USB_PHY_48] = _UNKNOWN_ID,
	[_HSI_KER] = CK_HSI,
	[_HSE_KER] = CK_HSE,
	[_HSE_KER_DIV2] = CK_HSE_DIV2,
	[_HSE_RTC] = _UNKNOWN_ID,
	[_CSI_KER] = CK_CSI,
	[_PLL1_P] = PLL1_P,
	[_PLL1_Q] = PLL1_Q,
	[_PLL1_R] = PLL1_R,
	[_PLL2_P] = PLL2_P,
	[_PLL2_Q] = PLL2_Q,
	[_PLL2_R] = PLL2_R,
	[_PLL3_P] = PLL3_P,
	[_PLL3_Q] = PLL3_Q,
	[_PLL3_R] = PLL3_R,
	[_PLL4_P] = PLL4_P,
	[_PLL4_Q] = PLL4_Q,
	[_PLL4_R] = PLL4_R,
	[_ACLK] = CK_AXI,
	[_PCLK1] = CK_AXI,
	[_PCLK2] = CK_AXI,
	[_PCLK3] = CK_AXI,
	[_PCLK4] = CK_AXI,
	[_PCLK5] = CK_AXI,
	[_HCLK5] = CK_AXI,
	[_HCLK6] = CK_AXI,
	[_HCLK2] = CK_AXI,
	[_CK_PER] = CK_PER,
	[_CK_MPU] = CK_MPU,
	[_CK_MCU] = CK_MCU,
};

static enum stm32mp1_parent_id osc_id2parent_id(enum stm32mp_osc_id osc_id)
{
	assert(osc_id >= OSC_HSI && osc_id < NB_OSC);
	COMPILE_TIME_ASSERT((int)OSC_HSI == (int)_HSI &&
			    (int)OSC_HSE == (int)_HSE &&
			    (int)OSC_CSI == (int)_CSI &&
			    (int)OSC_LSI == (int)_LSI &&
			    (int)OSC_LSE == (int)_LSE &&
			    (int)OSC_I2S_CKIN == (int)_I2S_CKIN &&
			    (int)OSC_USB_PHY_48 == (int)_USB_PHY_48);

	return (enum stm32mp1_parent_id)osc_id;
}

static enum stm32mp1_parent_id clock_id2parent_id(unsigned long id)
{
	size_t n = 0;

	COMPILE_TIME_ASSERT(STM32MP1_LAST_CLK < _UNKNOWN_ID);

	for (n = 0; n < ARRAY_SIZE(parent_id_clock_id); n++)
		if (parent_id_clock_id[n] == id)
			return (enum stm32mp1_parent_id)n;

	return _UNKNOWN_ID;
}

/* Identifiers for PLLs and their configuration resources */
enum stm32mp1_pll_id {
	_PLL1,
	_PLL2,
	_PLL3,
	_PLL4,
	_PLL_NB
};

enum stm32mp1_div_id {
	_DIV_P,
	_DIV_Q,
	_DIV_R,
	_DIV_NB,
};

enum stm32mp1_pllcfg {
	PLLCFG_M,
	PLLCFG_N,
	PLLCFG_P,
	PLLCFG_Q,
	PLLCFG_R,
	PLLCFG_O,
	PLLCFG_NB
};

enum stm32mp1_plltype {
	PLL_800,
	PLL_1600,
	PLL_TYPE_NB
};

/*
 * Clock generic gates clocks which state is controlled by a single RCC bit
 *
 * @offset: RCC register byte offset from RCC base where clock is controlled
 * @bit: Bit position in the RCC 32bit register
 * @clock_id: Identifier used for the clock in the clock driver API
 * @set_clr: Non-null if and only-if RCC register is a CLEAR/SET register
 *	(CLEAR register is at offset RCC_MP_ENCLRR_OFFSET from SET register)
 * @secure: One of N_S or SEC, defined below
 * @sel: _UNKNOWN_ID (fixed parent) or reference to parent clock selector
 *	(8bit storage of ID from enum stm32mp1_parent_sel)
 * @fixed: _UNKNOWN_ID (selectable paranet) or reference to parent clock
 *	(8bit storage of ID from enum stm32mp1_parent_id)
 */
struct stm32mp1_clk_gate {
	uint16_t offset;
	uint8_t bit;
	uint8_t clock_id;
	uint8_t set_clr;
	uint8_t secure;
	uint8_t sel; /* Relates to enum stm32mp1_parent_sel */
	uint8_t fixed; /* Relates to enum stm32mp1_parent_id */
};

/* Parent clock selection: select register info, parent clocks references */
struct stm32mp1_clk_sel {
	uint16_t offset;
	uint8_t src;
	uint8_t msk;
	uint8_t nb_parent;
	const uint8_t *parent;
};

#define REFCLK_SIZE 4
/* PLL control: type, control register offsets, up-to-4 selectable parent */
struct stm32mp1_clk_pll {
	enum stm32mp1_plltype plltype;
	uint16_t rckxselr;
	uint16_t pllxcfgr1;
	uint16_t pllxcfgr2;
	uint16_t pllxfracr;
	uint16_t pllxcr;
	uint16_t pllxcsgr;
	enum stm32mp_osc_id refclk[REFCLK_SIZE];
};

struct stm32mp1_pll {
	uint8_t refclk_min;
	uint8_t refclk_max;
	uint8_t divn_max;
};

/* Compact structure of 32bit cells, copied raw when suspending */
struct stm32mp1_pll_settings {
	uint32_t valid_id;
	uint32_t freq[PLAT_MAX_OPP_NB];
	uint32_t volt[PLAT_MAX_OPP_NB];
	uint32_t cfg[PLAT_MAX_OPP_NB][PLAT_MAX_PLLCFG_NB];
	uint32_t frac[PLAT_MAX_OPP_NB];
};

#define N_S	0	/* Non-secure can access RCC interface */
#define SEC	1	/* RCC[TZEN] protects RCC interface */

/* Clocks with selectable source and not set/clr register access */
#define _CLK_SELEC(_sec, _offset, _bit, _clock_id, _parent_sel)	\
	{							\
		.offset = (_offset),				\
		.bit = (_bit),					\
		.clock_id = (_clock_id),			\
		.set_clr = 0,					\
		.secure = (_sec),				\
		.sel = (_parent_sel),				\
		.fixed = _UNKNOWN_ID,				\
	}

/* Clocks with fixed source and not set/clr register access */
#define _CLK_FIXED(_sec, _offset, _bit, _clock_id, _parent)		\
	{							\
		.offset = (_offset),				\
		.bit = (_bit),					\
		.clock_id = (_clock_id),			\
		.set_clr = 0,					\
		.secure = (_sec),				\
		.sel = _UNKNOWN_SEL,				\
		.fixed = (_parent),				\
	}

/* Clocks with selectable source and set/clr register access */
#define _CLK_SC_SELEC(_sec, _offset, _bit, _clock_id, _parent_sel)	\
	{							\
		.offset = (_offset),				\
		.bit = (_bit),					\
		.clock_id = (_clock_id),			\
		.set_clr = 1,					\
		.secure = (_sec),				\
		.sel = (_parent_sel),				\
		.fixed = _UNKNOWN_ID,				\
	}

/* Clocks with fixed source and set/clr register access */
#define _CLK_SC_FIXED(_sec, _offset, _bit, _clock_id, _parent)	\
	{							\
		.offset = (_offset),				\
		.bit = (_bit),					\
		.clock_id = (_clock_id),			\
		.set_clr = 1,					\
		.secure = (_sec),				\
		.sel = _UNKNOWN_SEL,				\
		.fixed = (_parent),				\
	}

/*
 * Clocks with selectable source and set/clr register access
 * and enable bit position defined by a label (argument _bit)
 */
#define _CLK_SC2_SELEC(_sec, _offset, _bit, _clock_id, _parent_sel)	\
	{							\
		.offset = (_offset),				\
		.clock_id = (_clock_id),			\
		.bit = _offset ## _ ## _bit ## _POS,		\
		.set_clr = 1,					\
		.secure = (_sec),				\
		.sel = (_parent_sel),				\
		.fixed = _UNKNOWN_ID,				\
	}
#define _CLK_SC2_FIXED(_sec, _offset, _bit, _clock_id, _parent)	\
	{							\
		.offset = (_offset),				\
		.clock_id = (_clock_id),			\
		.bit = _offset ## _ ## _bit ## _POS,		\
		.set_clr = 1,					\
		.secure = (_sec),				\
		.sel = _UNKNOWN_SEL,				\
		.fixed = (_parent),				\
	}

#define _CLK_PARENT(idx, _offset, _src, _mask, _parent)		\
	[(idx)] = {						\
		.offset = (_offset),				\
		.src = (_src),					\
		.msk = (_mask),					\
		.parent = (_parent),				\
		.nb_parent = ARRAY_SIZE(_parent)		\
	}

#define _CLK_PLL(_idx, _type, _off1, _off2, _off3, _off4,	\
		 _off5, _off6, _p1, _p2, _p3, _p4)		\
	[(_idx)] = {						\
		.plltype = (_type),				\
		.rckxselr = (_off1),				\
		.pllxcfgr1 = (_off2),				\
		.pllxcfgr2 = (_off3),				\
		.pllxfracr = (_off4),				\
		.pllxcr = (_off5),				\
		.pllxcsgr = (_off6),				\
		.refclk[0] = (_p1),				\
		.refclk[1] = (_p2),				\
		.refclk[2] = (_p3),				\
		.refclk[3] = (_p4),				\
	}

#define NB_GATES	ARRAY_SIZE(stm32mp1_clk_gate)

static const struct stm32mp1_clk_gate stm32mp1_clk_gate[] = {
	_CLK_FIXED(SEC, RCC_DDRITFCR, 0, DDRC1, _ACLK),
	_CLK_FIXED(SEC, RCC_DDRITFCR, 1, DDRC1LP, _ACLK),
	_CLK_FIXED(SEC, RCC_DDRITFCR, 2, DDRC2, _ACLK),
	_CLK_FIXED(SEC, RCC_DDRITFCR, 3, DDRC2LP, _ACLK),
	_CLK_FIXED(SEC, RCC_DDRITFCR, 4, DDRPHYC, _PLL2_R),
	_CLK_FIXED(SEC, RCC_DDRITFCR, 5, DDRPHYCLP, _PLL2_R),
	_CLK_FIXED(SEC, RCC_DDRITFCR, 6, DDRCAPB, _PCLK4),
	_CLK_FIXED(SEC, RCC_DDRITFCR, 7, DDRCAPBLP, _PCLK4),
	_CLK_FIXED(SEC, RCC_DDRITFCR, 8, AXIDCG, _ACLK),
	_CLK_FIXED(SEC, RCC_DDRITFCR, 9, DDRPHYCAPB, _PCLK4),
	_CLK_FIXED(SEC, RCC_DDRITFCR, 10, DDRPHYCAPBLP, _PCLK4),

	_CLK_SC2_SELEC(SEC, RCC_MP_APB5ENSETR, SPI6EN, SPI6_K, _SPI6_SEL),
	_CLK_SC2_SELEC(SEC, RCC_MP_APB5ENSETR, I2C4EN, I2C4_K, _I2C46_SEL),
	_CLK_SC2_SELEC(SEC, RCC_MP_APB5ENSETR, I2C6EN, I2C6_K, _I2C46_SEL),
	_CLK_SC2_SELEC(SEC, RCC_MP_APB5ENSETR, USART1EN, USART1_K, _USART1_SEL),
	_CLK_SC2_FIXED(SEC, RCC_MP_APB5ENSETR, RTCAPBEN, RTCAPB, _PCLK5),
	_CLK_SC2_FIXED(SEC, RCC_MP_APB5ENSETR, TZC1EN, TZC1, _PCLK5),
	_CLK_SC2_FIXED(SEC, RCC_MP_APB5ENSETR, TZC2EN, TZC2, _PCLK5),
	_CLK_SC2_FIXED(SEC, RCC_MP_APB5ENSETR, TZPCEN, TZPC, _PCLK5),
	_CLK_SC2_FIXED(SEC, RCC_MP_APB5ENSETR, IWDG1APBEN, IWDG1, _PCLK5),
	_CLK_SC2_FIXED(SEC, RCC_MP_APB5ENSETR, BSECEN, BSEC, _PCLK5),
	_CLK_SC2_SELEC(SEC, RCC_MP_APB5ENSETR, STGENEN, STGEN_K, _STGEN_SEL),

	_CLK_SC2_FIXED(SEC, RCC_MP_AHB5ENSETR, GPIOZEN, GPIOZ, _HCLK5),
	_CLK_SC2_FIXED(SEC, RCC_MP_AHB5ENSETR, CRYP1EN, CRYP1, _HCLK5),
	_CLK_SC2_FIXED(SEC, RCC_MP_AHB5ENSETR, HASH1EN, HASH1, _HCLK5),
	_CLK_SC2_SELEC(SEC, RCC_MP_AHB5ENSETR, RNG1EN, RNG1_K, _RNG1_SEL),
	_CLK_SC2_FIXED(SEC, RCC_MP_AHB5ENSETR, BKPSRAMEN, BKPSRAM, _HCLK5),

	_CLK_SC2_FIXED(SEC, RCC_MP_TZAHB6ENSETR, MDMA, MDMA, _HCLK6),

	_CLK_SELEC(SEC, RCC_BDCR, RCC_BDCR_RTCCKEN_POS, RTC, _RTC_SEL),

	/* Non-secure clocks */
#ifdef CFG_WITH_NSEC_GPIOS
	_CLK_SC_FIXED(N_S, RCC_MP_AHB4ENSETR, 0, GPIOA, _UNKNOWN_ID),
	_CLK_SC_FIXED(N_S, RCC_MP_AHB4ENSETR, 1, GPIOB, _UNKNOWN_ID),
	_CLK_SC_FIXED(N_S, RCC_MP_AHB4ENSETR, 2, GPIOC, _UNKNOWN_ID),
	_CLK_SC_FIXED(N_S, RCC_MP_AHB4ENSETR, 3, GPIOD, _UNKNOWN_ID),
	_CLK_SC_FIXED(N_S, RCC_MP_AHB4ENSETR, 4, GPIOE, _UNKNOWN_ID),
	_CLK_SC_FIXED(N_S, RCC_MP_AHB4ENSETR, 5, GPIOF, _UNKNOWN_ID),
	_CLK_SC_FIXED(N_S, RCC_MP_AHB4ENSETR, 6, GPIOG, _UNKNOWN_ID),
	_CLK_SC_FIXED(N_S, RCC_MP_AHB4ENSETR, 7, GPIOH, _UNKNOWN_ID),
	_CLK_SC_FIXED(N_S, RCC_MP_AHB4ENSETR, 8, GPIOI, _UNKNOWN_ID),
	_CLK_SC_FIXED(N_S, RCC_MP_AHB4ENSETR, 9, GPIOJ, _UNKNOWN_ID),
	_CLK_SC_FIXED(N_S, RCC_MP_AHB4ENSETR, 10, GPIOK, _UNKNOWN_ID),
#endif
	_CLK_SC_FIXED(N_S, RCC_MP_APB1ENSETR, 6, TIM12_K, _PCLK1),
#ifdef CFG_WITH_NSEC_UARTS
	_CLK_SC_SELEC(N_S, RCC_MP_APB1ENSETR, 14, USART2_K, _UART24_SEL),
	_CLK_SC_SELEC(N_S, RCC_MP_APB1ENSETR, 15, USART3_K, _UART35_SEL),
	_CLK_SC_SELEC(N_S, RCC_MP_APB1ENSETR, 16, UART4_K, _UART24_SEL),
	_CLK_SC_SELEC(N_S, RCC_MP_APB1ENSETR, 17, UART5_K, _UART35_SEL),
	_CLK_SC_SELEC(N_S, RCC_MP_APB1ENSETR, 18, UART7_K, _UART78_SEL),
	_CLK_SC_SELEC(N_S, RCC_MP_APB1ENSETR, 19, UART8_K, _UART78_SEL),
#endif
	_CLK_SC_FIXED(N_S, RCC_MP_APB2ENSETR, 2, TIM15_K, _PCLK2),
#ifdef CFG_WITH_NSEC_UARTS
	_CLK_SC_SELEC(N_S, RCC_MP_APB2ENSETR, 13, USART6_K, _UART6_SEL),
#endif
	_CLK_SC_FIXED(N_S, RCC_MP_APB3ENSETR, 11, SYSCFG, _UNKNOWN_ID),
	_CLK_SC_SELEC(N_S, RCC_MP_APB4ENSETR, 8, DDRPERFM, _UNKNOWN_SEL),
	_CLK_SC_SELEC(N_S, RCC_MP_APB4ENSETR, 15, IWDG2, _UNKNOWN_SEL),

	_CLK_SELEC(N_S, RCC_DBGCFGR, 8, CK_DBG, _UNKNOWN_SEL),
};
DECLARE_KEEP_PAGER(stm32mp1_clk_gate);

const uint8_t stm32mp1_clk_on[] = {
	CK_HSE, CK_CSI, CK_LSI, CK_LSE, CK_HSI, CK_HSE_DIV2,
	PLL1_P, PLL1_Q, PLL1_R, PLL2_P, PLL2_Q, PLL2_R, PLL3_P, PLL3_Q, PLL3_R,
	CK_AXI, CK_MPU, CK_MCU,
};

/* Parents for secure aware clocks in the xxxSELR value ordering */
static const uint8_t stgen_parents[] = {
	_HSI_KER, _HSE_KER
};

static const uint8_t i2c46_parents[] = {
	_PCLK5, _PLL3_Q, _HSI_KER, _CSI_KER
};

static const uint8_t spi6_parents[] = {
	_PCLK5, _PLL4_Q, _HSI_KER, _CSI_KER, _HSE_KER, _PLL3_Q
};

static const uint8_t usart1_parents[] = {
	_PCLK5, _PLL3_Q, _HSI_KER, _CSI_KER, _PLL4_Q, _HSE_KER
};

static const uint8_t rng1_parents[] = {
	_CSI, _PLL4_R, _LSE, _LSI
};

static const uint8_t mpu_parents[] = {
	_HSI, _HSE, _PLL1_P, _PLL1_P /* specific div */
};

/* Parents for (some) non-secure clocks */
#ifdef CFG_WITH_NSEC_UARTS
static const uint8_t uart6_parents[] = {
	_PCLK2, _PLL4_Q, _HSI_KER, _CSI_KER, _HSE_KER
};

static const uint8_t uart234578_parents[] = {
	_PCLK1, _PLL4_Q, _HSI_KER, _CSI_KER, _HSE_KER
};
#endif

static const uint8_t axiss_parents[] = {
	_HSI, _HSE, _PLL2_P
};

static const uint8_t mcuss_parents[] = {
	_HSI, _HSE, _CSI, _PLL3_P
};

static const uint8_t rtc_parents[] = {
	_UNKNOWN_ID, _LSE, _LSI, _HSE_RTC
};

static const struct stm32mp1_clk_sel stm32mp1_clk_sel[_PARENT_SEL_NB] = {
	/* Secure aware clocks */
	_CLK_PARENT(_STGEN_SEL, RCC_STGENCKSELR, 0, 0x3, stgen_parents),
	_CLK_PARENT(_I2C46_SEL, RCC_I2C46CKSELR, 0, 0x7, i2c46_parents),
	_CLK_PARENT(_SPI6_SEL, RCC_SPI6CKSELR, 0, 0x7, spi6_parents),
	_CLK_PARENT(_USART1_SEL, RCC_UART1CKSELR, 0, 0x7, usart1_parents),
	_CLK_PARENT(_RNG1_SEL, RCC_RNG1CKSELR, 0, 0x3, rng1_parents),
	_CLK_PARENT(_RTC_SEL, RCC_BDCR, 16, 0x3, rtc_parents),
	_CLK_PARENT(_MPU_SEL, RCC_MPCKSELR, 0, 0x3, mpu_parents),
	/* Always non-secure clocks (maybe used in some way in secure world) */
#ifdef CFG_WITH_NSEC_UARTS
	_CLK_PARENT(_UART6_SEL, RCC_UART6CKSELR, 0, 0x7, uart6_parents),
	_CLK_PARENT(_UART24_SEL, RCC_UART24CKSELR, 0, 0x7, uart234578_parents),
	_CLK_PARENT(_UART35_SEL, RCC_UART35CKSELR, 0, 0x7, uart234578_parents),
	_CLK_PARENT(_UART78_SEL, RCC_UART78CKSELR, 0, 0x7, uart234578_parents),
#endif
	_CLK_PARENT(_AXISS_SEL, RCC_ASSCKSELR, 0, 0x3, axiss_parents),
	_CLK_PARENT(_MCUSS_SEL, RCC_MSSCKSELR, 0, 0x3, mcuss_parents),
};

/* Define characteristics of PLL according type */
static const struct stm32mp1_pll stm32mp1_pll[PLL_TYPE_NB] = {
	[PLL_800] = {
		.refclk_min = 4,
		.refclk_max = 16,
		.divn_max = 99,
	},
	[PLL_1600] = {
		.refclk_min = 8,
		.refclk_max = 16,
		.divn_max = 199,
	},
};

/* PLLNCFGR2 register divider by output */
static const uint8_t pllncfgr2[_DIV_NB] = {
	[_DIV_P] = RCC_PLLNCFGR2_DIVP_SHIFT,
	[_DIV_Q] = RCC_PLLNCFGR2_DIVQ_SHIFT,
	[_DIV_R] = RCC_PLLNCFGR2_DIVR_SHIFT,
};

static const struct stm32mp1_clk_pll stm32mp1_clk_pll[_PLL_NB] = {
	_CLK_PLL(_PLL1, PLL_1600,
		 RCC_RCK12SELR, RCC_PLL1CFGR1, RCC_PLL1CFGR2,
		 RCC_PLL1FRACR, RCC_PLL1CR, RCC_PLL1CSGR,
		 OSC_HSI, OSC_HSE, _UNKNOWN_OSC_ID, _UNKNOWN_OSC_ID),
	_CLK_PLL(_PLL2, PLL_1600,
		 RCC_RCK12SELR, RCC_PLL2CFGR1, RCC_PLL2CFGR2,
		 RCC_PLL2FRACR, RCC_PLL2CR, RCC_PLL2CSGR,
		 OSC_HSI, OSC_HSE, _UNKNOWN_OSC_ID, _UNKNOWN_OSC_ID),
	_CLK_PLL(_PLL3, PLL_800,
		 RCC_RCK3SELR, RCC_PLL3CFGR1, RCC_PLL3CFGR2,
		 RCC_PLL3FRACR, RCC_PLL3CR, RCC_PLL3CSGR,
		 OSC_HSI, OSC_HSE, OSC_CSI, _UNKNOWN_OSC_ID),
	_CLK_PLL(_PLL4, PLL_800,
		 RCC_RCK4SELR, RCC_PLL4CFGR1, RCC_PLL4CFGR2,
		 RCC_PLL4FRACR, RCC_PLL4CR, RCC_PLL4CSGR,
		 OSC_HSI, OSC_HSE, OSC_CSI, OSC_I2S_CKIN),
};

/* Prescaler table lookups for clock computation */
/* div = /1 /2 /4 /8 / 16 /64 /128 /512 */
static const uint8_t stm32mp1_mcu_div[16] = {
	0, 1, 2, 3, 4, 6, 7, 8, 9, 9, 9, 9, 9, 9, 9, 9
};

/* div = /1 /2 /4 /8 /16 : same divider for PMU and APBX */
#define stm32mp1_mpu_div	stm32mp1_mpu_apbx_div
#define stm32mp1_apbx_div	stm32mp1_mpu_apbx_div
static const uint8_t stm32mp1_mpu_apbx_div[8] = {
	0, 1, 2, 3, 4, 4, 4, 4
};

/* div = /1 /2 /3 /4 */
static const uint8_t stm32mp1_axi_div[8] = {
	1, 2, 3, 4, 4, 4, 4, 4
};

static const char __maybe_unused *const stm32mp1_clk_parent_name[_PARENT_NB] = {
	[_HSI] = "HSI",
	[_HSE] = "HSE",
	[_CSI] = "CSI",
	[_LSI] = "LSI",
	[_LSE] = "LSE",
	[_I2S_CKIN] = "I2S_CKIN",
	[_HSI_KER] = "HSI_KER",
	[_HSE_KER] = "HSE_KER",
	[_HSE_KER_DIV2] = "HSE_KER_DIV2",
	[_HSE_RTC] = "HSE_RTC",
	[_CSI_KER] = "CSI_KER",
	[_PLL1_P] = "PLL1_P",
	[_PLL1_Q] = "PLL1_Q",
	[_PLL1_R] = "PLL1_R",
	[_PLL2_P] = "PLL2_P",
	[_PLL2_Q] = "PLL2_Q",
	[_PLL2_R] = "PLL2_R",
	[_PLL3_P] = "PLL3_P",
	[_PLL3_Q] = "PLL3_Q",
	[_PLL3_R] = "PLL3_R",
	[_PLL4_P] = "PLL4_P",
	[_PLL4_Q] = "PLL4_Q",
	[_PLL4_R] = "PLL4_R",
	[_ACLK] = "ACLK",
	[_PCLK1] = "PCLK1",
	[_PCLK2] = "PCLK2",
	[_PCLK3] = "PCLK3",
	[_PCLK4] = "PCLK4",
	[_PCLK5] = "PCLK5",
	[_HCLK2] = "HCLK2",
	[_HCLK5] = "HCLK5",
	[_HCLK6] = "HCLK6",
	[_CK_PER] = "CK_PER",
	[_CK_MPU] = "CK_MPU",
	[_CK_MCU] = "CK_MCU",
	[_USB_PHY_48] = "USB_PHY_48",
};

/*
 * Oscillator frequency in Hz. This array shall be initialized
 * according to platform.
 */
static unsigned long stm32mp1_osc[NB_OSC];

/* Storage of the precomputed SoC settings for PLL1 various OPPs */
static struct stm32mp1_pll_settings pll1_settings;
static uint32_t current_opp_khz;

static unsigned long osc_frequency(enum stm32mp_osc_id idx)
{
	if (idx >= ARRAY_SIZE(stm32mp1_osc)) {
		DMSG("clk id %d not found", idx);
		return 0;
	}

	return stm32mp1_osc[idx];
}

static const struct stm32mp1_clk_gate *gate_ref(unsigned int idx)
{
	return &stm32mp1_clk_gate[idx];
}

static const struct stm32mp1_clk_sel *clk_sel_ref(unsigned int idx)
{
	return &stm32mp1_clk_sel[idx];
}

static const struct stm32mp1_clk_pll *pll_ref(unsigned int idx)
{
	return &stm32mp1_clk_pll[idx];
}

static int stm32mp1_clk_get_gated_id(unsigned long id)
{
	unsigned int i = 0;

	for (i = 0; i < NB_GATES; i++)
		if (gate_ref(i)->clock_id == id)
			return i;

	DMSG("clk id %lu not found", id);
	return -1;
}

static enum stm32mp1_parent_sel stm32mp1_clk_get_sel(int i)
{
	return (enum stm32mp1_parent_sel)gate_ref(i)->sel;
}

static enum stm32mp1_parent_id stm32mp1_clk_get_fixed_parent(int i)
{
	return (enum stm32mp1_parent_id)gate_ref(i)->fixed;
}

static int stm32mp1_clk_get_parent(unsigned long id)
{
	const struct stm32mp1_clk_sel *sel = NULL;
	enum stm32mp1_parent_id parent_id = 0;
	uint32_t p_sel = 0;
	int i = 0;
	enum stm32mp1_parent_id p = _UNKNOWN_ID;
	enum stm32mp1_parent_sel s = _UNKNOWN_SEL;
	vaddr_t rcc_base = stm32_rcc_base();

	parent_id = clock_id2parent_id(id);
	if (parent_id != _UNKNOWN_ID)
		return (int)parent_id;

	i = stm32mp1_clk_get_gated_id(id);
	if (i < 0)
		panic();

	p = stm32mp1_clk_get_fixed_parent(i);
	if (p < _PARENT_NB)
		return (int)p;

	s = stm32mp1_clk_get_sel(i);
	if (s == _UNKNOWN_SEL)
		return -1;
	if (s >= _PARENT_SEL_NB)
		panic();

	sel = clk_sel_ref(s);
	p_sel = (io_read32(rcc_base + sel->offset) >> sel->src) & sel->msk;
	if (p_sel < sel->nb_parent)
		return (int)sel->parent[p_sel];

	DMSG("No parent selected for clk %lu", id);
	return -1;
}

static unsigned long stm32mp1_pll_get_fref(const struct stm32mp1_clk_pll *pll)
{
	uint32_t selr = io_read32(stm32_rcc_base() + pll->rckxselr);
	uint32_t src = selr & RCC_SELR_REFCLK_SRC_MASK;

	return osc_frequency(pll->refclk[src]);
}

/*
 * pll_get_fvco() : return the VCO or (VCO / 2) frequency for the requested PLL
 * - PLL1 & PLL2 => return VCO / 2 with Fpll_y_ck = FVCO / 2 * (DIVy + 1)
 * - PLL3 & PLL4 => return VCO     with Fpll_y_ck = FVCO / (DIVy + 1)
 * => in all cases Fpll_y_ck = pll_get_fvco() / (DIVy + 1)
 */
static unsigned long stm32mp1_pll_get_fvco(const struct stm32mp1_clk_pll *pll)
{
	unsigned long refclk = 0;
	unsigned long fvco = 0;
	uint32_t cfgr1 = 0;
	uint32_t fracr = 0;
	uint32_t divm = 0;
	uint32_t divn = 0;

	cfgr1 = io_read32(stm32_rcc_base() + pll->pllxcfgr1);
	fracr = io_read32(stm32_rcc_base() + pll->pllxfracr);

	divm = (cfgr1 & RCC_PLLNCFGR1_DIVM_MASK) >> RCC_PLLNCFGR1_DIVM_SHIFT;
	divn = cfgr1 & RCC_PLLNCFGR1_DIVN_MASK;

	refclk = stm32mp1_pll_get_fref(pll);

	/*
	 * With FRACV :
	 *   Fvco = Fck_ref * ((DIVN + 1) + FRACV / 2^13) / (DIVM + 1)
	 * Without FRACV
	 *   Fvco = Fck_ref * ((DIVN + 1) / (DIVM + 1)
	 */
	if (fracr & RCC_PLLNFRACR_FRACLE) {
		unsigned long long numerator = 0;
		unsigned long long denominator = 0;
		uint32_t fracv = (fracr & RCC_PLLNFRACR_FRACV_MASK) >>
				 RCC_PLLNFRACR_FRACV_SHIFT;

		numerator = (((unsigned long long)divn + 1U) << 13) + fracv;
		numerator = refclk * numerator;
		denominator = ((unsigned long long)divm + 1U) << 13;
		fvco = (unsigned long)(numerator / denominator);
	} else {
		fvco = (unsigned long)(refclk * (divn + 1U) / (divm + 1U));
	}

	return fvco;
}

static unsigned long stm32mp1_read_pll_freq(enum stm32mp1_pll_id pll_id,
					    enum stm32mp1_div_id div_id)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	unsigned long dfout = 0;
	uint32_t cfgr2 = 0;
	uint32_t divy = 0;

	if (div_id >= _DIV_NB)
		return 0;

	cfgr2 = io_read32(stm32_rcc_base() + pll->pllxcfgr2);
	divy = (cfgr2 >> pllncfgr2[div_id]) & RCC_PLLNCFGR2_DIVX_MASK;

	dfout = stm32mp1_pll_get_fvco(pll) / (divy + 1U);

	return dfout;
}

static void pll_start(enum stm32mp1_pll_id pll_id)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uint32_t pllxcr = stm32_rcc_base() + pll->pllxcr;

	if (io_read32(pllxcr) & RCC_PLLNCR_PLLON)
		return;

	io_clrsetbits32(pllxcr,	RCC_PLLNCR_DIVPEN | RCC_PLLNCR_DIVQEN |
			RCC_PLLNCR_DIVREN, RCC_PLLNCR_PLLON);
}

#define PLLRDY_TIMEOUT_US	(200 * 1000)

static int pll_output(enum stm32mp1_pll_id pll_id, uint32_t output)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uint32_t pllxcr = stm32_rcc_base() + pll->pllxcr;
	uint64_t start = 0;

	start = timeout_init_us(PLLRDY_TIMEOUT_US);
	/* Wait PLL lock */
	while (!(io_read32(pllxcr) & RCC_PLLNCR_PLLRDY))
		if (timeout_elapsed(start))
			break;

	if (!(io_read32(pllxcr) & RCC_PLLNCR_PLLRDY)) {
		EMSG("PLL%d start failed @ 0x%"PRIx32": 0x%"PRIx32,
		     pll_id, pllxcr, io_read32(pllxcr));

		return -1;
	}

	/* Start the requested output */
	io_setbits32(pllxcr, output << RCC_PLLNCR_DIVEN_SHIFT);

	return 0;
}

static int pll_stop(enum stm32mp1_pll_id pll_id)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uint32_t pllxcr = stm32_rcc_base() + pll->pllxcr;
	uint64_t start = 0;

	/* Stop all output */
	io_clrbits32(pllxcr, RCC_PLLNCR_DIVPEN | RCC_PLLNCR_DIVQEN |
		     RCC_PLLNCR_DIVREN);

	/* Stop PLL */
	io_clrbits32(pllxcr, RCC_PLLNCR_PLLON);

	start = timeout_init_us(PLLRDY_TIMEOUT_US);
	/* Wait PLL stopped */
	while (io_read32(pllxcr) & RCC_PLLNCR_PLLRDY)
		if (timeout_elapsed(start))
			break;

	if (io_read32(pllxcr) & RCC_PLLNCR_PLLRDY) {
		EMSG("PLL%d stop failed @ 0x%"PRIx32": 0x%"PRIx32,
		     pll_id, pllxcr, io_read32(pllxcr));

		return -1;
	}

	return 0;
}

static uint32_t pll_compute_pllxcfgr2(uint32_t *pllcfg)
{
	uint32_t value = 0;

	value = (pllcfg[PLLCFG_P] << RCC_PLLNCFGR2_DIVP_SHIFT) &
		RCC_PLLNCFGR2_DIVP_MASK;
	value |= (pllcfg[PLLCFG_Q] << RCC_PLLNCFGR2_DIVQ_SHIFT) &
		 RCC_PLLNCFGR2_DIVQ_MASK;
	value |= (pllcfg[PLLCFG_R] << RCC_PLLNCFGR2_DIVR_SHIFT) &
		 RCC_PLLNCFGR2_DIVR_MASK;

	return value;
}

static void pll_config_output(enum stm32mp1_pll_id pll_id, uint32_t *pllcfg)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uintptr_t rcc_base = stm32_rcc_base();
	uint32_t value = 0;

	value = pll_compute_pllxcfgr2(pllcfg);

	io_write32(rcc_base + pll->pllxcfgr2, value);
}

static int pll_compute_pllxcfgr1(const struct stm32mp1_clk_pll *pll,
				 uint32_t *pllcfg, uint32_t *cfgr1)
{
	uint32_t rcc_base = stm32_rcc_base();
	enum stm32mp1_plltype type = pll->plltype;
	unsigned long refclk = 0;
	uint32_t ifrge = 0;
	uint32_t src = 0;

	src = io_read32(rcc_base + pll->rckxselr) &
	      RCC_SELR_REFCLK_SRC_MASK;

	refclk = osc_frequency(pll->refclk[src]) /
		 (pllcfg[PLLCFG_M] + 1U);

	if ((refclk < (stm32mp1_pll[type].refclk_min * 1000000U)) ||
	    (refclk > (stm32mp1_pll[type].refclk_max * 1000000U)))
		return -1;

	if ((type == PLL_800) && (refclk >= 8000000U))
		ifrge = 1U;

	*cfgr1 = (pllcfg[PLLCFG_N] << RCC_PLLNCFGR1_DIVN_SHIFT) &
		 RCC_PLLNCFGR1_DIVN_MASK;
	*cfgr1 |= (pllcfg[PLLCFG_M] << RCC_PLLNCFGR1_DIVM_SHIFT) &
		  RCC_PLLNCFGR1_DIVM_MASK;
	*cfgr1 |= (ifrge << RCC_PLLNCFGR1_IFRGE_SHIFT) &
		  RCC_PLLNCFGR1_IFRGE_MASK;

	return 0;
}

static int pll_config(enum stm32mp1_pll_id pll_id, uint32_t *pllcfg,
		      uint32_t fracv)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uint32_t rcc_base = stm32_rcc_base();
	uint32_t value = 0;
	int ret = 0;

	ret = pll_compute_pllxcfgr1(pll, pllcfg, &value);
	if (ret)
		return ret;

	io_write32(rcc_base + pll->pllxcfgr1, value);

	/* Fractional configuration */
	io_write32(rcc_base + pll->pllxfracr, value);

	/* Frac must be enabled only once its configuration is loaded */
	value = fracv << RCC_PLLNFRACR_FRACV_SHIFT;
	io_write32(rcc_base + pll->pllxfracr, value);
	value = io_read32(rcc_base + pll->pllxfracr);
	io_write32(rcc_base + pll->pllxfracr, value | RCC_PLLNFRACR_FRACLE);

	pll_config_output(pll_id, pllcfg);

	return 0;
}

static unsigned long get_clock_rate(enum stm32mp1_parent_id p)
{
	uint32_t reg = 0;
	unsigned long clock = 0;
	vaddr_t rcc_base = stm32_rcc_base();

	switch (p) {
	case _CK_MPU:
	/* MPU sub system */
		reg = io_read32(rcc_base + RCC_MPCKSELR);
		switch (reg & RCC_SELR_SRC_MASK) {
		case RCC_MPCKSELR_HSI:
			clock = osc_frequency(OSC_HSI);
			break;
		case RCC_MPCKSELR_HSE:
			clock = osc_frequency(OSC_HSE);
			break;
		case RCC_MPCKSELR_PLL:
			clock = stm32mp1_read_pll_freq(_PLL1, _DIV_P);
			break;
		case RCC_MPCKSELR_PLL_MPUDIV:
			reg = io_read32(rcc_base + RCC_MPCKDIVR);
			if (reg & RCC_MPUDIV_MASK)
				clock = stm32mp1_read_pll_freq(_PLL1, _DIV_P) >>
					stm32mp1_mpu_div[reg & RCC_MPUDIV_MASK];
			else
				clock = 0;
			break;
		default:
			break;
		}
		break;
	/* AXI sub system */
	case _ACLK:
	case _HCLK2:
	case _HCLK5:
	case _HCLK6:
	case _PCLK4:
	case _PCLK5:
		reg = io_read32(rcc_base + RCC_ASSCKSELR);
		switch (reg & RCC_SELR_SRC_MASK) {
		case RCC_ASSCKSELR_HSI:
			clock = osc_frequency(OSC_HSI);
			break;
		case RCC_ASSCKSELR_HSE:
			clock = osc_frequency(OSC_HSE);
			break;
		case RCC_ASSCKSELR_PLL:
			clock = stm32mp1_read_pll_freq(_PLL2, _DIV_P);
			break;
		default:
			break;
		}

		/* System clock divider */
		reg = io_read32(rcc_base + RCC_AXIDIVR);
		clock /= stm32mp1_axi_div[reg & RCC_AXIDIV_MASK];

		switch (p) {
		case _PCLK4:
			reg = io_read32(rcc_base + RCC_APB4DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		case _PCLK5:
			reg = io_read32(rcc_base + RCC_APB5DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		default:
			break;
		}
		break;
	/* MCU sub system */
	case _CK_MCU:
	case _PCLK1:
	case _PCLK2:
	case _PCLK3:
		reg = io_read32(rcc_base + RCC_MSSCKSELR);
		switch (reg & RCC_SELR_SRC_MASK) {
		case RCC_MSSCKSELR_HSI:
			clock = osc_frequency(OSC_HSI);
			break;
		case RCC_MSSCKSELR_HSE:
			clock = osc_frequency(OSC_HSE);
			break;
		case RCC_MSSCKSELR_CSI:
			clock = osc_frequency(OSC_CSI);
			break;
		case RCC_MSSCKSELR_PLL:
			clock = stm32mp1_read_pll_freq(_PLL3, _DIV_P);
			break;
		default:
			break;
		}

		/* MCU clock divider */
		reg = io_read32(rcc_base + RCC_MCUDIVR);
		clock >>= stm32mp1_mcu_div[reg & RCC_MCUDIV_MASK];

		switch (p) {
		case _PCLK1:
			reg = io_read32(rcc_base + RCC_APB1DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		case _PCLK2:
			reg = io_read32(rcc_base + RCC_APB2DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		case _PCLK3:
			reg = io_read32(rcc_base + RCC_APB3DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		case _CK_MCU:
		default:
			break;
		}
		break;
	case _CK_PER:
		reg = io_read32(rcc_base + RCC_CPERCKSELR);
		switch (reg & RCC_SELR_SRC_MASK) {
		case RCC_CPERCKSELR_HSI:
			clock = osc_frequency(OSC_HSI);
			break;
		case RCC_CPERCKSELR_HSE:
			clock = osc_frequency(OSC_HSE);
			break;
		case RCC_CPERCKSELR_CSI:
			clock = osc_frequency(OSC_CSI);
			break;
		default:
			break;
		}
		break;
	case _HSI:
	case _HSI_KER:
		clock = osc_frequency(OSC_HSI);
		break;
	case _CSI:
	case _CSI_KER:
		clock = osc_frequency(OSC_CSI);
		break;
	case _HSE:
	case _HSE_KER:
		clock = osc_frequency(OSC_HSE);
		break;
	case _HSE_KER_DIV2:
		clock = osc_frequency(OSC_HSE) >> 1;
		break;
	case _HSE_RTC:
		clock = osc_frequency(OSC_HSE);
		clock /= (io_read32(rcc_base + RCC_RTCDIVR) &
			  RCC_DIVR_DIV_MASK) + 1;
		break;
	case _LSI:
		clock = osc_frequency(OSC_LSI);
		break;
	case _LSE:
		clock = osc_frequency(OSC_LSE);
		break;
	/* PLL */
	case _PLL1_P:
		clock = stm32mp1_read_pll_freq(_PLL1, _DIV_P);
		break;
	case _PLL1_Q:
		clock = stm32mp1_read_pll_freq(_PLL1, _DIV_Q);
		break;
	case _PLL1_R:
		clock = stm32mp1_read_pll_freq(_PLL1, _DIV_R);
		break;
	case _PLL2_P:
		clock = stm32mp1_read_pll_freq(_PLL2, _DIV_P);
		break;
	case _PLL2_Q:
		clock = stm32mp1_read_pll_freq(_PLL2, _DIV_Q);
		break;
	case _PLL2_R:
		clock = stm32mp1_read_pll_freq(_PLL2, _DIV_R);
		break;
	case _PLL3_P:
		clock = stm32mp1_read_pll_freq(_PLL3, _DIV_P);
		break;
	case _PLL3_Q:
		clock = stm32mp1_read_pll_freq(_PLL3, _DIV_Q);
		break;
	case _PLL3_R:
		clock = stm32mp1_read_pll_freq(_PLL3, _DIV_R);
		break;
	case _PLL4_P:
		clock = stm32mp1_read_pll_freq(_PLL4, _DIV_P);
		break;
	case _PLL4_Q:
		clock = stm32mp1_read_pll_freq(_PLL4, _DIV_Q);
		break;
	case _PLL4_R:
		clock = stm32mp1_read_pll_freq(_PLL4, _DIV_R);
		break;
	/* Other */
	case _USB_PHY_48:
		clock = osc_frequency(OSC_USB_PHY_48);
		break;
	default:
		break;
	}

	return clock;
}

static void __clk_enable(const struct stm32mp1_clk_gate *gate)
{
	vaddr_t base = stm32_rcc_base();
	uint32_t bit = BIT(gate->bit);

	if (gate->set_clr)
		io_write32(base + gate->offset, bit);
	else
		io_setbits32_stm32shregs(base + gate->offset, bit);

	FMSG("Clock %u has been enabled", gate->clock_id);
}

static void __clk_disable(const struct stm32mp1_clk_gate *gate)
{
	vaddr_t base = stm32_rcc_base();
	uint32_t bit = BIT(gate->bit);

	if (gate->set_clr)
		io_write32(base + gate->offset + RCC_MP_ENCLRR_OFFSET, bit);
	else
		io_clrbits32_stm32shregs(base + gate->offset, bit);

	FMSG("Clock %u has been disabled", gate->clock_id);
}

static long get_timer_rate(long parent_rate, unsigned int apb_bus)
{
	uint32_t timgxpre = 0;
	uint32_t apbxdiv = 0;
	vaddr_t rcc_base = stm32_rcc_base();

	switch (apb_bus) {
	case 1:
		apbxdiv = io_read32(rcc_base + RCC_APB1DIVR) &
			  RCC_APBXDIV_MASK;
		timgxpre = io_read32(rcc_base + RCC_TIMG1PRER) &
			   RCC_TIMGXPRER_TIMGXPRE;
		break;
	case 2:
		apbxdiv = io_read32(rcc_base + RCC_APB2DIVR) &
			  RCC_APBXDIV_MASK;
		timgxpre = io_read32(rcc_base + RCC_TIMG2PRER) &
			   RCC_TIMGXPRER_TIMGXPRE;
		break;
	default:
		panic();
		break;
	}

	if (apbxdiv == 0)
		return parent_rate;

	return parent_rate * (timgxpre + 1) * 2;
}

static unsigned long _stm32_clock_get_rate(unsigned long id)
{
	enum stm32mp1_parent_id p = _UNKNOWN_ID;
	unsigned long rate = 0;

	p = stm32mp1_clk_get_parent(id);
	if (p < 0)
		return 0;

	rate = get_clock_rate(p);

	if ((id >= TIM2_K) && (id <= TIM14_K))
		rate = get_timer_rate(rate, 1);

	if ((id >= TIM1_K) && (id <= TIM17_K))
		rate = get_timer_rate(rate, 2);

	return rate;
}

/*
 * Get the parent ID of the target parent clock, or -1 if no parent found.
 */
static enum stm32mp1_parent_id get_parent_id_parent(enum stm32mp1_parent_id id)
{
	enum stm32mp1_parent_sel s = _UNKNOWN_SEL;
	enum stm32mp1_pll_id pll_id = _PLL_NB;
	uint32_t p_sel = 0;

	switch (id) {
	case _ACLK:
	case _HCLK5:
	case _HCLK6:
	case _PCLK4:
	case _PCLK5:
		s = _AXISS_SEL;
		break;
	case _PLL1_P:
	case _PLL1_Q:
	case _PLL1_R:
		pll_id = _PLL1;
		break;
	case _PLL2_P:
	case _PLL2_Q:
	case _PLL2_R:
		pll_id = _PLL2;
		break;
	case _PLL3_P:
	case _PLL3_Q:
	case _PLL3_R:
		pll_id = _PLL3;
		break;
	case _PLL4_P:
	case _PLL4_Q:
	case _PLL4_R:
		pll_id = _PLL4;
		break;
	case _PCLK1:
	case _PCLK2:
	case _HCLK2:
	case _CK_PER:
	case _CK_MPU:
	case _CK_MCU:
	case _USB_PHY_48:
		/* We do not expected to access these */
		panic();
		break;
	default:
		/* Other parents have no parent */
		return -1;
	}

	if (s != _UNKNOWN_SEL) {
		const struct stm32mp1_clk_sel *sel = clk_sel_ref(s);
		vaddr_t rcc_base = stm32_rcc_base();

		p_sel = (io_read32(rcc_base + sel->offset) >> sel->src) &
			sel->msk;

		if (p_sel < sel->nb_parent)
			return sel->parent[p_sel];
	} else {
		const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);

		p_sel = io_read32(stm32_rcc_base() + pll->rckxselr) &
			RCC_SELR_REFCLK_SRC_MASK;

		if (pll->refclk[p_sel] != _UNKNOWN_OSC_ID)
			return osc_id2parent_id(pll->refclk[p_sel]);
	}

	FMSG("No parent found for %s", stm32mp1_clk_parent_name[id]);
	return -1;
}

/* We are only interested in knowing if PLL3 shall be secure or not */
static void secure_parent_clocks(enum stm32mp1_parent_id parent_id)
{
	enum stm32mp1_parent_id grandparent_id = _UNKNOWN_ID;

	switch (parent_id) {
	case _ACLK:
	case _HCLK2:
	case _HCLK5:
	case _HCLK6:
	case _PCLK4:
	case _PCLK5:
		/* Intermediate clock mux or clock, go deeper in clock tree */
		break;
	case _HSI:
	case _HSI_KER:
	case _LSI:
	case _CSI:
	case _CSI_KER:
	case _HSE:
	case _HSE_KER:
	case _HSE_KER_DIV2:
	case _HSE_RTC:
	case _LSE:
	case _PLL1_P:
	case _PLL1_Q:
	case _PLL1_R:
	case _PLL2_P:
	case _PLL2_Q:
	case _PLL2_R:
		/* Always secure clocks, no need to go further */
		return;
	case _PLL3_P:
	case _PLL3_Q:
	case _PLL3_R:
		/* PLL3 is a shared resource, registered and don't go further */
		stm32mp_register_secure_periph(STM32MP1_SHRES_PLL3);
		return;
	default:
		DMSG("Cannot lookup parent clock %s",
		     stm32mp1_clk_parent_name[parent_id]);
		panic();
	}

	grandparent_id = get_parent_id_parent(parent_id);
	if (grandparent_id >= 0)
		secure_parent_clocks(grandparent_id);
}

void stm32mp_register_clock_parents_secure(unsigned long clock_id)
{
	enum stm32mp1_parent_id parent_id = stm32mp1_clk_get_parent(clock_id);

	if (parent_id < 0) {
		DMSG("No parent for clock %lu", clock_id);
		return;
	}

	secure_parent_clocks(parent_id);
}

static void save_current_opp(void)
{
	unsigned long freq_khz = UDIV_ROUND_NEAREST(_stm32_clock_get_rate(CK_MPU),
						    1000UL);
	if (freq_khz > (unsigned long)UINT32_MAX)
		panic();

	current_opp_khz = (uint32_t)freq_khz;
}

#ifdef CFG_EMBED_DTB
static const char *stm32mp_osc_node_label[NB_OSC] = {
	[OSC_LSI] = "clk-lsi",
	[OSC_LSE] = "clk-lse",
	[OSC_HSI] = "clk-hsi",
	[OSC_HSE] = "clk-hse",
	[OSC_CSI] = "clk-csi",
	[OSC_I2S_CKIN] = "i2s_ckin",
	[OSC_USB_PHY_48] = "ck_usbo_48m"
};

static unsigned int clk_freq_prop(const void *fdt, int node)
{
	const fdt32_t *cuint = NULL;
	int ret = 0;

	/* Disabled clocks report null rate */
	if (_fdt_get_status(fdt, node) == DT_STATUS_DISABLED)
		return 0;

	cuint = fdt_getprop(fdt, node, "clock-frequency", &ret);
	if (!cuint)
		panic();

	return fdt32_to_cpu(*cuint);
}

static void get_osc_freq_from_dt(const void *fdt)
{
	enum stm32mp_osc_id idx = _UNKNOWN_OSC_ID;
	int clk_node = fdt_path_offset(fdt, "/clocks");

	if (clk_node < 0)
		panic();

	COMPILE_TIME_ASSERT((int)OSC_HSI == 0);
	for (idx = OSC_HSI; idx < NB_OSC; idx++) {
		const char *name = stm32mp_osc_node_label[idx];
		int subnode = 0;

		fdt_for_each_subnode(subnode, fdt, clk_node) {
			const char *cchar = NULL;
			int ret = 0;

			cchar = fdt_get_name(fdt, subnode, &ret);
			if (!cchar)
				panic();

			if (strncmp(cchar, name, (size_t)ret) == 0) {
				stm32mp1_osc[idx] = clk_freq_prop(fdt, subnode);

				DMSG("Osc %s: %lu Hz", name, stm32mp1_osc[idx]);
				break;
			}
		}

		if (!stm32mp1_osc[idx])
			DMSG("Osc %s: no frequency info", name);
	}
}
#endif /*CFG_EMBED_DTB*/

static void __maybe_unused enable_rcc_tzen(void)
{
	io_setbits32(stm32_rcc_base() + RCC_TZCR, RCC_TZCR_TZEN);
}

static void __maybe_unused disable_rcc_tzen(void)
{
	IMSG("RCC is non-secure");
	io_clrbits32(stm32_rcc_base() + RCC_TZCR, RCC_TZCR_TZEN);
}

#ifdef CFG_EMBED_DTB
static TEE_Result stm32mp1_clk_fdt_init(const void *fdt, int node)
{
	unsigned int i = 0;
	int len = 0;
	int ignored = 0;

	get_osc_freq_from_dt(fdt);

	/*
	 * OP-TEE core is not in charge of configuring clock parenthood.
	 * This is expected from an earlier boot stage. Modifying the clock
	 * tree parenthood here may jeopardize already configured clocks.
	 * The sequence below ignores such DT directives with a friendly
	 * debug trace.
	 */
	if (fdt_getprop(fdt, node, "st,clksrc", &len)) {
		DMSG("Ignore source clocks configuration from DT");
		ignored++;
	}
	if (fdt_getprop(fdt, node, "st,clkdiv", &len)) {
		DMSG("Ignore clock divisors configuration from DT");
		ignored++;
	}
	if (fdt_getprop(fdt, node, "st,pkcs", &len)) {
		DMSG("Ignore peripheral clocks tree configuration from DT");
		ignored++;
	}
	for (i = (enum stm32mp1_pll_id)0; i < _PLL_NB; i++) {
		char name[] = "st,pll@X";

		snprintf(name, sizeof(name), "st,pll@%d", i);
		node = fdt_subnode_offset(fdt, node, name);
		if (node < 0)
			continue;

		if (fdt_getprop(fdt, node, "cfg", &len) ||
		    fdt_getprop(fdt, node, "frac", &len)) {
			DMSG("Ignore PLL%u configurations from DT", i);
			ignored++;
		}
	}

	if (ignored != 0)
		IMSG("DT clock tree configurations were ignored");

	return TEE_SUCCESS;
}
#endif /*CFG_EMBED_DTB*/

void stm32mp1_clk_mcuss_protect(bool enable)
{
	uintptr_t rcc_base = stm32_rcc_base();

	if (enable)
		io_setbits32(rcc_base + RCC_TZCR, RCC_TZCR_MCKPROT);
	else
		io_clrbits32(rcc_base + RCC_TZCR, RCC_TZCR_MCKPROT);
}

/*
 * Gets OPP parameters (frequency in KHz and voltage in mV) from an OPP table
 * subnode. Platform HW support capabilities are also checked.
 */
static int get_opp_freqvolt_from_dt_subnode(void *fdt, int subnode,
					    uint32_t *freq_khz,
					    uint32_t *voltage_mv)
{
	const fdt64_t *cuint64 = NULL;
	const fdt32_t *cuint32 = NULL;
	uint64_t read_freq_64 = 0;
	uint32_t read_voltage_32 = 0;

	assert(freq_khz);
	assert(voltage_mv);

	cuint32 = fdt_getprop(fdt, subnode, "opp-supported-hw", NULL);
	if (cuint32)
		if (!stm32mp_supports_cpu_opp(fdt32_to_cpu(*cuint32))) {
			DMSG("Invalid opp-supported-hw 0x%"PRIx32,
			     fdt32_to_cpu(*cuint32));
			return -FDT_ERR_BADVALUE;
		}

	cuint64 = fdt_getprop(fdt, subnode, "opp-hz", NULL);
	if (!cuint64) {
		DMSG("Missing opp-hz");
		return -FDT_ERR_NOTFOUND;
	}

	/* Frequency value expressed in KHz must fit on 32 bits */
	read_freq_64 = fdt64_to_cpu(*cuint64) / 1000ULL;
	if (read_freq_64 > (uint64_t)UINT32_MAX) {
		DMSG("Invalid opp-hz %"PRIu64, read_freq_64);
		return -FDT_ERR_BADVALUE;
	}

	cuint32 = fdt_getprop(fdt, subnode, "opp-microvolt", NULL);
	if (!cuint32) {
		DMSG("Missing opp-microvolt");
		return -FDT_ERR_NOTFOUND;
	}

	/* Millivolt value must fit on 16 bits */
	read_voltage_32 = fdt32_to_cpu(*cuint32) / 1000U;
	if (read_voltage_32 > UINT16_MAX) {
		DMSG("Invalid opp-microvolt %"PRIu32, read_voltage_32);
		return -FDT_ERR_BADVALUE;
	}

	*freq_khz = (uint32_t)read_freq_64;

	*voltage_mv = read_voltage_32;

	return 0;
}

/*
 * Parses OPP table in DT and finds all parameters supported by the HW
 * platform. If found, the corresponding frequency and voltage values are
 * respectively stored in @pll1_settings structure.
 * Note that @*count has to be set by caller to the effective size allocated
 * for both tables. Its value is then replaced by the number of filled elements.
 */
static int get_all_opp_freqvolt_from_dt(uint32_t *count)
{
	void *fdt = NULL;
	int node = 0;
	int subnode = 0;
	uint32_t idx = 0;

	assert(count);

	fdt = get_embedded_dt();
	node = fdt_node_offset_by_compatible(fdt, -1, DT_OPP_COMPAT);
	if (node < 0)
		return node;

	fdt_for_each_subnode(subnode, fdt, node) {
		uint32_t read_freq = 0;
		uint32_t read_voltage = 0;

		if (get_opp_freqvolt_from_dt_subnode(fdt, subnode, &read_freq,
						     &read_voltage))
			continue;

		if (idx >= *count)
			return -FDT_ERR_NOSPACE;

		pll1_settings.freq[idx] = read_freq;
		pll1_settings.volt[idx] = read_voltage;
		idx++;
	}

	if (!idx)
		return -FDT_ERR_NOTFOUND;

	*count = idx;

	return 0;
}

static int clk_compute_pll1_settings(unsigned long input_freq, int idx)
{
	unsigned long post_divm = 0;
	unsigned long long output_freq = pll1_settings.freq[idx] * 1000U;
	unsigned long long freq = 0;
	unsigned long long vco = 0;
	int divm = 0;
	int divn = 0;
	int divp = 0;
	int frac = 0;
	int i = 0;
	unsigned int diff = 0;
	unsigned int best_diff = UINT_MAX;

	/* Following parameters have always the same value */
	pll1_settings.cfg[idx][PLLCFG_Q] = 0;
	pll1_settings.cfg[idx][PLLCFG_R] = 0;
	pll1_settings.cfg[idx][PLLCFG_O] = PQR(1, 0, 0);

	for (divm = DIVM_MAX; divm >= DIVM_MIN; divm--)	{
		post_divm = input_freq / (unsigned long)(divm + 1);

		if ((post_divm < POST_DIVM_MIN) ||
		    (post_divm > POST_DIVM_MAX))
			continue;

		for (divp = DIVP_MIN; divp <= DIVP_MAX; divp++) {

			freq = output_freq * (divm + 1) * (divp + 1);

			divn = (int)((freq / input_freq) - 1);
			if ((divn < DIVN_MIN) || (divn > DIVN_MAX))
				continue;

			frac = (int)(((freq * FRAC_MAX) / input_freq) -
				     ((divn + 1) * FRAC_MAX));

			/* 2 loops to refine the fractional part */
			for (i = 2; i != 0; i--) {
				if (frac > FRAC_MAX)
					break;

				vco = (post_divm * (divn + 1)) +
				      ((post_divm * (unsigned long long)frac) /
				       FRAC_MAX);

				if ((vco < (VCO_MIN / 2)) ||
				    (vco > (VCO_MAX / 2))) {
					frac++;
					continue;
				}

				freq = vco / (divp + 1);
				if (output_freq < freq)
					diff = (unsigned int)(freq -
							      output_freq);
				else
					diff = (unsigned int)(output_freq -
							      freq);

				if (diff < best_diff)  {
					pll1_settings.cfg[idx][PLLCFG_M] = divm;
					pll1_settings.cfg[idx][PLLCFG_N] = divn;
					pll1_settings.cfg[idx][PLLCFG_P] = divp;
					pll1_settings.frac[idx] = frac;

					if (!diff)
						return 0;

					best_diff = diff;
				}

				frac++;
			}
		}
	}

	if (best_diff == UINT_MAX) {
		pll1_settings.cfg[idx][PLLCFG_O] = 0;
		return -1;
	}

	return 0;
}

static int clk_get_pll1_settings(uint32_t clksrc, int index)
{
	unsigned long input_freq = 0;
	unsigned int i = 0;

	for (i = 0; i < PLAT_MAX_OPP_NB; i++)
		if (pll1_settings.freq[i] == pll1_settings.freq[index])
			break;

	if (((i == PLAT_MAX_OPP_NB) &&
	     !stm32mp1_clk_pll1_settings_are_valid()) ||
	    ((i < PLAT_MAX_OPP_NB) && !pll1_settings.cfg[i][PLLCFG_O])) {
		/*
		 * Either PLL1 settings structure is completely empty,
		 * or these settings are not yet computed: do it.
		 */
		switch (clksrc) {
		case CLK_PLL12_HSI:
			input_freq = _stm32_clock_get_rate(CK_HSI);
			break;
		case CLK_PLL12_HSE:
			input_freq = _stm32_clock_get_rate(CK_HSE);
			break;
		default:
			panic();
		}

		return clk_compute_pll1_settings(input_freq, index);
	}

	if (i < PLAT_MAX_OPP_NB) {
		if (pll1_settings.cfg[i][PLLCFG_O])
			return 0;

		/*
		 * Index is in range and PLL1 settings are computed:
		 * use content to answer to the request.
		 */
		memcpy(&pll1_settings.cfg[index][0], &pll1_settings.cfg[i][0],
		       sizeof(uint32_t) * PLAT_MAX_PLLCFG_NB);
		pll1_settings.frac[index] = pll1_settings.frac[i];

		return 0;
	}

	return -1;
}

static int clk_save_current_pll1_settings(uint32_t buck1_voltage)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(_PLL1);
	uint32_t rcc_base = stm32_rcc_base();
	uint32_t freq = 0;
	unsigned int i = 0;

	freq = UDIV_ROUND_NEAREST(_stm32_clock_get_rate(CK_MPU), 1000L);

	for (i = 0; i < PLAT_MAX_OPP_NB; i++)
		if (pll1_settings.freq[i] == freq)
			break;

	if ((i == PLAT_MAX_OPP_NB) ||
	    ((pll1_settings.volt[i] != buck1_voltage) && buck1_voltage))
		return -1;

	pll1_settings.cfg[i][PLLCFG_M] = (io_read32(rcc_base + pll->pllxcfgr1) &
					  RCC_PLLNCFGR1_DIVM_MASK) >>
					 RCC_PLLNCFGR1_DIVM_SHIFT;

	pll1_settings.cfg[i][PLLCFG_N] = (io_read32(rcc_base + pll->pllxcfgr1) &
					  RCC_PLLNCFGR1_DIVN_MASK) >>
					 RCC_PLLNCFGR1_DIVN_SHIFT;

	pll1_settings.cfg[i][PLLCFG_P] = (io_read32(rcc_base + pll->pllxcfgr2) &
					  RCC_PLLNCFGR2_DIVP_MASK) >>
					 RCC_PLLNCFGR2_DIVP_SHIFT;

	pll1_settings.cfg[i][PLLCFG_Q] = (io_read32(rcc_base + pll->pllxcfgr2) &
					  RCC_PLLNCFGR2_DIVQ_MASK) >>
					 RCC_PLLNCFGR2_DIVQ_SHIFT;

	pll1_settings.cfg[i][PLLCFG_R] = (io_read32(rcc_base + pll->pllxcfgr2) &
					  RCC_PLLNCFGR2_DIVR_MASK) >>
					 RCC_PLLNCFGR2_DIVR_SHIFT;

	pll1_settings.cfg[i][PLLCFG_O] = io_read32(rcc_base + pll->pllxcr) >>
					 RCC_PLLNCR_DIVEN_SHIFT;

	pll1_settings.frac[i] = (io_read32(rcc_base + pll->pllxfracr) &
				 RCC_PLLNFRACR_FRACV_MASK) >>
				RCC_PLLNFRACR_FRACV_SHIFT;

	return i;
}

static uint32_t stm32mp1_clk_get_pll1_current_clksrc(void)
{
	uint32_t value = 0;
	const struct stm32mp1_clk_pll *pll = pll_ref(_PLL1);
	uint32_t rcc_base = stm32_rcc_base();

	value = io_read32(rcc_base + pll->rckxselr);

	switch (value & RCC_SELR_REFCLK_SRC_MASK) {
	case 0:
		return CLK_PLL12_HSI;
	case 1:
		return CLK_PLL12_HSE;
	default:
		panic();
	}
}

int stm32mp1_clk_compute_all_pll1_settings(uint32_t buck1_voltage)
{
	unsigned int i = 0;
	int ret = 0;
	int index = 0;
	uint32_t count = PLAT_MAX_OPP_NB;
	uint32_t clksrc = 0;

	ret = get_all_opp_freqvolt_from_dt(&count);
	switch (ret) {
	case 0:
		break;
	case -FDT_ERR_NOTFOUND:
		DMSG("Cannot find all OPP info in DT: use default settings.");
		return 0;
	default:
		EMSG("Inconsistent OPP settings found in DT, ignored.");
		return 0;
	}

	index = clk_save_current_pll1_settings(buck1_voltage);

	clksrc = stm32mp1_clk_get_pll1_current_clksrc();

	for (i = 0; i < count; i++) {
		if (index >= 0 && i == (unsigned int)index)
			continue;

		ret = clk_get_pll1_settings(clksrc, i);
		if (ret != 0)
			return ret;
	}

	pll1_settings.valid_id = PLL1_SETTINGS_VALID_ID;

	return 0;
}

void stm32mp1_clk_lp_save_opp_pll1_settings(uint8_t *data, size_t size)
{
	if ((size != sizeof(pll1_settings)) ||
	    !stm32mp1_clk_pll1_settings_are_valid())
		panic();

	memcpy(data, &pll1_settings, size);
}

bool stm32mp1_clk_pll1_settings_are_valid(void)
{
	return pll1_settings.valid_id == PLL1_SETTINGS_VALID_ID;
}

#ifdef CFG_DRIVERS_CLK
/*
 * Conversion between clk references and clock gates and clock on internals
 *
 * stm32mp1_clk first cells follow stm32mp1_clk_gate[] ordering.
 * stm32mp1_clk last cells follow stm32mp1_clk_on[] ordering.
 */
static struct clk stm32mp1_clk[ARRAY_SIZE(stm32mp1_clk_gate) +
			       ARRAY_SIZE(stm32mp1_clk_on)];

#define CLK_ON_INDEX_OFFSET	((int)ARRAY_SIZE(stm32mp1_clk_gate))

static bool clk_is_gate(struct clk *clk)
{
	int clk_index = clk - stm32mp1_clk;

	assert(clk_index >= 0 && clk_index < (int)ARRAY_SIZE(stm32mp1_clk));
	return clk_index < CLK_ON_INDEX_OFFSET;
}

static unsigned long clk_to_clock_id(struct clk *clk)
{
	int gate_index = clk - stm32mp1_clk;
	int on_index = gate_index - CLK_ON_INDEX_OFFSET;

	if (clk_is_gate(clk))
		return stm32mp1_clk_gate[gate_index].clock_id;

	return stm32mp1_clk_on[on_index];
}

static const struct stm32mp1_clk_gate *clk_to_gate_ref(struct clk *clk)
{
	int gate_index = clk - stm32mp1_clk;

	assert(clk_is_gate(clk));

	return stm32mp1_clk_gate + gate_index;
}

static int clock_id_to_gate_index(unsigned long clock_id)
{
	size_t n = 0;

	for (n = 0; n < ARRAY_SIZE(stm32mp1_clk_gate); n++)
		if (stm32mp1_clk_gate[n].clock_id == clock_id)
			return n;

	return -1;
}

static int clock_id_to_always_on_index(unsigned long clock_id)
{
	size_t n = 0;

	for (n = 0; n < ARRAY_SIZE(stm32mp1_clk_on); n++)
		if (stm32mp1_clk_on[n] == clock_id)
			return n;

	return -1;
}

static struct clk *clock_id_to_clk(unsigned long clock_id)
{
	int gate_index = clock_id_to_gate_index(clock_id);
	int on_index = clock_id_to_always_on_index(clock_id);

	if (gate_index >= 0)
		return stm32mp1_clk + gate_index;

	if (on_index >= 0)
		return stm32mp1_clk + CLK_ON_INDEX_OFFSET + on_index;

	return NULL;
}

struct clk *stm32mp_rcc_clock_id_to_clk(unsigned long clock_id)
{
	return clock_id_to_clk(clock_id);
}

unsigned int stm32mp_rcc_clk_to_clock_id(struct clk *clk)
{
	return clk_to_clock_id(clk);
}

#if CFG_TEE_CORE_LOG_LEVEL >= TRACE_DEBUG
struct clk_name {
	unsigned int clock_id;
	const char *name;
};

#define CLOCK_NAME(_binding, _name) \
	{ .clock_id = (_binding), .name = (_name) }

/* Store names only for some clocks */
const struct clk_name exposed_clk_name[] = {
	/* Clocks used by platform drivers not yet probed from DT */
	CLOCK_NAME(CK_DBG, "dbg"),
	CLOCK_NAME(CK_MCU, "mcu"),
	CLOCK_NAME(RTCAPB, "rtcapb"),
	CLOCK_NAME(BKPSRAM, "bkpsram"),
	CLOCK_NAME(RTC, "rtc"),
	CLOCK_NAME(CRYP1, "crpy1"),
	CLOCK_NAME(SYSCFG, "syscfg"),
	CLOCK_NAME(GPIOA, "gpioa"),
	CLOCK_NAME(GPIOB, "gpiob"),
	CLOCK_NAME(GPIOC, "gpioc"),
	CLOCK_NAME(GPIOD, "gpiod"),
	CLOCK_NAME(GPIOE, "gpioe"),
	CLOCK_NAME(GPIOF, "gpiof"),
	CLOCK_NAME(GPIOG, "gpiog"),
	CLOCK_NAME(GPIOH, "gpioh"),
	CLOCK_NAME(GPIOI, "gpioi"),
	CLOCK_NAME(GPIOJ, "gpioj"),
	CLOCK_NAME(GPIOK, "gpiok"),
	CLOCK_NAME(GPIOZ, "gpioz"),
	/* Clock exposed by SCMI. SCMI clock fmro DT bindings to come... */
	CLOCK_NAME(CK_HSE, "hse"),
	CLOCK_NAME(CK_HSI, "hsi"),
	CLOCK_NAME(CK_CSI, "csi"),
	CLOCK_NAME(CK_LSE, "lse"),
	CLOCK_NAME(CK_LSI, "lsi"),
	CLOCK_NAME(PLL2_Q, "pll2q"),
	CLOCK_NAME(PLL2_R, "pll2r"),
	CLOCK_NAME(PLL3_Q, "pll3q"),
	CLOCK_NAME(PLL3_R, "pll3r"),
	CLOCK_NAME(CRYP1, "cryp1"),
	CLOCK_NAME(HASH1, "hash1"),
	CLOCK_NAME(I2C4_K, "i2c4"),
	CLOCK_NAME(I2C6_K, "i2c6"),
	CLOCK_NAME(IWDG1, "iwdg"),
	CLOCK_NAME(RNG1_K, "rng1"),
	CLOCK_NAME(SPI6_K, "spi6"),
	CLOCK_NAME(USART1_K, "usart1"),
	CLOCK_NAME(CK_MCU, "mcu"),
};
DECLARE_KEEP_PAGER(exposed_clk_name);

static const char *clk_op_get_name(struct clk *clk)
{
	unsigned long clock_id = clk_to_clock_id(clk);
	size_t n = 0;

	for (n = 0; n < ARRAY_SIZE(exposed_clk_name); n++)
		if (exposed_clk_name[n].clock_id == clock_id)
			return exposed_clk_name[n].name;

	return NULL;
}
#else
static const char *clk_op_get_name(struct clk *clk __unused)
{
	return NULL;
}
#endif /*CFG_TEE_CORE_LOG_LEVEL*/

static unsigned long clk_op_compute_rate(struct clk *clk,
					 unsigned long parent_rate __unused)
{
	return _stm32_clock_get_rate(clk_to_clock_id(clk));
}

static TEE_Result clk_op_enable(struct clk *clk)
{
	if (clk_is_gate(clk))
		__clk_enable(clk_to_gate_ref(clk));

	return TEE_SUCCESS;
}
DECLARE_KEEP_PAGER(clk_op_enable);

static void clk_op_disable(struct clk *clk)
{
	if (clk_is_gate(clk))
		__clk_disable(clk_to_gate_ref(clk));
}
DECLARE_KEEP_PAGER(clk_op_disable);

/* This variable is weak to break its dependency chain when linked as unpaged */
const struct clk_ops stm32mp1_clk_ops
__weak __rodata_unpaged("stm32mp1_clk_ops") = {
	.enable = clk_op_enable,
	.disable = clk_op_disable,
	.get_rate = clk_op_compute_rate,
};

static TEE_Result register_stm32mp1_clocks(void)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	size_t n = 0;

	for (n = 0; n < ARRAY_SIZE(stm32mp1_clk); n++) {
		stm32mp1_clk[n].ops = &stm32mp1_clk_ops;
		stm32mp1_clk[n].name = clk_op_get_name(stm32mp1_clk + n);
		refcount_set(&stm32mp1_clk[n].enabled_count, 0);

		res = clk_register(stm32mp1_clk + n);
		if (res)
			return res;
	}

	return TEE_SUCCESS;
}

#ifdef CFG_DRIVERS_CLK_DT
static struct clk *stm32mp1_clk_dt_get_clk(struct dt_driver_phandle_args *pargs,
					   void *data __unused, TEE_Result *res)
{
	unsigned long clock_id = pargs->args[0];
	struct clk *clk = NULL;

	*res = TEE_ERROR_BAD_PARAMETERS;

	if (pargs->args_count != 1)
		return NULL;

	clk = clock_id_to_clk(clock_id);
	if (!clk)
		return NULL;

	*res = TEE_SUCCESS;
	return clk;
}

/* Non-null reference for compat data */
static const uint8_t non_secure_rcc;

static void enable_static_secure_clocks(void)
{
	unsigned int idx = 0;
	const unsigned long secure_enable[] = {
		DDRC1, DDRC1LP, DDRC2, DDRC2LP, DDRPHYC, DDRPHYCLP, DDRCAPB,
		AXIDCG, DDRPHYCAPB, DDRPHYCAPBLP, TZPC, TZC1, TZC2, STGEN_K,
		BSEC,
	};

	for (idx = 0; idx < ARRAY_SIZE(secure_enable); idx++) {
		clk_enable(stm32mp_rcc_clock_id_to_clk(secure_enable[idx]));
		stm32mp_register_clock_parents_secure(secure_enable[idx]);
	}

	if (CFG_TEE_CORE_NB_CORE > 1)
		clk_enable(stm32mp_rcc_clock_id_to_clk(RTCAPB));
}

static TEE_Result stm32mp1_clock_provider_probe(const void *fdt, int offs,
						const void *compat_data)
{
	TEE_Result res = TEE_ERROR_GENERIC;

	if (compat_data == &non_secure_rcc)
		disable_rcc_tzen();
	else
		enable_rcc_tzen();

	res = stm32mp1_clk_fdt_init(fdt, offs);
	if (res) {
		EMSG("Failed to initialize clocks from DT: %#"PRIx32, res);
		panic();
	}

	res = register_stm32mp1_clocks();
	if (res) {
		EMSG("Failed to register clocks: %#"PRIx32, res);
		panic();
	}

	res = clk_dt_register_clk_provider(fdt, offs, stm32mp1_clk_dt_get_clk,
					   NULL);
	if (res) {
		EMSG("Failed to register clock provider: %#"PRIx32, res);
		panic();
	}

	enable_static_secure_clocks();
	save_current_opp();

	return TEE_SUCCESS;
}

static const struct dt_device_match stm32mp1_clock_match_table[] = {
	{  .compatible = "st,stm32mp1-rcc", .compat_data = &non_secure_rcc, },
	{  .compatible = "st,stm32mp1-rcc-secure", },
	{ }
};

DEFINE_DT_DRIVER(stm32mp1_clock_dt_driver) = {
	.name = "stm32mp1_clock",
	.type = DT_DRIVER_CLK,
	.match_table = stm32mp1_clock_match_table,
	.probe = stm32mp1_clock_provider_probe,
};
#else /*CFG_DRIVERS_CLK_DT*/
static void stm32_clock_enable(unsigned long clock_id)
{
	int index = clock_id_to_gate_index(clock_id);

	if (index >= 0)
		__clk_enable(stm32mp1_clk_gate + index);

	assert(clock_id_to_always_on_index(clock_id) >= 0);
}

static void enable_static_secure_clocks(void)
{
	unsigned int idx = 0;
	const unsigned long secure_enable[] = {
		DDRC1, DDRC1LP, DDRC2, DDRC2LP, DDRPHYC, DDRPHYCLP, DDRCAPB,
		AXIDCG, DDRPHYCAPB, DDRPHYCAPBLP, TZPC, TZC1, TZC2, STGEN_K,
		BSEC,
	};
	for (idx = 0; idx < ARRAY_SIZE(secure_enable); idx++) {
		stm32_clock_enable(secure_enable[idx]);
		stm32mp_register_clock_parents_secure(secure_enable[idx]);
	}

	if (CFG_TEE_CORE_NB_CORE > 1)
		stm32_clock_enable(RTCAPB);
}

int stm32mp1_clk_compute_all_pll1_settings(uint32_t buck1_voltage __unused)
{
	return 0;
}

void stm32mp1_clk_lp_save_opp_pll1_settings(uint8_t *data __unused,
					    size_t size __unused)
{
}

bool stm32mp1_clk_pll1_settings_are_valid(void)
{
	return false;
}

static void enable_static_secure_clocks(void)
{
}

static TEE_Result stm32mp1_clk_early_init(void)
{
	TEE_Result __maybe_unused res = TEE_ERROR_GENERIC;

	res = register_stm32mp1_clocks();
	if (res) {
		EMSG("Failed to register clocks: %#"PRIx32, res);
		panic();
	}

	enable_static_secure_clocks();
	save_current_opp();

	return TEE_SUCCESS;
}

service_init(stm32mp1_clk_early_init);
#endif /* CFG_DRIVERS_CLK_DT */

#endif /* CFG_DRIVERS_CLK */

/* Start PMU OPP */
#define CLKSRC_TIMEOUT_US	(200 * 1000)
#define CLKDIV_TIMEOUT_US	(200 * 1000)
#define CLK_MPU_PLL1P		0x00000202
#define CLK_MPU_PLL1P_DIV	0x00000203

static int stm32mp1_set_clksrc(unsigned int clksrc)
{
	uintptr_t address = stm32_rcc_base() + (clksrc >> 4);
	uint64_t timeout_ref = 0;

	io_clrsetbits32(address, RCC_SELR_SRC_MASK, clksrc & RCC_SELR_SRC_MASK);

	timeout_ref = timeout_init_us(CLKSRC_TIMEOUT_US);
	while (!(io_read32(address) & RCC_SELR_SRCRDY))
		if (timeout_elapsed(timeout_ref))
			break;

	if (!(io_read32(address) & RCC_SELR_SRCRDY)) {
		EMSG("CLKSRC %u start failed @ 0x%"PRIxPTR": 0x%"PRIx32,
			      clksrc, address, io_read32(address));

		return -1;
	}

	return 0;
}

static int stm32mp1_set_clkdiv(unsigned int clkdiv, uintptr_t address)
{
	uint64_t timeout_ref = 0;

	io_clrsetbits32(address, RCC_DIVR_DIV_MASK, clkdiv & RCC_DIVR_DIV_MASK);

	timeout_ref = timeout_init_us(CLKDIV_TIMEOUT_US);
	while (!(io_read32(address) & RCC_DIVR_DIVRDY))
		if (timeout_elapsed(timeout_ref))
			break;

	if (!(io_read32(address) & RCC_DIVR_DIVRDY)) {
		EMSG("CLKDIV 0x%x start failed @ 0x%"PRIxPTR": 0x%"PRIx32,
		     clkdiv, address, io_read32(address));

		return -1;
	}

	return 0;
}

/*
 * Check if PLL1 can be configured on the fly.
 * @result  (-1) => config on the fly is not possible.
 *          (0)  => config on the fly is possible.
 *          (+1) => same parameters as those in place, no need to reconfig.
 * Return value is 0 if no error.
 */
static int is_pll_config_on_the_fly(enum stm32mp1_pll_id pll_id,
				    uint32_t *pllcfg, uint32_t fracv,
				    int *result)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uintptr_t rcc_base = stm32_rcc_base();
	uint32_t fracr = 0;
	uint32_t value = 0;
	int ret = 0;

	ret = pll_compute_pllxcfgr1(pll, pllcfg, &value);
	if (ret)
		return ret;

	if (io_read32(rcc_base + pll->pllxcfgr1) != value) {
		/* Different DIVN/DIVM, can't config on the fly */
		*result = -1;
		return 0;
	}

	*result = true;

	fracr = fracv << RCC_PLLNFRACR_FRACV_SHIFT;
	fracr |= RCC_PLLNFRACR_FRACLE;
	value = pll_compute_pllxcfgr2(pllcfg);

	if ((io_read32(rcc_base + pll->pllxfracr) == fracr) &&
	    (io_read32(rcc_base + pll->pllxcfgr2) == value))
		/* Same parameters, no need to config */
		*result = 1;
	else
		*result = 0;

	return 0;
}

static int stm32mp1_get_mpu_div(uint32_t freq_khz)
{
	unsigned long freq_pll1_p;
	unsigned long div;

	freq_pll1_p = get_clock_rate(_PLL1_P) / 1000UL;
	if ((freq_pll1_p % freq_khz) != 0U)
		return -1;

	div = freq_pll1_p / freq_khz;

	switch (div) {
	case 1UL:
	case 2UL:
	case 4UL:
	case 8UL:
	case 16UL:
		return __builtin_ffs(div) - 1;
	default:
		return -1;
	}
}

/* Configure PLL1 from input frequency OPP parameters */
static int pll1_config_from_opp_khz(uint32_t freq_khz)
{
	unsigned int idx = 0;
	int ret = 0;
	int div = 0;
	int config_on_the_fly = -1;

	for (idx = 0; idx < PLAT_MAX_OPP_NB; idx++)
		if (pll1_settings.freq[idx] == freq_khz)
			break;

	if (idx == PLAT_MAX_OPP_NB)
		return -1;

	div = stm32mp1_get_mpu_div(freq_khz);
	switch (div) {
	case -1:
		break;
	case 0:
		return stm32mp1_set_clksrc(CLK_MPU_PLL1P);
	default:
		ret = stm32mp1_set_clkdiv(div, stm32_rcc_base() +
					  RCC_MPCKDIVR);
		if (ret == 0)
			ret = stm32mp1_set_clksrc(CLK_MPU_PLL1P_DIV);

		return ret;
	}

	ret = is_pll_config_on_the_fly(_PLL1, &pll1_settings.cfg[idx][0],
				       pll1_settings.frac[idx],
				       &config_on_the_fly);
	if (ret)
		return ret;

	if (config_on_the_fly == 1)
		return 0;

	if (config_on_the_fly == -1) {
		/* Switch to HSI and stop PLL1 before reconfiguration */
		ret = stm32mp1_set_clksrc(CLK_MPU_HSI);
		if (ret)
			return ret;

		ret = pll_stop(_PLL1);
		if (ret)
			return ret;
	}

	ret = pll_config(_PLL1, &pll1_settings.cfg[idx][0],
			 pll1_settings.frac[idx]);
	if (ret)
		return ret;

	if (config_on_the_fly == -1) {
		/* Start PLL1 and switch back to after reconfiguration */
		pll_start(_PLL1);

		ret = pll_output(_PLL1, pll1_settings.cfg[idx][PLLCFG_O]);
		if (ret)
			return ret;

		ret = stm32mp1_set_clksrc(CLK_MPU_PLL1P);
		if (ret)
			return ret;
	}

	return 0;
}

int stm32mp1_set_opp_khz(uint32_t freq_khz)
{
	uint32_t mpu_src = 0;

	if (freq_khz == current_opp_khz)
		return 0;

	if (!stm32mp1_clk_pll1_settings_are_valid()) {
		/*
		 * No OPP table in DT or an error occurred during PLL1
		 * settings computation, system can only work on current
		 * operating point so return error.
		 */
		return -1;
	}

	/* Check that PLL1 is MPU clock source */
	mpu_src = io_read32(stm32_rcc_base() + RCC_MPCKSELR) &
		RCC_SELR_SRC_MASK;
	if ((mpu_src != RCC_MPCKSELR_PLL) &&
	    (mpu_src != RCC_MPCKSELR_PLL_MPUDIV))
		return -1;

	if (pll1_config_from_opp_khz(freq_khz)) {
		/* Restore original value */
		if (pll1_config_from_opp_khz(current_opp_khz)) {
			EMSG("No CPU operating point can be set");
			panic();
		}

		return -1;
	}

	current_opp_khz = freq_khz;

	return 0;
}

int stm32mp1_round_opp_khz(uint32_t *freq_khz)
{
	unsigned int i = 0;
	uint32_t round_opp = 0;

	if (!stm32mp1_clk_pll1_settings_are_valid()) {
		/*
		 * No OPP table in DT, or an error occurred during PLL1
		 * settings computation, system can only work on current
		 * operating point, so return current CPU frequency.
		 */
		*freq_khz = current_opp_khz;

		return 0;
	}

	for (i = 0; i < PLAT_MAX_OPP_NB; i++)
		if ((pll1_settings.freq[i] <= *freq_khz) &&
		    (pll1_settings.freq[i] > round_opp))
			round_opp = pll1_settings.freq[i];

	*freq_khz = round_opp;

	return 0;
}
/* End MPU OPP */

#ifndef CFG_DRIVERS_CLK
/*
 * Minimial clock support when there's no embedded DTB.
 * Implements legacy stm32_clock_*() function that only enables target clocks.
 */
bool stm32_clock_is_enabled(unsigned long clock_id)
{
	int index = clock_id_to_gate_index(clock_id);

	if (index >= 0)
		return __clk_is_enabled(stm32mp1_clk_gate + index);

	assert(clock_id_to_always_on_index(clock_id) >= 0);
	return true;
}

void stm32_clock_enable(unsigned long clock_id)
{
	int index = clock_id_to_gate_index(clock_id);

	if (index >= 0)
		__clk_enable(stm32mp1_clk_gate + index);

	assert(clock_id_to_always_on_index(clock_id) >= 0);
}

void stm32_clock_disable(unsigned long clock_id __unused)
{
	/* No refcounting: do not disable the clock */
}
#endif /*!CFG_DRIVERS_CLK*/
