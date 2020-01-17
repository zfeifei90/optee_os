// SPDX-License-Identifier:	GPL-2.0+	BSD-3-Clause
/*
 * Copyright (C) 2017-2019, STMicroelectronics - All Rights Reserved
 */

#include <assert.h>
#include <initcall.h>
#include <kernel/delay.h>
#include <kernel/dt.h>
#include <drivers/stm32mp1_clk.h>
#include <drivers/stm32mp1_rcc.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <dt-bindings/clock/stm32mp1-clksrc.h>
#include <initcall.h>
#include <io.h>
#include <keep.h>
#include <kernel/generic_boot.h>
#include <kernel/panic.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <kernel/spinlock.h>
#include <stdint.h>
#include <stdio.h>
#include <stm32_util.h>
#include <stm32mp_dt.h>
#include <stm32mp_pm.h>
#include <trace.h>
#include <util.h>

#ifdef CFG_DT
#include <drivers/stm32mp1_clkfunc.h>
#include <libfdt.h>
#endif

#define CLKSRC_TIMEOUT_US	200000U
#define PLLRDY_TIMEOUT_US	200000U

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

enum stm32mp1_parent_id {
/* Oscillators are defined in enum stm32mp_osc_id */

/* Other parent source */
	_HSI_KER = NB_OSC,
	_HSE_KER,
	_HSE_KER_DIV2,
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
	_HCLK6,
	_HCLK2,
	_CK_PER,
	_CK_MPU,
	_CK_MCU,
	_PARENT_NB,
	_UNKNOWN_ID = 0xff,
};

/* Lists only the parent clock we are interested in */
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
	_SDMMC12_SEL,
	_SDMMC3_SEL,
	_ASS_SEL,
	_MSS_SEL,
	_USBPHY_SEL,
	_USBO_SEL,
	_PARENT_SEL_NB,
	_UNKNOWN_SEL = 0xff,
};

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

enum stm32mp1_clksrc_id {
	CLKSRC_MPU,
	CLKSRC_AXI,
	CLKSRC_MCU,
	CLKSRC_PLL12,
	CLKSRC_PLL3,
	CLKSRC_PLL4,
	CLKSRC_RTC,
	CLKSRC_MCO1,
	CLKSRC_MCO2,
	CLKSRC_NB
};

enum stm32mp1_clkdiv_id {
	CLKDIV_MPU,
	CLKDIV_AXI,
	CLKDIV_MCU,
	CLKDIV_APB1,
	CLKDIV_APB2,
	CLKDIV_APB3,
	CLKDIV_APB4,
	CLKDIV_APB5,
	CLKDIV_RTC,
	CLKDIV_MCO1,
	CLKDIV_MCO2,
	CLKDIV_NB
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

enum stm32mp1_pllcsg {
	PLLCSG_MOD_PER,
	PLLCSG_INC_STEP,
	PLLCSG_SSCG_MODE,
	PLLCSG_NB
};

enum stm32mp1_plltype {
	PLL_800,
	PLL_1600,
	PLL_TYPE_NB
};

struct stm32mp1_pll {
	uint8_t refclk_min;
	uint8_t refclk_max;
	uint8_t divn_max;
};

struct stm32mp1_clk_gate {
	uint16_t offset;
	uint8_t bit;
	uint8_t index;
	uint8_t set_clr;
	uint8_t sel; /* Relates to enum stm32mp1_parent_sel */
	uint8_t fixed; /* Relates to enum stm32mp1_parent_id */
};

struct stm32mp1_clk_sel {
	uint16_t offset;
	uint8_t src;
	uint8_t msk;
	uint8_t nb_parent;
	const uint8_t *parent;
};

#define REFCLK_SIZE 4
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

/* Compact structure of 32bit cells, copied raw when suspending */
struct __attribute__((__packed__)) stm32mp1_pll_settings {
	uint32_t valid_id;
	uint32_t freq[PLAT_MAX_OPP_NB];
	uint32_t volt[PLAT_MAX_OPP_NB];
	uint32_t cfg[PLAT_MAX_OPP_NB][PLAT_MAX_PLLCFG_NB];
	uint32_t frac[PLAT_MAX_OPP_NB];
};

/* Clocks with selectable source and not set/clr register access */
#define _CLK_SELEC(off, b, idx, s)			\
	{						\
		.offset = (off),			\
		.bit = (b),				\
		.index = (idx),				\
		.set_clr = 0,				\
		.sel = (s),				\
		.fixed = _UNKNOWN_ID,			\
	}

/* Clocks with fixed source and not set/clr register access */
#define _CLK_FIXED(off, b, idx, f)			\
	{						\
		.offset = (off),			\
		.bit = (b),				\
		.index = (idx),				\
		.set_clr = 0,				\
		.sel = _UNKNOWN_SEL,			\
		.fixed = (f),				\
	}

/* Clocks with selectable source and set/clr register access */
#define _CLK_SC_SELEC(off, b, idx, s)			\
	{						\
		.offset = (off),			\
		.bit = (b),				\
		.index = (idx),				\
		.set_clr = 1,				\
		.sel = (s),				\
		.fixed = _UNKNOWN_ID,			\
	}

/* Clocks with fixed source and set/clr register access */
#define _CLK_SC_FIXED(off, b, idx, f)			\
	{						\
		.offset = (off),			\
		.bit = (b),				\
		.index = (idx),				\
		.set_clr = 1,				\
		.sel = _UNKNOWN_SEL,			\
		.fixed = (f),				\
	}

/*
 * Clocks with selectable source and set/clr register access
 * and enable bit position defined by a label (argument b)
 */
#define _CLK_SC2_SELEC(off, b, idx, s)			\
	{						\
		.offset = (off),			\
		.index = (idx),				\
		.bit = off ## _ ## b ## _POS,		\
		.set_clr = 1,				\
		.sel = (s),				\
		.fixed = _UNKNOWN_ID,			\
	}
#define _CLK_SC2_FIXED(off, b, idx, f)			\
	{						\
		.offset = (off),			\
		.index = (idx),				\
		.bit = off ## _ ## b ## _POS,		\
		.set_clr = 1,				\
		.sel = _UNKNOWN_SEL,			\
		.fixed = (f),				\
	}

#define _CLK_PARENT(idx, off, s, m, p)			\
	[(idx)] = {					\
		.offset = (off),			\
		.src = (s),				\
		.msk = (m),				\
		.parent = (p),				\
		.nb_parent = ARRAY_SIZE(p)		\
	}

#define _CLK_PLL(idx, type, off1, off2, off3,		\
		 off4, off5, off6,			\
		 p1, p2, p3, p4)			\
	[(idx)] = {					\
		.plltype = (type),			\
		.rckxselr = (off1),			\
		.pllxcfgr1 = (off2),			\
		.pllxcfgr2 = (off3),			\
		.pllxfracr = (off4),			\
		.pllxcr = (off5),			\
		.pllxcsgr = (off6),			\
		.refclk[0] = (p1),			\
		.refclk[1] = (p2),			\
		.refclk[2] = (p3),			\
		.refclk[3] = (p4),			\
	}

static const uint8_t stm32mp1_clks[][2] = {
	{ CK_PER, _CK_PER },
	{ CK_MPU, _CK_MPU },
	{ CK_AXI, _ACLK },
	{ CK_MCU, _CK_MCU },
	{ CK_HSE, _HSE },
	{ CK_CSI, _CSI },
	{ CK_LSI, _LSI },
	{ CK_LSE, _LSE },
	{ CK_HSI, _HSI },
	{ CK_HSE_DIV2, _HSE_KER_DIV2 },
};

#define NB_GATES	ARRAY_SIZE(stm32mp1_clk_gate)

static const struct stm32mp1_clk_gate stm32mp1_clk_gate[] = {
	_CLK_FIXED(RCC_DDRITFCR, 0, DDRC1, _ACLK),
	_CLK_FIXED(RCC_DDRITFCR, 1, DDRC1LP, _ACLK),
	_CLK_FIXED(RCC_DDRITFCR, 2, DDRC2, _ACLK),
	_CLK_FIXED(RCC_DDRITFCR, 3, DDRC2LP, _ACLK),
	_CLK_FIXED(RCC_DDRITFCR, 4, DDRPHYC, _PLL2_R),
	_CLK_FIXED(RCC_DDRITFCR, 5, DDRPHYCLP, _PLL2_R),
	_CLK_FIXED(RCC_DDRITFCR, 6, DDRCAPB, _PCLK4),
	_CLK_FIXED(RCC_DDRITFCR, 7, DDRCAPBLP, _PCLK4),
	_CLK_FIXED(RCC_DDRITFCR, 8, AXIDCG, _ACLK),
	_CLK_FIXED(RCC_DDRITFCR, 9, DDRPHYCAPB, _PCLK4),
	_CLK_FIXED(RCC_DDRITFCR, 10, DDRPHYCAPBLP, _PCLK4),

	_CLK_SC2_SELEC(RCC_MP_APB5ENSETR, SPI6EN, SPI6_K, _SPI6_SEL),
	_CLK_SC2_SELEC(RCC_MP_APB5ENSETR, I2C4EN, I2C4_K, _I2C46_SEL),
	_CLK_SC2_SELEC(RCC_MP_APB5ENSETR, I2C6EN, I2C6_K, _I2C46_SEL),
	_CLK_SC2_SELEC(RCC_MP_APB5ENSETR, USART1EN, USART1_K, _USART1_SEL),
	_CLK_SC2_FIXED(RCC_MP_APB5ENSETR, RTCAPBEN, RTCAPB, _PCLK5),
	_CLK_SC2_FIXED(RCC_MP_APB5ENSETR, TZC1EN, TZC1, _PCLK5),
	_CLK_SC2_FIXED(RCC_MP_APB5ENSETR, TZC2EN, TZC2, _PCLK5),
	_CLK_SC2_FIXED(RCC_MP_APB5ENSETR, TZPCEN, TZPC, _PCLK5),
	_CLK_SC2_FIXED(RCC_MP_APB5ENSETR, IWDG1APBEN, IWDG1, _PCLK5),
	_CLK_SC2_FIXED(RCC_MP_APB5ENSETR, BSECEN, BSEC, _PCLK5),
	_CLK_SC2_SELEC(RCC_MP_APB5ENSETR, STGENEN, STGEN_K, _STGEN_SEL),

	_CLK_SC2_FIXED(RCC_MP_AHB5ENSETR, GPIOZEN, GPIOZ, _PCLK5),
	_CLK_SC2_FIXED(RCC_MP_AHB5ENSETR, CRYP1EN, CRYP1, _PCLK5),
	_CLK_SC2_FIXED(RCC_MP_AHB5ENSETR, HASH1EN, HASH1, _PCLK5),
	_CLK_SC2_SELEC(RCC_MP_AHB5ENSETR, RNG1EN, RNG1_K, _RNG1_SEL),
	_CLK_SC2_FIXED(RCC_MP_AHB5ENSETR, BKPSRAMEN, BKPSRAM, _PCLK5),

	/* Non-secure clocks */
#ifdef CFG_WITH_NSEC_GPIOS
	_CLK_SC_FIXED(RCC_MP_AHB4ENSETR, 0, GPIOA, _UNKNOWN_ID),
	_CLK_SC_FIXED(RCC_MP_AHB4ENSETR, 1, GPIOB, _UNKNOWN_ID),
	_CLK_SC_FIXED(RCC_MP_AHB4ENSETR, 2, GPIOC, _UNKNOWN_ID),
	_CLK_SC_FIXED(RCC_MP_AHB4ENSETR, 3, GPIOD, _UNKNOWN_ID),
	_CLK_SC_FIXED(RCC_MP_AHB4ENSETR, 4, GPIOE, _UNKNOWN_ID),
	_CLK_SC_FIXED(RCC_MP_AHB4ENSETR, 5, GPIOF, _UNKNOWN_ID),
	_CLK_SC_FIXED(RCC_MP_AHB4ENSETR, 6, GPIOG, _UNKNOWN_ID),
	_CLK_SC_FIXED(RCC_MP_AHB4ENSETR, 7, GPIOH, _UNKNOWN_ID),
	_CLK_SC_FIXED(RCC_MP_AHB4ENSETR, 8, GPIOI, _UNKNOWN_ID),
	_CLK_SC_FIXED(RCC_MP_AHB4ENSETR, 9, GPIOJ, _UNKNOWN_ID),
	_CLK_SC_FIXED(RCC_MP_AHB4ENSETR, 10, GPIOK, _UNKNOWN_ID),
#endif
#ifdef CFG_WITH_NSEC_UARTS
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 14, USART2_K, _UART24_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 15, USART3_K, _UART35_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 16, UART4_K, _UART24_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 17, UART5_K, _UART35_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 18, UART7_K, _UART78_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 19, UART8_K, _UART78_SEL),
	_CLK_SC_SELEC(RCC_MP_APB2ENSETR, 13, USART6_K, _UART6_SEL),
#endif
	_CLK_SC_SELEC(RCC_MP_APB4ENSETR, 0, LTDC_PX, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_APB4ENSETR, 8, DDRPERFM, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_APB4ENSETR, 15, IWDG2, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_APB4ENSETR, 16, USBPHY_K, _USBPHY_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB2ENSETR, 0, DMA1, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB2ENSETR, 1, DMA2, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB2ENSETR, 8, USBO_K, _USBO_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB2ENSETR, 16, SDMMC3_K, _SDMMC3_SEL),
	_CLK_SELEC(RCC_DBGCFGR, 8, CK_DBG, _UNKNOWN_SEL),
	_CLK_SC_FIXED(RCC_MP_APB1ENSETR, 6, TIM12_K, _PCLK1),
	_CLK_SC_FIXED(RCC_MP_APB2ENSETR, 2, TIM15_K, _PCLK2),
	_CLK_SC_FIXED(RCC_MP_APB3ENSETR, 11, SYSCFG, _UNKNOWN_ID),
	_CLK_SC_SELEC(RCC_MP_AHB6ENSETR, 0, MDMA, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB6ENSETR, 5, GPU, _UNKNOWN_SEL),
	_CLK_SC_FIXED(RCC_MP_AHB6ENSETR, 10, ETHMAC, _ACLK),
	_CLK_SC_SELEC(RCC_MP_AHB6ENSETR, 16, SDMMC1_K, _SDMMC12_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB6ENSETR, 17, SDMMC2_K, _SDMMC12_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB6ENSETR, 24, USBH, _UNKNOWN_SEL),
};
KEEP_PAGER(stm32mp1_clk_gate);

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

/* Parents for (some) non-secure clocks */
static const uint8_t uart6_parents[] = {
	_PCLK2, _PLL4_Q, _HSI_KER, _CSI_KER, _HSE_KER
};

static const uint8_t uart234578_parents[] = {
	_PCLK1, _PLL4_Q, _HSI_KER, _CSI_KER, _HSE_KER
};

static const uint8_t sdmmc12_parents[] = {
	_HCLK6, _PLL3_R, _PLL4_P, _HSI_KER
};

static const uint8_t sdmmc3_parents[] = {
	_HCLK2, _PLL3_R, _PLL4_P, _HSI_KER
};

static const uint8_t ass_parents[] = {
	_HSI, _HSE, _PLL2
};

static const uint8_t mss_parents[] = {
	_HSI, _HSE, _CSI, _PLL3
};

static const uint8_t usbphy_parents[] = {
	_HSE_KER, _PLL4_R, _HSE_KER_DIV2
};

static const uint8_t usbo_parents[] = {
	_PLL4_R, _USB_PHY_48
};

static const struct stm32mp1_clk_sel stm32mp1_clk_sel[_PARENT_SEL_NB] = {
	/* Secure aware clocks */
	_CLK_PARENT(_STGEN_SEL, RCC_STGENCKSELR, 0, 0x3, stgen_parents),
	_CLK_PARENT(_I2C46_SEL, RCC_I2C46CKSELR, 0, 0x7, i2c46_parents),
	_CLK_PARENT(_SPI6_SEL, RCC_SPI6CKSELR, 0, 0x7, spi6_parents),
	_CLK_PARENT(_USART1_SEL, RCC_UART1CKSELR, 0, 0x7, usart1_parents),
	_CLK_PARENT(_RNG1_SEL, RCC_RNG1CKSELR, 0, 0x3, rng1_parents),
	/* Always non-secure clocks (maybe used in some way in secure world) */
	_CLK_PARENT(_UART6_SEL, RCC_UART6CKSELR, 0, 0x7, uart6_parents),
	_CLK_PARENT(_UART24_SEL, RCC_UART24CKSELR, 0, 0x7, uart234578_parents),
	_CLK_PARENT(_UART35_SEL, RCC_UART35CKSELR, 0, 0x7, uart234578_parents),
	_CLK_PARENT(_UART78_SEL, RCC_UART78CKSELR, 0, 0x7, uart234578_parents),
	_CLK_PARENT(_SDMMC12_SEL, RCC_SDMMC12CKSELR, 0, 0x7, sdmmc12_parents),
	_CLK_PARENT(_SDMMC3_SEL, RCC_SDMMC3CKSELR, 0, 0x7, sdmmc3_parents),
	_CLK_PARENT(_ASS_SEL, RCC_ASSCKSELR, 0, 0x3, ass_parents),
	_CLK_PARENT(_MSS_SEL, RCC_MSSCKSELR, 0, 0x3, mss_parents),
	_CLK_PARENT(_USBPHY_SEL, RCC_USBCKSELR, 0, 0x3, usbphy_parents),
	_CLK_PARENT(_USBO_SEL, RCC_USBCKSELR, 4, 0x1, usbo_parents),
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
		 _HSI, _HSE, _UNKNOWN_OSC_ID, _UNKNOWN_OSC_ID),
	_CLK_PLL(_PLL2, PLL_1600,
		 RCC_RCK12SELR, RCC_PLL2CFGR1, RCC_PLL2CFGR2,
		 RCC_PLL2FRACR, RCC_PLL2CR, RCC_PLL2CSGR,
		 _HSI, _HSE, _UNKNOWN_OSC_ID, _UNKNOWN_OSC_ID),
	_CLK_PLL(_PLL3, PLL_800,
		 RCC_RCK3SELR, RCC_PLL3CFGR1, RCC_PLL3CFGR2,
		 RCC_PLL3FRACR, RCC_PLL3CR, RCC_PLL3CSGR,
		 _HSI, _HSE, _CSI, _UNKNOWN_OSC_ID),
	_CLK_PLL(_PLL4, PLL_800,
		 RCC_RCK4SELR, RCC_PLL4CFGR1, RCC_PLL4CFGR2,
		 RCC_PLL4FRACR, RCC_PLL4CR, RCC_PLL4CSGR,
		 _HSI, _HSE, _CSI, _I2S_CKIN),
};

/* Prescaler table lookups for clock computation */
/* div = /1 /2 /4 /8 / 16 /64 /128 /512 */
static const uint8_t stm32mp1_mcu_div[16] = {
	0, 1, 2, 3, 4, 6, 7, 8, 9, 9, 9, 9, 9, 9, 9, 9
};

/* div = /1 /2 /4 /8 /16 : same divider for PMU and APBX */
#define stm32mp1_mpu_div stm32mp1_mpu_apbx_div
#define stm32mp1_apbx_div stm32mp1_mpu_apbx_div
static const uint8_t stm32mp1_mpu_apbx_div[8] = {
	0, 1, 2, 3, 4, 4, 4, 4
};

/* div = /1 /2 /3 /4 */
static const uint8_t stm32mp1_axi_div[8] = {
	1, 2, 3, 4, 4, 4, 4, 4
};

#if TRACE_LEVEL >= TRACE_DEBUG
static const char *const __maybe_unused stm32mp1_clk_parent_name[_PARENT_NB] = {
	[_HSI] = "HSI",
	[_HSE] = "HSE",
	[_CSI] = "CSI",
	[_LSI] = "LSI",
	[_LSE] = "LSE",
	[_I2S_CKIN] = "I2S_CKIN",
	[_HSI_KER] = "HSI_KER",
	[_HSE_KER] = "HSE_KER",
	[_HSE_KER_DIV2] = "HSE_KER_DIV2",
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
	[_HCLK6] = "KCLK6",
	[_HCLK2] = "HCLK2",
	[_CK_PER] = "CK_PER",
	[_CK_MPU] = "CK_MPU",
	[_CK_MCU] = "CK_MCU",
	[_USB_PHY_48] = "USB_PHY_48",
};
#endif

/* RCC clock device driver private */
static unsigned long stm32mp1_osc[NB_OSC];
static unsigned int gate_refcounts[NB_GATES];
static unsigned int refcount_lock;
static struct stm32mp1_pll_settings pll1_settings;
static uint32_t current_opp_khz;

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

static unsigned int get_id_from_rcc_bit(unsigned int offset, unsigned int bit)
{
	unsigned int idx;

	for (idx = 0; idx < NB_GATES; idx++) {
		const struct stm32mp1_clk_gate *gate = gate_ref(idx);

		if ((offset == gate->offset) && (bit == gate->bit)) {
			return gate->index;
		}

		if ((gate->set_clr != 0U) &&
		    (offset == (gate->offset + RCC_MP_ENCLRR_OFFSET)) &&
		    (bit == gate->bit)) {
			return gate->index;
		}
	}

	/* Currently only supported gated clocks */
	return ~0U;
}

static unsigned long stm32mp1_clk_get_fixed(enum stm32mp_osc_id idx)
{
	if (idx >= NB_OSC) {
		DMSG("clk id %d not found", idx);
		return 0;
	}

	return stm32mp1_osc[idx];
}

static int stm32mp1_clk_get_gated_id(unsigned long id)
{
	unsigned int i;

	for (i = 0; i < NB_GATES; i++) {
		if (gate_ref(i)->index == id) {
			return i;
		}
	}

	DMSG("clk id %lu not found", id);
	return -1;
}

static enum stm32mp1_parent_sel stm32mp1_clk_get_sel(int i)
{
	return (enum stm32mp1_parent_sel)(gate_ref(i)->sel);
}

static enum stm32mp1_parent_id stm32mp1_clk_get_fixed_parent(int i)
{
	return (enum stm32mp1_parent_id)gate_ref(i)->fixed;
}

static void pll_start(enum stm32mp1_pll_id pll_id)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uint32_t pllxcr = stm32_rcc_base() + pll->pllxcr;

	mmio_clrsetbits_32(pllxcr,
			   RCC_PLLNCR_DIVPEN | RCC_PLLNCR_DIVQEN |
			   RCC_PLLNCR_DIVREN,
			   RCC_PLLNCR_PLLON);
}

static int pll_output(enum stm32mp1_pll_id pll_id, uint32_t output)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uint32_t pllxcr = stm32_rcc_base() + pll->pllxcr;
	uint64_t start;

	start = utimeout_init(PLLRDY_TIMEOUT_US);
	/* Wait PLL lock */
	while ((mmio_read_32(pllxcr) & RCC_PLLNCR_PLLRDY) == 0U) {
		if (utimeout_elapsed(PLLRDY_TIMEOUT_US, start)) {
			EMSG("PLL%d start failed @ 0x%"PRIx32": 0x%"PRIx32,
			     pll_id, pllxcr, mmio_read_32(pllxcr));
			return -1;
		}
	}

	/* Start the requested output */
	mmio_setbits_32(pllxcr, output << RCC_PLLNCR_DIVEN_SHIFT);

	return 0;
}

static int pll_stop(enum stm32mp1_pll_id pll_id)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uint32_t pllxcr = stm32_rcc_base() + pll->pllxcr;
	uint64_t start;

	/* Stop all output */
	mmio_clrbits_32(pllxcr, RCC_PLLNCR_DIVPEN | RCC_PLLNCR_DIVQEN |
			RCC_PLLNCR_DIVREN);

	/* Stop PLL */
	mmio_clrbits_32(pllxcr, RCC_PLLNCR_PLLON);

	start = utimeout_init(PLLRDY_TIMEOUT_US);
	/* Wait PLL stopped */
	while ((mmio_read_32(pllxcr) & RCC_PLLNCR_PLLRDY) != 0U) {
		if (utimeout_elapsed(PLLRDY_TIMEOUT_US, start)) {
			EMSG("PLL%d stop failed @ 0x%"PRIx32": 0x%"PRIx32,
			     pll_id, pllxcr, mmio_read_32(pllxcr));

			return -1;
		}
	}

	return 0;
}

static uint32_t pll_compute_pllxcfgr2(uint32_t *pllcfg)
{
	uint32_t value;

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
	uint32_t value;

	value = pll_compute_pllxcfgr2(pllcfg);

	mmio_write_32(rcc_base + pll->pllxcfgr2, value);
}

static int pll_compute_pllxcfgr1(const struct stm32mp1_clk_pll *pll,
				 uint32_t *pllcfg, uint32_t *cfgr1)
{
	uint32_t rcc_base = stm32_rcc_base();
	enum stm32mp1_plltype type = pll->plltype;
	unsigned long refclk;
	uint32_t ifrge = 0;
	uint32_t src;

	src = mmio_read_32(rcc_base + pll->rckxselr) &
	      RCC_SELR_REFCLK_SRC_MASK;

	refclk = stm32mp1_clk_get_fixed(pll->refclk[src]) /
		 (pllcfg[PLLCFG_M] + 1U);

	if ((refclk < (stm32mp1_pll[type].refclk_min * 1000000U)) ||
	    (refclk > (stm32mp1_pll[type].refclk_max * 1000000U))) {
		return -1;
	}

	if ((type == PLL_800) && (refclk >= 8000000U)) {
		ifrge = 1U;
	}

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
	uint32_t value;
	int ret;

	ret = pll_compute_pllxcfgr1(pll, pllcfg, &value);
	if (ret != 0) {
		return ret;
	}

	mmio_write_32(rcc_base + pll->pllxcfgr1, value);

	/* Fractional configuration */
	value = 0;
	mmio_write_32(rcc_base + pll->pllxfracr, value);

	/* Frac must be enabled only once its configuration is loaded */
	value = fracv << RCC_PLLNFRACR_FRACV_SHIFT;
	mmio_write_32(rcc_base + pll->pllxfracr, value);
	value = mmio_read_32(rcc_base + pll->pllxfracr);
	mmio_write_32(rcc_base + pll->pllxfracr, value | RCC_PLLNFRACR_FRACLE);

	pll_config_output(pll_id, pllcfg);

	return 0;
}

static int stm32mp1_set_clksrc(unsigned int clksrc)
{
	uintptr_t address = stm32_rcc_base() + (clksrc >> 4);
	uint64_t timeout_ref;

	mmio_clrsetbits_32(address, RCC_SELR_SRC_MASK,
			   clksrc & RCC_SELR_SRC_MASK);

	timeout_ref = utimeout_init(CLKSRC_TIMEOUT_US);
	while ((mmio_read_32(address) & RCC_SELR_SRCRDY) == 0U) {
		if (utimeout_elapsed(CLKSRC_TIMEOUT_US, timeout_ref)) {
			EMSG("CLKSRC %x start failed @%"PRIxPTR": 0x%"PRIx32"\n",
			      clksrc, address, mmio_read_32(address));
			return -1;
		}
	}

	return 0;
}

static int stm32mp1_clk_get_parent(unsigned long id)
{
	const struct stm32mp1_clk_sel *sel;
	unsigned int j;
	uint32_t p_sel;
	int i;
	enum stm32mp1_parent_id p;
	enum stm32mp1_parent_sel s;
	uintptr_t rcc_base = stm32_rcc_base();

	for (j = 0U; j < ARRAY_SIZE(stm32mp1_clks); j++) {
		if (stm32mp1_clks[j][0] == id) {
			return (int)stm32mp1_clks[j][1];
		}
	}

	i = stm32mp1_clk_get_gated_id(id);
	if (i < 0) {
		panic();
	}

	p = stm32mp1_clk_get_fixed_parent(i);
	if (p < _PARENT_NB) {
		return (int)p;
	}

	s = stm32mp1_clk_get_sel(i);
	if (s == _UNKNOWN_SEL) {
		return -1;
	}
	if (s >= _PARENT_SEL_NB) {
		panic();
	}

	sel = clk_sel_ref(s);
	p_sel = (mmio_read_32(rcc_base + sel->offset) >> sel->src) & sel->msk;
	if (p_sel < sel->nb_parent) {
		return (int)sel->parent[p_sel];
	}

	DMSG("No parent selected for clk %lu", id);
	return -1;
}

static unsigned long stm32mp1_pll_get_fref(const struct stm32mp1_clk_pll *pll)
{
	uint32_t selr = mmio_read_32(stm32_rcc_base() + pll->rckxselr);
	uint32_t src = selr & RCC_SELR_REFCLK_SRC_MASK;

	return stm32mp1_clk_get_fixed(pll->refclk[src]);
}

/*
 * pll_get_fvco() : return the VCO or (VCO / 2) frequency for the requested PLL
 * - PLL1 & PLL2 => return VCO / 2 with Fpll_y_ck = FVCO / 2 * (DIVy + 1)
 * - PLL3 & PLL4 => return VCO     with Fpll_y_ck = FVCO / (DIVy + 1)
 * => in all cases Fpll_y_ck = pll_get_fvco() / (DIVy + 1)
 */
static unsigned long stm32mp1_pll_get_fvco(const struct stm32mp1_clk_pll *pll)
{
	unsigned long refclk, fvco;
	uint32_t cfgr1, fracr, divm, divn;

	cfgr1 = mmio_read_32(stm32_rcc_base() + pll->pllxcfgr1);
	fracr = mmio_read_32(stm32_rcc_base() + pll->pllxfracr);

	divm = (cfgr1 & (RCC_PLLNCFGR1_DIVM_MASK)) >> RCC_PLLNCFGR1_DIVM_SHIFT;
	divn = cfgr1 & RCC_PLLNCFGR1_DIVN_MASK;

	refclk = stm32mp1_pll_get_fref(pll);

	/*
	 * With FRACV :
	 *   Fvco = Fck_ref * ((DIVN + 1) + FRACV / 2^13) / (DIVM + 1)
	 * Without FRACV
	 *   Fvco = Fck_ref * ((DIVN + 1) / (DIVM + 1)
	 */
	if ((fracr & RCC_PLLNFRACR_FRACLE) != 0U) {
		unsigned long long numerator;
		unsigned long long denominator;
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
	unsigned long dfout;
	uint32_t cfgr2, divy;

	if (div_id >= _DIV_NB) {
		return 0;
	}

	cfgr2 = mmio_read_32(stm32_rcc_base() + pll->pllxcfgr2);
	divy = (cfgr2 >> pllncfgr2[div_id]) & RCC_PLLNCFGR2_DIVX_MASK;

	dfout = stm32mp1_pll_get_fvco(pll) / (divy + 1U);

	return dfout;
}

static unsigned long get_clock_rate(int p)
{
	uint32_t reg, clkdiv;
	unsigned long clock = 0;
	uintptr_t rcc_base = stm32_rcc_base();

	switch (p) {
	case _CK_MPU:
	/* MPU sub system */
		reg = mmio_read_32(rcc_base + RCC_MPCKSELR);
		switch (reg & RCC_SELR_SRC_MASK) {
		case RCC_MPCKSELR_HSI:
			clock = stm32mp1_clk_get_fixed(_HSI);
			break;
		case RCC_MPCKSELR_HSE:
			clock = stm32mp1_clk_get_fixed(_HSE);
			break;
		case RCC_MPCKSELR_PLL:
			clock = stm32mp1_read_pll_freq(_PLL1, _DIV_P);
			break;
		case RCC_MPCKSELR_PLL_MPUDIV:
			clock = stm32mp1_read_pll_freq(_PLL1, _DIV_P);

			reg = mmio_read_32(rcc_base + RCC_MPCKDIVR);
			clkdiv = reg & RCC_MPUDIV_MASK;
			if (clkdiv != 0U) {
				clock /= stm32mp1_mpu_div[clkdiv];
			}
			break;
		default:
			break;
		}
		break;
	/* AXI sub system */
	case _ACLK:
	case _HCLK2:
	case _HCLK6:
	case _PCLK4:
	case _PCLK5:
		reg = mmio_read_32(rcc_base + RCC_ASSCKSELR);
		switch (reg & RCC_SELR_SRC_MASK) {
		case RCC_ASSCKSELR_HSI:
			clock = stm32mp1_clk_get_fixed(_HSI);
			break;
		case RCC_ASSCKSELR_HSE:
			clock = stm32mp1_clk_get_fixed(_HSE);
			break;
		case RCC_ASSCKSELR_PLL:
			clock = stm32mp1_read_pll_freq(_PLL2, _DIV_P);
			break;
		default:
			break;
		}

		/* System clock divider */
		reg = mmio_read_32(rcc_base + RCC_AXIDIVR);
		clock /= stm32mp1_axi_div[reg & RCC_AXIDIV_MASK];

		switch (p) {
		case _PCLK4:
			reg = mmio_read_32(rcc_base + RCC_APB4DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		case _PCLK5:
			reg = mmio_read_32(rcc_base + RCC_APB5DIVR);
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
		reg = mmio_read_32(rcc_base + RCC_MSSCKSELR);
		switch (reg & RCC_SELR_SRC_MASK) {
		case RCC_MSSCKSELR_HSI:
			clock = stm32mp1_clk_get_fixed(_HSI);
			break;
		case RCC_MSSCKSELR_HSE:
			clock = stm32mp1_clk_get_fixed(_HSE);
			break;
		case RCC_MSSCKSELR_CSI:
			clock = stm32mp1_clk_get_fixed(_CSI);
			break;
		case RCC_MSSCKSELR_PLL:
			clock = stm32mp1_read_pll_freq(_PLL3, _DIV_P);
			break;
		default:
			break;
		}

		/* MCU clock divider */
		reg = mmio_read_32(rcc_base + RCC_MCUDIVR);
		clock >>= stm32mp1_mcu_div[reg & RCC_MCUDIV_MASK];

		switch (p) {
		case _PCLK1:
			reg = mmio_read_32(rcc_base + RCC_APB1DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		case _PCLK2:
			reg = mmio_read_32(rcc_base + RCC_APB2DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		case _PCLK3:
			reg = mmio_read_32(rcc_base + RCC_APB3DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		case _CK_MCU:
		default:
			break;
		}
		break;
	case _CK_PER:
		reg = mmio_read_32(rcc_base + RCC_CPERCKSELR);
		switch (reg & RCC_SELR_SRC_MASK) {
		case RCC_CPERCKSELR_HSI:
			clock = stm32mp1_clk_get_fixed(_HSI);
			break;
		case RCC_CPERCKSELR_HSE:
			clock = stm32mp1_clk_get_fixed(_HSE);
			break;
		case RCC_CPERCKSELR_CSI:
			clock = stm32mp1_clk_get_fixed(_CSI);
			break;
		default:
			break;
		}
		break;
	case _HSI:
	case _HSI_KER:
		clock = stm32mp1_clk_get_fixed(_HSI);
		break;
	case _CSI:
	case _CSI_KER:
		clock = stm32mp1_clk_get_fixed(_CSI);
		break;
	case _HSE:
	case _HSE_KER:
		clock = stm32mp1_clk_get_fixed(_HSE);
		break;
	case _HSE_KER_DIV2:
		clock = stm32mp1_clk_get_fixed(_HSE) >> 1;
		break;
	case _LSI:
		clock = stm32mp1_clk_get_fixed(_LSI);
		break;
	case _LSE:
		clock = stm32mp1_clk_get_fixed(_LSE);
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
		clock = stm32mp1_clk_get_fixed(_USB_PHY_48);
		break;
	default:
		break;
	}

	return clock;
}

static void __clk_enable(struct stm32mp1_clk_gate const *gate)
{
	uintptr_t base = stm32_rcc_base();
	uint32_t bit = BIT(gate->bit);

	if (gate->set_clr != 0U) {
		mmio_write_32(base + gate->offset, bit);
	} else {
		io_mask32_stm32shregs(base + gate->offset, bit, bit);
	}

	FMSG("Clock %u has been enabled", gate->index);
}

static void __clk_disable(struct stm32mp1_clk_gate const *gate)
{
	uintptr_t base = stm32_rcc_base();
	uint32_t bit = BIT(gate->bit);

	if (gate->set_clr != 0U) {
		mmio_write_32(base + gate->offset + RCC_MP_ENCLRR_OFFSET, bit);
	} else {
		io_mask32_stm32shregs(base + gate->offset, 0, bit);
	}

	FMSG("Clock %u has been disabled", gate->index);
}

static bool __clk_is_enabled(struct stm32mp1_clk_gate const *gate)
{
	uintptr_t base = stm32_rcc_base();

	return mmio_read_32(base + gate->offset) & BIT(gate->bit);
}

bool stm32mp1_clk_is_enabled(unsigned long id)
{
	int i = stm32mp1_clk_get_gated_id(id);

	if (i < 0) {
		return false;
	}

	return __clk_is_enabled(gate_ref(i));
}

unsigned int stm32mp1_clk_get_refcount(unsigned long id)
{
	int i = stm32mp1_clk_get_gated_id(id);

	return gate_refcounts[i];
}

void __stm32mp1_clk_enable(unsigned long id, bool secure)
{
	int i = stm32mp1_clk_get_gated_id(id);
	uint32_t exceptions;

	if (i < 0) {
		DMSG("Invalid clock %lu: %d", id, i);
		panic();
	}

	exceptions = may_spin_lock(&refcount_lock);

	if (incr_shrefcnt(&gate_refcounts[i], secure) != 0) {
		__clk_enable(gate_ref(i));
	}

	may_spin_unlock(&refcount_lock, exceptions);
}

void __stm32mp1_clk_disable(unsigned long id, bool secure)
{
	int i = stm32mp1_clk_get_gated_id(id);
	uint32_t exceptions;

	if (i < 0) {
		DMSG("Invalid clock %lu: %d", id, i);
		panic();
	}

	exceptions = may_spin_lock(&refcount_lock);

	if (decr_shrefcnt(&gate_refcounts[i], secure) != 0) {
		__clk_disable(gate_ref(i));
	}

	may_spin_unlock(&refcount_lock, exceptions);
}

static long get_timer_rate(long parent_rate, unsigned int apb_bus)
{
	uint32_t timgxpre;
	uint32_t apbxdiv;
	uintptr_t rcc_base = stm32_rcc_base();

	switch (apb_bus) {
	case 1:
		apbxdiv = mmio_read_32(rcc_base + RCC_APB1DIVR) &
			  RCC_APBXDIV_MASK;
		timgxpre = mmio_read_32(rcc_base + RCC_TIMG1PRER) &
			   RCC_TIMGXPRER_TIMGXPRE;
		break;
	case 2:
		apbxdiv = mmio_read_32(rcc_base + RCC_APB2DIVR) &
			  RCC_APBXDIV_MASK;
		timgxpre = mmio_read_32(rcc_base + RCC_TIMG2PRER) &
			   RCC_TIMGXPRER_TIMGXPRE;
		break;
	default:
		panic();
		break;
	}

	if (apbxdiv == 0) {
		return parent_rate;
	}

	return parent_rate * (timgxpre + 1) * 2;
}

unsigned long stm32mp1_clk_get_rate(unsigned long id)
{
	int p;
	unsigned long rate;

	p = stm32mp1_clk_get_parent(id);
	if (p < 0) {
		return 0;
	}

	rate = get_clock_rate(p);

	if ((id >= TIM2_K) && (id <= TIM14_K)) {
		rate = get_timer_rate(rate, 1);
	}
	if ((id >= TIM1_K) && (id <= TIM17_K)) {
		rate = get_timer_rate(rate, 2);
	}

	return rate;
}

#ifdef CFG_DT
static int clk_compute_pll1_settings(unsigned long input_freq, int idx)
{
	unsigned long post_divm;
	unsigned long long output_freq = pll1_settings.freq[idx] * 1000U;
	unsigned long long freq;
	unsigned long long vco;
	int divm;
	int divn;
	int divp;
	int frac;
	int i;
	unsigned int diff;
	unsigned int best_diff = UINT_MAX;

	/* Following parameters have always the same value */
	pll1_settings.cfg[idx][PLLCFG_Q] = 0;
	pll1_settings.cfg[idx][PLLCFG_R] = 0;
	pll1_settings.cfg[idx][PLLCFG_O] = PQR(1, 0, 0);

	for (divm = DIVM_MAX; divm >= DIVM_MIN; divm--)	{
		post_divm = input_freq / (unsigned long)(divm + 1);

		if ((post_divm < POST_DIVM_MIN) ||
		    (post_divm > POST_DIVM_MAX)) {
			continue;
		}

		for (divp = DIVP_MIN; divp <= DIVP_MAX; divp++) {

			freq = output_freq * (divm + 1) * (divp + 1);

			divn = (int)((freq / input_freq) - 1);
			if ((divn < DIVN_MIN) || (divn > DIVN_MAX)) {
				continue;
			}

			frac = (int)(((freq * FRAC_MAX) / input_freq) -
				     ((divn + 1) * FRAC_MAX));

			/* 2 loops to refine the fractional part */
			for (i = 2; i != 0; i--) {
				if (frac > FRAC_MAX) {
					break;
				}

				vco = (post_divm * (divn + 1)) +
				      ((post_divm * (unsigned long long)frac) /
				       FRAC_MAX);

				if ((vco < (VCO_MIN / 2)) ||
				    (vco > (VCO_MAX / 2))) {
					frac++;
					continue;
				}

				freq = vco / (divp + 1);
				if (output_freq < freq) {
					diff = (unsigned int)(freq -
							      output_freq);
				} else {
					diff = (unsigned int)(output_freq -
							      freq);
				}

				if (diff < best_diff)  {
					pll1_settings.cfg[idx][PLLCFG_M] = divm;
					pll1_settings.cfg[idx][PLLCFG_N] = divn;
					pll1_settings.cfg[idx][PLLCFG_P] = divp;
					pll1_settings.frac[idx] = frac;

					if (diff == 0) {
						return 0;
					}

					best_diff = diff;
				}

				frac++;
			}
		}
	}

	if (best_diff == UINT_MAX) {
		return -1;
	}

	return 0;
}

static int clk_get_pll1_settings(uint32_t clksrc, int index)
{
	unsigned long input_freq;
	unsigned int i;

	for (i = 0; i < PLAT_MAX_OPP_NB; i++) {
		if (pll1_settings.freq[i] == pll1_settings.freq[index]) {
			break;
		}
	}

	if (((i == PLAT_MAX_OPP_NB) &&
	     !stm32mp1_clk_pll1_settings_are_valid()) ||
	    ((i < PLAT_MAX_OPP_NB) &&
	     (pll1_settings.cfg[i][PLLCFG_O] == 0U))) {
		/*
		 * Either PLL1 settings structure is completely empty,
		 * or these settings are not yet computed: do it.
		 */
		switch (clksrc) {
		case CLK_PLL12_HSI:
			input_freq = stm32mp1_clk_get_rate(CK_HSI);
			break;
		case CLK_PLL12_HSE:
			input_freq = stm32mp1_clk_get_rate(CK_HSE);
			break;
		default:
			panic();
		}

		return clk_compute_pll1_settings(input_freq, index);
	}

	if ((i < PLAT_MAX_OPP_NB) &&
	    (pll1_settings.cfg[i][PLLCFG_O] != 0U)) {
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
	uint32_t freq;
	unsigned int i;

	freq = UDIV_ROUND_NEAREST(stm32mp1_clk_get_rate(CK_MPU), 1000L);

	for (i = 0; i < PLAT_MAX_OPP_NB; i++) {
		if (pll1_settings.freq[i] == freq) {
			break;
		}
	}

	if ((i == PLAT_MAX_OPP_NB) ||
	    ((pll1_settings.volt[i] != buck1_voltage) &&
	     (buck1_voltage != 0U))) {
		return -1;
	}

	pll1_settings.cfg[i][PLLCFG_M] =
		(mmio_read_32(rcc_base + pll->pllxcfgr1) &
		 RCC_PLLNCFGR1_DIVM_MASK) >> RCC_PLLNCFGR1_DIVM_SHIFT;

	pll1_settings.cfg[i][PLLCFG_N] =
		(mmio_read_32(rcc_base + pll->pllxcfgr1) &
		 RCC_PLLNCFGR1_DIVN_MASK) >> RCC_PLLNCFGR1_DIVN_SHIFT;

	pll1_settings.cfg[i][PLLCFG_P] =
		(mmio_read_32(rcc_base + pll->pllxcfgr2) &
		 RCC_PLLNCFGR2_DIVP_MASK) >> RCC_PLLNCFGR2_DIVP_SHIFT;

	pll1_settings.cfg[i][PLLCFG_Q] =
		(mmio_read_32(rcc_base + pll->pllxcfgr2) &
		 RCC_PLLNCFGR2_DIVQ_MASK) >> RCC_PLLNCFGR2_DIVQ_SHIFT;

	pll1_settings.cfg[i][PLLCFG_R] =
		(mmio_read_32(rcc_base + pll->pllxcfgr2) &
		 RCC_PLLNCFGR2_DIVR_MASK) >> RCC_PLLNCFGR2_DIVR_SHIFT;

	pll1_settings.cfg[i][PLLCFG_O] =
		mmio_read_32(rcc_base + pll->pllxcr) >>
		RCC_PLLNCR_DIVEN_SHIFT;

	pll1_settings.frac[i] =
		(mmio_read_32(rcc_base + pll->pllxfracr) &
		 RCC_PLLNFRACR_FRACV_MASK) >> RCC_PLLNFRACR_FRACV_SHIFT;

	return i;
}

static uint32_t stm32mp1_clk_get_pll1_current_clksrc(void)
{
	uint32_t value;
	const struct stm32mp1_clk_pll *pll = pll_ref(_PLL1);
	uint32_t rcc_base = stm32_rcc_base();

	value = mmio_read_32(rcc_base + pll->rckxselr);

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
	unsigned int i;
	int ret;
	int index;
	uint32_t count = PLAT_MAX_OPP_NB;
	uint32_t clksrc;
	void *fdt;

	fdt = get_dt_blob();
	if (fdt == NULL) {
		panic();
	}

	ret = fdt_get_all_opp_freqvolt(fdt, &count, pll1_settings.freq,
				       pll1_settings.volt);
	switch (ret) {
	case 0:
		break;
	case -FDT_ERR_NOTFOUND:
		DMSG("Cannot find all OPP info in DT: use default settings");
		return 0;
	default:
		EMSG("Inconsistent OPP settings found in DT, ignored.");
		return 0;
	}

	index = clk_save_current_pll1_settings(buck1_voltage);

	clksrc = stm32mp1_clk_get_pll1_current_clksrc();

	for (i = 0; i < count; i++) {
		if ((index >= 0) && (i == (unsigned int)index))
			continue;

		ret = clk_get_pll1_settings(clksrc, i);
		if (ret)
			return ret;
	}

	pll1_settings.valid_id = PLL1_SETTINGS_VALID_ID;

	return 0;
}

void stm32mp1_clk_lp_save_opp_pll1_settings(uint8_t *data, size_t size)
{
	if ((size != sizeof(pll1_settings)) ||
	    !stm32mp1_clk_pll1_settings_are_valid()) {
		panic();
	}

	memcpy(data, &pll1_settings, size);
}

bool stm32mp1_clk_pll1_settings_are_valid(void)
{
	return pll1_settings.valid_id == PLL1_SETTINGS_VALID_ID;
}

static void stm32mp1_osc_clk_init(const char *name,
				  enum stm32mp_osc_id index)
{
	uint32_t frequency;
	void *fdt;

	fdt = get_dt_blob();
	if (fdt == NULL) {
		panic();
	}

	stm32mp1_osc[index] = 0;

	if (fdt_osc_read_freq(fdt, name, &frequency) == 0) {
		stm32mp1_osc[index] = frequency;
	}
}

static void stm32mp1_osc_init(void)
{
	enum stm32mp_osc_id i;
	char **name __maybe_unused = (char **)&stm32mp_osc_node_label[0];

	for (i = (enum stm32mp_osc_id)0 ; i < NB_OSC; i++) {
		stm32mp1_osc_clk_init(stm32mp_osc_node_label[i], i);
		DMSG("Osc %s frequency: %lu", name[i], stm32mp1_osc[i]);
	}
}
#else /* CFG_DT */
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
#endif

/*
 * Lookup platform clock from enable bit location in RCC registers.
 * Return a valid clock ID on success, return ~0 on error.
 */
unsigned long stm32mp1_clk_rcc2id(size_t offset, size_t bit)
{
	return get_id_from_rcc_bit(offset, bit);
}

/*
 * Get the parent ID of the target parent clock, for tagging as secure
 * shared clock dependencies.
 */
static int get_parent_id_parent(unsigned int parent_id)
{
	enum stm32mp1_parent_sel s = _UNKNOWN_SEL;
	enum stm32mp1_pll_id pll_id;
	uint32_t p_sel;

	switch (parent_id) {
	case _ACLK:
	case _PCLK4:
	case _PCLK5:
		s = _ASS_SEL;
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
	case _HCLK6:
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
		uintptr_t rcc_base = stm32_rcc_base();

		p_sel = (mmio_read_32(rcc_base + sel->offset) >> sel->src) &
			sel->msk;

		if (p_sel < sel->nb_parent) {
			return (int)sel->parent[p_sel];
		}
	} else {
		const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);

		p_sel = mmio_read_32(stm32_rcc_base() + pll->rckxselr) &
			RCC_SELR_REFCLK_SRC_MASK;

		if (pll->refclk[p_sel] != _UNKNOWN_OSC_ID) {
			return (int)pll->refclk[p_sel];
		}
	}

	FMSG("No parent selected for %s", stm32mp1_clk_parent_name[parent_id]);
	return -1;
}

static void secure_parent_clocks(unsigned long parent_id)
{
	int grandparent_id;

	switch (parent_id) {
	/* Secure only the parents for these clocks */
	case _ACLK:
	case _HCLK2:
	case _HCLK6:
	case _PCLK4:
	case _PCLK5:
		break;
	/* PLLs */
	case _PLL1_P:
		stm32mp_register_secure_periph(STM32MP1_SHRES_PLL1_P);
		break;
	case _PLL1_Q:
		stm32mp_register_secure_periph(STM32MP1_SHRES_PLL1_Q);
		break;
	case _PLL1_R:
		stm32mp_register_secure_periph(STM32MP1_SHRES_PLL1_R);
		break;

	case _PLL2_P:
		stm32mp_register_secure_periph(STM32MP1_SHRES_PLL2_P);
		break;
	case _PLL2_Q:
		stm32mp_register_secure_periph(STM32MP1_SHRES_PLL2_Q);
		break;
	case _PLL2_R:
		stm32mp_register_secure_periph(STM32MP1_SHRES_PLL2_R);
		break;

	case _PLL3_P:
		stm32mp_register_secure_periph(STM32MP1_SHRES_PLL3_P);
		break;
	case _PLL3_Q:
		stm32mp_register_secure_periph(STM32MP1_SHRES_PLL3_Q);
		break;
	case _PLL3_R:
		stm32mp_register_secure_periph(STM32MP1_SHRES_PLL3_R);
		break;

	/* Source clocks */
	case _HSI:
	case _HSI_KER:
		stm32mp_register_secure_periph(STM32MP1_SHRES_HSI);
		break;
	case _LSI:
		stm32mp_register_secure_periph(STM32MP1_SHRES_LSI);
		break;
	case _CSI:
	case _CSI_KER:
		stm32mp_register_secure_periph(STM32MP1_SHRES_CSI);
		break;
	case _HSE:
	case _HSE_KER:
	case _HSE_KER_DIV2:
		stm32mp_register_secure_periph(STM32MP1_SHRES_HSE);
		break;
	case _LSE:
		stm32mp_register_secure_periph(STM32MP1_SHRES_LSE);
		break;

	default:
		panic();
	}

	grandparent_id = get_parent_id_parent(parent_id);
	if (grandparent_id >= 0) {
		secure_parent_clocks(grandparent_id);
	}
}

void stm32mp_register_clock_parents_secure(unsigned long clock_id)
{
	int parent_id;

	switch (clock_id) {
	case PLL1:
		parent_id = get_parent_id_parent(_PLL1_P);
		break;
	case PLL2:
		parent_id = get_parent_id_parent(_PLL2_P);
		break;
	case PLL3:
		parent_id = get_parent_id_parent(_PLL3_P);
		break;
	case PLL4:
		EMSG("PLL4 cannot be secure");
		panic();
	default:
		/* Others are expected gateable clock */
		parent_id = stm32mp1_clk_get_parent(clock_id);
		break;
	}

	if (parent_id < 0) {
		DMSG("No parent for clock %lu", clock_id);
		panic();
	}

	secure_parent_clocks(parent_id);
}

#ifdef CFG_DT
/*
 * Check that the device tree does not provide clock tree configuration
 * information. Such configuration would not be applied since the early boot
 * loader is in charge of configuring the clock tree and enabling the PLLs.
 */
static void init_clock_tree_from_dt(void)
{
	void *fdt;
	uintptr_t rcc_base = stm32_rcc_base();
	int node = -1;
	unsigned int i;
	int len;
	int ignored = 0;

	fdt = get_dt_blob();
	if (fdt != NULL) {
		node = fdt_get_rcc_node(fdt);
	}

	if ((fdt == NULL) || (node < 0)) {
		panic("RCC DT");
	}

	if ((_fdt_get_status(fdt, node) & DT_STATUS_OK_SEC) == 0) {
		panic("RCC disabled");
	}

	assert(virt_to_phys((void *)stm32_rcc_base()) ==
	       fdt_rcc_read_addr(fdt));

	/* Expect booting from a secure setup */
	if ((mmio_read_32(rcc_base + RCC_TZCR) & RCC_TZCR_TZEN) == 0) {
		panic("RCC TZC[TZEN]");
	}

	/* Get oscillator frequency to handle freq get/set operations */
	stm32mp1_osc_init();

	node = fdt_get_rcc_node(fdt);
	assert(node >= 0);

	/*
	 * OP-TEE core is not in charge of the clock tree configuration.
	 * This is expected from an earlier boot stage. Modifying the clock
	 * tree here may jeopardize the already configured clock tree.
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
		char name[12];

		snprintf(name, sizeof(name), "st,pll@%d", i);
		node = fdt_rcc_subnode_offset(fdt, name);

		if (node <= 0) {
			continue;
		}

		if (fdt_getprop(fdt, node, "cfg", &len) ||
		    fdt_getprop(fdt, node, "frac", &len)) {
			DMSG("Ignore PLL%u configurations from DT", i);
			ignored++;
		}
	}

	if (ignored != 0) {
		IMSG("DT clock tree configurations were ignored");
	}
}
#else
static void init_clock_tree_from_dt(void)
{
	uintptr_t rcc_base = stm32_rcc_base();

	/* Expect booting from a secure setup */
	if ((mmio_read_32(rcc_base + RCC_TZCR) & RCC_TZCR_TZEN) == 0) {
		panic("RCC TZC[TZEN]");
	}
}
#endif /*CFG_DT*/

/* Sync secure clock refcount after all drivers probe/inits,  */
void stm32mp_update_earlyboot_clocks_state(void)
{
	unsigned int idx;

	for (idx = 0; idx < NB_GATES; idx++) {
		unsigned long clock_id = gate_ref(idx)->index;

		/*
		 * Drop non-secure refcount set on shareable clocks that are
		 * not shared. Secure clock should not hold a non-secure
		 * refcount. Non-secure clock cannot hold any refcount.
		 */
		if (__clk_is_enabled(gate_ref(idx)) &&
		    stm32mp_clock_is_shareable(clock_id) &&
		    !stm32mp_clock_is_shared(clock_id)) {
			stm32mp1_clk_disable_non_secure(clock_id);
		}

		/*
		 * Disable secure clocks enabled from early boot but not explicitly
		 * enabled from the secure world.
		 */
		if (__clk_is_enabled(gate_ref(idx)) &&
		    !stm32mp_clock_is_non_secure(clock_id) &&
		    !gate_refcounts[idx]) {
			__clk_disable(gate_ref(idx));
		}
	}

	/* Dump clocks state */
	for (idx = 0; idx < NB_GATES; idx++) {
		unsigned long __maybe_unused clock_id = gate_ref(idx)->index;
		int __maybe_unused p = stm32mp1_clk_get_parent(clock_id);

		FMSG("stm32mp clock %3lu is %sabled (refcnt %d) (parent %d %s)",
			clock_id,
			__clk_is_enabled(gate_ref(idx)) ? "en" : "dis",
			gate_refcounts[idx],
			p, p < 0 ? "n.a" : stm32mp1_clk_parent_name[p]);
	}
}

/* Set a non-secure refcount on shareable clock that were enabled from boot */
static void sync_earlyboot_clocks_state(void)
{
	unsigned int idx;

	for (idx = 0; idx < NB_GATES; idx++) {
		assert(!gate_refcounts[idx]);
	}

	/*
	 * Set a non-secure refcount for shareable clocks enabled from boot.
	 * It will be dropped after core inits for secure-only clocks.
	 */
	for (idx = 0; idx < NB_GATES; idx++) {
		struct stm32mp1_clk_gate const *gate = gate_ref(idx);

		if (__clk_is_enabled(gate) &&
		    stm32mp_clock_is_shareable(gate->index)) {
			gate_refcounts[idx] = SHREFCNT_NONSECURE_FLAG;
		}
	}

	/*
	 * Register secure clock parents and init a refcount for
	 * secure only resources that are not registered from a driver probe.
	 * - DDR controller and phy clocks.
	 * - TZC400, ETZPC and STGEN clocks.
	 * - RTCAPB clocks on multi-core
	 */
	stm32mp_register_clock_parents_secure(DDRC1);
	stm32mp1_clk_enable_secure(DDRC1);
	stm32mp_register_clock_parents_secure(DDRC1LP);
	stm32mp1_clk_enable_secure(DDRC1LP);
	stm32mp_register_clock_parents_secure(DDRC2);
	stm32mp1_clk_enable_secure(DDRC2);
	stm32mp_register_clock_parents_secure(DDRC2LP);
	stm32mp1_clk_enable_secure(DDRC2LP);
	stm32mp_register_clock_parents_secure(DDRPHYC);
	stm32mp1_clk_enable_secure(DDRPHYC);
	stm32mp_register_clock_parents_secure(DDRPHYCLP);
	stm32mp1_clk_enable_secure(DDRPHYCLP);
	stm32mp_register_clock_parents_secure(DDRCAPB);
	stm32mp1_clk_enable_secure(DDRCAPB);
	stm32mp_register_clock_parents_secure(AXIDCG);
	stm32mp1_clk_enable_secure(AXIDCG);
	stm32mp_register_clock_parents_secure(DDRPHYCAPB);
	stm32mp1_clk_enable_secure(DDRPHYCAPB);
	stm32mp_register_clock_parents_secure(DDRPHYCAPBLP);
	stm32mp1_clk_enable_secure(DDRPHYCAPBLP);

	stm32mp_register_clock_parents_secure(TZPC);
	stm32mp1_clk_enable_secure(TZPC);
	stm32mp_register_clock_parents_secure(TZC1);
	stm32mp1_clk_enable_secure(TZC1);
	stm32mp_register_clock_parents_secure(TZC2);
	stm32mp1_clk_enable_secure(TZC2);
	stm32mp_register_clock_parents_secure(STGEN_K);
	stm32mp1_clk_enable_secure(STGEN_K);

	stm32mp_register_clock_parents_secure(BSEC);
	stm32mp1_clk_enable_secure(BSEC);

	stm32mp_register_clock_parents_secure(BKPSRAM);

	stm32mp_register_clock_parents_secure(RTCAPB);
	stm32mp1_clk_enable_secure(RTCAPB);

	/* The low power sequences mandates RNG1 and CRYP1 support */
	stm32mp_register_clock_parents_secure(RNG1_K);
	stm32mp_register_clock_parents_secure(CRYP1);
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
	uint32_t fracr;
	uint32_t value;
	int ret;

	ret = pll_compute_pllxcfgr1(pll, pllcfg, &value);
	if (ret != 0) {
		return ret;
	}

	if (mmio_read_32(rcc_base + pll->pllxcfgr1) != value) {
		/* Different DIVN/DIVM, can't config on the fly */
		*result = -1;
		return 0;
	}

	fracr = fracv << RCC_PLLNFRACR_FRACV_SHIFT;
	fracr |= RCC_PLLNFRACR_FRACLE;
	value = pll_compute_pllxcfgr2(pllcfg);

	if ((mmio_read_32(rcc_base + pll->pllxfracr) == fracr) &&
	    (mmio_read_32(rcc_base + pll->pllxcfgr2) == value)) {
		/* Same parameters, no need to config */
		*result = 1;
	} else {
		*result = 0;
	}

	return 0;
}

/* Configure PLL1 from input frequency OPP parameters */
static int pll1_config_from_opp_khz(uint32_t freq_khz)
{
	unsigned int i;
	int ret;
	int config_on_the_fly = -1;

	for (i = 0; i < PLAT_MAX_OPP_NB; i++) {
		if (pll1_settings.freq[i] == freq_khz) {
			break;
		}
	}

	if (i == PLAT_MAX_OPP_NB) {
		return -1;
	}

	ret = is_pll_config_on_the_fly(_PLL1, &pll1_settings.cfg[i][0],
				       pll1_settings.frac[i],
				       &config_on_the_fly);
	if (ret != 0) {
		return ret;
	}

	if (config_on_the_fly == 1) {
		/* No need to reconfig, setup already OK */
		return 0;
	}

	if (config_on_the_fly == -1) {
		/* Switch to HSI and stop PLL1 before reconfiguration */
		ret = stm32mp1_set_clksrc(CLK_MPU_HSI);
		if (ret != 0) {
			return ret;
		}

		ret = pll_stop(_PLL1);
		if (ret != 0) {
			return ret;
		}
	}

	ret = pll_config(_PLL1, &pll1_settings.cfg[i][0],
			 pll1_settings.frac[i]);
	if (ret != 0) {
		return ret;
	}

	if (config_on_the_fly == -1) {
		/* Start PLL1 and switch back to after reconfiguration */
		pll_start(_PLL1);

		ret = pll_output(_PLL1, pll1_settings.cfg[i][PLLCFG_O]);
		if (ret != 0) {
			return ret;
		}

		ret = stm32mp1_set_clksrc(CLK_MPU_PLL1P);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

int stm32mp1_set_opp_khz(uint32_t freq_khz)
{
	if (freq_khz == current_opp_khz) {
		/* OPP already set, nothing to do */
		return 0;
	}

	if (!stm32mp1_clk_pll1_settings_are_valid()) {
		/*
		 * No OPP table in DT or an error occurred during PLL1
		 * settings computation, system can only work on current
		 * operating point so return error.
		 */
		return -1;
	}

	/* Check that PLL1 (without MPUDIV) is MPU clock source */
	if (((mmio_read_32(stm32_rcc_base() + RCC_MPCKSELR) &
	     RCC_SELR_SRC_MASK)) != RCC_MPCKSELR_PLL) {
		return -1;
	}

	if (pll1_config_from_opp_khz(freq_khz) != 0) {
		/* Restore original value */
		if (pll1_config_from_opp_khz(current_opp_khz) != 0) {
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
	unsigned int i;
	uint32_t round_opp = 0U;

	if (!stm32mp1_clk_pll1_settings_are_valid()) {
		/*
		 * No OPP table in DT, or an error occurred during PLL1
		 * settings computation, system can only work on current
		 * operating point, so return current CPU frequency.
		 */
		*freq_khz = current_opp_khz;

		return 0;
	}

	for (i = 0; i < PLAT_MAX_OPP_NB; i++) {
		if ((pll1_settings.freq[i] <= *freq_khz) &&
		    (pll1_settings.freq[i] > round_opp)) {
			round_opp = pll1_settings.freq[i];
		}
	}

	*freq_khz = round_opp;

	return 0;
}

static void _clock_mpu_suspend(void)
{
	uintptr_t mpckselr = stm32_rcc_base() + RCC_MPCKSELR;

	if (((mmio_read_32(mpckselr) & RCC_SELR_SRC_MASK)) ==
	    RCC_MPCKSELR_PLL) {
		if (stm32mp1_set_clksrc(CLK_MPU_PLL1P_DIV)) {
			panic();
		}
	}
}

static void _clock_mpu_resume(void)
{
	uintptr_t mpckselr = stm32_rcc_base() + RCC_MPCKSELR;

	if (((mmio_read_32(mpckselr) & RCC_SELR_SRC_MASK)) ==
	    RCC_MPCKSELR_PLL_MPUDIV) {
		if (stm32mp1_set_clksrc(CLK_MPU_PLL1P)) {
			panic();
		}
	}
}

static void _clock_resume(void)
{
	unsigned int idx;

	_clock_mpu_resume();

	/* Sync secure and shared clocks physical state on functional state */
	for (idx = 0; idx < NB_GATES; idx++) {
		struct stm32mp1_clk_gate const *gate = gate_ref(idx);

		if (stm32mp_clock_is_non_secure(gate->index)) {
			continue;
		}

		if (gate_refcounts[idx]) {
			DMSG("Force clock %d enable", gate->index);
			__clk_enable(gate);
		} else {
			DMSG("Force clock %d disable", gate->index);
			__clk_disable(gate);
		}
	}
}

void stm32mp_clock_suspend_resume(enum pm_op op)
{
	switch (op) {
	case PM_OP_SUSPEND:
		_clock_mpu_suspend();
		break;
	case PM_OP_RESUME:
		_clock_resume();
		break;
	default:
		panic();
	}
}

static TEE_Result stm32mp1_clk_probe(void)
{
	unsigned long freq_khz;

	assert(PLLCFG_NB == PLAT_MAX_PLLCFG_NB);

	init_clock_tree_from_dt();

	sync_earlyboot_clocks_state();

	/* Save current CPU operating point value */
	freq_khz = UDIV_ROUND_NEAREST(stm32mp1_clk_get_rate(CK_MPU), 1000UL);
	if (freq_khz > (unsigned long)UINT32_MAX) {
		panic();
	}

	current_opp_khz = (uint32_t)freq_khz;

	return TEE_SUCCESS;
}
/* Setup clock support before driver initialization */
service_init(stm32mp1_clk_probe);

