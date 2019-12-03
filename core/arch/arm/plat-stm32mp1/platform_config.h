/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2017-2019, STMicroelectronics
 */

#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <mm/generic_ram_layout.h>

/* Enable/disable use of the core0 reset control from RCC */
#undef STM32MP1_USE_MPU0_RESET

/* Make stacks aligned to data cache line length */
#define STACK_ALIGNMENT			32

#if defined(CFG_WITH_PAGER)
#if defined(CFG_WITH_LPAE)
/*
 * Optimize unpaged memory size:
 * - one table for the level2 table for overall vmem range
 * - two tables for TEE RAM fine grain mapping [2ffc.0000 301f.ffff]
 * - one table for internal RAMs (PM: ROMed core TEE RAM & DDR first page)
 * - one table for a 2MByte dynamiq shared virtual memory (SHM_VASPACE)
 */
#define MAX_XLAT_TABLES			5
#else
/*
 * Optimize unpaged memory size:
 * - two tables for TEE RAM mapping [2ffc.0000 300f.ffff]
 * - one table for secure internal RAMs (PM: ROMed core TEE RAM)
 * - one table for non-secure internal RAMs (PM: DDR first page)
 * - two tables for a 2MByte dynamiq shared virtual memory (SHM_VASPACE)
 */
#define MAX_XLAT_TABLES			6
#endif /*CFG_WITH_LPAE*/
#else
/* Be generous, there is plenty of secure DDR */
#define MAX_XLAT_TABLES			10
#endif /*CFG_WITH_PAGER*/

/* Expected platform default size, if not found in device tree */
#define STM32MP1_DDR_SIZE_DFLT		(1 * 1024 * 1024 * 1024)

#define GIC_IOMEM_BASE			0xa0021000ul
#define GIC_IOMEM_SIZE			0x00007000
#define DDR_BASE			0xc0000000ul
#define SYSRAM_BASE			0x2ffc0000
#define BKPSRAM_BASE			0x54000000

#define BSEC_BASE			0x5c005000
#define CRYP1_BASE			0x54001000
#define DBGMCU_BASE			0x50081000
#define DDRCTRL_BASE			0x5a003000
#define DDRPHYC_BASE			0x5a004000
#define ETZPC_BASE			0x5c007000
#define EXTI_BASE			0x5000D000
#define GPIOA_BASE			0x50002000
#define GPIOB_BASE			0x50003000
#define GPIOC_BASE			0x50004000
#define GPIOD_BASE			0x50005000
#define GPIOE_BASE			0x50006000
#define GPIOF_BASE			0x50007000
#define GPIOG_BASE			0x50008000
#define GPIOH_BASE			0x50009000
#define GPIOI_BASE			0x5000a000
#define GPIOJ_BASE			0x5000b000
#define GPIOK_BASE			0x5000c000
#define GPIOZ_BASE			0x54004000
#define HASH1_BASE			0x54002000
#define I2C4_BASE			0x5c002000
#define I2C6_BASE			0x5c009000
#define IWDG1_BASE			0x5c003000
#define IWDG2_BASE			0x5a002000
#define PWR_BASE			0x50001000
#define RCC_BASE			0x50000000
#define RNG1_BASE			0x54003000
#define RTC_BASE			0x5c004000
#define SPI6_BASE			0x5c001000
#define STGEN_BASE			0x5C008000
#define SYSCFG_BASE			0x50020000
#define TAMP_BASE			0x5c00a000
#define USART1_BASE			0x5c000000
#define USART2_BASE			0x4000e000
#define USART3_BASE			0x4000f000
#define UART4_BASE			0x40010000
#define UART5_BASE			0x40011000
#define USART6_BASE			0x44003000
#define UART7_BASE			0x40018000
#define UART8_BASE			0x40019000
#define TZC_BASE			0x5c006000

/* BSEC OTP resources */
#define STM32MP1_OTP_MAX_ID		0x5FU
#define STM32MP1_UPPER_OTP_START	0x20U

#define OTP_MAX_SIZE			(STM32MP1_OTP_MAX_ID + 1U)

#define HW2_OTP				"hw2_otp"

#define HW2_OTP_IWDG_HW_ENABLE_SHIFT	3
#define HW2_OTP_IWDG_FZ_STOP_SHIFT	5
#define HW2_OTP_IWDG_FZ_STANDBY_SHIFT	7

/*
 * GPIO banks: 11 non secure banks (A to K) and 1 secure bank (Z)
 * Bank register's base address is computed from the bank ID listed here.
 */
#define GPIOS_NSEC_COUNT		11
#define GPIOS_NSEC_BASE			GPIOA_BASE
#define GPIOS_NSEC_SIZE			(GPIOS_NSEC_COUNT * SMALL_PAGE_SIZE)

#define STM32MP1_GPIOZ_MAX_COUNT	1
#define STM32MP1_GPIOZ_PIN_MAX_COUNT	8

#define GPIO_BANK_OFFSET		0x1000U

/* Bank IDs used in GPIO driver API */
#define GPIO_BANK_A			0U
#define GPIO_BANK_B			1U
#define GPIO_BANK_C			2U
#define GPIO_BANK_D			3U
#define GPIO_BANK_E			4U
#define GPIO_BANK_F			5U
#define GPIO_BANK_G			6U
#define GPIO_BANK_H			7U
#define GPIO_BANK_I			8U
#define GPIO_BANK_J			9U
#define GPIO_BANK_K			10U
#define GPIO_BANK_Z			25U

/* IWDG resources */
#define IWDG1_INST			0
#define IWDG2_INST			1

#define STM32MP1_IRQ_IWDG1		182U
#define STM32MP1_IRQ_IWDG2		183U

/* TAMP resources */
#define TAMP_BKP_REGISTER_OFF		0x100

/* RCC platform resources */
#define RCC_WAKEUP_IT			177

/* SoC revision */
#define STM32MP1_REV_A			0x00001000
#define STM32MP1_REV_B			0x00002000
#define STM32MP1_REV_Z			0x00002001

/* DBGMCU resources */
#define DBGMCU_IDC			0x0
#define DBGMCU_IDC_REV_ID_MASK		GENMASK_32(31, 16)
#define DBGMCU_IDC_REV_ID_SHIFT		16

/* BKPSRAM layout */
#define BKPSRAM_SIZE			SMALL_PAGE_SIZE
#define BKPSRAM_PM_OFFSET		0x000
#define BKPSRAM_PM_SIZE			(BKPSRAM_PM_MAILBOX_SIZE + \
						BKPSRAM_PM_CONTEXT_SIZE)

#define BKPSRAM_PM_MAILBOX_OFFSET	BKPSRAM_PM_OFFSET
#define BKPSRAM_PM_MAILBOX_SIZE		0x100
#define BKPSRAM_PM_CONTEXT_OFFSET	(BKPSRAM_PM_MAILBOX_OFFSET + \
						BKPSRAM_PM_MAILBOX_SIZE)
#define BKPSRAM_PM_CONTEXT_SIZE		0x700

/* SYSRAM */
#define SYSRAM_SIZE			0x40000

/* GIC resources */
#define GICD_OFFSET			0x00000000
#define GICC_OFFSET			0x00001000
#define GICH_OFFSET			0x00003000
#define GICV_OFFSET			0x00005000

#define GIC_NON_SEC_SGI_0		0
#define GIC_SEC_SGI_0			8
#define GIC_SEC_SGI_1			9
#define GIC_SPI_SEC_PHY_TIMER		29

#define TARGET_CPU0_GIC_MASK		BIT(0)
#define TARGET_CPU1_GIC_MASK		BIT(1)
#define TARGET_CPUS_GIC_MASK		GENMASK_32(CFG_TEE_CORE_NB_CORE - 1, 0)

#define STM32MP_GIC_PRIORITY_CSTOP	0xc0

#endif /*PLATFORM_CONFIG_H*/
