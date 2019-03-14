/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2017-2018, STMicroelectronics
 */

#ifndef __STM32_UTIL_H__
#define __STM32_UTIL_H__

#include <io.h>
#include <drivers/stm32_etzpc.h>
#include <drivers/stm32mp1_clk.h>
#include <kernel/panic.h>
#include <mm/core_mmu.h>
#include <sm/sm.h>
#include <stdint.h>
#include <stdbool.h>

/* SoC versioning */
uint32_t stm32mp1_dbgmcu_get_chip_version(void);

/* SiP & OEM platform services */
bool stm32_sip_service(struct sm_ctx *ctx,
		       uint32_t *a0, uint32_t *a1, uint32_t *a2, uint32_t *a3);
bool stm32_oem_service(struct sm_ctx *ctx,
		       uint32_t *a0, uint32_t *a1, uint32_t *a2, uint32_t *a3);


/* Platform util for the STGEN driver */
uintptr_t stm32_get_stgen_base(void);

/* Platform util for the GIC */
uintptr_t get_gicc_base(void);
uintptr_t get_gicd_base(void);
void stm32mp_gic_set_end_of_interrupt(uint32_t it);

/* Platform util for clock gating. ID refers to clock DT bindings ID. */
void stm32_clock_enable(unsigned long id);
void stm32_clock_disable(unsigned long id);

static inline unsigned long stm32_clock_get_rate(unsigned long id)
{
	return stm32mp1_clk_get_rate(id);
}

static inline unsigned long stm32_clock_is_enabled(unsigned long id)
{
	return stm32mp1_clk_is_enabled(id);
}

/* Platform util for the GPIO driver */
uintptr_t stm32_get_gpio_bank_base(unsigned int bank);
uint32_t stm32_get_gpio_bank_offset(unsigned int bank);
int stm32_get_gpio_bank_clock(unsigned int bank);

/* Platform util for the IWDG driver */
unsigned long stm32_get_iwdg_otp_config(uintptr_t pbase);
int stm32mp_iwdg_irq2instance(size_t irq);
size_t stm32mp_iwdg_instance2irq(int instance);
unsigned int stm32mp_iwdg_iomem2instance(uintptr_t pbase);

/* Platform util for the BSEC driver */
unsigned int stm32mp_get_otp_max(void);
unsigned int stm32mp_get_otp_upper_start(void);

/* Platform util for the ETZPC driver */
uintptr_t stm32mp_get_etzpc_base(void);
enum etzpc_decprot_attributes stm32mp_etzpc_binding2decprot(uint32_t mode);

/* Platform util for the RTC driver */
bool stm32_rtc_get_read_twice(void);

/* Backup registers and RAM utils */
uintptr_t stm32mp_bkpreg(unsigned int idx);

uintptr_t stm32mp1_bkpsram_base(void);

/* Platform util for PMIC support */
bool stm32mp_with_pmic(void);

/* Power management service */
void stm32mp_register_online_cpu(void);

/*
 * Lock/unlock access to shared registers
 *
 * @lock - NULL or pointer to spin lock
 */
uint32_t lock_stm32shregs(void);
void unlock_stm32shregs(uint32_t exceptions);

void io_mask32_stm32shregs(uintptr_t va, uint32_t value, uint32_t mask);

static inline void stm32shregs_setbits(uintptr_t va, uint32_t value)
{
	io_mask32_stm32shregs(va, value, value);
}

static inline void stm32shregs_clrbits(uintptr_t va, uint32_t value)
{
	io_mask32_stm32shregs(va, 0, value);
}

static inline void stm32shregs_clrsetbits(uintptr_t va, uint32_t mask,
					  uint32_t value)
{
	io_mask32_stm32shregs(va, value, mask);
}

/*
 * Generic spinlock function that bypass spinlock if MMU is disabled or
 * lock is NULL.
 */
uint32_t may_spin_lock(unsigned int *lock);
void may_spin_unlock(unsigned int *lock, uint32_t exceptions);

/* Reset function for early watchdog management */
void stm32mp_platform_reset(int cpu);

/* Clock calibration */
int stm32mp_start_clock_calib(unsigned int clock_id);

/*
 * Shared reference counter: increments by 2 on secure increment
 * request, decrements by 2 on secure decrement request. Bit #0
 * is set to 1 on non-secure increment request and reset to 0 on
 * non-secure decrement request. These counter initializes to
 * either 0, 1 or 2 upon their expect default state.
 * Increment refcount and return if incremented from 0.
 * Counters saturates once above UINT_MAX / 2.
 */
#define SHREFCNT_NONSECURE_FLAG		0x1ul
#define SHREFCNT_SECURE_STEP		0x2ul
#define SHREFCNT_MAX			(UINT_MAX / 2)

/* Return 1 if refcnt decrements to 0, else return 0 */
static inline int incr_shrefcnt(unsigned int *refcnt, bool secure)
{
	int rc = !*refcnt;

	if (secure) {
		if (*refcnt < SHREFCNT_MAX) {
			*refcnt += SHREFCNT_SECURE_STEP;
			assert(*refcnt < SHREFCNT_MAX);
		}
	} else {
		*refcnt |= SHREFCNT_NONSECURE_FLAG;
	}

	return rc;
}

/* Return 1 if refcnt decrements to 0, else return 0 */
static inline int decr_shrefcnt(unsigned int *refcnt, bool secure)
{
	int  rc = 0;

	if (secure) {
		if (*refcnt < SHREFCNT_MAX) {
			if (*refcnt < SHREFCNT_SECURE_STEP) {
				panic();
			}

			*refcnt -= SHREFCNT_SECURE_STEP;
			rc = !*refcnt;
		}
	} else {
		rc = (*refcnt == SHREFCNT_NONSECURE_FLAG);
		*refcnt &= ~SHREFCNT_NONSECURE_FLAG;
	}

	return rc;
}

static inline int incr_refcnt(unsigned int *refcnt)
{
	return incr_shrefcnt(refcnt, true);
}

static inline int decr_refcnt(unsigned int *refcnt)
{
	return decr_shrefcnt(refcnt, true);
}

#define STM32MP1_SHRES_GPIOZ(i)		(STM32MP1_SHRES_GPIOZ_0 + i)

enum stm32mp_shres {
	STM32MP1_SHRES_GPIOZ_0 = 0,
	STM32MP1_SHRES_GPIOZ_1,
	STM32MP1_SHRES_GPIOZ_2,
	STM32MP1_SHRES_GPIOZ_3,
	STM32MP1_SHRES_GPIOZ_4,
	STM32MP1_SHRES_GPIOZ_5,
	STM32MP1_SHRES_GPIOZ_6,
	STM32MP1_SHRES_GPIOZ_7,
	STM32MP1_SHRES_IWDG1,
	STM32MP1_SHRES_USART1,
	STM32MP1_SHRES_SPI6,
	STM32MP1_SHRES_I2C4,
	STM32MP1_SHRES_RNG1,
	STM32MP1_SHRES_HASH1,
	STM32MP1_SHRES_CRYP1,
	STM32MP1_SHRES_I2C6,
	STM32MP1_SHRES_RTC,
	STM32MP1_SHRES_MCU,
	STM32MP1_SHRES_HSI,
	STM32MP1_SHRES_LSI,
	STM32MP1_SHRES_HSE,
	STM32MP1_SHRES_LSE,
	STM32MP1_SHRES_CSI,
	STM32MP1_SHRES_PLL1,
	STM32MP1_SHRES_PLL1_P,
	STM32MP1_SHRES_PLL1_Q,
	STM32MP1_SHRES_PLL1_R,
	STM32MP1_SHRES_PLL2,
	STM32MP1_SHRES_PLL2_P,
	STM32MP1_SHRES_PLL2_Q,
	STM32MP1_SHRES_PLL2_R,
	STM32MP1_SHRES_PLL3,
	STM32MP1_SHRES_PLL3_P,
	STM32MP1_SHRES_PLL3_Q,
	STM32MP1_SHRES_PLL3_R,

	STM32MP1_SHRES_COUNT
};

void stm32mp_register_secure_periph(unsigned int id);
void stm32mp_register_shared_periph(unsigned int id);
void stm32mp_register_non_secure_periph(unsigned int id);
void stm32mp_register_secure_periph_iomem(uintptr_t base);
void stm32mp_register_non_secure_periph_iomem(uintptr_t base);
void stm32mp_register_secure_gpio(unsigned int bank, unsigned int pin);
void stm32mp_register_non_secure_gpio(unsigned int bank, unsigned int pin);
void stm32mp_register_etzpc_decprot(unsigned int id,
				    enum etzpc_decprot_attributes attr);

bool stm32mp_periph_is_shared(unsigned long id);
bool stm32mp_periph_is_non_secure(unsigned long id);
bool stm32mp_periph_is_secure(unsigned long id);
bool stm32mp_periph_is_unregistered(unsigned long id);

bool stm32mp_gpio_bank_is_shared(unsigned int bank);
bool stm32mp_gpio_bank_is_non_secure(unsigned int bank);
bool stm32mp_gpio_bank_is_secure(unsigned int bank);

bool stm32mp_clock_is_shareable(unsigned long clock_id);
bool stm32mp_clock_is_shared(unsigned long clock_id);
bool stm32mp_clock_is_non_secure(unsigned long clock_id);

/*
 * Set bit fields, clear bit flieds, clear bits and set bits utils
 */
static inline void mmio_write_8(uintptr_t addr, uint8_t value)
{
	write8(value, addr);
}

static inline uint8_t mmio_read_8(uintptr_t addr)
{
	return read8(addr);
}

static inline void mmio_write_32(uintptr_t addr, uint32_t value)
{
	write32(value, addr);
}

static inline uint32_t mmio_read_32(uintptr_t addr)
{
	return read32(addr);
}

static inline void mmio_setbits_32(uintptr_t addr, uint32_t mask)
{
	write32(read32(addr) | mask, addr);
}

static inline void mmio_clrbits_32(uintptr_t addr, uint32_t mask)
{
	write32(read32(addr) & ~mask, addr);
}

static inline void mmio_clrsetbits_32(uintptr_t addr, uint32_t clear_mask,
				      uint32_t set_mask)
{
	write32((read32(addr) & ~clear_mask) | set_mask, addr);
}

#endif /*__STM32_UTIL_H__*/
