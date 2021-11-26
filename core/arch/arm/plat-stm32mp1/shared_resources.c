// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2017-2022, STMicroelectronics
 */

#include <config.h>
#include <drivers/stm32_etzpc.h>
#include <drivers/stm32_gpio.h>
#include <drivers/stm32mp_dt_bindings.h>
#ifdef CFG_STM32MP13
#include <drivers/stm32mp13_rcc.h>
#else /* assume CFG_STM32MP15 */
#include <drivers/stm32mp1_rcc.h>
#endif
#include <initcall.h>
#include <io.h>
#include <keep.h>
#include <kernel/boot.h>
#include <kernel/panic.h>
#include <kernel/pm.h>
#include <kernel/spinlock.h>
#include <libfdt.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <stdbool.h>
#include <stm32_util.h>
#include <string.h>

/*
 * Once one starts to get the resource registering state, one cannot register
 * new resources. This ensures resource state cannot change.
 */
static bool registering_locked __unused;

/*
 * Shared register access: upon shared resource lock
 */
static unsigned int shregs_lock = SPINLOCK_UNLOCK;

/* Shared resource lock: assume not required if MMU is disabled */
static uint32_t lock_stm32shregs(void)
{
	return may_spin_lock(&shregs_lock);
}

static void unlock_stm32shregs(uint32_t exceptions)
{
	may_spin_unlock(&shregs_lock, exceptions);
}

void io_mask32_stm32shregs(vaddr_t va, uint32_t value, uint32_t mask)
{
	uint32_t exceptions = lock_stm32shregs();

	io_mask32(va, value, mask);

	unlock_stm32shregs(exceptions);
}

void io_clrsetbits32_stm32shregs(vaddr_t va, uint32_t clr, uint32_t set)
{
	uint32_t exceptions = lock_stm32shregs();

	io_clrsetbits32(va, clr, set);

	unlock_stm32shregs(exceptions);
}

/*
 * Shared peripherals and resources registration
 *
 * Each resource assignation is stored in a table. The state defaults
 * to PERIPH_UNREGISTERED if the resource is not explicitly assigned.
 *
 * Resource driver that as not embedded (a.k.a their related CFG_xxx build
 * directive is disabled) are assigned to the non-secure world.
 *
 * Each IO of the GPIOZ IO can be secure or non-secure.
 *
 * It is the platform responsibility the ensure resource assignation
 * matches the access permission firewalls configuration.
 */
enum shres_state {
	SHRES_UNREGISTERED = 0,
	SHRES_SECURE,
	SHRES_NON_SECURE,
};

/* Use a byte array to store each resource state */
static uint8_t shres_state[STM32MP1_SHRES_COUNT] __maybe_unused = {
#ifdef CFG_STM32MP15
#if !defined(CFG_STM32_IWDG)
	[STM32MP1_SHRES_IWDG1] = SHRES_NON_SECURE,
#endif
#if !defined(CFG_STM32_UART)
	[STM32MP1_SHRES_USART1] = SHRES_NON_SECURE,
#endif
#if !defined(CFG_STM32_SPI)
	[STM32MP1_SHRES_SPI6] = SHRES_NON_SECURE,
#endif
#if !defined(CFG_STM32_I2C)
	[STM32MP1_SHRES_I2C4] = SHRES_NON_SECURE,
	[STM32MP1_SHRES_I2C6] = SHRES_NON_SECURE,
#endif
#if !defined(CFG_STM32_RNG)
	[STM32MP1_SHRES_RNG1] = SHRES_NON_SECURE,
#endif
#if !defined(CFG_STM32_HASH)
	[STM32MP1_SHRES_HASH1] = SHRES_NON_SECURE,
#endif
#if !defined(CFG_STM32_CRYP)
	[STM32MP1_SHRES_CRYP1] = SHRES_NON_SECURE,
#endif
#if !defined(CFG_STM32_RTC)
	[STM32MP1_SHRES_RTC] = SHRES_NON_SECURE,
#endif
#endif
};

static const char __maybe_unused *shres2str_id_tbl[STM32MP1_SHRES_COUNT] = {
#ifdef CFG_STM32MP15
	[STM32MP1_SHRES_IWDG1] = "IWDG1",
	[STM32MP1_SHRES_USART1] = "USART1",
	[STM32MP1_SHRES_SPI6] = "SPI6",
	[STM32MP1_SHRES_I2C4] = "I2C4",
	[STM32MP1_SHRES_RNG1] = "RNG1",
	[STM32MP1_SHRES_HASH1] = "HASH1",
	[STM32MP1_SHRES_CRYP1] = "CRYP1",
	[STM32MP1_SHRES_I2C6] = "I2C6",
	[STM32MP1_SHRES_RTC] = "RTC",
	[STM32MP1_SHRES_MCU] = "MCU",
	[STM32MP1_SHRES_PLL3] = "PLL3",
#endif
	[STM32MP1_SHRES_MDMA] = "MDMA",
};

static __maybe_unused const char *shres2str_id(enum stm32mp_shres id)
{
	return shres2str_id_tbl[id];
}

static const char *shres2str_state_tbl[4] __maybe_unused = {
	[SHRES_UNREGISTERED] = "unregistered",
	[SHRES_NON_SECURE] = "non-secure",
	[SHRES_SECURE] = "secure",
};

static __maybe_unused const char *shres2str_state(enum shres_state id)
{
	return shres2str_state_tbl[id];
}

#ifdef CFG_STM32MP15
static void register_periph(enum stm32mp_shres id, enum shres_state state)
{
	assert(id < STM32MP1_SHRES_COUNT &&
	       (state == SHRES_SECURE || state == SHRES_NON_SECURE));

	if (registering_locked)
		panic();

	if (shres_state[id] != SHRES_UNREGISTERED &&
	    shres_state[id] != state) {
		DMSG("Cannot change %s from %s to %s",
		     shres2str_id(id),
		     shres2str_state(shres_state[id]),
		     shres2str_state(state));
		panic();
	}

	if (shres_state[id] == SHRES_UNREGISTERED)
		DMSG("Register %s as %s",
		     shres2str_id(id), shres2str_state(state));

	shres_state[id] = state;

	/* Explore clock tree to lock secure clock dependencies */
	if (state == SHRES_SECURE) {
		switch (id) {
		case STM32MP1_SHRES_IWDG1:
			stm32mp_register_clock_parents_secure(IWDG1);
			break;
		case STM32MP1_SHRES_USART1:
			stm32mp_register_clock_parents_secure(USART1_K);
			break;
		case STM32MP1_SHRES_SPI6:
			stm32mp_register_clock_parents_secure(SPI6_K);
			break;
		case STM32MP1_SHRES_I2C4:
			stm32mp_register_clock_parents_secure(I2C4_K);
			break;
		case STM32MP1_SHRES_RNG1:
			stm32mp_register_clock_parents_secure(RNG1_K);
			break;
		case STM32MP1_SHRES_HASH1:
			stm32mp_register_clock_parents_secure(HASH1);
			break;
		case STM32MP1_SHRES_CRYP1:
			stm32mp_register_clock_parents_secure(CRYP1);
			break;
		case STM32MP1_SHRES_I2C6:
			stm32mp_register_clock_parents_secure(I2C6_K);
			break;
		case STM32MP1_SHRES_RTC:
			stm32mp_register_clock_parents_secure(RTC);
			break;
		default:
			/* No expected resource dependency */
			break;
		}
	}
}

/* Register resource by ID */
void stm32mp_register_secure_periph(enum stm32mp_shres id)
{
	register_periph(id, SHRES_SECURE);
}

void stm32mp_register_non_secure_periph(enum stm32mp_shres id)
{
	register_periph(id, SHRES_NON_SECURE);
}

/* Register resource by IO memory base address */
static void register_periph_iomem(vaddr_t base, enum shres_state state)
{
	enum stm32mp_shres id = STM32MP1_SHRES_COUNT;

	switch (base) {
	case IWDG1_BASE:
		id = STM32MP1_SHRES_IWDG1;
		break;
	case USART1_BASE:
		id = STM32MP1_SHRES_USART1;
		break;
	case SPI6_BASE:
		id = STM32MP1_SHRES_SPI6;
		break;
	case I2C4_BASE:
		id = STM32MP1_SHRES_I2C4;
		break;
	case I2C6_BASE:
		id = STM32MP1_SHRES_I2C6;
		break;
	case RTC_BASE:
		id = STM32MP1_SHRES_RTC;
		break;
	case RNG1_BASE:
		id = STM32MP1_SHRES_RNG1;
		break;
	case CRYP1_BASE:
		id = STM32MP1_SHRES_CRYP1;
		break;
	case HASH1_BASE:
		id = STM32MP1_SHRES_HASH1;
		break;

#ifdef CFG_WITH_NSEC_UARTS
	case USART2_BASE:
	case USART3_BASE:
	case UART4_BASE:
	case UART5_BASE:
	case USART6_BASE:
	case UART7_BASE:
	case UART8_BASE:
	/* Fall through */
#endif
	case IWDG2_BASE:
		/* Allow drivers to register some non-secure resources */
		DMSG("IO for non-secure resource 0x%lx", base);
		if (state != SHRES_NON_SECURE)
			panic();

		return;

	default:
		panic();
		break;
	}

	register_periph(id, state);
}

void stm32mp_register_secure_periph_iomem(vaddr_t base)
{
	register_periph_iomem(base, SHRES_SECURE);
}

void stm32mp_register_non_secure_periph_iomem(vaddr_t base)
{
	register_periph_iomem(base, SHRES_NON_SECURE);
}

static void lock_registering(void)
{
	registering_locked = true;
}

bool stm32mp_periph_is_secure(enum stm32mp_shres id)
{
	lock_registering();

	return shres_state[id] == SHRES_SECURE;
}
#else
void stm32mp_register_secure_periph_iomem(vaddr_t base __maybe_unused)
{
}

void stm32mp_register_non_secure_periph_iomem(vaddr_t base __maybe_unused)
{
}

bool stm32mp_periph_is_secure(enum stm32mp_shres id __maybe_unused)
{
	return true;
}
#endif /* CFG_STM32MP15 */

bool stm32mp_nsec_can_access_clock(unsigned long clock_id)
{
	enum stm32mp_shres shres_id = STM32MP1_SHRES_COUNT;

	/* Oscillators and PLLs are visible from non-secure world */
	COMPILE_TIME_ASSERT(CK_HSE == 0 &&
			    (CK_HSE + 1) == CK_CSI &&
			    (CK_HSE + 2) == CK_LSI &&
			    (CK_HSE + 3) == CK_LSE &&
			    (CK_HSE + 4) == CK_HSI &&
			    (CK_HSE + 5) == CK_HSE_DIV2 &&
			    (PLL1_P + 1) == PLL1_Q &&
			    (PLL1_P + 2) == PLL1_R &&
			    (PLL1_P + 3) == PLL2_P &&
			    (PLL1_P + 4) == PLL2_Q &&
			    (PLL1_P + 5) == PLL2_R &&
			    (PLL1_P + 6) == PLL3_P &&
			    (PLL1_P + 7) == PLL3_Q &&
			    (PLL1_P + 8) == PLL3_R);

	if (clock_id <= CK_HSE_DIV2 ||
	    (clock_id >= PLL1_P && clock_id <= PLL3_R))
		return true;

	switch (clock_id) {
	case BSEC:
	case CK_AXI:
	case CK_CSI:
	case CK_HSE:
	case CK_HSE_DIV2:
	case CK_HSI:
	case CK_LSE:
	case CK_LSI:
	case CK_MPU:
	case PLL1_P:
	case PLL1_Q:
	case PLL1_R:
	case PLL2_P:
	case PLL2_Q:
	case PLL2_R:
	case PLL3_P:
	case PLL3_Q:
	case PLL3_R:
	case RTCAPB:
#ifdef CFG_STM32MP13
	case CK_MLAHB:
	case CK_PER:
	case PLL4_P:
	case PLL4_Q:
	case PLL4_R:
	case PCLK1:
	case PCLK2:
	case PCLK3:
	case PCLK4:
	case PCLK5:
	case PCLK6:
	case CK_TIMG1:
	case CK_TIMG2:
	case CK_TIMG3:
	case RTC:
#endif
		return true;
#ifdef CFG_STM32MP15
	case GPIOZ:
		return true;
	case SPI6_K:
		shres_id = STM32MP1_SHRES_SPI6;
		break;
	case I2C4_K:
		shres_id = STM32MP1_SHRES_I2C4;
		break;
	case I2C6_K:
		shres_id = STM32MP1_SHRES_I2C6;
		break;
	case USART1_K:
		shres_id = STM32MP1_SHRES_USART1;
		break;
	case IWDG1:
		shres_id = STM32MP1_SHRES_IWDG1;
		break;
	case CRYP1:
		shres_id = STM32MP1_SHRES_CRYP1;
		break;
	case HASH1:
		shres_id = STM32MP1_SHRES_HASH1;
		break;
	case RNG1_K:
		shres_id = STM32MP1_SHRES_RNG1;
		break;
	case RTC:
		shres_id = STM32MP1_SHRES_RTC;
		break;
	case CK_MCU:
		shres_id = STM32MP1_SHRES_MCU;
		break;
#endif
	default:
		return false;
	}

	return !stm32mp_periph_is_secure(shres_id);
}

bool stm32mp_nsec_can_access_reset(unsigned int reset_id)
{
	enum stm32mp_shres shres_id = STM32MP1_SHRES_COUNT;

	switch (reset_id) {
#ifdef CFG_STM32MP15
	case SPI6_R:
		shres_id = STM32MP1_SHRES_SPI6;
		break;
	case I2C4_R:
		shres_id = STM32MP1_SHRES_I2C4;
		break;
	case I2C6_R:
		shres_id = STM32MP1_SHRES_I2C6;
		break;
	case USART1_R:
		shres_id = STM32MP1_SHRES_USART1;
		break;
	case CRYP1_R:
		shres_id = STM32MP1_SHRES_CRYP1;
		break;
	case HASH1_R:
		shres_id = STM32MP1_SHRES_HASH1;
		break;
	case RNG1_R:
		shres_id = STM32MP1_SHRES_RNG1;
		break;
	case MCU_R:
	case MCU_HOLD_BOOT_R:
		shres_id = STM32MP1_SHRES_MCU;
		break;
#endif
	case MDMA_R:
		shres_id = STM32MP1_SHRES_MDMA;
		break;
	default:
		return false;
	}

	return !stm32mp_periph_is_secure(shres_id);
}

#ifdef CFG_STM32MP15
static bool mckprot_resource(enum stm32mp_shres id)
{
	switch (id) {
	case STM32MP1_SHRES_MCU:
	case STM32MP1_SHRES_PLL3:
		return true;
	default:
		return false;
	}
}

static void check_rcc_secure_configuration(void)
{
	bool secure = stm32_rcc_is_secure();
	bool mckprot = stm32_rcc_is_mckprot();
	bool need_secure = false;
	bool need_mckprot = false;
	enum stm32mp_shres id = STM32MP1_SHRES_COUNT;
	uint32_t state = 0;

	if (stm32_bsec_get_state(&state))
		panic();

	if (state == BSEC_STATE_SEC_CLOSED && !secure)
		panic("Closed device mandates secure RCC");

	for (id = 0; id < STM32MP1_SHRES_COUNT; id++) {
		if  (shres_state[id] != SHRES_SECURE)
			continue;

		need_secure = true;
		if (mckprot_resource(id))
			need_mckprot = true;

		if ((mckprot_resource(id) && !mckprot) || !secure)
			EMSG("Error: RCC TZEN=%u MCKPROT=%u while %s is secure",
			     secure, mckprot, shres2str_id(id));
	}

	DMSG("RCC/PWR secure hardening: TZEN %sable, MCKPROT %sable",
	     secure ? "en" : "dis", mckprot ? "en" : "dis");

	if (need_secure && !secure)
		panic();

	stm32mp1_clk_mcuss_protect(need_mckprot);
}

static TEE_Result stm32mp1_init_final_shres(void)
{
	enum stm32mp_shres id = STM32MP1_SHRES_COUNT;

	lock_registering();

	for (id = (enum stm32mp_shres)0; id < STM32MP1_SHRES_COUNT; id++) {
		uint8_t __maybe_unused *state = &shres_state[id];

		DMSG("stm32mp %-8s (%2u): %-14s",
		     shres2str_id(id), id, shres2str_state(*state));
	}

	check_rcc_secure_configuration();

	return TEE_SUCCESS;
}
/* Finalize shres after drivers initialization, hence driver_init_late() */
driver_init_late(stm32mp1_init_final_shres);
#endif
