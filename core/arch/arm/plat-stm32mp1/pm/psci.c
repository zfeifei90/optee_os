// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2017-2018, STMicroelectronics
 * Copyright (c) 2015-2018, ARM Limited and Contributors. All rights reserved.
 */

#include <arm.h>
#include <boot_api.h>
#include <drivers/stm32_etzpc.h>
#include <drivers/stm32_rng.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <dt-bindings/power/stm32mp1-power.h>
#include <initcall.h>
#include <io.h>
#include <keep.h>
#include <kernel/cache_helpers.h>
#include <kernel/delay.h>
#include <kernel/generic_boot.h>
#include <kernel/interrupt.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/spinlock.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <sm/pm.h>
#include <sm/psci.h>
#include <stm32_util.h>
#include <stm32mp_pm.h>
#include <string.h>
#include <trace.h>
#include <util.h>

#include "context.h"
#include "power.h"

/*
 * SMP boot support and access to the mailbox
 */
#define CORE_OFF		0
#define CORE_AWAKE		1
#define CORE_RET		2
#define CORE_ON			3

static int core_state[CFG_TEE_CORE_NB_CORE];
static unsigned int __maybe_unused state_lock = SPINLOCK_UNLOCK;

static uint32_t __maybe_unused lock_state_access(void)
{
	return may_spin_lock(&state_lock);
}

static void __maybe_unused unlock_state_access(uint32_t exceptions)
{
	may_spin_unlock(&state_lock, exceptions);
}

int psci_affinity_info(uint32_t affinity, uint32_t lowest_affinity_level)
{
	unsigned int pos = get_core_pos_mpidr(affinity);

	DMSG("core %zu, state %u", pos, core_state[pos]);

	if ((pos >= CFG_TEE_CORE_NB_CORE) ||
	    (lowest_affinity_level > PSCI_AFFINITY_LEVEL_ON)) {
		return PSCI_RET_INVALID_PARAMETERS;
	}

	switch (core_state[pos]) {
	case CORE_OFF:
	case CORE_RET:
		return PSCI_AFFINITY_LEVEL_OFF;
	case CORE_AWAKE:
		return PSCI_AFFINITY_LEVEL_ON_PENDING;
	case CORE_ON:
		return PSCI_AFFINITY_LEVEL_ON;
	default:
		panic();
	}
}

#if CFG_TEE_CORE_NB_CORE == 1
/*
 * Function called when a CPU is booted through the OP-TEE.
 * All cores shall register when online.
 */
void stm32mp_register_online_cpu(void)
{
	assert((core_state[0] == CORE_OFF) ||
	       (core_state[0] == CORE_RET));
	core_state[0] = CORE_ON;
}
#else
void stm32mp_register_online_cpu(void)
{
	size_t pos = get_core_pos();
	uint32_t excep = lock_state_access();

	if (pos == 0) {
		assert((core_state[pos] == CORE_OFF) ||
		       (core_state[pos] == CORE_RET));
	} else {
		if (core_state[pos] != CORE_AWAKE) {
			core_state[pos] = CORE_OFF;
			unlock_state_access(excep);
			stm32_pm_cpu_power_down_wfi();
			/* No return */
		}
	}

	core_state[pos] = CORE_ON;
	unlock_state_access(excep);
}

#define GICD_SGIR		0xF00
static void raise_sgi0_as_secure(void)
{
	dsb_ishst();
	write32(GIC_NON_SEC_SGI_0 | SHIFT_U32(TARGET_CPU1_GIC_MASK, 16),
		get_gicd_base() + GICD_SGIR);
}

static void release_secondary_early_hpen(size_t pos __unused)
{
	/* Need to send SIG#0 over Group0 after individual core 1 reset */
	raise_sgi0_as_secure();
	udelay(20);

	stm32_clock_enable(RTCAPB);

	write32(TEE_LOAD_ADDR,
		stm32mp_bkpreg(BCKR_CORE1_BRANCH_ADDRESS));
	write32(BOOT_API_A7_CORE1_MAGIC_NUMBER,
		stm32mp_bkpreg(BCKR_CORE1_MAGIC_NUMBER));

	stm32_clock_disable(RTCAPB);

	dsb_ishst();
	itr_raise_sgi(GIC_SEC_SGI_0, TARGET_CPU1_GIC_MASK);
}

/* Override default psci_cpu_on() with platform specific sequence */
int psci_cpu_on(uint32_t core_id, uint32_t entry, uint32_t context_id)
{
	size_t pos = get_core_pos_mpidr(core_id);
	uint32_t excep;
	int rc;

	if (!pos || pos >= CFG_TEE_CORE_NB_CORE)
		return PSCI_RET_INVALID_PARAMETERS;

	DMSG("core %zu, ns_entry 0x%" PRIx32 ", state %u",
		pos, entry, core_state[pos]);

	excep = lock_state_access();

	switch (core_state[pos]) {
	case CORE_ON:
		rc = PSCI_RET_ALREADY_ON;
		break;
	case CORE_AWAKE:
		rc = PSCI_RET_ON_PENDING;
		break;
	case CORE_RET:
		rc = PSCI_RET_DENIED;
		break;
	case CORE_OFF:
		core_state[pos] = CORE_AWAKE;
		rc = PSCI_RET_SUCCESS;
		break;
	default:
		panic();
	}

	unlock_state_access(excep);

	if (rc == PSCI_RET_SUCCESS) {
		generic_boot_set_core_ns_entry(pos, entry, context_id);
		release_secondary_early_hpen(pos);
	}

	return rc;
}

/* Override default psci_cpu_off() with platform specific sequence */
int psci_cpu_off(void)
{
	unsigned int pos = get_core_pos();
	uint32_t excep;

	if (pos == 0) {
		EMSG("PSCI_CPU_OFF not supported for core0, use system_off");
		return PSCI_RET_INTERNAL_FAILURE;
	}

	DMSG("core %u", pos);

	excep = lock_state_access();

	assert(core_state[pos] == CORE_ON);
	core_state[pos] = CORE_OFF;

	unlock_state_access(excep);

	/* Enable BKPREG access for the disabled CPU */
	stm32_clock_enable(RTCAPB);

	thread_mask_exceptions(THREAD_EXCP_ALL);
	stm32_pm_cpu_power_down_wfi();
	panic();
}
#endif

static int enter_cstop_suspend(unsigned int soc_mode)
{
	int rc = 1;

	if (read_isr() != 0) {
		return rc;
	}

	if (stm32_enter_cstop(soc_mode)) {
		goto resume;
	}

	if (need_to_backup_cpu_context(soc_mode)) {
		stm32_pm_cpu_power_down_wfi();
	} else {
		stm32_pm_cpu_wfi();
		rc = 0;
	}

resume:
	stm32_exit_cstop();

	return rc;
}

static int plat_suspend(uint32_t arg)
{
	unsigned int soc_mode = arg;
	size_t pos = get_core_pos();
	int rc = 1;

	if (read_isr() != 0) {
		return rc;
	}

	/* No need to lock state access as CPU is alone when here */
	assert(core_state[pos] == CORE_ON);
	core_state[pos] = CORE_RET;

	stm32mp_pm_save_context(soc_mode);

	rc = enter_cstop_suspend(soc_mode);

	stm32mp_pm_restore_context(soc_mode);
	stm32mp_pm_wipe_context();

	assert(core_state[pos] == CORE_RET);
	core_state[pos] = CORE_ON;

	return rc;
}

static void plat_resume(uint32_t arg)
{
	unsigned int soc_mode = arg;

	plat_cpu_reset_late();
	main_init_gic();

	assert(core_state[get_core_pos()] == CORE_ON);

	stm32mp_pm_restore_context(soc_mode);
}

#if !defined(CFG_STM32_RNG)
static bool plat_can_suspend(void)
{
	/* RNG is mandated for suspending TEE RAM content */
	return false;
}
#else
#if CFG_TEE_CORE_NB_CORE == 1
static bool plat_can_suspend(void)
{
	return true;
}
#else /*CFG_TEE_CORE_NB_CORE==1*/
static bool plat_can_suspend(void)
{
	size_t pos = get_core_pos();
	size_t n;
	uint32_t excep;
	bool rc = true;

	excep = lock_state_access();

	for (n = 0; n < ARRAY_SIZE(core_state); n++) {
		if (n == pos) {
			continue;
		}
		if (core_state[n] == CORE_AWAKE) {
			/* State core as lost and proceed suspend */
			core_state[n] = CORE_OFF;
		}
		if (core_state[n] != CORE_OFF) {
			rc = false;
		}
	}

	unlock_state_access(excep);

	return rc;
}
#endif /*CFG_TEE_CORE_NB_CORE==1*/
#endif /*CFG_STM32_RNG*/

/* Override default psci_system_suspend() with platform specific sequence */
int psci_system_suspend(uintptr_t entry, uint32_t context_id __unused,
			struct sm_nsec_ctx *nsec)
{
	int ret = PSCI_RET_INVALID_PARAMETERS;
	uint32_t soc_mode;
	int __maybe_unused pos = get_core_pos();

	DMSG("core %u", pos);

	if (!plat_can_suspend()) {
		return PSCI_RET_DENIED;
	}

	soc_mode = stm32mp1_get_lp_soc_mode(PSCI_MODE_SYSTEM_SUSPEND);

	switch (soc_mode) {
	case STM32_PM_CSLEEP_RUN:
		stm32_enter_csleep();
		nsec->mon_lr = (uint32_t)entry;
		return PSCI_RET_SUCCESS;
	case STM32_PM_SHUTDOWN:
		stm32_enter_cstop_shutdown(soc_mode);
		panic();
	default:
		/* Others are suspended mode: at least some context to backup */
		break;
	}

	assert(cpu_mmu_enabled() && core_state[pos] == CORE_ON);

	if (need_to_backup_cpu_context(soc_mode)) {
		sm_save_unbanked_regs(&nsec->ub_regs);
		/*
		 * sm_pm_cpu_suspend(arg, func) saves the CPU core context in TEE RAM
		 * then calls func(arg) to run the platform lower power sequence.
		 *
		 * If platform fails to suspend, sm_pm_cpu_suspend() returns with a
		 * non null return code. When sm_pm_cpu_suspend() returns 0 platform
		 * context must be restored.
		 */
		ret = sm_pm_cpu_suspend((uint32_t)soc_mode, plat_suspend);
		if (ret == 0) {
			plat_resume((uint32_t)soc_mode);
			sm_restore_unbanked_regs(&nsec->ub_regs);
		}
	} else {
		ret = plat_suspend((uint32_t)soc_mode);
	}

	if (ret == 0) {
		nsec->mon_lr = (uint32_t)entry;
		IMSG("Resumed");
		return PSCI_RET_SUCCESS;
	}

	return PSCI_RET_INTERNAL_FAILURE;
}

/* Override default psci_system_off() with platform specific sequence */
void __noreturn psci_system_off(void)
{
	uint32_t soc_mode = stm32mp1_get_lp_soc_mode(PSCI_MODE_SYSTEM_OFF);

	DMSG("core %u", get_core_pos());

	stm32_enter_cstop_shutdown(soc_mode);
}

/* Override default psci_system_reset() with platform specific sequence */
void __noreturn psci_system_reset(void)
{
	uint32_t soc_mode = stm32mp1_get_lp_soc_mode(PSCI_MODE_SYSTEM_OFF);

	DMSG("core %u", get_core_pos());

	stm32_enter_cstop_reset(soc_mode);
}

void __noreturn stm32mp_platform_reset(int __unused cpu)
{
	psci_system_reset();
}

/* Override default psci_cpu_on() with platform supported features */
int psci_features(uint32_t psci_fid)
{
	switch (psci_fid) {
	case PSCI_PSCI_FEATURES:
	case PSCI_VERSION:
#if CFG_TEE_CORE_NB_CORE > 1
	case PSCI_CPU_ON:
	case PSCI_CPU_OFF:
#endif
	case PSCI_SYSTEM_SUSPEND:
	case PSCI_SYSTEM_RESET:
	case PSCI_SYSTEM_OFF:
		return PSCI_RET_SUCCESS;
	default:
		return PSCI_RET_NOT_SUPPORTED;
	}
}

/* Override default psci_version() to enable PSCI_VERSION_1_0 API */
uint32_t psci_version(void)
{
	return PSCI_VERSION_1_0;
}

