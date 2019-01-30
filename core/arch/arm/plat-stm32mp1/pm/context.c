// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2018, STMicroelectronics - All Rights Reserved
 * Copyright (c) 2017-2018, ARM Limited and Contributors. All rights reserved.
 */

#include <arm32.h>
#include <boot_api.h>
#include <drivers/gic.h>
#include <drivers/stm32_rng.h>
#include <drivers/stm32_rtc.h>
#include <drivers/stm32mp1_clk.h>
#include <drivers/stm32mp1_ddrc.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <dt-bindings/power/stm32mp1-power.h>
#include <generated/context_asm_defines.h>
#include <initcall.h>
#include <kernel/cache_helpers.h>
#include <kernel/delay.h>
#include <keep.h>
#include <kernel/panic.h>
#include <mm/core_memprot.h>
#include <mm/mobj.h>
#include <platform_config.h>
#include <stdlib.h>
#include <stm32_util.h>
#include <stm32mp_pm.h>
#include <string.h>

#include "context.h"
#include "power.h"

#define TRAINING_AREA_SIZE		64

#define STANDBY_CONTEXT_MAGIC		(0x00010000 + TRAINING_AREA_SIZE)

/*
 * Context saved in TEE RAM during lower power sequence.
 * Can be allocated if to big for static allocation.
 *
 * @stgen_cnt_h: Upper 32bit of the STGEN counter
 * @stgen_cnt_l: Lower 32bit of the STGEN counter
 * @rtc: RTC time read at suspend
 */
struct pm_context {
	uint32_t stgen_cnt_h;
	uint32_t stgen_cnt_l;
	struct stm32_rtc_calendar rtc;
};

static struct pm_context plat_ctx;

/*
 * BKPSRAM contains a mailbox used with early boot stages for resume sequence.
 * The mailbox content data that must be restored before OP-TEE is resumed.
 *
 * @magic: magic value read by early boot stage for consistency
 * @zq0cr0_zdata: DDRPHY configuration to be restored.
 * @ddr_training_backup: DDR area saved at suspend and backed up at resume
 */
struct pm_mailbox {
	uint32_t magic;
	uint32_t core0_resume_ep;
	uint32_t zq0cr0_zdata;
	uint8_t ddr_training_backup[TRAINING_AREA_SIZE];
};

/*
 * BKPSRAM contains OP-TEE resume instruction sequence which restores
 * TEE RAM content. The BKPSRAM contains restoration materials
 * (key, tag) and the resume entry point in restored TEE RAM.
 */
static struct retram_resume_ctx *get_retram_resume_ctx(void)
{
	uintptr_t bkpsram_base = stm32mp1_bkpsram_base();
	uintptr_t context_base = bkpsram_base + BKPSRAM_PM_CONTEXT_OFFSET;

	return (struct retram_resume_ctx *)context_base;
}

static struct pm_mailbox *get_pm_mailbox(void)
{
	uintptr_t bkpsram_base = stm32mp1_bkpsram_base();
	uintptr_t mailbox_base = bkpsram_base + BKPSRAM_PM_MAILBOX_OFFSET;

	return (struct pm_mailbox *)mailbox_base;
}

#if TRACE_LEVEL >= TRACE_DEBUG
static void __maybe_unused dump_context(void)
{
	struct pm_mailbox *mailbox = get_pm_mailbox();
	struct retram_resume_ctx *ctx = get_retram_resume_ctx();

	stm32_clock_enable(RTCAPB);

	DMSG("Backup registers: address 0x%" PRIx32 ", magic 0x%" PRIx32,
		*(uint32_t *)stm32mp_bkpreg(BCKR_CORE1_BRANCH_ADDRESS),
		*(uint32_t *)stm32mp_bkpreg(BCKR_CORE1_MAGIC_NUMBER));

	stm32_clock_disable(RTCAPB);

	stm32_clock_enable(BKPSRAM);

	DMSG("BKPSRAM mailbox:  0x%" PRIx32 ", zd 0x%" PRIx32 ", ep 0x%" PRIx32,
		mailbox->magic, mailbox->zq0cr0_zdata,
		mailbox->core0_resume_ep);

	DMSG("BKPSRAM context:  teeram backup @%" PRIx32 ", resume @0x%" PRIx32,
		ctx->teeram_bkp_pa, ctx->resume_pa);

	stm32_clock_disable(BKPSRAM);
}
#else
static void __maybe_unused dump_context(void)
{
}
#endif

/*
 * Save and restore functions
 */
static void save_time(void)
{
        uintptr_t stgen = stm32_get_stgen_base();

	plat_ctx.stgen_cnt_h = read32(stgen + CNTCVU_OFFSET);
	plat_ctx.stgen_cnt_l = read32(stgen + CNTCVL_OFFSET);
	if (plat_ctx.stgen_cnt_l < 10) {
		plat_ctx.stgen_cnt_h = read32(stgen + CNTCVU_OFFSET);
	}

	stm32_rtc_get_calendar(&plat_ctx.rtc);
}

#if TRACE_LEVEL >= TRACE_DEBUG
static void print_ccm_decryption_duration(void)
{
	uintptr_t stgen = stm32_get_stgen_base();
	struct retram_resume_ctx *ctx = get_retram_resume_ctx();


	stm32_clock_enable(BKPSRAM);

	DMSG("CCM decryption duration %llums",
		((unsigned long long)ctx->stgen_cnt * 1000) /
		mmio_read_32(stgen + CNTFID_OFFSET));

	stm32_clock_enable(BKPSRAM);
}
#else
static void print_ccm_decryption_duration(void)
{
}
#endif

static void restore_time(void)
{
	struct stm32_rtc_calendar current_calendar;
	unsigned long long stdby_time_in_ms;
	unsigned long long cnt;
	uintptr_t stgen = stm32_get_stgen_base();
	struct retram_resume_ctx __maybe_unused *ctx = get_retram_resume_ctx();

	stm32_rtc_get_calendar(&current_calendar);
	stdby_time_in_ms = stm32_rtc_diff_calendar(&current_calendar,
						   &plat_ctx.rtc);

	cnt = ((uint64_t)plat_ctx.stgen_cnt_h << 32) | plat_ctx.stgen_cnt_l;
	cnt += (stdby_time_in_ms * mmio_read_32(stgen + CNTFID_OFFSET)) / 1000U;

	mmio_clrbits_32(stgen + CNTCR_OFFSET, CNTCR_EN);
	mmio_write_32(stgen + CNTCVL_OFFSET, (uint32_t)cnt);
	mmio_write_32(stgen + CNTCVU_OFFSET, (uint32_t)(cnt >> 32));
	mmio_setbits_32(stgen + CNTCR_OFFSET, CNTCR_EN);

	print_ccm_decryption_duration();
}

static bool __maybe_unused pm_cb_is_valid(void (*cb)(enum pm_op op, void *hdl),
					  void *hdl)
{
	void *cb_voidp = (void *)(uintptr_t)cb;
	paddr_t cb_phy = virt_to_phys(cb_voidp);
	paddr_t hdl_phy = virt_to_phys(hdl);
	bool __maybe_unused valid;

	valid = (phys_to_virt(cb_phy, MEM_AREA_TEE_RAM_RX) == cb_voidp) &&
		((phys_to_virt(hdl_phy, MEM_AREA_TEE_RAM_RX) == hdl) ||
		 (phys_to_virt(hdl_phy, MEM_AREA_TEE_RAM_RO) == hdl) ||
		 (phys_to_virt(hdl_phy, MEM_AREA_TEE_RAM_RW) == hdl));

	if (!valid) {
		EMSG("pm_cb mandates unpaged arguments %p %p", cb_voidp, hdl);
	}

	return valid;
}

struct pm_cb {
	void (*callback)(enum pm_op op, void *handle);
	void *handle;
};
static struct pm_cb *pm_cb;
static size_t pm_cb_count;

void stm32mp_register_pm_cb(void (*callback)(enum pm_op op, void *handle),
			    void *handle)
{
	assert(pm_cb_is_valid(callback, handle));

	pm_cb_count++;
	pm_cb = realloc(pm_cb, sizeof(struct pm_cb) * pm_cb_count);
	if (!pm_cb){
		panic();
	}

	pm_cb[pm_cb_count - 1].callback = callback;
	pm_cb[pm_cb_count - 1].handle = handle;
}

static void save_soc_context(void)
{
	const enum pm_op suspend = PM_OP_SUSPEND;
	size_t n;

	for (n = 0; n < pm_cb_count; n++) {
		pm_cb[n].callback(suspend, pm_cb[n].handle);
	}

	/* Suspend core services */
	stm32mp_gic_suspend_resume(suspend);
	stm32mp_clock_suspend_resume(suspend);
}

static void restore_soc_context(void)
{
	const enum pm_op resume = PM_OP_RESUME;
	size_t n;

	/* Resume core services */
	stm32mp_gic_suspend_resume(resume);
	stm32mp_clock_suspend_resume(resume);

	for (n = 0; n < pm_cb_count; n++) {
		pm_cb[n].callback(resume, pm_cb[n].handle);
	}
}

uintptr_t stm32mp_pm_retram_resume_ep(void)
{
	struct retram_resume_ctx *ctx = get_retram_resume_ctx();

	return (uintptr_t)&ctx->resume_sequence;
}

/* Clear the content of the PM mailbox */
void stm32mp_pm_wipe_context(void)
{
	struct retram_resume_ctx *ctx = get_retram_resume_ctx();
	struct pm_mailbox *mailbox = get_pm_mailbox();

	stm32_clock_enable(BKPSRAM);

	memset(ctx, 0xa5, sizeof(*ctx));
	memset(mailbox, 0xa5, sizeof(*mailbox));

	stm32_clock_disable(BKPSRAM);
}

static struct mobj *teeram_bkp_mobj;

static void init_retram_resume_resources(void)
{
	struct retram_resume_ctx *ctx = get_retram_resume_ctx();
	const size_t size = (uintptr_t)stm32mp_bkpsram_image_end -
			    (uintptr_t)stm32mp_bkpsram_resume;
	paddr_t __maybe_unused pa;

	COMPILE_TIME_ASSERT(sizeof(struct pm_mailbox) <
			    BKPSRAM_PM_MAILBOX_SIZE);
	COMPILE_TIME_ASSERT(sizeof(struct retram_resume_ctx) <
			    BKPSRAM_PM_CONTEXT_SIZE);
	assert((sizeof(*ctx) + size) < BKPSRAM_PM_CONTEXT_SIZE);

	teeram_bkp_mobj = mobj_mm_alloc(mobj_sec_ddr, TEE_RAM_PH_SIZE,
					&tee_mm_sec_ddr);
	if (teeram_bkp_mobj == NULL) {
		panic();
	}
	assert((mobj_get_va(teeram_bkp_mobj, 0) != NULL) &&
	       (mobj_get_pa(teeram_bkp_mobj, 0, 0, &pa) == 0));

	stm32_clock_enable(BKPSRAM);
	memset(ctx, 0, sizeof(*ctx));
	stm32_clock_disable(BKPSRAM);
}

/*
 * When returning from STANDBY, the 64 first bytes of DDR will be overwritten
 * during DDR DQS training. This area must then be saved before going to
 * standby in the PM mailbox with the earlier boot stages.
 */
static void save_ddr_training_area(void)
{
	struct pm_mailbox *mailbox = get_pm_mailbox();
	size_t size = sizeof(mailbox->ddr_training_backup);
	struct mobj __maybe_unused *mobj = NULL;
	paddr_t pa = DDR_BASE;
	void *va = phys_to_virt(pa, MEM_AREA_RAM_NSEC);

#if !defined(CFG_STM32MP_MAP_NSEC_LOW_DDR)
	/* Config switch helps not requesting mobj_mapped_shm_alloc() unpaged */
	if (!va) {
		mobj = mobj_mapped_shm_alloc(&pa, SMALL_PAGE_SIZE, 0, 0);
		va = mobj_get_va(mobj, 0);
	}
#endif

	memcpy(&mailbox->ddr_training_backup[0], va, size);

	mobj_free(mobj);
}

static void load_earlyboot_pm_mailbox(void)
{
	struct pm_mailbox *mailbox = get_pm_mailbox();

	COMPILE_TIME_ASSERT(sizeof(struct pm_mailbox) <
			    BKPSRAM_PM_MAILBOX_SIZE);

	assert(stm32_clock_is_enabled(BKPSRAM));

	memset(mailbox, 0, sizeof(*mailbox));

	mailbox->zq0cr0_zdata = get_ddrphy_calibration();

	save_ddr_training_area();
}

#ifdef CFG_STM32_RNG
/*
 * CRYP relies on standard format for CCM IV/B0/CRT0 data. Our sequence uses
 * no AAD, 4 bytes to encode the payload byte size and a 11 byte nonce.
 */
#define PM_CCM_Q			4
#define PM_CCM_Q_FLAGS			(PM_CCM_Q - 1)
#define PM_CCM_TAG_LEN			16
#define PM_CCM_TAG_FLAGS		(((PM_CCM_TAG_LEN - 2) / 2) << 3)

static void save_teeram_in_ddr(void)
{
	struct retram_resume_ctx *ctx = get_retram_resume_ctx();
	size_t __maybe_unused size = (uintptr_t)stm32mp_bkpsram_image_end -
				     (uintptr_t)stm32mp_bkpsram_resume;
	paddr_t pa;
	struct ccm_unpg_ctx *ccm = &ctx->ccm_ctx;
	void *teeram = phys_to_virt(TEE_RAM_START, MEM_AREA_ROM_SEC);
	void *teeram_bkp = mobj_get_va(teeram_bkp_mobj, 0);

	COMPILE_TIME_ASSERT(PM_CTX_CCM_KEY_SIZE == sizeof(ccm->key));
	COMPILE_TIME_ASSERT(PM_CTX_CCM_CTR1_SIZE == sizeof(ccm->ctr1));
	COMPILE_TIME_ASSERT(PM_CTX_CCM_B0_SIZE == sizeof(ccm->b0));
	COMPILE_TIME_ASSERT(PM_CTX_CCM_CTR0_SIZE == sizeof(ccm->ctr0));
	COMPILE_TIME_ASSERT(PM_CTX_CCM_TAG_SIZE == sizeof(ccm->tag));

	assert(stm32_clock_is_enabled(BKPSRAM) &&
	       stm32_clock_is_enabled(CRYP1));

	memcpy(ctx->resume_sequence,
	       (void *)(uintptr_t)stm32mp_bkpsram_resume, size);

	memset(ctx, 0, sizeof(*ctx));
	ctx->resume_pa = virt_to_phys((void *)(uintptr_t)stm32mp_sysram_resume);
	if (mobj_get_pa(teeram_bkp_mobj, 0, 0, &pa)) {
		panic();
	}
	ctx->teeram_bkp_pa = (uint32_t)pa;
	ctx->cryp1_base = (uint32_t)phys_to_virt(CRYP1_BASE, MEM_AREA_IO_SEC);
	ctx->rcc_base = (uint32_t)phys_to_virt(RCC_BASE, MEM_AREA_IO_SEC);
	ctx->stgen_base = (uint32_t)phys_to_virt(STGEN_BASE, MEM_AREA_IO_SEC);

	if (stm32_rng_read((uint8_t *)ccm->key, sizeof(ccm->key))) {
		panic();
	}

	assert(((PM_CCM_TAG_FLAGS & ~0x38U) | (PM_CCM_Q_FLAGS & ~0x07U)) == 0);
	COMPILE_TIME_ASSERT(PM_CCM_Q <= 4);
	COMPILE_TIME_ASSERT(TEE_RAM_PH_SIZE > UINT16_MAX);
	COMPILE_TIME_ASSERT(TEE_RAM_PH_SIZE < UINT32_MAX);

	if (stm32_rng_read((uint8_t *)ccm->ctr1, sizeof(ccm->ctr1))) {
		panic();
	}
	ccm->ctr1[0] &= GENMASK_32(24, 0);
	memcpy(ccm->b0, ccm->ctr1, sizeof(ccm->b0));
	memcpy(ccm->ctr0, ccm->ctr1, sizeof(ccm->ctr0));

	ccm->ctr0[0] |= PM_CCM_Q_FLAGS << 24;
	ccm->ctr0[3] = 0;
	ccm->ctr1[0] |= PM_CCM_Q_FLAGS << 24;
	ccm->ctr1[3] = 1;
	ccm->b0[0] |= (PM_CCM_Q_FLAGS | PM_CCM_TAG_FLAGS) << 24;
	ccm->b0[3] = TEE_RAM_PH_SIZE;

	stm32mp_ccm_encrypt_teeram(ctx, teeram_bkp, teeram, TEE_RAM_PH_SIZE);
	dcache_clean_range(teeram_bkp, TEE_RAM_PH_SIZE);

	memcpy(ctx->ccm_ref_tag, ccm->tag, sizeof(ctx->ccm_ref_tag));

	DMSG("CCM encryption duration %llums",
		((unsigned long long)ctx->stgen_cnt * 1000) /
		mmio_read_32(ctx->stgen_base + CNTFID_OFFSET));
	ctx->stgen_cnt = 0;
}
#else
static void save_teeram_in_ddr(void)
{
	panic("Mandates RNG support");
}
#endif /*CFG_STM32_RNG*/

/* Finalize the PM mailbox now that everything is loaded */
static void enable_pm_mailbox(unsigned int suspend)
{
	struct pm_mailbox *mailbox = get_pm_mailbox();
	uint32_t magic = BOOT_API_A7_CORE0_MAGIC_NUMBER;
	uint32_t hint = 0;

	assert(stm32_clock_is_enabled(BKPSRAM) &&
	       stm32_clock_is_enabled(RTCAPB));

	if (suspend) {
		hint = virt_to_phys(&get_retram_resume_ctx()->resume_sequence);
	}

	write32(magic, stm32mp_bkpreg(BCKR_CORE1_MAGIC_NUMBER));
	write32(hint, stm32mp_bkpreg(BCKR_CORE1_BRANCH_ADDRESS));

	mailbox->core0_resume_ep = hint;
	mailbox->magic = STANDBY_CONTEXT_MAGIC;
}

static void gate_pm_context_clocks(bool enable)
{
	static bool clocks_enabled;

	if (enable) {
		assert(!clocks_enabled);
		stm32_clock_enable(BKPSRAM);
		stm32_clock_enable(RTCAPB);
		stm32_clock_enable(CRYP1);
		clocks_enabled = true;
	} else {
		/* Suspended TEE RAM state left the clocks enabled */
		if (clocks_enabled) {
			stm32_clock_disable(BKPSRAM);
			stm32_clock_disable(RTCAPB);
			stm32_clock_disable(CRYP1);
			clocks_enabled = false;
		}
	}
}

/*
 * Context (TEE RAM content + peripherals) must be restored
 * only if system may reach STANDBY state.
 */
void stm32mp_pm_save_context(unsigned int soc_mode)
{
	save_time();

	if (!need_to_backup_cpu_context(soc_mode)) {
		return;
	}

	gate_pm_context_clocks(true);
	load_earlyboot_pm_mailbox();
	save_soc_context();
	save_teeram_in_ddr();
	enable_pm_mailbox(1);
}

void stm32mp_pm_restore_context(unsigned int soc_mode)
{
	if (need_to_backup_cpu_context(soc_mode)) {
		restore_soc_context();
		gate_pm_context_clocks(false);
	}

	restore_time();
}

void stm32mp_pm_shutdown_context(void)
{
	gate_pm_context_clocks(true);
	load_earlyboot_pm_mailbox();
	enable_pm_mailbox(0);
	gate_pm_context_clocks(false);
}

static TEE_Result init_pm_support(void)
{
	init_retram_resume_resources();

	stm32mp_pm_wipe_context();

	return TEE_SUCCESS;
}
driver_init(init_pm_support);
