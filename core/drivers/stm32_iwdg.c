// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2017-2018, STMicroelectronics - All Rights Reserved
 * Copyright (c) 2017-2018, ARM Limited and Contributors. All rights reserved.
 */

#include <assert.h>
#include <drivers/stm32_iwdg.h>
#include <io.h>
#include <keep.h>
#include <kernel/delay.h>
#include <kernel/dt.h>
#include <kernel/generic_boot.h>
#include <kernel/interrupt.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <libfdt.h>
#include <mm/core_memprot.h>
#include <stm32_util.h>
#include <stm32mp_dt.h>
#include <stm32mp_pm.h>
#include <string.h>
#include <trace.h>

/* IWDG Compatibility */
#define IWDG_COMPAT		"st,stm32mp1-iwdg"
#define IWDG_TIMEOUT_US		100000U

/* IWDG registers offsets */
#define IWDG_KR_OFFSET		0x00U
#define IWDG_PR_OFFSET		0x04U
#define IWDG_RLR_OFFSET		0x08U
#define IWDG_SR_OFFSET		0x0CU
#define IWDG_EWCR_OFFSET	0x14U

/* Registers values */
#define IWDG_KR_ACCESS_KEY	0x5555
#define IWDG_KR_RELOAD_KEY	0xAAAA
#define IWDG_KR_START_KEY	0xCCCC

#define IWDG_PR_DIV_4		0x00
#define IWDG_PR_DIV_256		0x06

#define IWDG_RLR_MAX_VAL	0xFFF

#define IWDG_SR_EWU		BIT(3)

#define IWDG_EWCR_EWIE		BIT(15)
#define IWDG_EWCR_EWIC		BIT(14)
#define IWDG_EWCR_EWIT_MASK	GENMASK_32(11, 0)

struct stm32_iwdg_instance {
	uintptr_t pbase;
	uintptr_t vbase;
	unsigned long clock;
	uint8_t instance;
	uint8_t flags;
};

static struct stm32_iwdg_instance *stm32_iwdg;
static size_t stm32_iwdg_count;

static uintptr_t get_base(struct stm32_iwdg_instance *iwdg)
{
	if (!cpu_mmu_enabled()) {
		return iwdg->pbase;
	}

	return iwdg->vbase;
}

static int stm32_iwdg_get_dt_node(void *fdt, struct dt_node_info *info,
				  int offset)
{
	int node;

	node = fdt_get_node(fdt, info, offset, IWDG_COMPAT);
	if (node < 0) {
		if (offset == -1) {
			DMSG("No IDWG found");
		}
		return -FDT_ERR_NOTFOUND;
	}

	return node;
}

static struct stm32_iwdg_instance *get_iwdg(unsigned int instance)
{
	size_t i;

	for (i = 0; i < stm32_iwdg_count; i++) {
		if (stm32_iwdg[i].instance == instance) {
			return &stm32_iwdg[i];
		}
	}

	return NULL;
}

static enum itr_return stm32_iwdg_it_handler(struct itr_handler *handler)
{
	unsigned int __maybe_unused cpu = get_core_pos();
	int instance = stm32mp_iwdg_irq2instance(handler->it);
	struct stm32_iwdg_instance *iwdg = get_iwdg(instance);
	uintptr_t iwdg_base = get_base(iwdg);

	DMSG("CPU %u IT Watchdog %d\n", cpu, instance + 1);

	stm32_iwdg_refresh(instance);

	stm32_clock_enable(iwdg->clock);

	mmio_setbits_32(iwdg_base + IWDG_EWCR_OFFSET, IWDG_EWCR_EWIC);

	stm32_clock_disable(iwdg->clock);

	/*
	 * Ack interrupt as we do not return from next call.
	 * And interrupt is no more considered as pending here.
	 */
	stm32mp_gic_set_end_of_interrupt(handler->it);

	stm32_cores_reset();

	return ITRR_HANDLED;
}
KEEP_PAGER(stm32_iwdg_it_handler);

static int stm32_iwdg_get_secure_timeout(void *fdt, int node)
{
	const fdt32_t *cuint;

	cuint = fdt_getprop(fdt, node, "secure-timeout-sec", NULL);
	if (cuint == NULL) {
		return -1;
	}

	return (int)fdt32_to_cpu(*cuint);
}

static int stm32_iwdg_conf_etimeout(void *fdt, int node,
				    struct stm32_iwdg_instance *iwdg)
{
	int id_lsi;
	int dt_secure_timeout = stm32_iwdg_get_secure_timeout(fdt, node);
	uint32_t reload, status;
	uint64_t timeout_ref;
	unsigned long long reload_ll;
	uintptr_t iwdg_base = get_base(iwdg);
	struct itr_handler *itr;

	if (dt_secure_timeout < 0) {
		return 0;
	}

	if (dt_secure_timeout == 0) {
		return -1;
	}

	id_lsi = fdt_get_clock_id_by_name(fdt, node, "lsi");
	if (id_lsi < 0) {
		return -1;
	}

	/* Prescaler fix to 256 */
	reload_ll = (unsigned long long)dt_secure_timeout *
		    stm32mp1_clk_get_rate(id_lsi);
	reload = ((uint32_t)(reload_ll >> 8) - 1U) & IWDG_EWCR_EWIT_MASK;

	stm32_clock_enable(iwdg->clock);

	write32(IWDG_KR_START_KEY, iwdg_base + IWDG_KR_OFFSET);
	write32(IWDG_KR_ACCESS_KEY, iwdg_base + IWDG_KR_OFFSET);
	write32(IWDG_PR_DIV_256, iwdg_base + IWDG_PR_OFFSET);
	write32(IWDG_EWCR_EWIE | reload, iwdg_base + IWDG_EWCR_OFFSET);

	timeout_ref = utimeout_init(IWDG_TIMEOUT_US);
	do {
		status = read32(iwdg_base + IWDG_SR_OFFSET) & IWDG_SR_EWU;
		if (utimeout_elapsed(IWDG_TIMEOUT_US, timeout_ref)) {
			stm32_clock_disable(iwdg->clock);
			return -1;
		}
	} while (status != 0U);

	stm32_clock_disable(iwdg->clock);

	itr = calloc(1, sizeof(struct itr_handler));
	if (itr == NULL) {
		panic("out of memory");
	}

	itr->it = stm32mp_iwdg_instance2irq(iwdg->instance);
	itr->handler = stm32_iwdg_it_handler;
	itr_add(itr);
	itr_enable(itr->it);

	return 0;
}

void stm32_iwdg_refresh(uint32_t instance)
{
	struct stm32_iwdg_instance *iwdg = get_iwdg(instance);
	uintptr_t iwdg_base = get_base(iwdg);

	assert(iwdg);

	stm32_clock_enable(iwdg->clock);

	write32(IWDG_KR_RELOAD_KEY, iwdg_base + IWDG_KR_OFFSET);

	stm32_clock_disable(iwdg->clock);
}

static TEE_Result iwdg_init(void)
{
	int node = -1;
	int res;
	struct dt_node_info dt_info;
	void *fdt;
	size_t count;

	fdt = get_dt_blob();
	if (!fdt) {
		panic();
	}

	assert((stm32_iwdg == NULL) && (stm32_iwdg_count == 0));
	count = 0;

	for (node = stm32_iwdg_get_dt_node(fdt, &dt_info, node);
	     node != -FDT_ERR_NOTFOUND;
	     node = stm32_iwdg_get_dt_node(fdt, &dt_info, node)) {
		struct stm32_iwdg_instance iwdg;
		enum teecore_memtypes memtype;
		uint32_t hw_init;

		memset(&iwdg, 0, sizeof(iwdg));
		iwdg.pbase = dt_info.base;
		iwdg.clock = (unsigned long)dt_info.clock;
		iwdg.instance = (uint8_t)stm32mp_iwdg_iomem2instance(iwdg.pbase);

		memtype = ((dt_info.status & DT_STATUS_OK_NSEC) != 0) ?
			  MEM_AREA_IO_NSEC : MEM_AREA_IO_SEC;
		iwdg.vbase = (uintptr_t)phys_to_virt(iwdg.pbase, memtype);

		/* DT can specify low power cases */
		if (fdt_getprop(fdt, node, "stm32,enable-on-stop", NULL) ==
		    NULL) {
			iwdg.flags |= IWDG_DISABLE_ON_STOP;
		}

		if (fdt_getprop(fdt, node, "stm32,enable-on-standby", NULL) ==
		    NULL) {
			iwdg.flags |= IWDG_DISABLE_ON_STANDBY;
		}

		hw_init = stm32_get_iwdg_otp_config(iwdg.pbase);

		if ((hw_init & IWDG_HW_ENABLED) != 0) {
			if (dt_info.status == DT_STATUS_DISABLED) {
				panic("IWDG HW enabled");
			}
			iwdg.flags |= IWDG_HW_ENABLED;
		}

		if ((hw_init & IWDG_DISABLE_ON_STOP) != 0) {
			iwdg.flags |= IWDG_DISABLE_ON_STOP;
		}

		if ((hw_init & IWDG_DISABLE_ON_STANDBY) != 0) {
			iwdg.flags |= IWDG_DISABLE_ON_STANDBY;
		}

		if (dt_info.status == DT_STATUS_DISABLED) {
			continue;
		}

		DMSG("IWDG%u found, %ssecure", iwdg.instance + 1,
		     ((dt_info.status & DT_STATUS_OK_NSEC) != 0) ? "non " : "");

		if ((dt_info.status & DT_STATUS_OK_NSEC) != 0) {
			stm32mp_register_non_secure_periph_iomem(iwdg.pbase);
		} else {
			stm32mp_register_secure_periph_iomem(iwdg.pbase);
		}

		stm32_clock_enable(iwdg.clock);
		stm32_clock_disable(iwdg.clock);

		res = stm32_iwdg_conf_etimeout(fdt, node, &iwdg);
		if (res != 0) {
			EMSG("IWDG%x early timeout config failed (%d)\n",
			     iwdg.instance + 1, res);
			panic();
		}

		stm32_iwdg = realloc(stm32_iwdg, (count + 1) * sizeof(iwdg));
		if (stm32_iwdg == NULL) {
			panic("out of memory");
		}

		memcpy(&stm32_iwdg[count], &iwdg, sizeof(iwdg));
		count++;
	}

	stm32_iwdg_count = count;

	DMSG("%u IWDG instance%s found", count, count > 1 ? "s" : "");

	return TEE_SUCCESS;
}
driver_init(iwdg_init);
