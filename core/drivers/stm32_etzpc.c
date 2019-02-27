// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2015-2017, ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2017-2018, STMicroelectronics
 */

#include <assert.h>
#include <drivers/stm32_etzpc.h>
#include <kernel/dt.h>
#include <kernel/generic_boot.h>
#include <initcall.h>
#include <io.h>
#include <keep.h>
#include <kernel/panic.h>
#include <mm/core_memprot.h>
#include <stdint.h>
#include <stm32_util.h>
#include <stm32mp_dt.h>
#include <stm32mp_pm.h>
#include <util.h>

#ifdef CFG_DT
#include <libfdt.h>
#endif

#define ETZPC_COMPAT		"st,stm32-etzpc"
#define ETZPC_LOCK_MASK		0x1U
#define ETZPC_MODE_SHIFT	8
#define ETZPC_MODE_MASK		GENMASK_32(1, 0)
#define ETZPC_ID_SHIFT		16
#define ETZPC_ID_MASK		GENMASK_32(7, 0)

/* ID Registers */
#define ETZPC_TZMA0_SIZE		0x000U
#define ETZPC_DECPROT0			0x010U
#define ETZPC_DECPROT_LOCK0		0x030U
#define ETZPC_HWCFGR			0x3F0U
#define ETZPC_VERR			0x3F4U

/* ID Registers fields */
#define ETZPC_TZMA0_SIZE_LOCK		BIT(31)
#define ETZPC_DECPROT0_MASK		GENMASK_32(1, 0)
#define ETZPC_HWCFGR_NUM_TZMA_SHIFT	0
#define ETZPC_HWCFGR_NUM_PER_SEC_SHIFT	8
#define ETZPC_HWCFGR_NUM_AHB_SEC_SHIFT	16
#define ETZPC_HWCFGR_CHUNCKS1N4_SHIFT	24

#define DECPROT_SHIFT			1
#define IDS_PER_DECPROT_REGS		16U
#define IDS_PER_DECPROT_LOCK_REGS	32U

#define ETZPC_TZMA_ALL_SECURE		0x3FF
#define ETZPC_TZMA_ALL_NO_SECURE	0x000

/*
 * etzpc_instance.
 * base : register virtual base address set during init given by user
 * pbase : register physical base address set during init given by user
 * chunk_size : supported TZMA size steps
 * num_tzma: number of TZMA zone read from register at init
 * num_ahb_sec : number of securable AHB master zone read from register
 * num_per_sec : number of securable AHB & APB Peripherals read from register
 * revision : IP revision read from register at init
 * periph_cfg : buffer storing holding the DECPROT configuration for peripherals
 */
struct etzpc_instance {
	uintptr_t base;
	paddr_t pbase;
	uint8_t chunck_size;
	uint8_t num_tzma;
	uint8_t num_per_sec;
	uint8_t num_ahb_sec;
	uint8_t revision;
	uint8_t *periph_cfg;
};

/* Only 1 instance of the ETZPC is expected per platform */
static struct etzpc_instance etzpc_dev;

#ifdef CFG_DT
struct dt_id_attr {
	/* The effective size of the array is meaningless here */
	fdt32_t id_attr[1];
};
#endif

/*
 * Implementation uses uint8_t to store each securable DECPROT configuration.
 * When resuming from deep suspend, the DECPROT configurations are restored.
 */
#define PERIPH_LOCK_BIT		BIT(7)
#define PERIPH_ATTR_MASK	GENMASK_32(2, 0)

static uintptr_t etzpc_base(void)
{
	if (!cpu_mmu_enabled())
		return etzpc_dev.pbase;

	if (!etzpc_dev.base)
		etzpc_dev.base = (uintptr_t)phys_to_virt(etzpc_dev.pbase,
							 MEM_AREA_IO_SEC);

	return etzpc_dev.base;
}

static bool __maybe_unused valid_decprot_id(unsigned int id)
{
	return id < (unsigned int)etzpc_dev.num_per_sec;
}

static bool __maybe_unused valid_tzma_id(unsigned int id)
{
	return id < (unsigned int)etzpc_dev.num_tzma;
}

#ifdef CFG_DT
static int etzpc_dt_conf_decprot(void *fdt, int node)
{
	const struct dt_id_attr *conf_list;
	unsigned int i;
	int len = 0;

	conf_list = (const struct dt_id_attr *)fdt_getprop(fdt, node,
							   "st,decprot", &len);
	if (conf_list == NULL) {
		IMSG("No ETZPC configuration in DT, use default");
		return 0;
	}

	for (i = 0; i < len / sizeof(uint32_t); i++) {
		enum etzpc_decprot_attributes attr;
		uint32_t value;
		uint32_t id;
		uint32_t mode;

		value = fdt32_to_cpu(conf_list->id_attr[i]);

		id = ((value >> ETZPC_ID_SHIFT) & ETZPC_ID_MASK);
		if (!valid_decprot_id(id)) {
			EMSG("Invalid DECPROT %" PRIu32, id);
			return -1;
		}

		mode = (value >> ETZPC_MODE_SHIFT) & ETZPC_MODE_MASK;
		attr = stm32mp_etzpc_binding2decprot(mode);

		stm32mp_register_etzpc_decprot(id, attr);

		etzpc_configure_decprot(id, attr);

		if ((value & ETZPC_LOCK_MASK) != 0U) {
			etzpc_lock_decprot(id);
		}
	}

	return 0;
}
#endif

void etzpc_configure_decprot(uint32_t decprot_id,
			     enum etzpc_decprot_attributes decprot_attr)
{
	uintptr_t offset = 4U * (decprot_id / IDS_PER_DECPROT_REGS);
	uint32_t shift = (decprot_id % IDS_PER_DECPROT_REGS) << DECPROT_SHIFT;
	uint32_t masked_decprot = (uint32_t)decprot_attr & ETZPC_DECPROT0_MASK;
	uintptr_t base = etzpc_base();

	assert(valid_decprot_id(decprot_id));

	mmio_clrsetbits_32(base + ETZPC_DECPROT0 + offset,
			   (uint32_t)ETZPC_DECPROT0_MASK << shift,
			   masked_decprot << shift);

	etzpc_dev.periph_cfg[decprot_id] = (uint8_t)decprot_attr;

	assert((decprot_attr & ~PERIPH_ATTR_MASK) == 0);
	COMPILE_TIME_ASSERT(TZPC_DECPROT_MAX <= UINT8_MAX);
}

enum etzpc_decprot_attributes etzpc_get_decprot(uint32_t decprot_id)
{
	uintptr_t offset = 4U * (decprot_id / IDS_PER_DECPROT_REGS);
	uint32_t shift = (decprot_id % IDS_PER_DECPROT_REGS) << DECPROT_SHIFT;
	uintptr_t base_decprot = etzpc_base() + offset;
	uint32_t value;

	assert(valid_decprot_id(decprot_id));

	value = (read32(base_decprot + ETZPC_DECPROT0) >> shift) &
		ETZPC_DECPROT0_MASK;

	return (enum etzpc_decprot_attributes)value;
}

void etzpc_lock_decprot(uint32_t decprot_id)
{
	uintptr_t offset = 4U * (decprot_id / IDS_PER_DECPROT_LOCK_REGS);
	uint32_t shift = BIT(decprot_id % IDS_PER_DECPROT_LOCK_REGS);
	uintptr_t base_decprot = etzpc_base() + offset;

	assert(valid_decprot_id(decprot_id));

	write32(shift, base_decprot + ETZPC_DECPROT_LOCK0);

	etzpc_dev.periph_cfg[decprot_id] |= PERIPH_LOCK_BIT;
}

void etzpc_configure_tzma(uint32_t tzma_id, uint16_t tzma_value)
{
	assert(valid_tzma_id(tzma_id));

	write32(tzma_value, etzpc_base() + ETZPC_TZMA0_SIZE +
		(sizeof(uint32_t) * tzma_id));
}

uint16_t etzpc_get_tzma(uint32_t tzma_id)
{
	assert(valid_tzma_id(tzma_id));

	return read32(etzpc_base() + ETZPC_TZMA0_SIZE +
		      (sizeof(uint32_t) * tzma_id));
}

void etzpc_lock_tzma(uint32_t tzma_id)
{
	assert(valid_tzma_id(tzma_id));

	mmio_setbits_32(etzpc_base() + ETZPC_TZMA0_SIZE +
			(sizeof(uint32_t) * tzma_id), ETZPC_TZMA0_SIZE_LOCK);
}

bool etzpc_get_lock_tzma(uint32_t tzma_id)
{
	uint32_t tzma_size;

	assert(valid_tzma_id(tzma_id));

	tzma_size = read32(etzpc_base() + ETZPC_TZMA0_SIZE +
			   (sizeof(uint32_t) * tzma_id));

	return (tzma_size & ETZPC_TZMA0_SIZE_LOCK) != 0;
}

static void etzpc_suspend_resume(enum pm_op op, void __unused *handle)
{
	unsigned int n;

	if (op == PM_OP_SUSPEND) {
		return;
	}

	/* OP-TEE owns the whole in SYSRAM */
	etzpc_configure_tzma(1, ETZPC_TZMA_ALL_SECURE);

	for (n = 0; n < etzpc_dev.num_per_sec; n++) {
		unsigned int attr = etzpc_dev.periph_cfg[n] & PERIPH_ATTR_MASK;

		etzpc_configure_decprot(n, (enum etzpc_decprot_attributes)attr);

		if (etzpc_dev.periph_cfg[n] & PERIPH_LOCK_BIT) {
			etzpc_lock_decprot(n);
		}
	}
}
KEEP_PAGER(etzpc_suspend_resume);

static TEE_Result etzpc_init(void)
{
	void *fdt __maybe_unused;
	int node __maybe_unused;
	struct dt_node_info etzpc_info __maybe_unused;
	uintptr_t base;
	uint32_t hwcfg;
	size_t n;

#ifdef CFG_DT
	fdt = get_dt_blob();
	if (!fdt) {
		panic();
	}

	node = fdt_get_node(fdt, &etzpc_info, -1, ETZPC_COMPAT);
	if (node >= 0) {
		etzpc_dev.pbase = etzpc_info.base;
		assert(etzpc_dev.pbase == stm32mp_get_etzpc_base());

		if (etzpc_info.status != DT_STATUS_OK_SEC) {
			EMSG("ETZPC DT status");
			panic();
		}
	}
#endif

	if (!etzpc_dev.pbase) {
		etzpc_dev.pbase = stm32mp_get_etzpc_base();
	}

	base = etzpc_base();

	hwcfg = read32(base + ETZPC_HWCFGR);
	etzpc_dev.num_tzma = (uint8_t)(hwcfg >> ETZPC_HWCFGR_NUM_TZMA_SHIFT);
	etzpc_dev.num_per_sec = (uint8_t)(hwcfg >>
					  ETZPC_HWCFGR_NUM_PER_SEC_SHIFT);
	etzpc_dev.num_ahb_sec = (uint8_t)(hwcfg >>
					  ETZPC_HWCFGR_NUM_AHB_SEC_SHIFT);
	etzpc_dev.chunck_size = (uint8_t)(hwcfg >>
					  ETZPC_HWCFGR_CHUNCKS1N4_SHIFT);

	etzpc_dev.revision = read8(base + ETZPC_VERR);

	etzpc_dev.periph_cfg = calloc(etzpc_dev.num_per_sec,
				      sizeof(*etzpc_dev.periph_cfg));
	if (etzpc_dev.periph_cfg == NULL) {
		panic();
	}
	for (n = 0; n < etzpc_dev.num_per_sec; n++) {
		etzpc_configure_decprot(n, etzpc_get_decprot(n));
	}

	DMSG("ETZPC version 0x%" PRIx8, etzpc_dev.revision);

#ifdef CFG_DT
	if (etzpc_dt_conf_decprot(fdt, node)) {
		panic();
	}
#endif

	/* OP-TEE owns the whole in SYSRAM */
	etzpc_configure_tzma(1, ETZPC_TZMA_ALL_SECURE);

	stm32mp_register_pm_cb(etzpc_suspend_resume, NULL);

	return TEE_SUCCESS;
}
driver_init(etzpc_init);
