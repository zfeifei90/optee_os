// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2018, STMicroelectronics - All Rights Reserved
 */

#include <assert.h>
#include <drivers/stm32_rng.h>
#include <drivers/stm32_reset.h>
#include <io.h>
#include <libfdt.h>
#include <platform_config.h>
#include <kernel/dt.h>
#include <kernel/delay.h>
#include <kernel/generic_boot.h>
#include <kernel/panic.h>
#include <libfdt.h>
#include <mm/core_memprot.h>
#include <stdbool.h>
#include <stm32mp_dt.h>
#include <stm32_util.h>

#define DT_RNG_COMPAT		"st,stm32-rng"
#define RNG_CR			0x00U
#define RNG_SR			0x04U
#define RNG_DR			0x08U

#define RNG_CR_RNGEN		BIT(2)
#define RNG_CR_IE		BIT(3)
#define RNG_CR_CED		BIT(5)

#define RNG_SR_DRDY		BIT(0)
#define RNG_SR_CECS		BIT(1)
#define RNG_SR_SECS		BIT(2)
#define RNG_SR_CEIS		BIT(5)
#define RNG_SR_SEIS		BIT(6)

#define RNG_TIMEOUT_US		100000

struct stm32_rng_instance {
	uintptr_t pbase;
	uintptr_t vbase;
	unsigned long clock;
};

static struct stm32_rng_instance *stm32_rng;

static uintptr_t get_base(void)
{
	if (!cpu_mmu_enabled()) {
		return stm32_rng->pbase;
	}

	return stm32_rng->vbase;
}

int stm32_rng_read(uint8_t *out, size_t size)
{
	uint8_t *buf = out;
	size_t len = size;
	uint64_t timeout_ref;
	uint32_t data32;
	uintptr_t rng_base = get_base();
	int rc = 0;
	int count;

	if (stm32_rng == 0) {
		return -1;
	}

	stm32_clock_enable(stm32_rng->clock);

	if ((read32(rng_base + RNG_CR) & RNG_CR_RNGEN) == 0U) {
		write32(RNG_CR_RNGEN | RNG_CR_CED, rng_base + RNG_CR);
	}

	while (len != 0) {
		timeout_ref = utimeout_init(RNG_TIMEOUT_US);
		do {
			uint32_t status = read32(rng_base + RNG_SR);

			if ((status & (RNG_SR_SECS | RNG_SR_SEIS)) != 0U) {
				size_t i;

				/* Recommended by the SoC reference manual */
				io_mask32(rng_base + RNG_SR, 0, RNG_SR_SEIS);
				dmb();
				for (i = 12; i != 0; i--) {
					(void)read32(rng_base + RNG_DR);
				}
				dmb();

				if ((read32(rng_base + RNG_SR) & RNG_SR_SEIS) !=
				    0U) {
					panic("RNG noise");
				}
			}

			if (utimeout_elapsed(RNG_TIMEOUT_US, timeout_ref)) {
				rc = -1;
				goto bail;
			}
		} while ((read32(rng_base + RNG_SR) & RNG_SR_DRDY) == 0U);

		count = 4;
		while (len != 0) {
			data32 = read32(rng_base + RNG_DR);
			count--;

			memcpy(buf, &data32, MIN(len, sizeof(uint32_t)));
			buf += MIN(len, sizeof(uint32_t));
			len -= MIN(len, sizeof(uint32_t));

			if (count == 0) {
				break;
			}
		}
	}

bail:
	stm32_clock_disable(stm32_rng->clock);

	if (rc != 0) {
		memset(out, 0, buf - out);
	}

	return rc;
}

static TEE_Result stm32_rng_init(void)
{
	void *fdt;
	struct dt_node_info dt_rng;
	int node;
	uint8_t __maybe_unused test[43];
	enum teecore_memtypes memtype;

	fdt = get_dt_blob();
	if (!fdt) {
		panic();
	}

	node = fdt_get_node(fdt, &dt_rng, -1, DT_RNG_COMPAT);
	if (node < 0) {
		return TEE_SUCCESS;
	}

	if ((dt_rng.status & DT_STATUS_OK_SEC) == 0) {
		return TEE_SUCCESS;
	}

	assert(dt_rng.base == RNG1_BASE);

	if (stm32_rng) {
		panic();
	}
	stm32_rng = calloc(1, sizeof(*stm32_rng));
	if (!stm32_rng) {
		panic();
	}

	stm32_rng->pbase = dt_rng.base;

	if ((dt_rng.status & DT_STATUS_OK_NSEC) != 0) {
		memtype = MEM_AREA_IO_NSEC;
		stm32mp_register_non_secure_periph_iomem(stm32_rng->pbase);
	} else {
		memtype = MEM_AREA_IO_SEC;
		stm32mp_register_secure_periph_iomem(stm32_rng->pbase);
	}

	stm32_rng->vbase = (uintptr_t)phys_to_virt(stm32_rng->pbase, memtype);

	if (dt_rng.clock < 0) {
		panic();
	}
	stm32_rng->clock = (unsigned long)dt_rng.clock;

	stm32_clock_enable(stm32_rng->clock);
	stm32_clock_disable(stm32_rng->clock);

	if (dt_rng.reset >= 0) {
		stm32_reset_assert((unsigned long)dt_rng.reset);
		udelay(20);
		stm32_reset_deassert((unsigned long)dt_rng.reset);
	}

	DMSG("Init RNG done");

#if TRACE_LEVEL >= TRACE_DEBUG
	memset(test, 0xa5, sizeof(test));
	if (stm32_rng_read(test, sizeof(test))) {
		panic("RNG test");
	}

	DHEXDUMP(test, sizeof(test));
#endif

	return TEE_SUCCESS;
}
driver_init(stm32_rng_init);
