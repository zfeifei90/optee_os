 // SPDX-License-Identifier: BSD-2-Clause
 /*
  * Copyright (C) 2020-2021, STMicroelectronics - All Rights Reserved
  */

#include <crypto/crypto.h>
#include <drivers/clk.h>
#include <drivers/stm32mp1_rcc.h>
#include <drivers/stm32mp_dt_bindings.h>
#include <kernel/pseudo_ta.h>
#include <kernel/user_ta.h>
#include <mm/core_memprot.h>
#include <remoteproc_pta.h>
#include <stm32_util.h>
#include <string.h>

#define PTA_NAME "remoteproc.pta"

#define STM32_M4_FW_ID		0

/* Firmware states */
enum rproc_load_state {
	REMOTEPROC_OFF = 0,
	REMOTEPROC_ON,
};

/*
 * struct rproc_ta_memory_region - Represent a remote processor memory mapping
 * @pa - Memory physical base address from current CPU space
 * @da - Memory physical base address from remote processor space
 * @size - Memory region byte size
 */
struct rproc_ta_memory_region {
	paddr_t pa;
	paddr_t da;
	size_t size;
};

static const struct rproc_ta_memory_region rproc_ta_mp1_m4_mems[] = {
	/* MCU SRAM */
	{ .pa = MCUSRAM_BASE, .da = 0x10000000, .size = MCUSRAM_SIZE },
	/* Alias of the MCU SRAM */
	{ .pa = MCUSRAM_BASE, .da = 0x30000000, .size = MCUSRAM_SIZE },
	/* RETRAM */
	{ .pa = RETRAM_BASE, .da = 0x00000000, .size = RETRAM_SIZE },
};

static enum rproc_load_state rproc_ta_state;

static TEE_Result rproc_pta_capabilities(uint32_t pt,
					 TEE_Param params[TEE_NUM_PARAMS])
{
	const uint32_t exp_pt = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
						TEE_PARAM_TYPE_VALUE_OUTPUT,
						TEE_PARAM_TYPE_VALUE_OUTPUT,
						TEE_PARAM_TYPE_NONE);

	if (pt != exp_pt)
		return TEE_ERROR_BAD_PARAMETERS;

	/* Support only ELF format */
	params[1].value.a = PTA_REMOTEPROC_ELF_FMT;

	/*
	 * Due to stm32mp1 pager, secure memory is too expensive. Support hash
	 * protected image only, so that firmware image can be loaded from
	 * non-secure memory.
	 */
	params[2].value.a = PTA_REMOTEPROC_FW_WITH_HASH_TABLE;

	return TEE_SUCCESS;
}

static TEE_Result da_to_pa(paddr_t da, size_t size, paddr_t *pa)
{
	const struct rproc_ta_memory_region *mems = rproc_ta_mp1_m4_mems;
	size_t i = 0;

	DMSG("da addr: %#"PRIxPA" size: %zu", da, size);

	for (i = 0; i < ARRAY_SIZE(rproc_ta_mp1_m4_mems); i++) {
		if (da >= mems[i].da &&
		    (da + size) <= (mems[i].da + mems[i].size)) {
			*pa = da - mems[i].da + mems[i].pa;
			DMSG("da %#"PRIxPA" to pa %#"PRIxPA, da, *pa);

			return TEE_SUCCESS;
		}
	}

	return TEE_ERROR_ACCESS_DENIED;
}

static TEE_Result rproc_pta_load_segment(uint32_t pt,
					 TEE_Param params[TEE_NUM_PARAMS])
{
	const uint32_t exp_pt = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
						TEE_PARAM_TYPE_MEMREF_INPUT,
						TEE_PARAM_TYPE_VALUE_INPUT,
						TEE_PARAM_TYPE_MEMREF_INPUT);
	TEE_Result res = TEE_ERROR_GENERIC;
	paddr_t pa = 0;
	vaddr_t dst = 0;
	uint8_t *src = params[1].memref.buffer;
	size_t size = params[1].memref.size;
	paddr_t da = (paddr_t)reg_pair_to_64(params[2].value.b,
					     params[2].value.a);

	if (pt != exp_pt)
		return TEE_ERROR_BAD_PARAMETERS;

	/* Only STM32_M4_FW_ID supported */
	if (params[0].value.a != STM32_M4_FW_ID) {
		EMSG("Unsupported firmware ID %#"PRIx32, params[0].value.a);
		return TEE_ERROR_NOT_SUPPORTED;
	}

	if (rproc_ta_state != REMOTEPROC_OFF)
		return TEE_ERROR_BAD_STATE;

	/* Get the physical address in A7 mapping */
	res = da_to_pa(da, size, &pa);
	if (res)
		return res;

	/* Get the associated va */
	dst = core_mmu_get_va(pa, MEM_AREA_IO_SEC, 1);

	if (params[3].memref.buffer)
		IMSG("Hash verification not yet supported");

	/* Copy the segment to the remote processor memory*/
	memcpy((void *)dst, src, size);

	/* TODO: computed hash and compare it with the expected hash*/

	return TEE_SUCCESS;
}

static TEE_Result rproc_pta_set_memory(uint32_t pt,
				       TEE_Param params[TEE_NUM_PARAMS])
{
	const uint32_t exp_pt = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
						TEE_PARAM_TYPE_VALUE_INPUT,
						TEE_PARAM_TYPE_VALUE_INPUT,
						TEE_PARAM_TYPE_VALUE_INPUT);
	TEE_Result res = TEE_ERROR_GENERIC;
	paddr_t pa = 0;
	vaddr_t dst = 0;
	paddr_t da = params[1].value.a;
	size_t size = params[2].value.a;
	char value = (char)params[3].value.a;

	if (pt != exp_pt)
		return TEE_ERROR_BAD_PARAMETERS;

	/* Only STM32_M4_FW_ID supported */
	if (params[0].value.a != STM32_M4_FW_ID) {
		EMSG("Unsupported firmware ID %#"PRIx32, params[0].value.a);
		return TEE_ERROR_NOT_SUPPORTED;
	}

	if (rproc_ta_state != REMOTEPROC_OFF)
		return TEE_ERROR_BAD_STATE;

	/* Get the physical address in CPU mapping */
	res = da_to_pa(da, size, &pa);
	if (res)
		return res;

	dst = core_mmu_get_va(pa, MEM_AREA_IO_SEC, 1);

	memset((void *)dst, value, size);

	return TEE_SUCCESS;
}

static TEE_Result rproc_pta_da_to_pa(uint32_t pt,
				     TEE_Param params[TEE_NUM_PARAMS])
{
	const uint32_t exp_pt = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
						TEE_PARAM_TYPE_VALUE_INPUT,
						TEE_PARAM_TYPE_VALUE_INPUT,
						TEE_PARAM_TYPE_VALUE_OUTPUT);
	TEE_Result res = TEE_ERROR_GENERIC;
	paddr_t da = params[1].value.a;
	size_t size = params[2].value.a;
	paddr_t pa = 0;

	DMSG("Conversion for address %#"PRIxPA" size %zu", da, size);

	if (pt != exp_pt)
		return TEE_ERROR_BAD_PARAMETERS;

	/* Only STM32_M4_FW_ID supported */
	if (params[0].value.a != STM32_M4_FW_ID) {
		EMSG("Unsupported firmware ID %#"PRIx32, params[0].value.a);
		return TEE_ERROR_NOT_SUPPORTED;
	}

	/* Target address is expected 32bit, ensure 32bit MSB are zero */
	if (params[1].value.b || params[2].value.b)
		return TEE_ERROR_BAD_PARAMETERS;

	res = da_to_pa(da, size, &pa);
	if (res)
		return res;

	reg_pair_from_64((uint64_t)pa, &params[3].value.b, &params[3].value.a);

	return TEE_SUCCESS;
}

static TEE_Result rproc_pta_start(uint32_t pt,
				  TEE_Param params[TEE_NUM_PARAMS])
{
	const uint32_t exp_pt = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
						TEE_PARAM_TYPE_NONE,
						TEE_PARAM_TYPE_NONE,
						TEE_PARAM_TYPE_NONE);
	vaddr_t rcc_base = stm32_rcc_base();
	struct clk *mcu_clk = stm32mp_rcc_clock_id_to_clk(CK_MCU);

	if (pt != exp_pt)
		return TEE_ERROR_BAD_PARAMETERS;

	/* Only STM32_M4_FW_ID supported */
	if (params[0].value.a != STM32_M4_FW_ID) {
		EMSG("Unsupported firmware ID %#"PRIx32, params[0].value.a);
		return TEE_ERROR_NOT_SUPPORTED;
	}

	if (rproc_ta_state != REMOTEPROC_OFF)
		return TEE_ERROR_BAD_STATE;

	clk_enable(mcu_clk);

	/*
	 * The firmware is started by deasserting the hold boot and
	 * asserting back to avoid auto restart on a crash.
	 * No need to release the MCU reset as it is automatically released by
	 * the hardware.
	 */
	io_setbits32(rcc_base + RCC_MP_GCR, RCC_MP_GCR_BOOT_MCU);
	io_clrbits32(rcc_base + RCC_MP_GCR, RCC_MP_GCR_BOOT_MCU);

	rproc_ta_state = REMOTEPROC_ON;

	return TEE_SUCCESS;
}

static TEE_Result rproc_pta_stop(uint32_t pt,
				 TEE_Param params[TEE_NUM_PARAMS])
{
	const uint32_t exp_pt = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
						TEE_PARAM_TYPE_NONE,
						TEE_PARAM_TYPE_NONE,
						TEE_PARAM_TYPE_NONE);
	struct clk *mcu_clk = stm32mp_rcc_clock_id_to_clk(CK_MCU);
	vaddr_t rcc_base = stm32_rcc_base();

	if (pt != exp_pt)
		return TEE_ERROR_BAD_PARAMETERS;

	/* Only STM32_M4_FW_ID supported */
	if (params[0].value.a != STM32_M4_FW_ID) {
		EMSG("Unsupported firmware ID %#"PRIx32, params[0].value.a);
		return TEE_ERROR_NOT_SUPPORTED;
	}

	if (rproc_ta_state != REMOTEPROC_ON)
		return TEE_ERROR_BAD_STATE;

	/* The firmware is stopped (reset with holdboot is active) */
	io_clrbits32(rcc_base + RCC_MP_GCR, RCC_MP_GCR_BOOT_MCU);

	stm32_reset_set(MCU_R);

	clk_disable(mcu_clk);

	rproc_ta_state = REMOTEPROC_OFF;

	return TEE_SUCCESS;
}

static TEE_Result rproc_pta_invoke_command(void *pSessionContext __unused,
					   uint32_t cmd_id,
					   uint32_t param_types,
					   TEE_Param params[TEE_NUM_PARAMS])
{
	switch (cmd_id) {
	case PTA_REMOTEPROC_HW_CAPABILITIES:
		return rproc_pta_capabilities(param_types, params);
	case PTA_REMOTEPROC_LOAD_SEGMENT_SHA256:
		return rproc_pta_load_segment(param_types, params);
	case PTA_REMOTEPROC_SET_MEMORY:
		return rproc_pta_set_memory(param_types, params);
	case PTA_REMOTEPROC_FIRMWARE_START:
		return rproc_pta_start(param_types, params);
	case PTA_REMOTEPROC_FIRMWARE_STOP:
		return rproc_pta_stop(param_types, params);
	case PTA_REMOTEPROC_FIRMWARE_DA_TO_PA:
		return rproc_pta_da_to_pa(param_types, params);
	default:
		break;
	}

	return TEE_ERROR_NOT_IMPLEMENTED;
}

/*
 * Trusted Application entry points
 */
static TEE_Result
	rproc_pta_open_session(uint32_t param_types __unused,
			       TEE_Param params[TEE_NUM_PARAMS] __unused,
			       void **sess_ctx __unused)
{
	struct ts_session *s = ts_get_calling_session();

	/* TODO: check that we're called the remove proc TA (check UUID) */
	if (!s || !is_user_ta_ctx(s->ctx))
		return TEE_ERROR_ACCESS_DENIED;

	return TEE_SUCCESS;
}

pseudo_ta_register(.uuid = PTA_REMOTEPROC_UUID, .name = PTA_NAME,
		   .flags = PTA_DEFAULT_FLAGS,
		   .invoke_command_entry_point = rproc_pta_invoke_command,
		   .open_session_entry_point = rproc_pta_open_session);
