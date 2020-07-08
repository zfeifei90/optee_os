// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2017-2019, STMicroelectronics
 */

#include <drivers/stm32mp1_rcc.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <inttypes.h>
#include <kernel/panic.h>
#include <io.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <stm32_util.h>
#include <trace.h>

#include "rcc_svc.h"
#include "stm32mp1_smc.h"

#undef FULL_RCC_ACCESS

static bool offset_is_clear_register(uint32_t __maybe_unused offset)
{
#ifdef FULL_RCC_ACCESS
	switch (offset) {
	case RCC_OCENCLRR:
	case RCC_MP_SREQCLRR:
	case RCC_APB5RSTCLRR:
	case RCC_AHB5RSTCLRR:
	case RCC_MP_APB5ENCLRR:
	case RCC_MP_AHB5ENCLRR:
	case RCC_MP_APB5LPENCLRR:
	case RCC_MP_AHB5LPENCLRR:
	case RCC_MP_IWDGFZCLRR:
		return true;
	default:
		return false;
	}
#else
	/* All allowed registers are non set/clear registers  */
	return false;
#endif
}

static void access_allowed_mask(uint32_t request, uint32_t offset,
				uint32_t value, uint32_t allowed_mask)
{
	vaddr_t va = stm32_rcc_base() + offset;

	if (!allowed_mask)
		return;

	switch (request) {
	case STM32_SIP_SVC_REG_WRITE:
		if (offset_is_clear_register(offset)) {
			/* CLR registers show SET state, not CLR state */
			io_write32(va, value & allowed_mask);
		} else {
			io_mask32_stm32shregs(va, value, allowed_mask);
		}
		FMSG("wrt 0x%" PRIx32 "=0x%" PRIx32 " => 0x%" PRIx32,
			offset, value, io_read32(va));
		break;

	case STM32_SIP_SVC_REG_SET:
		if (offset_is_clear_register(offset)) {
			/* CLR registers show SET state, not CLR state */
			io_write32(va, value & allowed_mask);
		} else {
			io_setbits32_stm32shregs(va, value & allowed_mask);
		}
		FMSG("set 0x%" PRIx32 "=0x%" PRIx32 " => 0x%" PRIx32,
			offset, value, io_read32(va));
		break;

	case STM32_SIP_SVC_REG_CLEAR:
		/* Nothing to do on CLR registers */
		if (!offset_is_clear_register(offset))
			io_clrbits32_stm32shregs(va, value & allowed_mask);
		FMSG("clear 0x%" PRIx32 "=0x%" PRIx32 " => 0x%" PRIx32,
			offset, value, io_read32(va));
		break;

	default:
		break;
	}
}

static void raw_allowed_access_request(uint32_t request,
				       uint32_t offset, uint32_t value)
{
	uint32_t allowed_mask = 0;

	switch (offset) {
	case RCC_MP_CIER:
	case RCC_MP_CIFR:
		allowed_mask = RCC_MP_CIFR_WKUPF;
		break;

#ifdef FULL_RCC_ACCESS
	case RCC_OCENSETR:
	case RCC_OCENCLRR:
	case RCC_HSICFGR:
	case RCC_CSICFGR:
	case RCC_MP_BOOTCR:		/* Allowed MPU/MCU reboot cfg */
	case RCC_MP_GCR:		/* Allowed MPU/MCU reboot cfg */
	case RCC_MP_GRSTCSETR:		/* Allowed MCU and system reset */
	case RCC_BR_RSTSCLRR:		/* Allowed system reset status */
	case RCC_MC_RSTSCLRR:		/* Allowed system reset status */
	case RCC_MP_RSTSCLRR:		/* Allowed system reset status */
	case RCC_BDCR:
	case RCC_RDLSICR:
	case RCC_APB5RSTSETR:
	case RCC_APB5RSTCLRR:
	case RCC_MP_APB5ENSETR:
	case RCC_MP_APB5ENCLRR:
	case RCC_MP_APB5LPENSETR:
	case RCC_MP_APB5LPENCLRR:
	case RCC_AHB5RSTSETR:
	case RCC_AHB5RSTCLRR:
	case RCC_MP_AHB5ENSETR:
	case RCC_MP_AHB5ENCLRR:
	case RCC_MP_AHB5LPENSETR:
	case RCC_MP_AHB5LPENCLRR:
	case RCC_RTCDIVR:
	case RCC_I2C46CKSELR:
	case RCC_SPI6CKSELR:
	case RCC_UART1CKSELR:
	case RCC_RNG1CKSELR:
	case RCC_MP_IWDGFZSETR:
	case RCC_MP_IWDGFZCLRR:
		allowed_mask = UINT32_MAX;
		break;
#endif
	default:
		panic();
	}

	access_allowed_mask(request, offset, value, allowed_mask);
}

uint32_t rcc_scv_handler(uint32_t x1, uint32_t x2, uint32_t x3)
{
	uint32_t request = x1;
	uint32_t offset = x2;
	uint32_t value = x3;

	/*
	 * Argument x2 can be either the register physical address of the
	 * register offset toward RCC_BASE.
	 */
	if (offset & ~RCC_OFFSET_MASK) {
		if ((offset & ~RCC_OFFSET_MASK) != RCC_BASE)
			return STM32_SIP_SVC_INVALID_PARAMS;

		offset &= RCC_OFFSET_MASK;
	}

	DMSG_RAW("RCC service: %s 0x%" PRIx32 " at offset 0x%" PRIx32,
		 request == STM32_SIP_SVC_REG_WRITE ? "write" :
		 request == STM32_SIP_SVC_REG_SET ? "set" : "clear",
		 value, offset);

	raw_allowed_access_request(request, offset, value);

	return STM32_SIP_SVC_OK;
}

uint32_t rcc_opp_scv_handler(uint32_t x1, uint32_t x2, uint32_t *res)
{
	uint32_t cmd = x1;
	uint32_t opp = x2 / 1000U; /* KHz */

	switch (cmd) {
	case STM32_SIP_SVC_RCC_OPP_SET:
		if (stm32mp1_set_opp_khz(opp))
			return STM32_SIP_SVC_FAILED;
		break;

	case STM32_SIP_SVC_RCC_OPP_ROUND:
		if(stm32mp1_round_opp_khz(&opp))
			return STM32_SIP_SVC_FAILED;

		if (MUL_OVERFLOW(opp, 1000, res))
			return STM32_SIP_SVC_FAILED;
		break;

	default:
		return STM32_SIP_SVC_INVALID_PARAMS;
	}

	return STM32_SIP_SVC_OK;
}
