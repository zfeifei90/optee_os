// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2017-2019, STMicroelectronics
 */

#include <arm.h>
#include <assert.h>
#include <drivers/stm32_bsec.h>
#include <kernel/thread.h>
#include <sm/optee_smc.h>
#include <sm/sm.h>
#include <stm32_util.h>
#include <stdint.h>
#include <string.h>
#include <tee_api_types.h>
#include <trace.h>

#include "bsec_svc.h"
#include "low_power_svc.h"
#include "pwr_svc.h"
#include "rcc_svc.h"
#include "stm32mp1_smc.h"

static uint32_t __maybe_unused calib_scv_handler(uint32_t x1)
{
	unsigned long clock_id = x1;

	return (stm32mp_start_clock_calib(clock_id) == 0) ?
		STM32_SIP_OK : STM32_SIP_FAILED;
}

/* STM32 SiP Service UUID */
static const TEE_UUID stm32mp1_sip_svc_uid = {
	0x50aa78a7, 0x9bf4, 0x4a14,
	{ 0x8a, 0x5e, 0x26, 0x4d, 0x59, 0x94, 0xc2, 0x14 }
};

static void get_sip_func_uid(uint32_t *a0, uint32_t *a1,
			     uint32_t *a2, uint32_t *a3)
{
	const void *uid = &stm32mp1_sip_svc_uid;

	memcpy(a0, (char *)uid, sizeof(uint32_t));
	memcpy(a1, (char *)uid + sizeof(uint32_t), sizeof(uint32_t));
	memcpy(a2, (char *)uid + (sizeof(uint32_t) * 2), sizeof(uint32_t));
	memcpy(a3, (char *)uid + (sizeof(uint32_t) * 3), sizeof(uint32_t));
}

bool stm32_sip_service(struct sm_ctx __unused *ctx,
		       uint32_t *a0, uint32_t *a1, uint32_t *a2, uint32_t *a3)
{
	switch (OPTEE_SMC_FUNC_NUM(*a0)) {
	case STM32_SIP_FUNC_CALL_COUNT:
		/* This service is meaningless, return a dummy value */
		*a0 = 0;
		break;
	case STM32_SIP_FUNC_VERSION:
		*a0 = STM32_SIP_SVC_VERSION_MAJOR;
		*a1 = STM32_SIP_SVC_VERSION_MINOR;
		break;
	case STM32_SIP_FUNC_UID:
		get_sip_func_uid(a0, a1, a2, a3);
		break;
#ifdef CFG_STM32_BSEC_SIP
	case STM32_SIP_FUNC_BSEC:
		*a0 = bsec_main(*a1, *a2, *a3, a1);
		break;
#endif
#ifdef CFG_STM32_RCC_SIP
	case STM32_SIP_FUNC_RCC:
		*a0 = rcc_scv_handler(*a1, *a2, *a3);
		break;
	case STM32_SIP_FUNC_RCC_OPP:
		*a0 = rcc_opp_scv_handler(*a1, *a2, a1);
		break;
#endif
#ifdef CFG_STM32_CLOCKSRC_CALIB
	case STM32_SIP_RCC_CAL:
		*a0 = calib_scv_handler(*a1);
		break;
#endif
#ifdef CFG_STM32_PWR_SIP
	case STM32_SIP_FUNC_PWR:
		*a0 = pwr_scv_handler(*a1, *a2, *a3);
		break;
#endif
#ifdef CFG_STM32_POWER_SERVICES
	case STM32_SIP_FUNC_SR_MODE:
		*a0 = sr_mode_scv_handler(*a1, *a2);
		break;

	case STM32_SIP_FUNC_CSTOP:
		*a0 = cstop_scv_handler(ctx, *a1, *a2, *a3);
		break;

	case STM32_SIP_FUNC_STANDBY:
		*a0 = standby_scv_handler(ctx, *a1, *a2, *a3);
		break;

	case STM32_SIP_FUNC_SHUTDOWN:
		*a0 = shutdown_scv_handler();
		break;

	case STM32_SIP_FUNC_PD_DOMAIN:
		*a0 = pm_domain_scv_handler(*a1, *a2);
		break;
#endif

	default:
		return true;
	}

	return false;
}

bool stm32_oem_service(struct sm_ctx __unused *ctx,
		       uint32_t *a0, uint32_t *a1 __maybe_unused,
		       uint32_t *a2 __maybe_unused, uint32_t *a3 __unused)
{
	switch (OPTEE_SMC_FUNC_NUM(*a0)) {
#ifdef CFG_STM32_POWER_SERVICES
	case STM32_OEM_FUNC_LP_FORCE_PARAMS:
		*a0 = pm_set_lp_state_scv_handler(*a1, *a2);
		break;
#endif

	default:
		return true;
	}

	return false;
}
