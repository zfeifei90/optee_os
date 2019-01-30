/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2017-2018, STMicroelectronics
 */
#ifndef __STM32MP_PM_H__
#define __STM32MP_PM_H__

#ifndef ASM

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PSCI_MODE_SYSTEM_SUSPEND	0
#define PSCI_MODE_SYSTEM_OFF		1

enum stm32mp1_pm_domain {
	STM32MP1_PD_VSW,
	STM32MP1_PD_CORE_RET,
	STM32MP1_PD_CORE,
	STM32MP1_PD_MAX_PM_DOMAIN
};

enum pm_op {
	PM_OP_SUSPEND,
	PM_OP_RESUME,
};

/*
 * Drivers can register a callback for the suspend and resume sequences and
 * a private cookie passed as argument to the callback.
 * The same callback is used for suspend and resume. First argument of the
 * callback defines whether the device shall suspend or resume.
 *
 * Driver should tag its callback resource as unpaged (i.e KEEP_PAGER())
 * since the callback is called from an unpaged execution context.
 */
void stm32mp_register_pm_cb(void (*callback)(enum pm_op op, void *handle),
			    void *handle);

void stm32_cores_reset(void);

void stm32mp_gic_suspend_resume(enum pm_op op);
void stm32mp_clock_suspend_resume(enum pm_op op);

#endif /*ASM*/

#endif /*__STM32MP_PM_H__*/

