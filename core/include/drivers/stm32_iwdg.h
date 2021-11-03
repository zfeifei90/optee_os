/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2018-2021, STMicroelectronics - All Rights Reserved
 */

#ifndef __STM32_IWDG_H__
#define __STM32_IWDG_H__

#include <drivers/clk.h>
#include <io.h>
#include <mm/core_memprot.h>
#include <stdint.h>

/* Values for struct stm32_iwdg_platdata::flags */
#define IWDG_HW_ENABLED			BIT(0)
#define IWDG_DISABLE_ON_STOP		BIT(1)
#define IWDG_DISABLE_ON_STANDBY		BIT(2)

struct stm32_iwdg_platdata {
	struct io_pa_va base;
	struct clk *clock;
	struct clk *clk_lsi;
	int irq;
	uint8_t flags;
	int timeout;
	int sec_timeout;
};

__weak int stm32_iwdg_get_platdata(struct stm32_iwdg_platdata *pdata);

void stm32_iwdg_refresh(void);

#endif /*__STM32_IWDG_H__*/
