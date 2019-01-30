/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2017-2018, STMicroelectronics
 */

#ifndef __STM32_UART_H__
#define __STM32_UART_H__

#include <drivers/serial.h>
#include <drivers/stm32_gpio.h>

struct stm32_uart_pdata {
	struct io_pa_va base;
	struct serial_chip chip;
	bool secure;
	struct stm32_pinctrl *pinctrl;
	size_t pinctrl_count;
	unsigned long clock;
};

void stm32_uart_init(struct stm32_uart_pdata *pd, vaddr_t base);
struct stm32_uart_pdata *probe_uart_from_dt_node(void *fdt, int node);

#endif /*__STM32_UART_H__*/
