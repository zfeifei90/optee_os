/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2017-2018, STMicroelectronics - All Rights Reserved
 */

#ifndef __STM32MP1_CLKFUNC_H__
#define __STM32MP1_CLKFUNC_H__

#include <libfdt.h>
#include <stdbool.h>

extern const char *stm32mp_osc_node_label[NB_OSC];

int fdt_osc_read_freq(void *fdt, const char *name, uint32_t *freq);
bool fdt_osc_read_bool(void *fdt, enum stm32mp_osc_id osc_id,
			const char *prop_name);
uint32_t fdt_osc_read_uint32_default(void *fdt, enum stm32mp_osc_id osc_id,
				     const char *prop_name,
				     uint32_t dflt_value);

int fdt_get_rcc_node(void *fdt);
uint32_t fdt_rcc_read_addr(void *fdt);
int fdt_rcc_read_uint32_array(void *fdt, const char *prop_name,
			      uint32_t *array, uint32_t count);
int fdt_rcc_subnode_offset(void *fdt, const char *name);
const fdt32_t *fdt_rcc_read_prop(void *fdt, const char *prop_name, int *lenp);
bool fdt_get_rcc_secure_status(void *fdt);

uintptr_t get_stgen_base(void);

unsigned long get_uart_clock_freq(uint32_t instance);

#endif /* __STM32MP1_CLKFUNC_H__ */
