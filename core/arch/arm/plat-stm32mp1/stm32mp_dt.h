/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2018-2019, STMicroelectronics - All Rights Reserved
 * Copyright (c) 2017-2018, ARM Limited and Contributors. All rights reserved.
 */

#ifndef __STM32MP1_DT_H__
#define __STM32MP1_DT_H__

#include <stdbool.h>
#include <stdint.h>

#define DT_DDR_COMPAT		"st,stm32mp1-ddr"
#define DT_OPP_COMPAT		"operating-points-v2"

struct dt_node_info {
	uint32_t base;
	int32_t clock;
	int32_t reset;
	unsigned int status;
};

bool fdt_check_node(void *fdt, int node);
uint32_t fdt_read_uint32_default(void *fdt, int node, const char *prop_name,
				 uint32_t dflt_value);
int fdt_read_uint32_array(void *fdt, int node, const char *prop_name,
			  uint32_t *array, uint32_t count);
void fdt_fill_device_info(void *fdt, struct dt_node_info *info, int node);
int fdt_get_node(void *fdt, struct dt_node_info *info, int offset,
		 const char *compat);
int fdt_get_stdout_node_offset(void *fdt);
int fdt_get_node_by_compatible(void *fdt, const char *compatible);
int fdt_match_instance_by_compatible(void *fdt, const char *compatible,
				    uintptr_t address);
uintptr_t fdt_get_peripheral_base(void *fdt, const char *compatible);
uint32_t fdt_get_ddr_size(void *fdt);
int fdt_get_all_opp_freqvolt(void *fdt, uint32_t *count,
			     uint32_t *freq_khz_array,
			     uint32_t *voltage_mv_array);
const char *fdt_get_board_model(void *fdt);

int fdt_get_clock_id(void *fdt, int node);
int fdt_get_clock_id_by_name(void *fdt, int node, const char *name);
int fdt_get_gpio_bank_pinctrl_node(void *fdt, unsigned int bank);
int fdt_get_gpioz_nbpins_from_dt(void *fdt);

#endif /* __STM32MP1_DT_H__ */
