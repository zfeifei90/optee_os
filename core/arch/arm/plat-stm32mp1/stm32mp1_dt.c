// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2018, STMicroelectronics - All Rights Reserved
 * Copyright (c) 2017-2018, ARM Limited and Contributors. All rights reserved.
 */

#include <assert.h>
#include <drivers/stm32mp1_clk.h>
#include <drivers/stm32mp1_clkfunc.h>
#include <drivers/stm32_gpio.h>
#include <kernel/dt.h>
#include <kernel/generic_boot.h>
#include <kernel/panic.h>
#include <libfdt.h>
#include <platform_config.h>
#include <stm32_util.h>
#include <stm32mp_dt.h>
#include <trace.h>

/*******************************************************************************
 * This function check the presence of a node (generic use of fdt library).
 * Returns true if present else returns false.
 ******************************************************************************/
bool fdt_check_node(void *fdt, int node)
{
	int len;
	const char *cchar;

	cchar = fdt_get_name(fdt, node, &len);

	return (cchar != NULL) && (len >= 0);
}

/*******************************************************************************
 * This function reads a value of a node property (generic use of fdt
 * library).
 * Returns value if success, and a default value if property not found.
 * Default value is passed as parameter.
 ******************************************************************************/
uint32_t fdt_read_uint32_default(void *fdt, int node, const char *prop_name,
				 uint32_t dflt_value)
{
	const fdt32_t *cuint;
	int lenp;

	cuint = fdt_getprop(fdt, node, prop_name, &lenp);
	if (cuint == NULL) {
		return dflt_value;
	}

	return fdt32_to_cpu(*cuint);
}

/*******************************************************************************
 * This function reads a series of parameters in a node property
 * (generic use of fdt library).
 * It reads the values inside the device tree, from property name and node.
 * The number of parameters is also indicated as entry parameter.
 * Returns 0 if success, and a negative value else.
 * If success, values are stored at the third parameter address.
 ******************************************************************************/
int fdt_read_uint32_array(void *fdt, int node, const char *prop_name,
			  uint32_t *array, uint32_t count)
{
	const fdt32_t *cuint;
	int len;
	uint32_t i;

	cuint = fdt_getprop(fdt, node, prop_name, &len);
	if (cuint == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	if ((uint32_t)len != (count * sizeof(uint32_t))) {
		return -FDT_ERR_BADLAYOUT;
	}

	for (i = 0; i < ((uint32_t)len / sizeof(uint32_t)); i++) {
		*array = fdt32_to_cpu(*cuint);
		array++;
		cuint++;
	}

	return 0;
}

/*******************************************************************************
 * This function gets the clock ID of the given node.
 * It reads the value indicated inside the device tree.
 * Returns ID on success and a negative FDT error code on failure.
 ******************************************************************************/
int fdt_get_clock_id(void *fdt, int node)
{
	const fdt32_t *cuint;

	cuint = fdt_getprop(fdt, node, "clocks", NULL);
	if (cuint == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	cuint++;
	return (int)fdt32_to_cpu(*cuint);
}

/*******************************************************************************
 * This function gets the clock ID of the given node using clock-names.
 * It reads the value indicated inside the device tree.
 * Returns ID on success and a negative FDT error code on failure.
 ******************************************************************************/
int fdt_get_clock_id_by_name(void *fdt, int node, const char *name)
{
	const fdt32_t *cuint;
	int index;
	int len;

	index = fdt_stringlist_search(fdt, node, "clock-names", name);
	if (index < 0) {
		return index;
	}

	cuint = fdt_getprop(fdt, node, "clocks", &len);
	if (cuint == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	if ((index * (int)sizeof(uint32_t)) > len) {
		return -FDT_ERR_BADVALUE;
	}

	cuint += (index << 1) + 1;
	return (int)fdt32_to_cpu(*cuint);
}

/*******************************************************************************
 * This function fills the generic information from a given node.
 ******************************************************************************/
void fdt_fill_device_info(void *fdt, struct dt_node_info *info, int node)
{
	const fdt32_t *cuint;

	cuint = fdt_getprop(fdt, node, "reg", NULL);
	if (cuint != NULL) {
		info->base = fdt32_to_cpu(*cuint);
	} else {
		info->base = 0;
	}

	cuint = fdt_getprop(fdt, node, "clocks", NULL);
	if (cuint != NULL) {
		cuint++;
		info->clock = (int)fdt32_to_cpu(*cuint);
	} else {
		info->clock = -1;
	}

	cuint = fdt_getprop(fdt, node, "resets", NULL);
	if (cuint != NULL) {
		cuint++;
		info->reset = (int)fdt32_to_cpu(*cuint);
	} else {
		info->reset = -1;
	}

	info->status = _fdt_get_status(fdt, node);
}

/*******************************************************************************
 * This function retrieve the generic information from DT.
 * Returns node if success, and a negative value else.
 ******************************************************************************/
int fdt_get_node(void *fdt, struct dt_node_info *info, int offset,
		const char *compat)
{
	int node;

	node = fdt_node_offset_by_compatible(fdt, offset, compat);
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	fdt_fill_device_info(fdt, info, node);

	return node;
}

/*******************************************************************************
 * This function gets the stdout path node.
 * It reads the value indicated inside the device tree.
 * Returns node on success and a negative FDT error code on failure.
 ******************************************************************************/
int fdt_get_stdout_node_offset(void *fdt)
{
	int node;
	const char *cchar;

	node = fdt_path_offset(fdt, "/secure-chosen");
	if (node < 0) {
		node = fdt_path_offset(fdt, "/chosen");
		if (node < 0) {
			return -FDT_ERR_NOTFOUND;
		}
	}

	cchar = fdt_getprop(fdt, node, "stdout-path", NULL);
	if (cchar == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	node = -FDT_ERR_NOTFOUND;
	if (strchr(cchar, (int)':') != NULL) {
		const char *name;
		char *str = (char *)cchar;
		int len = 0;

		while (strncmp(":", str, 1)) {
			len++;
			str++;
		}

		name = fdt_get_alias_namelen(fdt, cchar, len);

		if (name != NULL) {
			node = fdt_path_offset(fdt, name);
		}
	} else {
		node = fdt_path_offset(fdt, cchar);
	}

	return node;
}

/*******************************************************************************
 * This function gets DDR size information from the DT.
 * Returns value in bytes if success, and STM32MP1_DDR_SIZE_DFLT else.
 ******************************************************************************/
uint32_t fdt_get_ddr_size(void *fdt)
{
	int node;

	node = fdt_node_offset_by_compatible(fdt, -1, DT_DDR_COMPAT);
	if (node < 0) {
		IMSG("%s: Cannot read DDR node in DT\n", __func__);
		return STM32MP1_DDR_SIZE_DFLT;
	}

	return fdt_read_uint32_default(fdt, node, "st,mem-size",
				       STM32MP1_DDR_SIZE_DFLT);
}

/*******************************************************************************
 * This function retrieves board model from DT.
 * Returns string taken from model node, NULL otherwise
 ******************************************************************************/
const char *fdt_get_board_model(void *fdt)
{
	int node = fdt_path_offset(fdt, "/");

	if (node < 0) {
		return NULL;
	}

	return (const char *)fdt_getprop(fdt, node, "model", NULL);
}

/*******************************************************************************
 * This function gets GPIO bank PINCTRL node information from the DT.
 * Returns node value.
 ******************************************************************************/
int fdt_get_gpio_bank_pinctrl_node(void *fdt, unsigned int bank)
{
	switch (bank) {
	case GPIO_BANK_A:
	case GPIO_BANK_B:
	case GPIO_BANK_C:
	case GPIO_BANK_D:
	case GPIO_BANK_E:
	case GPIO_BANK_F:
	case GPIO_BANK_G:
	case GPIO_BANK_H:
	case GPIO_BANK_I:
	case GPIO_BANK_J:
	case GPIO_BANK_K:
		return fdt_path_offset(fdt, "/soc/pin-controller");
	case GPIO_BANK_Z:
		return fdt_path_offset(fdt, "/soc/pin-controller-z");
	default:
		panic();
	}
}

/*******************************************************************************
 * This function gets GPIOZ pin number information from the DT.
 * It also checks node consistency.
 ******************************************************************************/
int fdt_get_gpioz_nbpins_from_dt(void *fdt)
{
	int pinctrl_node;
	int pinctrl_subnode;

	pinctrl_node = fdt_get_gpio_bank_pinctrl_node(fdt, GPIO_BANK_Z);
	if (pinctrl_node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	fdt_for_each_subnode(pinctrl_subnode, fdt, pinctrl_node) {
		uint32_t bank_offset;
		const fdt32_t *cuint;

		if (fdt_getprop(fdt, pinctrl_subnode,
				"gpio-controller", NULL) == NULL) {
			continue;
		}

		cuint = fdt_getprop(fdt, pinctrl_subnode, "reg", NULL);
		if (cuint == NULL) {
			continue;
		}

		bank_offset = stm32_get_gpio_bank_offset(GPIO_BANK_Z);
		if (fdt32_to_cpu(*cuint) != bank_offset) {
			continue;
		}

		if (_fdt_get_status(fdt, pinctrl_subnode) ==
		     DT_STATUS_DISABLED) {
			return 0;
		}

		cuint = fdt_getprop(fdt, pinctrl_subnode, "ngpios", NULL);
		if (cuint == NULL) {
			return -FDT_ERR_NOTFOUND;
		}

		return (int)fdt32_to_cpu(*cuint);
	}

	return 0;
}
