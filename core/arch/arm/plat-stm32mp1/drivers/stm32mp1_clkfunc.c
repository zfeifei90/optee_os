// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 */

#include <dt-bindings/clock/stm32mp1-clksrc.h>
#include <kernel/dt.h>
#include <kernel/generic_boot.h>
#include <libfdt.h>
#include <stm32_util.h>
#include <drivers/stm32mp1_clk.h>
#include <drivers/stm32mp1_clkfunc.h>
#include <stm32mp_dt.h>

#define DT_RCC_NODE_NAME	"rcc@50000000"
#define DT_RCC_CLK_COMPAT	"st,stm32mp1-rcc"
#define DT_RCC_COMPAT		"syscon"
#define DT_STGEN_COMPAT		"st,stm32-stgen"
#define DT_UART_COMPAT		"st,stm32h7-uart"

const char *stm32mp_osc_node_label[NB_OSC] = {
	[_LSI] = "clk-lsi",
	[_LSE] = "clk-lse",
	[_HSI] = "clk-hsi",
	[_HSE] = "clk-hse",
	[_CSI] = "clk-csi",
	[_I2S_CKIN] = "i2s_ckin",
	[_USB_PHY_48] = "ck_usbo_48m"
};

/*******************************************************************************
 * This function reads the frequency of an oscillator from its name.
 * It reads the value indicated inside the device tree.
 * Returns 0 on success, and a negative FDT/ERRNO error code on failure.
 * On success, value is stored in the second parameter.
 ******************************************************************************/
int fdt_osc_read_freq(void *fdt, const char *name, uint32_t *freq)
{
	int node, subnode;

	node = fdt_path_offset(fdt, "/clocks");
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	fdt_for_each_subnode(subnode, fdt, node) {
		const char *cchar;
		int ret;

		cchar = fdt_get_name(fdt, subnode, &ret);
		if (cchar == NULL) {
			return ret;
		}

		if (strncmp(cchar, name, (size_t)ret) == 0) {
			const fdt32_t *cuint;

			if (_fdt_get_status(fdt, subnode) ==
			    DT_STATUS_DISABLED) {
				goto exit;
			}

			cuint = fdt_getprop(fdt, subnode, "clock-frequency",
					    &ret);
			if (cuint == NULL) {
				return ret;
			}

			*freq = fdt32_to_cpu(*cuint);

			return 0;
		}
	}

exit:
	/* Oscillator not found or disabled, freq=0 */
	*freq = 0;
	return 0;
}

/*******************************************************************************
 * This function checks the presence of an oscillator property from its id.
 * The search is done inside the device tree.
 * Returns true/false regarding search result.
 ******************************************************************************/
bool fdt_osc_read_bool(void *fdt, enum stm32mp_osc_id osc_id,
			const char *prop_name)
{
	int node, subnode;

	if (osc_id >= NB_OSC) {
		return false;
	}

	node = fdt_path_offset(fdt, "/clocks");
	if (node < 0) {
		return false;
	}

	fdt_for_each_subnode(subnode, fdt, node) {
		const char *cchar;
		int ret;

		cchar = fdt_get_name(fdt, subnode, &ret);
		if (cchar == NULL) {
			return false;
		}

		if (strncmp(cchar, stm32mp_osc_node_label[osc_id],
			    (size_t)ret) != 0) {
			continue;
		}

		if (fdt_getprop(fdt, subnode, prop_name, NULL) != NULL) {
			return true;
		}
	}

	return false;
}

/*******************************************************************************
 * This function reads a value of a oscillator property from its id.
 * Returns value on success, and a default value if property not found.
 * Default value is passed as parameter.
 ******************************************************************************/
uint32_t fdt_osc_read_uint32_default(void *fdt, enum stm32mp_osc_id osc_id,
				     const char *prop_name, uint32_t dflt_value)
{
	int node, subnode;

	if (osc_id >= NB_OSC) {
		return dflt_value;
	}

	node = fdt_path_offset(fdt, "/clocks");
	if (node < 0) {
		return dflt_value;
	}

	fdt_for_each_subnode(subnode, fdt, node) {
		const char *cchar;
		int ret;

		cchar = fdt_get_name(fdt, subnode, &ret);
		if (cchar == NULL) {
			return dflt_value;
		}

		if (strncmp(cchar, stm32mp_osc_node_label[osc_id],
			    (size_t)ret) != 0) {
			continue;
		}

		return fdt_read_uint32_default(fdt, subnode, prop_name,
					       dflt_value);
	}

	return dflt_value;
}

/*******************************************************************************
 * This function reads the rcc base address.
 * It reads the value indicated inside the device tree.
 * Returns address on success, and 0 on failure.
 ******************************************************************************/
uint32_t fdt_rcc_read_addr(void *fdt)
{
	int node, subnode;

	node = fdt_path_offset(fdt, "/soc");
	if (node < 0) {
		return 0;
	}

	fdt_for_each_subnode(subnode, fdt, node) {
		const char *cchar;
		int ret;

		cchar = fdt_get_name(fdt, subnode, &ret);
		if (cchar == NULL) {
			return 0;
		}

		if (strncmp(cchar, DT_RCC_NODE_NAME, (size_t)ret) == 0) {
			const fdt32_t *cuint;

			cuint = fdt_getprop(fdt, subnode, "reg", NULL);
			if (cuint == NULL) {
				return 0;
			}

			return fdt32_to_cpu(*cuint);
		}
	}

	return 0;
}

/*******************************************************************************
 * This function returns the RCC node in the device tree.
 ******************************************************************************/
int fdt_get_rcc_node(void *fdt)
{
	return fdt_get_node_by_compatible(fdt, DT_RCC_CLK_COMPAT);
}

/*******************************************************************************
 * This function reads a series of parameters in rcc-clk section.
 * It reads the values indicated inside the device tree, from property name.
 * The number of parameters is also indicated as entry parameter.
 * Returns 0 on success, and a negative FDT/ERRNO error code on failure.
 * On success, values are stored at the second parameter address.
 ******************************************************************************/
int fdt_rcc_read_uint32_array(void *fdt, const char *prop_name,
			      uint32_t *array, uint32_t count)
{
	int node = fdt_get_rcc_node(fdt);

	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	return fdt_read_uint32_array(fdt, node, prop_name, array, count);
}

/*******************************************************************************
 * This function gets the subnode offset in rcc-clk section from its name.
 * It reads the values indicated inside the device tree.
 * Returns offset on success, and a negative FDT/ERRNO error code on failure.
 ******************************************************************************/
int fdt_rcc_subnode_offset(void *fdt, const char *name)
{
	int node, subnode;

	node = fdt_get_rcc_node(fdt);
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	subnode = fdt_subnode_offset(fdt, node, name);
	if (subnode <= 0) {
		return -FDT_ERR_NOTFOUND;
	}

	return subnode;
}

/*******************************************************************************
 * This function gets the pointer to a rcc-clk property from its name.
 * It reads the values indicated inside the device tree.
 * Length of the property is stored in the second parameter.
 * Returns pointer on success, and NULL value on failure.
 ******************************************************************************/
const fdt32_t *fdt_rcc_read_prop(void *fdt, const char *prop_name, int *lenp)
{
	const fdt32_t *cuint;
	int node, len;

	node = fdt_get_rcc_node(fdt);
	if (node < 0) {
		return NULL;
	}

	cuint = fdt_getprop(fdt, node, prop_name, &len);
	if (cuint == NULL) {
		return NULL;
	}

	*lenp = len;
	return cuint;
}

/*******************************************************************************
 * This function reads the stgen base address.
 * It reads the value indicated inside the device tree.
 * Returns address on success, and NULL value on failure.
 ******************************************************************************/
uintptr_t get_stgen_base(void)
{
	void *fdt;

	fdt = get_dt_blob();
	if (fdt == NULL) {
		return 0;
	}

	return fdt_get_peripheral_base(fdt, DT_STGEN_COMPAT);
}

/*******************************************************************************
 * This function gets the frequency of the specified uart instance.
 * From this instance, all the uarts nodes in DT are parsed, and the register
 * base is compared to the instance. If match between these two values, then
 * the clock source is read from the DT and we deduce the frequency.
 * Returns clock frequency on success, 0 value on failure.
 ******************************************************************************/
unsigned long get_uart_clock_freq(uint32_t instance)
{
	int node;
	void *fdt;
	int clk_id;

	fdt = get_dt_blob();
	if (fdt == NULL) {
		return 0;
	}

	node = fdt_match_instance_by_compatible(fdt, DT_UART_COMPAT, instance);
	if (node < 0) {
		return 0;
	}

	clk_id = fdt_get_clock_id(fdt, node);
	if (clk_id < 0) {
		return 0;
	}

	return stm32mp1_clk_get_rate((unsigned long)clk_id);
}
