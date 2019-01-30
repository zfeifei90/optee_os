// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2017-2018, STMicroelectronics - All Rights Reserved
 */

#include <kernel/delay.h>
#include <drivers/stm32_i2c.h>
#include <drivers/stm32mp1_clk.h>
#include <drivers/stm32mp1_pmic.h>
#include <drivers/stpmic1.h>
#include <io.h>
#include <keep.h>
#include <kernel/dt.h>
#include <kernel/generic_boot.h>
#include <kernel/panic.h>
#include <libfdt.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <stdbool.h>
#include <stm32_util.h>
#include <stm32mp_dt.h>
#include <stm32mp_pm.h>
#include <trace.h>
#include <util.h>

#define STPMIC1_LDO12356_OUTPUT_MASK	(uint8_t)(GENMASK_32(6, 2))
#define STPMIC1_LDO12356_OUTPUT_SHIFT	2
#define STPMIC1_LDO3_MODE		(uint8_t)(BIT(7))
#define STPMIC1_LDO3_DDR_SEL		31U
#define STPMIC1_LDO3_1800000		(9U << STPMIC1_LDO12356_OUTPUT_SHIFT)

#define STPMIC1_BUCK_OUTPUT_SHIFT	2
#define STPMIC1_BUCK3_1V8		(39U << STPMIC1_BUCK_OUTPUT_SHIFT)

#define MODE_STANDBY                    8U

#define STPMIC1_DEFAULT_START_UP_DELAY_MS	1

static struct i2c_handle_s i2c_handle;
static uint32_t pmic_i2c_addr;

bool stm32mp_with_pmic(void)
{
	return (i2c_handle.dt_status & DT_STATUS_OK_SEC) != 0;
}

static int dt_get_pmic_node(void *fdt)
{
	return fdt_node_offset_by_compatible(fdt, -1, "st,stpmic1");
}

static int dt_pmic_status(void)
{
	void *fdt = get_dt_blob();

	if (fdt) {
		int node = dt_get_pmic_node(fdt);

		if (node > 0) {
			return _fdt_get_status(fdt, node);
		}
	}

	return -1;
}

static bool dt_pmic_is_secure(void)
{
	int status = dt_pmic_status();

	return ((unsigned)status == DT_STATUS_OK_SEC) &&
	       (i2c_handle.dt_status == DT_STATUS_OK_SEC);
}

/*
 * @idx: Private identifier provided by the target PMIC driver
 * @flags: Operations expected when entering a low power sequence
 * @voltage: Target voltage to apply during low power sequences
 */
struct regu_bo_config {
	uint8_t flags;
	struct stpmic1_bo_cfg cfg;
};

#define REGU_BO_FLAG_ENABLE_REGU		BIT(0)
#define REGU_BO_FLAG_SET_VOLTAGE		BIT(1)
#define REGU_BO_FLAG_PULL_DOWN			BIT(2)
#define REGU_BO_FLAG_MASK_RESET			BIT(3)

static struct regu_bo_config *regu_bo_config;
static size_t regu_bo_count;

static int save_boot_on_config(void)
{
	int pmic_node, regulators_node, regulator_node;
	void *fdt;

	assert(!regu_bo_config && !regu_bo_count);

	fdt = get_dt_blob();
	if (fdt == NULL) {
		panic();
	}

	pmic_node = dt_get_pmic_node(fdt);
	if (pmic_node < 0) {
		panic();
	}

	regulators_node = fdt_subnode_offset(fdt, pmic_node, "regulators");

	fdt_for_each_subnode(regulator_node, fdt, regulators_node) {
		const fdt32_t *cuint;
		const char *name;
		struct regu_bo_config regu_cfg;
		uint16_t mv;

		if (fdt_getprop(fdt, regulator_node, "regulator-boot-on",
				NULL) == NULL) {
			continue;
		}

		memset(&regu_cfg, 0, sizeof(regu_cfg));
		name = fdt_get_name(fdt, regulator_node, NULL);

		regu_cfg.flags |= REGU_BO_FLAG_ENABLE_REGU;

		if (fdt_getprop(fdt, regulator_node, "regulator-pull-down",
				NULL) != NULL) {
			stpmic1_bo_pull_down_cfg(name, &regu_cfg.cfg);
			regu_cfg.flags |= REGU_BO_FLAG_PULL_DOWN;
		}

		if (fdt_getprop(fdt, regulator_node, "st,mask-reset",
				NULL) != NULL) {
			stpmic1_bo_mask_reset_cfg(name, &regu_cfg.cfg);
			regu_cfg.flags |= REGU_BO_FLAG_MASK_RESET;
		}

		cuint = fdt_getprop(fdt, regulator_node,
				    "regulator-min-microvolt", NULL);
		if (cuint != NULL) {
			/* DT uses microvolts, whereas driver awaits millivolts */
			mv = (uint16_t)(fdt32_to_cpu(*cuint) / 1000U);

			if (!stpmic1_bo_voltage_cfg(name, mv, &regu_cfg.cfg)) {
				regu_cfg.flags |= REGU_BO_FLAG_SET_VOLTAGE;
			}
		}

		/* Save config in the Boot On configuration list */
		regu_bo_count++;
		regu_bo_config = realloc(regu_bo_config,
					 regu_bo_count * sizeof(regu_cfg));
		if (regu_bo_config == NULL) {
			panic();
		}

		memcpy(&regu_bo_config[regu_bo_count - 1], &regu_cfg,
			sizeof(regu_cfg));
	}

	return 0;
}

void stm32mp_pmic_apply_boot_on_config(void)
{
	size_t i;

	for (i = 0; i < regu_bo_count; i++) {
		struct regu_bo_config *regu_cfg = &regu_bo_config[i];

		if (regu_cfg->flags & REGU_BO_FLAG_SET_VOLTAGE) {
			if (stpmic1_bo_voltage_unpg(&regu_cfg->cfg)) {
				panic();
			}
		}

		if (regu_cfg->flags & REGU_BO_FLAG_ENABLE_REGU) {
			if (stpmic1_bo_enable_unpg(&regu_cfg->cfg)) {
				panic();
			}
		}

		if (regu_cfg->flags & REGU_BO_FLAG_PULL_DOWN) {
			if (stpmic1_bo_pull_down_unpg(&regu_cfg->cfg)) {
				panic();
			}
		}

		if (regu_cfg->flags & REGU_BO_FLAG_MASK_RESET) {
			if (stpmic1_bo_mask_reset_unpg(&regu_cfg->cfg)) {
				panic();
			}
		}
	}
}

/*
 * @idx: Private identifier provided by the target PMIC driver
 * @flags: Operations expected when entering a low power sequence
 * @voltage: Target voltage to apply during low power sequences
 */
struct regu_lp_config {
	uint8_t flags;
	struct stpmic1_lp_cfg cfg;
};

#define REGU_LP_FLAG_LOAD_PWRCTRL	BIT(0)
#define REGU_LP_FLAG_ON_IN_SUSPEND	BIT(1)
#define REGU_LP_FLAG_OFF_IN_SUSPEND	BIT(2)
#define REGU_LP_FLAG_SET_VOLTAGE	BIT(3)
#define REGU_LP_FLAG_MODE_STANDBY	BIT(4)

struct regu_lp_state {
	const char *name;
	size_t cfg_count;
	struct regu_lp_config *cfg;
};

#define REGU_LP_STATE_DISK		0
#define REGU_LP_STATE_STANDBY		1
#define REGU_LP_STATE_MEM		2
#define REGU_LP_STATE_COUNT		3

static struct regu_lp_state regu_lp_state[REGU_LP_STATE_COUNT] = {
	[REGU_LP_STATE_DISK] = { .name = "standby-ddr-off", },
	[REGU_LP_STATE_STANDBY] = { .name = "standby-ddr-sr", },
	[REGU_LP_STATE_MEM] = { .name = "lp-stop", },
};

static unsigned int regu_lp_state2idx(const char *name)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(regu_lp_state); i++) {
		struct regu_lp_state *state = &regu_lp_state[i];

		if (!strncmp(name, state->name, strlen(state->name))) {
			return i;
		}
	}

	panic();
}

static int save_low_power_config(const char *lp_state)
{
	int pmic_node, regulators_node, regulator_node;
	void *fdt;
	unsigned int state_idx = regu_lp_state2idx(lp_state);
	struct regu_lp_state *state = &regu_lp_state[state_idx];

	assert(!state->cfg && !state->cfg_count);

	fdt = get_dt_blob();
	if (fdt == NULL) {
		panic();
	}

	pmic_node = dt_get_pmic_node(fdt);
	if (pmic_node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	regulators_node = fdt_subnode_offset(fdt, pmic_node, "regulators");

	fdt_for_each_subnode(regulator_node, fdt, regulators_node) {
		const fdt32_t *cuint;
		const char *reg_name;
		int regulator_state_node;
		struct regu_lp_config *regu_cfg;

		state->cfg_count++;
		state->cfg = realloc(state->cfg,
				     state->cfg_count * sizeof(*state->cfg));
		if (state->cfg == NULL) {
			panic();
		}

		regu_cfg = &state->cfg[state->cfg_count - 1];

		memset(regu_cfg, 0, sizeof(*regu_cfg));

		reg_name = fdt_get_name(fdt, regulator_node, NULL);

		if (stpmic1_lp_cfg(reg_name, &regu_cfg->cfg) != 0) {
			EMSG("Invalid regu name %s", reg_name);
			return -1;
		}

		/*
		 * Always copy active configuration (Control register) to
		 * PWRCTRL Control register, even if regulator_state_node
		 * does not exist.
		 */
		regu_cfg->flags |= REGU_LP_FLAG_LOAD_PWRCTRL;

		/* Then apply configs from regulator_state_node */
		regulator_state_node = fdt_subnode_offset(fdt,
							  regulator_node,
							  lp_state);
		if (regulator_state_node <= 0) {
			continue;
		}

		if (fdt_getprop(fdt, regulator_state_node,
				"regulator-on-in-suspend", NULL) != NULL) {
			regu_cfg->flags |= REGU_LP_FLAG_ON_IN_SUSPEND;
		}

		if (fdt_getprop(fdt, regulator_state_node,
				"regulator-off-in-suspend", NULL) != NULL) {
			regu_cfg->flags |= REGU_LP_FLAG_OFF_IN_SUSPEND;
		}

		cuint = fdt_getprop(fdt, regulator_state_node,
				    "regulator-suspend-microvolt", NULL);
		if (cuint != NULL) {
			uint32_t mv = fdt32_to_cpu(*cuint) / 1000U;

			if (stpmic1_lp_voltage_cfg(reg_name, mv,
						    &regu_cfg->cfg) == 0) {
				regu_cfg->flags |= REGU_LP_FLAG_SET_VOLTAGE;
			}
		}

		cuint = fdt_getprop(fdt, regulator_state_node,
				    "regulator-mode", NULL);
		if (cuint != NULL) {
			if (fdt32_to_cpu(*cuint) == MODE_STANDBY) {
				regu_cfg->flags |= REGU_LP_FLAG_MODE_STANDBY;
			}
		}
	}

	return 0;
}

/*
 * int stm32mp_pmic_set_lp_config(char *lp_state)
 *
 * Load the low power configuration stored in regu_lp_state[].
 */
void stm32mp_pmic_apply_lp_config(const char *lp_state)
{
	unsigned int state_idx = regu_lp_state2idx(lp_state);
	struct regu_lp_state *state = &regu_lp_state[state_idx];
	size_t i;

	if (stpmic1_powerctrl_on() != 0) {
		panic();
	}

	for (i = 0; i < state->cfg_count; i++) {
		struct stpmic1_lp_cfg *cfg = &state->cfg[i].cfg;

		if ((state->cfg[i].flags & REGU_LP_FLAG_LOAD_PWRCTRL) != 0) {
			if (stpmic1_lp_load_unpg(cfg) != 0) {
				panic();
			}
		}

		if ((state->cfg[i].flags & REGU_LP_FLAG_ON_IN_SUSPEND) != 0) {
			if (stpmic1_lp_on_off_unpg(cfg, 1) != 0) {
				panic();
			}
		}

		if ((state->cfg[i].flags & REGU_LP_FLAG_OFF_IN_SUSPEND) != 0) {
			if (stpmic1_lp_on_off_unpg(cfg, 0) != 0) {
				panic();
			}
		}

		if ((state->cfg[i].flags & REGU_LP_FLAG_SET_VOLTAGE) != 0) {
			if (stpmic1_lp_voltage_unpg(cfg) != 0) {
				panic();
			}
		}

		if ((state->cfg[i].flags & REGU_LP_FLAG_MODE_STANDBY) != 0) {
			if (stpmic1_lp_mode_unpg(cfg, 1) != 0) {
				panic();
			}
		}
	}
}

static int save_power_configurations(void)
{
	unsigned int i;

	if (save_boot_on_config() != 0) {
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(regu_lp_state); i++) {
		if (save_low_power_config(regu_lp_state[i].name) != 0) {
			return -1;
		}
	}

	return 0;
}

/*
 * Get PMIC and its I2C bus configuration from the device tree.
 * Return 0 on success, negative on error, 1 if no PMIC node is defined.
 */
static int dt_pmic_i2c_config(struct dt_node_info *i2c_info,
			      struct stm32_pinctrl **pinctrl,
			      size_t *pinctrl_count,
			      struct stm32_i2c_init_s *init)
{
	int pmic_node;
	int i2c_node;
	void *fdt;
	const fdt32_t *cuint;

	fdt = get_dt_blob();
	if (!fdt) {
		return -1;
	}

	pmic_node = dt_get_pmic_node(fdt);
	if (pmic_node < 0) {
		return 1;
	}

	cuint = fdt_getprop(fdt, pmic_node, "reg", NULL);
	if (cuint == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	pmic_i2c_addr = fdt32_to_cpu(*cuint) << 1;
	if (pmic_i2c_addr > UINT16_MAX) {
		return -1;
	}

	i2c_node = fdt_parent_offset(fdt, pmic_node);
	if (i2c_node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	fdt_fill_device_info(fdt, i2c_info, i2c_node);
	if (i2c_info->base == 0U) {
		return -FDT_ERR_NOTFOUND;
	}

	return stm32_i2c_get_setup_from_fdt(fdt, i2c_node, init,
					    pinctrl, pinctrl_count);
}

/*
 * PMIC and resource initialization
 */

/* Return true if PMIC is available, false if not found, panics on errors */
static bool initialize_pmic_i2c(void)
{
	int ret;
	struct dt_node_info i2c_info;
	struct i2c_handle_s *i2c = &i2c_handle;
	struct stm32_pinctrl *pinctrl;
	size_t pinctrl_count;
	struct stm32_i2c_init_s i2c_init;

	ret = dt_pmic_i2c_config(&i2c_info, &pinctrl, &pinctrl_count,
				 &i2c_init);
	if (ret < 0) {
		EMSG("I2C configuration failed %d\n", ret);
		panic();
	}
	if (ret != 0) {
		return false;
	}

	/* Initialize PMIC I2C */
	i2c->pbase = i2c_info.base;
	i2c->vbase = (uintptr_t)phys_to_virt(i2c_info.base, MEM_AREA_IO_SEC);
	assert(i2c->vbase);
	i2c->dt_status = i2c_info.status;
	i2c->clock = i2c_info.clock;
	i2c_init.own_address1 = pmic_i2c_addr;
	i2c_init.addressing_mode = I2C_ADDRESSINGMODE_7BIT;
	i2c_init.dual_address_mode = I2C_DUALADDRESS_DISABLE;
	i2c_init.own_address2 = 0;
	i2c_init.own_address2_masks = I2C_OAR2_OA2NOMASK;
	i2c_init.general_call_mode = I2C_GENERALCALL_DISABLE;
	i2c_init.no_stretch_mode = I2C_NOSTRETCH_DISABLE;
	i2c_init.analog_filter = 1;
	i2c_init.digital_filter_coef = 0;

	i2c->pinctrl = pinctrl;
	i2c->pinctrl_count = pinctrl_count;

	stm32mp_get_pmic();

	ret = stm32_i2c_init(i2c, &i2c_init);
	if (ret != 0) {
		EMSG("Cannot initialize I2C %x (%d)\n", i2c->pbase, ret);
		panic();
	}

	if (!stm32_i2c_is_device_ready(i2c, pmic_i2c_addr, 1,
				       I2C_TIMEOUT_BUSY_MS)) {
		EMSG("I2C device not ready\n");
		panic();
	}

	stpmic1_bind_i2c(i2c, (uint16_t)pmic_i2c_addr);

	stm32mp_put_pmic();

	return true;
}

/*
 * Automated suspend/resume at system suspend/resume is expected
 * only when the PMIC is secure. If it is non secure, only atomic
 * execution context acn get/put the PMIC resources.
 */
static void pmic_suspend_resume(enum pm_op op, void __unused *handle)
{
	switch (op) {
	case PM_OP_SUSPEND:
		stm32_i2c_suspend(&i2c_handle);
		break;
	case PM_OP_RESUME:
		stm32_i2c_resume(&i2c_handle);
		break;
	default:
		panic();
	}
}
KEEP_PAGER(pmic_suspend_resume);

/* stm32mp_get/put_pmic allows secure atomic sequences to use non secure PMIC */
void stm32mp_get_pmic(void)
{
	stm32_i2c_resume(&i2c_handle);
}

void stm32mp_put_pmic(void)
{
	stm32_i2c_suspend(&i2c_handle);
}

static void register_non_secure_pmic(void)
{
	size_t n;

	if (!i2c_handle.pbase) {
		return;
	}

	stm32mp_register_non_secure_periph_iomem(i2c_handle.pbase);
	stm32mp_register_clock_parents_secure(i2c_handle.clock);

	for (n = 0; n < i2c_handle.pinctrl_count; n++) {
		stm32mp_register_non_secure_gpio(i2c_handle.pinctrl[n].bank,
						 i2c_handle.pinctrl[n].pin);
	}
}

static void register_secure_pmic(void)
{
	size_t n;

	stm32mp_register_pm_cb(pmic_suspend_resume, NULL);
	stm32mp_register_secure_periph_iomem(i2c_handle.pbase);

	for (n = 0; n < i2c_handle.pinctrl_count; n++) {
		stm32mp_register_secure_gpio(i2c_handle.pinctrl[n].bank,
					     i2c_handle.pinctrl[n].pin);
	}
}

static TEE_Result initialize_pmic(void)
{
	unsigned long pmic_version;

	if (!initialize_pmic_i2c()) {
		DMSG("No PMIC");
		register_non_secure_pmic();
		return TEE_SUCCESS;
	}

	stm32mp_get_pmic();

	if (stpmic1_get_version(&pmic_version)) {
		panic("Failed to access PMIC");
	}

	DMSG("PMIC version = 0x%02lx", pmic_version);
	stpmic1_dump_regulators();

	if (save_power_configurations() != 0) {
		panic();
	}

	if (dt_pmic_is_secure()) {
		register_secure_pmic();
	} else {
		register_non_secure_pmic();
	}

	stm32mp_put_pmic();

	return TEE_SUCCESS;
}
driver_init(initialize_pmic);

