// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2019-2021, STMicroelectronics
 */
#include <assert.h>
#include <compiler.h>
#include <config.h>
#include <confine_array_index.h>
#include <drivers/clk.h>
#include <drivers/clk_dt.h>
#include <drivers/scmi-msg.h>
#include <drivers/scmi.h>
#include <drivers/stm32_firewall.h>
#include <drivers/stm32mp_dt_bindings.h>
#include <initcall.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <platform_config.h>
#include <stdint.h>
#include <speculation_barrier.h>
#include <stm32_util.h>
#include <string.h>
#include <tee_api_defines.h>
#include <util.h>

#define TIMEOUT_US_1MS		1000

#define SCMI_CLOCK_NAME_SIZE	16
#define SCMI_RD_NAME_SIZE	16
#define SCMI_VOLTD_NAME_SIZE	16

/*
 * struct stm32_scmi_clk - Data for the exposed clock
 * @clock_id: Clock identifier in RCC clock driver
 * @name: Clock string ID exposed to channel
 * @enabled: State of the SCMI clock
 */
struct stm32_scmi_clk {
	unsigned long clock_id;
	struct clk *clk;
	const char *name;
	bool enabled;
};

/*
 * struct stm32_scmi_rd - Data for the exposed reset controller
 * @reset_id: Reset identifier in RCC reset driver
 * @base: Physical controller address
 * @name: Reset string ID exposed to channel
 */
struct stm32_scmi_rd {
	unsigned long reset_id;
	paddr_t base;
	const char *name;
};

/* Locate all non-secure SMT message buffers in last page of SYSRAM */
#define SMT_BUFFER_BASE		CFG_STM32MP1_SCMI_SHM_BASE
#define SMT_BUFFER0_BASE	SMT_BUFFER_BASE
#define SMT_BUFFER1_BASE	(SMT_BUFFER_BASE + 0x200)

#if (SMT_BUFFER1_BASE + SMT_BUF_SLOT_SIZE > \
	CFG_STM32MP1_SCMI_SHM_BASE + CFG_STM32MP1_SCMI_SHM_SIZE)
#error "SCMI shared memory mismatch"
#endif

register_phys_mem(MEM_AREA_IO_NSEC, CFG_STM32MP1_SCMI_SHM_BASE,
		  CFG_STM32MP1_SCMI_SHM_SIZE);

#define CLOCK_CELL(_scmi_id, _id, _name, _init_enabled) \
	[_scmi_id] = { \
		.clock_id = _id, \
		.name = _name, \
		.enabled = _init_enabled, \
	}

#define RESET_CELL(_scmi_id, _id, _base, _name) \
	[_scmi_id] = { \
		.reset_id = _id, \
		.base = _base, \
		.name = _name, \
	}

struct channel_resources {
	struct scmi_msg_channel *channel;
	struct stm32_scmi_clk *clock;
	size_t clock_count;
	struct stm32_scmi_rd *rd;
	size_t rd_count;
};

#ifdef CFG_STM32MP13
static struct stm32_scmi_clk stm32_scmi0_clock[] = {
	CLOCK_CELL(CK_SCMI0_HSE, CK_HSE, "ck_hse", true),
	CLOCK_CELL(CK_SCMI0_HSI, CK_HSI, "ck_hsi", true),
	CLOCK_CELL(CK_SCMI0_CSI, CK_CSI, "ck_csi", true),
	CLOCK_CELL(CK_SCMI0_LSE, CK_LSE, "ck_lse", true),
	CLOCK_CELL(CK_SCMI0_LSI, CK_LSI, "ck_lsi", true),
	CLOCK_CELL(CK_SCMI0_HSE_DIV2, CK_HSE_DIV2, "clk-hse-div2", true),
	CLOCK_CELL(CK_SCMI0_PLL2_Q, PLL2_Q, "pll2_q", true),
	CLOCK_CELL(CK_SCMI0_PLL2_R, PLL2_R, "pll2_r", true),
	CLOCK_CELL(CK_SCMI0_PLL3_P, PLL3_P, "pll3_p", true),
	CLOCK_CELL(CK_SCMI0_PLL3_Q, PLL3_Q, "pll3_q", true),
	CLOCK_CELL(CK_SCMI0_PLL3_R, PLL3_R, "pll3_r", true),
	CLOCK_CELL(CK_SCMI0_PLL4_P, PLL4_P, "pll4_p", true),
	CLOCK_CELL(CK_SCMI0_PLL4_Q, PLL4_Q, "pll4_q", true),
	CLOCK_CELL(CK_SCMI0_PLL4_R, PLL4_R, "pll4_r", true),
	CLOCK_CELL(CK_SCMI0_MPU, CK_MPU, "ck_mpu", true),
	CLOCK_CELL(CK_SCMI0_AXI, CK_AXI, "ck_axi", true),
	CLOCK_CELL(CK_SCMI0_MLAHB, CK_MLAHB, "ck_mlahb", true),
	CLOCK_CELL(CK_SCMI0_CKPER, CK_PER, "ck_per", true),
	CLOCK_CELL(CK_SCMI0_PCLK1, PCLK1, "pclk1", true),
	CLOCK_CELL(CK_SCMI0_PCLK2, PCLK2, "pclk2", true),
	CLOCK_CELL(CK_SCMI0_PCLK3, PCLK3, "pclk3", true),
	CLOCK_CELL(CK_SCMI0_PCLK4, PCLK4, "pclk4", true),
	CLOCK_CELL(CK_SCMI0_PCLK5, PCLK5, "pclk5", true),
	CLOCK_CELL(CK_SCMI0_PCLK6, PCLK6, "pclk6", true),
	CLOCK_CELL(CK_SCMI0_CKTIMG1, CK_TIMG1, "timg1_ck", true),
	CLOCK_CELL(CK_SCMI0_CKTIMG2, CK_TIMG2, "timg2_ck", true),
	CLOCK_CELL(CK_SCMI0_CKTIMG3, CK_TIMG3, "timg3_ck", true),
	CLOCK_CELL(CK_SCMI0_RTC, RTC, "ck_rtc", true),
	CLOCK_CELL(CK_SCMI0_RTCAPB, RTCAPB, "rtcapb", true),
	CLOCK_CELL(CK_SCMI0_BSEC, BSEC, "bsec", true),
};

static struct stm32_scmi_rd stm32_scmi0_reset_domain[] = {
	RESET_CELL(RST_SCMI0_LTDC, LTDC_R, LTDC_BASE, "ltdc"),
	RESET_CELL(RST_SCMI0_MDMA, MDMA_R, MDMA_BASE, "mdma"),
};

/* Currently supporting Clocks and Reset Domains */
static const uint8_t plat_protocol_list[] = {
	SCMI_PROTOCOL_ID_CLOCK,
	SCMI_PROTOCOL_ID_RESET_DOMAIN,
#ifdef CFG_SCMI_MSG_REGULATOR_CONSUMER
	SCMI_PROTOCOL_ID_VOLTAGE_DOMAIN,
#endif
	0 /* Null termination */
};

static const struct channel_resources scmi_channel[] = {
	[0] = {
		.channel = &(struct scmi_msg_channel){
			.shm_addr = { .pa = SMT_BUFFER0_BASE },
			.shm_size = SMT_BUF_SLOT_SIZE,
		},
		.clock = stm32_scmi0_clock,
		.clock_count = ARRAY_SIZE(stm32_scmi0_clock),
		.rd = stm32_scmi0_reset_domain,
		.rd_count = ARRAY_SIZE(stm32_scmi0_reset_domain),
	},
	[1] = {
		.channel = &(struct scmi_msg_channel){
			.shm_addr = { .pa = SMT_BUFFER1_BASE },
			.shm_size = SMT_BUF_SLOT_SIZE,
		},
	},
};

#endif /* CFG_STM32MP13 */

#ifdef CFG_STM32MP15
static struct stm32_scmi_clk stm32_scmi0_clock[] = {
	CLOCK_CELL(CK_SCMI0_HSE, CK_HSE, "ck_hse", true),
	CLOCK_CELL(CK_SCMI0_HSI, CK_HSI, "ck_hsi", true),
	CLOCK_CELL(CK_SCMI0_CSI, CK_CSI, "ck_csi", true),
	CLOCK_CELL(CK_SCMI0_LSE, CK_LSE, "ck_lse", true),
	CLOCK_CELL(CK_SCMI0_LSI, CK_LSI, "ck_lsi", true),
	CLOCK_CELL(CK_SCMI0_PLL2_Q, PLL2_Q, "pll2_q", true),
	CLOCK_CELL(CK_SCMI0_PLL2_R, PLL2_R, "pll2_r", true),
	CLOCK_CELL(CK_SCMI0_MPU, CK_MPU, "ck_mpu", true),
	CLOCK_CELL(CK_SCMI0_AXI, CK_AXI, "ck_axi", true),
	CLOCK_CELL(CK_SCMI0_BSEC, BSEC, "bsec", true),
	CLOCK_CELL(CK_SCMI0_CRYP1, CRYP1, "cryp1", false),
	CLOCK_CELL(CK_SCMI0_GPIOZ, GPIOZ, "gpioz", false),
	CLOCK_CELL(CK_SCMI0_HASH1, HASH1, "hash1", false),
	CLOCK_CELL(CK_SCMI0_I2C4, I2C4_K, "i2c4_k", false),
	CLOCK_CELL(CK_SCMI0_I2C6, I2C6_K, "i2c6_k", false),
	CLOCK_CELL(CK_SCMI0_IWDG1, IWDG1, "iwdg1", false),
	CLOCK_CELL(CK_SCMI0_RNG1, RNG1_K, "rng1_k", true),
	CLOCK_CELL(CK_SCMI0_RTC, RTC, "ck_rtc", true),
	CLOCK_CELL(CK_SCMI0_RTCAPB, RTCAPB, "rtcapb", true),
	CLOCK_CELL(CK_SCMI0_SPI6, SPI6_K, "spi6_k", false),
	CLOCK_CELL(CK_SCMI0_USART1, USART1_K, "usart1_k", false),
};

static struct stm32_scmi_clk stm32_scmi1_clock[] = {
	CLOCK_CELL(CK_SCMI1_PLL3_Q, PLL3_Q, "pll3_q", true),
	CLOCK_CELL(CK_SCMI1_PLL3_R, PLL3_R, "pll3_r", true),
	CLOCK_CELL(CK_SCMI1_MCU, CK_MCU, "ck_mcu", false),
};

static struct stm32_scmi_rd stm32_scmi0_reset_domain[] = {
	RESET_CELL(RST_SCMI0_SPI6, SPI6_R, SPI6_BASE, "spi6"),
	RESET_CELL(RST_SCMI0_I2C4, I2C4_R, I2C4_BASE, "i2c4"),
	RESET_CELL(RST_SCMI0_I2C6, I2C6_R, I2C6_BASE, "i2c6"),
	RESET_CELL(RST_SCMI0_USART1, USART1_R, USART1_BASE, "usart1"),
	RESET_CELL(RST_SCMI0_STGEN, STGEN_R, STGEN_BASE, "stgen"),
	RESET_CELL(RST_SCMI0_GPIOZ, GPIOZ_R, GPIOZ_BASE, "gpioz"),
	RESET_CELL(RST_SCMI0_CRYP1, CRYP1_R, CRYP1_BASE, "cryp1"),
	RESET_CELL(RST_SCMI0_HASH1, HASH1_R, HASH1_BASE, "hash1"),
	RESET_CELL(RST_SCMI0_RNG1, RNG1_R, RNG1_BASE, "rng1"),
	RESET_CELL(RST_SCMI0_MDMA, MDMA_R, MDMA_BASE, "mdma"),
	RESET_CELL(RST_SCMI0_MCU, MCU_R, 0, "mcu"),
	RESET_CELL(RST_SCMI0_MCU_HOLD_BOOT, MCU_HOLD_BOOT_R, 0,
		   "mcu_hold_boot"),
};

/* Currently supporting Clocks and Reset Domains */
static const uint8_t plat_protocol_list[] = {
	SCMI_PROTOCOL_ID_CLOCK,
	SCMI_PROTOCOL_ID_RESET_DOMAIN,
#ifdef CFG_SCMI_MSG_REGULATOR_CONSUMER
	SCMI_PROTOCOL_ID_VOLTAGE_DOMAIN,
#endif
	0 /* Null termination */
};

static const struct channel_resources scmi_channel[] = {
	[0] = {
		.channel = &(struct scmi_msg_channel){
			.shm_addr = { .pa = SMT_BUFFER0_BASE },
			.shm_size = SMT_BUF_SLOT_SIZE,
		},
		.clock = stm32_scmi0_clock,
		.clock_count = ARRAY_SIZE(stm32_scmi0_clock),
		.rd = stm32_scmi0_reset_domain,
		.rd_count = ARRAY_SIZE(stm32_scmi0_reset_domain),
	},
	[1] = {
		.channel = &(struct scmi_msg_channel){
			.shm_addr = { .pa = SMT_BUFFER1_BASE },
			.shm_size = SMT_BUF_SLOT_SIZE,
		},
		.clock = stm32_scmi1_clock,
		.clock_count = ARRAY_SIZE(stm32_scmi1_clock),
	},
};
#endif /* CFG_STM32MP15 */

static const struct channel_resources *find_resource(unsigned int channel_id)
{
	assert(channel_id < ARRAY_SIZE(scmi_channel));

	return scmi_channel + channel_id;
}

struct scmi_msg_channel *plat_scmi_get_channel(unsigned int channel_id)
{
	const size_t max_id = ARRAY_SIZE(scmi_channel);
	unsigned int confined_id = confine_array_index(channel_id, max_id);

	if (channel_id >= max_id)
		return NULL;

	return find_resource(confined_id)->channel;
}

static size_t __maybe_unused plat_scmi_protocol_count_paranoid(void)
{
	unsigned int n = 0;
	unsigned int count = 0;
	const size_t channel_count = ARRAY_SIZE(scmi_channel);

	for (n = 0; n < channel_count; n++)
		if (scmi_channel[n].clock_count)
			break;
	if (n < channel_count)
		count++;

	for (n = 0; n < channel_count; n++)
		if (scmi_channel[n].rd_count)
			break;
	if (n < channel_count)
		count++;

	for (n = 0; n < channel_count; n++)
		if (IS_ENABLED(CFG_SCMI_MSG_REGULATOR_CONSUMER) &&
		    plat_scmi_voltd_count(n))
			break;
	if (n < channel_count)
		count++;

	return count;
}

static const char vendor[] = "ST";
static const char sub_vendor[] = "";

const char *plat_scmi_vendor_name(void)
{
	return vendor;
}

const char *plat_scmi_sub_vendor_name(void)
{
	return sub_vendor;
}

size_t plat_scmi_protocol_count(void)
{
	const size_t count = ARRAY_SIZE(plat_protocol_list) - 1;

	assert(count == plat_scmi_protocol_count_paranoid());

	return count;
}

const uint8_t *plat_scmi_protocol_list(unsigned int channel_id __unused)
{
	assert(plat_scmi_protocol_count_paranoid() ==
	       (ARRAY_SIZE(plat_protocol_list) - 1));

	return plat_protocol_list;
}

/*
 * Platform SCMI clocks
 */
static struct stm32_scmi_clk *find_clock(unsigned int channel_id,
					 unsigned int scmi_id)
{
	const struct channel_resources *resource = find_resource(channel_id);
	size_t n = 0;

	if (resource) {
		for (n = 0; n < resource->clock_count; n++)
			if (n == scmi_id)
				return &resource->clock[n];
	}

	return NULL;
}

size_t plat_scmi_clock_count(unsigned int channel_id)
{
	const struct channel_resources *resource = find_resource(channel_id);

	if (!resource)
		return 0;

	return resource->clock_count;
}

const char *plat_scmi_clock_get_name(unsigned int channel_id,
				     unsigned int scmi_id)
{
	struct stm32_scmi_clk *clock = find_clock(channel_id, scmi_id);

	if (!clock || !stm32mp_nsec_can_access_clock(clock->clock_id))
		return NULL;

	return clock->name;
}

int32_t plat_scmi_clock_rates_array(unsigned int channel_id,
				    unsigned int scmi_id, size_t start_index,
				    unsigned long *array, size_t *nb_elts)
{
	struct stm32_scmi_clk *clock = find_clock(channel_id, scmi_id);

	if (!clock)
		return SCMI_NOT_FOUND;

	if (!stm32mp_nsec_can_access_clock(clock->clock_id))
		return SCMI_DENIED;

	/* Exposed clocks are currently fixed rate clocks */
	if (start_index)
		return SCMI_INVALID_PARAMETERS;

	if (!array)
		*nb_elts = 1;
	else if (*nb_elts == 1)
		*array = clk_get_rate(clock->clk);
	else
		return SCMI_GENERIC_ERROR;

	return SCMI_SUCCESS;
}

unsigned long plat_scmi_clock_get_rate(unsigned int channel_id,
				       unsigned int scmi_id)
{
	struct stm32_scmi_clk *clock = find_clock(channel_id, scmi_id);

	if (!clock || !stm32mp_nsec_can_access_clock(clock->clock_id))
		return 0;

	return clk_get_rate(clock->clk);
}

int32_t plat_scmi_clock_get_state(unsigned int channel_id, unsigned int scmi_id)
{
	struct stm32_scmi_clk *clock = find_clock(channel_id, scmi_id);

	if (!clock || !stm32mp_nsec_can_access_clock(clock->clock_id))
		return 0;

	return (int32_t)clock->enabled;
}

int32_t plat_scmi_clock_set_state(unsigned int channel_id, unsigned int scmi_id,
				  bool enable_not_disable)
{
	struct stm32_scmi_clk *clock = find_clock(channel_id, scmi_id);

	if (!clock)
		return SCMI_NOT_FOUND;

	if (!stm32mp_nsec_can_access_clock(clock->clock_id))
		return SCMI_DENIED;

	if (enable_not_disable) {
		if (!clock->enabled) {
			FMSG("SCMI clock %u enable", scmi_id);
			clk_enable(clock->clk);
			clock->enabled = true;
		}
	} else {
		if (clock->enabled) {
			FMSG("SCMI clock %u disable", scmi_id);
			clk_disable(clock->clk);
			clock->enabled = false;
		}
	}

	return SCMI_SUCCESS;
}

/*
 * Platform SCMI reset domains
 */
static struct stm32_scmi_rd *find_rd(unsigned int channel_id,
				     unsigned int scmi_id)
{
	const struct channel_resources *resource = find_resource(channel_id);
	size_t n = 0;

	if (resource) {
		for (n = 0; n < resource->rd_count; n++)
			if (n == scmi_id)
				return &resource->rd[n];
	}

	return NULL;
}

const char *plat_scmi_rd_get_name(unsigned int channel_id, unsigned int scmi_id)
{
	const struct stm32_scmi_rd *rd = find_rd(channel_id, scmi_id);

	if (!rd)
		return NULL;

	return rd->name;
}

size_t plat_scmi_rd_count(unsigned int channel_id)
{
	const struct channel_resources *resource = find_resource(channel_id);

	if (!resource)
		return 0;

	return resource->rd_count;
}

int32_t plat_scmi_rd_autonomous(unsigned int channel_id, unsigned int scmi_id,
				uint32_t state)
{
	const struct stm32_scmi_rd *rd = find_rd(channel_id, scmi_id);
	const struct stm32_firewall_cfg nsec_cfg[] = {
		{ FWLL_NSEC_RW | FWLL_MASTER(0) },
		{ }, /* Null terminated */
	};

	if (!rd)
		return SCMI_NOT_FOUND;

	if (rd->base && stm32_firewall_check_access(rd->base, 0, nsec_cfg))
		return SCMI_DENIED;

#ifdef CFG_STM32MP15
	if (rd->reset_id == MCU_HOLD_BOOT_R)
		return SCMI_NOT_SUPPORTED;
#endif

	/* Supports only reset with context loss */
	if (state)
		return SCMI_NOT_SUPPORTED;

	FMSG("SCMI reset %u cycle", scmi_id);

	if (stm32_reset_assert(rd->reset_id, TIMEOUT_US_1MS))
		return SCMI_HARDWARE_ERROR;

	if (stm32_reset_deassert(rd->reset_id, TIMEOUT_US_1MS))
		return SCMI_HARDWARE_ERROR;

	return SCMI_SUCCESS;
}

int32_t plat_scmi_rd_set_state(unsigned int channel_id, unsigned int scmi_id,
			       bool assert_not_deassert)
{
	const struct stm32_scmi_rd *rd = find_rd(channel_id, scmi_id);
	const struct stm32_firewall_cfg nsec_cfg[] = {
		{ FWLL_NSEC_RW | FWLL_MASTER(0) },
		{ }, /* Null terminated */
	};

	if (!rd)
		return SCMI_NOT_FOUND;

	if (rd->base && stm32_firewall_check_access(rd->base, 0, nsec_cfg))
		return SCMI_DENIED;

#ifdef CFG_STM32MP15
	if (rd->reset_id == MCU_HOLD_BOOT_R) {
		FMSG("SCMI MCU hold boot %s",
		     assert_not_deassert ? "set" : "release");
		stm32_reset_assert_deassert_mcu(assert_not_deassert);
		return SCMI_SUCCESS;
	}
#endif

	if (assert_not_deassert) {
		FMSG("SCMI reset %u set", scmi_id);
		stm32_reset_set(rd->reset_id);
	} else {
		FMSG("SCMI reset %u release", scmi_id);
		stm32_reset_release(rd->reset_id);
	}

	return SCMI_SUCCESS;
}

/*
 * Initialize platform SCMI resources
 */
static TEE_Result stm32mp1_init_scmi_server(void)
{
	size_t i = 0;
	size_t j = 0;

	for (i = 0; i < ARRAY_SIZE(scmi_channel); i++) {
		const struct channel_resources *res = scmi_channel + i;
		struct scmi_msg_channel *chan = res->channel;
		size_t voltd_count = 0;

		/* Enforce non-secure shm mapped as device memory */
		chan->shm_addr.va = (vaddr_t)phys_to_virt(chan->shm_addr.pa,
							  MEM_AREA_IO_NSEC, 1);
		assert(chan->shm_addr.va);

		scmi_smt_init_agent_channel(chan);

		for (j = 0; j < res->clock_count; j++) {
			struct stm32_scmi_clk *clk = &res->clock[j];

			if (!clk->name ||
			    strlen(clk->name) >= SCMI_CLOCK_NAME_SIZE)
				panic("SCMI clock name invalid");

			clk->clk = stm32mp_rcc_clock_id_to_clk(clk->clock_id);
			assert(clk->clk);

			/* Sync SCMI clocks with their targeted initial state */
			if (clk->enabled &&
			    stm32mp_nsec_can_access_clock(clk->clock_id))
				clk_enable(clk->clk);
		}

		for (j = 0; j < res->rd_count; j++) {
			struct stm32_scmi_rd *rd = &res->rd[j];

			if (!rd->name ||
			    strlen(rd->name) >= SCMI_RD_NAME_SIZE)
				panic("SCMI reset domain name invalid");
		}

		if (IS_ENABLED(CFG_SCMI_MSG_REGULATOR_CONSUMER))
		    voltd_count = plat_scmi_voltd_count(i);

		for (j = 0; j < voltd_count; j++) {
			const char *name = plat_scmi_voltd_get_name(i, j);

			if (!name || strlen(name) >= SCMI_VOLTD_NAME_SIZE)
				panic("SCMI voltage domain name invalid");
		}
	}

	return TEE_SUCCESS;
}

driver_init_late(stm32mp1_init_scmi_server);
