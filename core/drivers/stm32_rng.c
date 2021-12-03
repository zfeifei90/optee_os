// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2018-2022, STMicroelectronics
 */

#include <assert.h>
#include <crypto/crypto.h>
#include <drivers/clk.h>
#include <drivers/clk_dt.h>
#include <drivers/stm32_rng.h>
#include <io.h>
#include <kernel/delay.h>
#include <kernel/dt.h>
#include <kernel/boot.h>
#include <kernel/panic.h>
#include <kernel/thread.h>
#include <libfdt.h>
#include <mm/core_memprot.h>
#include <rng_support.h>
#include <stdbool.h>
#include <stm32_util.h>
#include <string.h>
#include <tee/tee_cryp_utl.h>

#define RNG_CR			0x00U
#define RNG_SR			0x04U
#define RNG_DR			0x08U

#define RNG_CR_RNGEN		BIT(2)
#define RNG_CR_IE		BIT(3)
#define RNG_CR_CED		BIT(5)
#define RNG_CR_CONDRST		BIT(30)
#define RNG_CR_CLKDIV		GENMASK_32(19, 16)
#define RNG_CR_CLKDIV_SHIFT	U(16)

#define RNG_SR_DRDY		BIT(0)
#define RNG_SR_CECS		BIT(1)
#define RNG_SR_SECS		BIT(2)
#define RNG_SR_CEIS		BIT(5)
#define RNG_SR_SEIS		BIT(6)

#define RNG_TIMEOUT_US_10MS	U(10000)
#define RNG_TIMEOUT_US_1MS	U(1000)
#define RNG_FIFO_BYTE_DEPTH	U(16)

#define RNG_NIST_CONFIG_A	U(0x0F00D00)
#define RNG_NIST_CONFIG_B	U(0x1801000)
#define RNG_NIST_CONFIG_MASK	GENMASK_32(25, 8)

#define RNG_MAX_CLOCK_FREQ	U(750000)

struct stm32_rng_driver_data {
	bool has_cond_reset;
};

struct stm32_rng_device {
	struct stm32_rng_platdata pdata;
	struct stm32_rng_driver_data *ddata;
	unsigned int lock;
	bool error_conceal;
	uint64_t error_to_ref;
};

struct my_dt_device_match {
	const char *compatible;
	const void *data;
};

const struct stm32_rng_driver_data mp13_data[] = {
	{ .has_cond_reset = true },
};

const struct stm32_rng_driver_data mp15_data[] = {
	{ .has_cond_reset = false },
};

static const struct my_dt_device_match rng_match_table[] = {
	{ .compatible = "st,stm32mp13-rng", .data = &mp13_data},
	{ .compatible = "st,stm32-rng", .data = &mp15_data},
	{ }
};

static struct stm32_rng_device stm32_rng = { };

/* Seed error management */
static void conceal_seed_error(struct stm32_rng_device *dev)
{
	struct stm32_rng_platdata *pdata = &dev->pdata;
	struct stm32_rng_driver_data *ddata = dev->ddata;
	uintptr_t base = pdata->base;
	uint32_t sr = 0;

	if (ddata->has_cond_reset) {
		if (!dev->error_conceal) {
			/* Conceal by resetting the subsystem */
			sr = io_read32(base + RNG_SR);
			if (sr & RNG_SR_SECS) {
				io_setbits32(base + RNG_CR, RNG_CR_CONDRST);
				io_clrbits32(base + RNG_CR, RNG_CR_CONDRST);

				dev->error_to_ref =
					timeout_init_us(RNG_TIMEOUT_US_10MS);
				dev->error_conceal = true;

				/* Wait subsystem reset cycle completes */
				return;
			}
		} else {
			/* Measure time before possible reschedule */
			bool timed_out = timeout_elapsed(dev->error_to_ref);

			if ((io_read32(base + RNG_CR) & RNG_CR_CONDRST)) {
				if (timed_out)
					panic();

				/* Wait subsystem reset cycle completes */
				return;
			}

			dev->error_conceal = false;
		}

		io_clrbits32(base + RNG_SR, RNG_SR_SEIS);
	} else {
		size_t i = 0;

		io_mask32(base + RNG_SR, 0, RNG_SR_SEIS);

		for (i = 12; i != 0; i--)
			(void)io_read32(base + RNG_DR);
	}

	if (io_read32(base + RNG_SR) & RNG_SR_SEIS)
		panic("RNG noise");
}

static TEE_Result stm32_rng_read_available(struct stm32_rng_device *rng_dev,
					   uint8_t *out, size_t size)
{
	uintptr_t base = rng_dev->pdata.base;
	uint8_t *buf = out;

	assert(size <= RNG_FIFO_BYTE_DEPTH);

	if (rng_dev->error_conceal || io_read32(base + RNG_SR) & RNG_SR_SEIS) {
		conceal_seed_error(rng_dev);
		return TEE_ERROR_NO_DATA;
	}

	if (!(io_read32(base + RNG_SR) & RNG_SR_DRDY))
		return TEE_ERROR_NO_DATA;

	/* RNG is ready: read up to 4 32bit words */
	while (size) {
		uint32_t data32 = io_read32(rng_dev->pdata.base + RNG_DR);
		size_t sz = MIN(size, sizeof(uint32_t));

		memcpy(buf, &data32, sz);
		buf += sz;
		size -= sz;
	}

	return TEE_SUCCESS;
}

static uint32_t stm32_rng_clock_freq_restrain(struct stm32_rng_device *dev)
{
	unsigned long clock_rate = 0;
	uint32_t clock_div = 0;

	clock_rate = clk_get_rate(dev->pdata.clock);

	/*
	 * Get the exponent to apply on the CLKDIV field in RNG_CR register
	 * No need to handle the case when clock-div > 0xF as it is physically
	 * impossible
	 */
	while ((clock_rate >> clock_div) > RNG_MAX_CLOCK_FREQ)
		clock_div++;

	DMSG("RNG clk rate : %lu", clk_get_rate(dev->pdata.clock) >> clock_div);

	return clock_div;
}

static TEE_Result stm32_rng_init(struct stm32_rng_device *dev)
{
	struct stm32_rng_platdata *pdata = &dev->pdata;
	struct stm32_rng_driver_data *ddata = dev->ddata;
	uintptr_t base = pdata->base;
	uint64_t timeout_ref = 0;
	uint32_t clock_div = 0;
	uint32_t sr = 0;
	uint32_t cr_ced_mask = 0;

	if (!pdata->clock_error)
		cr_ced_mask = RNG_CR_CED;

	/* Clean error indications */
	io_write32(base + RNG_SR, 0);

	if (ddata->has_cond_reset) {
		clock_div = stm32_rng_clock_freq_restrain(dev);

		/* Update configuration fields */
		io_clrsetbits32(base + RNG_CR, RNG_NIST_CONFIG_MASK,
				RNG_NIST_CONFIG_B | RNG_CR_CONDRST |
				cr_ced_mask);
		io_clrsetbits32(base + RNG_CR, RNG_CR_CLKDIV,
				(clock_div << RNG_CR_CLKDIV_SHIFT));

		/* No need to wait for RNG_CR_CONDRST toggle as we enable clk */
		io_clrsetbits32(base + RNG_CR, RNG_CR_CONDRST, RNG_CR_RNGEN);
	} else {
		io_setbits32(base + RNG_CR, RNG_CR_RNGEN | cr_ced_mask);
	}

	timeout_ref = timeout_init_us(RNG_TIMEOUT_US_10MS);
	while (!(io_read32(base + RNG_SR) & RNG_SR_DRDY))
		if (timeout_elapsed(timeout_ref))
			break;

	if (!(io_read32(base + RNG_SR) & RNG_SR_DRDY))
		return TEE_ERROR_GENERIC;

	sr = io_read32(base + RNG_SR);
	if (sr & (RNG_SR_CEIS | RNG_SR_CECS))
		DMSG("Clock error detected");

	if (sr & (RNG_SR_SECS | RNG_SR_SEIS))
		conceal_seed_error(dev);

	return TEE_SUCCESS;
}

#ifdef CFG_EMBED_DTB
static int get_node_from_multiple_compatible(const void *fdt,
					     struct stm32_rng_device *dev)
{
	const struct my_dt_device_match *cur = rng_match_table;

	while (cur->compatible) {
		int node = -1;

		node = fdt_node_offset_by_compatible(fdt, node,
						     cur->compatible);
		if (node >= 0) {
			dev->ddata = (struct stm32_rng_driver_data *)cur->data;
			return node;
		}

		cur++;
	}

	/* Compatible string not found */
	return -1;
}

static int stm32_rng_parse_fdt(struct stm32_rng_device *dev)
{
	int node = -1;
	struct io_pa_va base = { };
	struct dt_node_info dt_rng = { };
	void *fdt = NULL;
	struct stm32_rng_platdata *pdata = &dev->pdata;

	fdt = get_embedded_dt();
	if (!fdt)
		return -1;

	node = get_node_from_multiple_compatible(fdt, dev);
	if (node < 0)
		return -1;

	_fdt_fill_device_info(fdt, &dt_rng, node);
	if (dt_rng.status == DT_STATUS_DISABLED ||
	    dt_rng.clock == DT_INFO_INVALID_CLOCK ||
	    dt_rng.reg == DT_INFO_INVALID_REG ||
	    dt_rng.reset == DT_INFO_INVALID_RESET) {
		return -1;
	}

	/* ETZPC get state */
	base.pa = dt_rng.reg;

	/* Here depends on State io_pa_or_va_nsec */
	pdata->base = io_pa_or_va_secure(&base, 1);
	pdata->reset = dt_rng.reset;
	pdata->clock_error = false;

	pdata->clock = clk_dt_get_by_idx(fdt, node, 0);
	if (!pdata->clock)
		panic();

	if (fdt_getprop(fdt, node, "clock-error-detect", NULL))
		pdata->clock_error = true;

	return 0;
}

__weak int stm32_rng_get_platdata(struct stm32_rng_platdata *pdata __unused)
{
	/* In DT config, the platform data are filled by DT file */
	return 0;
}
#else /* CFG_EMBED_DTB */
static int stm32_rng_parse_fdt(struct stm32_rng_device *dev)
{
	/* Do nothing, there is no fdt to parse in this case */
	return 0;
}

/*
 * This function can be overridden by platform to define pdata of stm32 driver
 */
__weak
int stm32_rng_get_platdata(struct stm32_tamp_platdata *pdata __unused)
{
	return -1;
}
#endif /* CFG_EMBED_DTB */

int stm32_rng_probe(void)
{
	int ret = 0;
	paddr_t pbase __unused = 0;

	ret = stm32_rng_get_platdata(&stm32_rng.pdata);
	if (ret)
		return ret;

	ret = stm32_rng_parse_fdt(&stm32_rng);
	if (ret)
		return ret;

	ret = clk_enable(stm32_rng.pdata.clock);
	if (ret)
		return ret;

	ret = stm32_reset_assert(stm32_rng.pdata.reset, RNG_TIMEOUT_US_1MS);
	if (ret)
		return ret;

	ret = stm32_reset_deassert(stm32_rng.pdata.reset, RNG_TIMEOUT_US_1MS);
	if (ret)
		return ret;

	ret = stm32_rng_init(&stm32_rng);
	if (ret)
		return ret;

	clk_disable(stm32_rng.pdata.clock);

	pbase = virt_to_phys((void *)stm32_rng.pdata.base);

#ifdef CFG_STM32MP15
	stm32mp_register_secure_periph_iomem(pbase);
#endif

	return 0;
}

/* Masks interrupts while reading available data from RNG FIFO */
static TEE_Result get_rng_bytes_relaxed(uint8_t *out, size_t len)
{
	uint64_t timeout_ref = timeout_init_us(RNG_TIMEOUT_US_10MS);
	TEE_Result res = TEE_ERROR_GENERIC;
	uint32_t exceptions = 0;
	bool timed_out = false;
	uint8_t *buf = out;

	clk_enable(stm32_rng.pdata.clock);

	while (len) {
		size_t burst_len = MIN(len, RNG_FIFO_BYTE_DEPTH);

		exceptions = may_spin_lock(&stm32_rng.lock);

		res = stm32_rng_read_available(&stm32_rng, buf, burst_len);
		if (res) {
			assert(res == TEE_ERROR_NO_DATA);

			/* Measure timeout before we're possibly rescheduled */
			timed_out = timeout_elapsed(timeout_ref);
		} else {
			buf += burst_len;
			len -= burst_len;
			timeout_ref = timeout_init_us(RNG_TIMEOUT_US_10MS);
		}

		may_spin_unlock(&stm32_rng.lock, exceptions);

		if (len && timed_out)
			break;
	}

	clk_disable(stm32_rng.pdata.clock);

	if (!len)
		return TEE_SUCCESS;

	if (timed_out)
		return TEE_ERROR_NO_DATA;

	return TEE_ERROR_GENERIC;
}

#ifdef CFG_WITH_SOFTWARE_PRNG
TEE_Result stm32_rng_read(void *buf, size_t blen)
#else
TEE_Result crypto_rng_read(void *buf, size_t blen)
#endif
{
	TEE_Result res = TEE_ERROR_GENERIC;
	uint8_t *b = buf;

	if (!b)
		return TEE_ERROR_BAD_PARAMETERS;

	if (!stm32_rng.pdata.base) {
		EMSG("No RNG");
		return TEE_ERROR_NOT_SUPPORTED;
	}

	res = get_rng_bytes_relaxed(b, blen);
	if (res)
		memset(b, 0, blen);

	return res;
}

uint8_t hw_get_random_byte(void)
{
	uint8_t data = 0;

	if (get_rng_bytes_relaxed(&data, 1))
		panic();

	return data;
}

#ifndef CFG_WITH_SOFTWARE_PRNG
/* Override weak plat_rng_init with platform handler */
void plat_rng_init(void)
{
	if (stm32_rng_probe())
		panic();
}
#endif

