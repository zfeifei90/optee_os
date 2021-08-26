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

#define RNG_SR_DRDY		BIT(0)
#define RNG_SR_CECS		BIT(1)
#define RNG_SR_SECS		BIT(2)
#define RNG_SR_CEIS		BIT(5)
#define RNG_SR_SEIS		BIT(6)

#define RNG_TIMEOUT_US_10MS	U(10000)
#define RNG_TIMEOUT_US_1MS	U(1000)
#define RNG_FIFO_BYTE_DEPTH	U(16)

#define RNG_NIST_CONFIG_A	0x0F00D00
#define RNG_NIST_CONFIG_B	0x1801000
#define RNG_NIST_CONFIG_MASK	GENMASK_32(25, 8)

struct stm32_rng_driver_data {
	bool has_cond_reset;
};

struct stm32_rng_device {
	struct stm32_rng_platdata pdata;
	struct stm32_rng_driver_data *ddata;
	unsigned int lock;
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
	uint64_t timeout_ref = 0;

	if (ddata->has_cond_reset) {
		sr = io_read32(base + RNG_SR);
		if (sr & RNG_SR_SECS) {
			io_setbits32(base + RNG_CR, RNG_CR_CONDRST);
			io_clrbits32(base + RNG_CR, RNG_CR_CONDRST);

			/* Wait for CR == 0 */
			timeout_ref = timeout_init_us(RNG_TIMEOUT_US_10MS);
			while (io_read32(base + RNG_CR) & RNG_CR_CONDRST)
				if (timeout_elapsed(timeout_ref))
					break;

			/* Panic on timeout not due to thread scheduling */
			if (io_read32(base + RNG_CR) & RNG_CR_CONDRST)
				panic();
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

static TEE_Result stm32_rng_init(struct stm32_rng_device *dev)
{
	struct stm32_rng_platdata *pdata = &dev->pdata;
	struct stm32_rng_driver_data *ddata = dev->ddata;
	uintptr_t base = pdata->base;
	uint64_t timeout_ref = 0;
	uint32_t cr = 0;
	uint32_t sr = 0;

	cr = io_read32(base + RNG_CR);
	if (!pdata->clock_error)
		cr |= RNG_CR_CED;

	/* Clean error indications */
	io_write32(base + RNG_SR, 0);

	if (ddata->has_cond_reset) {
		io_setbits32(base + RNG_CR, RNG_CR_CONDRST | cr);
		io_clrsetbits32(base + RNG_CR, RNG_NIST_CONFIG_MASK,
				RNG_NIST_CONFIG_B);
		io_clrsetbits32(base + RNG_CR, RNG_CR_CONDRST,
				RNG_CR_RNGEN | cr);
	} else {
		io_setbits32(base + RNG_CR, RNG_CR_RNGEN | cr);
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

static TEE_Result stm32_rng_read_raw(struct stm32_rng_device *rng_dev,
				     uint8_t *out, size_t *size)
{
	TEE_Result rc = TEE_ERROR_SECURITY;
	uint32_t exceptions = thread_mask_exceptions(THREAD_EXCP_ALL);
	uintptr_t base = rng_dev->pdata.base;
	uint32_t req_size = *size;
	size_t tot_size = 0;
	uint8_t *buf = out;
	uint64_t timeout_ref = timeout_init_us(RNG_TIMEOUT_US_10MS);

	clk_enable(rng_dev->pdata.clock);

	while (req_size && !timeout_elapsed(timeout_ref)) {
		uint32_t len = 0;
		uint32_t sr = io_read32(base + RNG_SR);

		if (sr & RNG_SR_SEIS) {
			conceal_seed_error(rng_dev);
			sr = io_read32(base + RNG_SR);
		}

		if (!(sr & RNG_SR_DRDY))
			continue;

		len = MIN(RNG_FIFO_BYTE_DEPTH, req_size);
		/* RNG is ready: read up to 4 32bit words */
		while (len) {
			uint32_t data32 = io_read32(base + RNG_DR);
			size_t sz = MIN(len, sizeof(uint32_t));

			memcpy(buf, &data32, sz);
			buf += sz;
			len -= sz;
			req_size -= sz;
			tot_size += sz;
		}

		timeout_ref = timeout_init_us(RNG_TIMEOUT_US_10MS);
	}

	clk_disable(rng_dev->pdata.clock);

	if (timeout_elapsed(timeout_ref))
		return TEE_ERROR_GENERIC;

	rc = TEE_SUCCESS;
	*size = tot_size;

	thread_unmask_exceptions(exceptions);

	return rc;
}

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

#ifdef CFG_WITH_SOFTWARE_PRNG
TEE_Result stm32_rng_read(void *buf, size_t blen)
#else
TEE_Result crypto_rng_read(void *buf, size_t blen)
#endif
{
	TEE_Result rc = 0;
	uint32_t exceptions = 0;
	size_t out_size = 0;
	uint8_t *b = buf;

	if (!b)
		return TEE_ERROR_BAD_PARAMETERS;

	if (!stm32_rng.pdata.base) {
		EMSG("No RNG");
		return TEE_ERROR_NOT_SUPPORTED;
	}

	while (out_size < blen) {
		/* Read by chunks of the size the RNG FIFO depth */
		size_t sz = blen - out_size;

		exceptions = may_spin_lock(&stm32_rng.lock);

		rc = stm32_rng_read_raw(&stm32_rng, b, &sz);

		may_spin_unlock(&stm32_rng.lock, exceptions);

		if (rc)
			goto bail;

		out_size += sz;
		b += sz;
	}

bail:
	if (rc)
		memset(b, 0, blen);

	return rc;
}

uint8_t hw_get_random_byte(void)
{
	uint8_t data = 0;
	size_t len = 1;

	if (stm32_rng_read_raw(&stm32_rng, &data, &len) != TEE_SUCCESS)
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

