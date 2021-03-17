// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2021, STMicroelectronics
 */

#include <assert.h>
#include <config.h>
#include <crypto/crypto.h>
#include <drivers/clk.h>
#include <drivers/stm32_gpio.h>
#include <drivers/stm32_rtc.h>
#include <drivers/stm32_tamp.h>
#include <io.h>
#include <kernel/boot.h>
#include <kernel/delay.h>
#include <kernel/dt.h>
#include <kernel/interrupt.h>
#include <mm/core_memprot.h>
#include <sm/psci.h>  /* If psci_reset */
#include <stdbool.h>
#include <stm32_util.h>
#include <stm32mp_pm.h> /* If stm32_cores_reset() */

#include <libfdt.h>

/* STM32 Registers */
#define _TAMP_CR1			0x00U
#define _TAMP_CR2			0x04U
#define _TAMP_CR3			0x08U
#define _TAMP_FLTCR			0x0CU
#define _TAMP_ATCR1			0x10U
#define _TAMP_ATSEEDR			0x14U
#define _TAMP_ATOR			0x18U
#define _TAMP_ATCR2			0x1CU
#define _TAMP_SECCFGR			0x20U
#define _TAMP_SMCR			0x20U
#define _TAMP_PRIVCFGR			0x24U
#define _TAMP_IER			0x2CU
#define _TAMP_SR			0x30U
#define _TAMP_MISR			0x34U
#define _TAMP_SMISR			0x38U
#define _TAMP_SCR			0x3CU
#define _TAMP_COUNTR			0x40U
#define _TAMP_COUNT2R			0x44U
#define _TAMP_OR			0x50U
#define _TAMP_ERCFGR			0X54U
#define _TAMP_HWCFGR2			0x3ECU
#define _TAMP_HWCFGR1			0x3F0U
#define _TAMP_VERR			0x3F4U
#define _TAMP_IPIDR			0x3F8U
#define _TAMP_SIDR			0x3FCU

/* _TAMP_CR1 bit filds */
#define _TAMP_CR1_ITAMP(id)		BIT((id) + 16U)
#define _TAMP_CR1_ETAMP(id)		BIT((id))

/* _TAMP_CR2 bit filds */
#define _TAMP_CR2_ETAMPTRG(id)		BIT((id) + 24U)
#define _TAMP_CR2_BKERASE		BIT(23)
#define _TAMP_CR2_BKBLOCK		BIT(22)
#define _TAMP_CR2_ETAMPMSK_MAX_ID	3U
#define _TAMP_CR2_ETAMPMSK(id)		BIT((id) + 16U)
#define _TAMP_CR2_ETAMPNOER(id)		BIT((id))

/* _TAMP_CR3 bit filds */
#define _TAMP_CR3_ITAMPNOER_ALL		GENMASK_32(12, 0)
#define _TAMP_CR3_ITAMPNOER(id)		BIT((id))

/* _TAMP_FLTCR bit fields */
#define _TAMP_FLTCR_TAMPFREQ		GENMASK_32(2, 0)
#define _TAMP_FLTCR_TAMPFLT		GENMASK_32(4, 3)
#define _TAMP_FLTCR_TAMPPRCH		GENMASK_32(6, 5)
#define _TAMP_FLTCR_TAMPPUDIS		BIT(7)

/* _TAMP_ATCR bit fields */
#define _TAMP_ATCR1_ATCKSEL		GENMASK_32(18, 16)
#define _TAMP_ATCR1_ATPER		GENMASK_32(26, 24)
#define _TAMP_ATCR1_COMMON_MASK		GENMASK_32(31, 16)
#define _TAMP_ATCR1_ETAMPAM(id)		BIT((id))
#define _TAMP_ATCR1_ATOSEL_MASK(i)	GENMASK_32(((i) + 1) * 2U + 7U, \
						   (i) * 2U + 8U)
#define _TAMP_ATCR1_ATOSEL(i, o)	(((o) - 1U) << ((i) * 2U + 8U))

/* _TAMP_ATOR bit fields */
#define _TAMP_PRNG			GENMASK_32(7, 0)
#define _TAMP_SEEDF			BIT(14)
#define _TAMP_INITS			BIT(15)

/* _TAMP_IER bit fields */
#define _TAMP_IER_ITAMP(id)		BIT((id) + 16U)
#define _TAMP_IER_ETAMP(id)		BIT((id))

/* _TAMP_SR bit fields */
#define _TAMP_SR_ETAMPXF_MASK		GENMASK_32(7, 0)
#define _TAMP_SR_ITAMPXF_MASK		GENMASK_32(31, 16)
#define _TAMP_SR_ITAMP(id)		BIT((id) + 16U)
#define _TAMP_SR_ETAMP(id)		BIT((id))

/* _TAMP_SCR bit fields */
#define _TAMP_SCR_ITAMP(id)		BIT((id) + 16U)
#define _TAMP_SCR_ETAMP(id)		BIT((id))

/* _TAMP_SECCFGR bit fields */
#define _TAMP_SECCFGR_BKPRWSEC_MASK	GENMASK_32(7, 0)
#define _TAMP_SECCFGR_BKPRWSEC_SHIFT	0U
#define _TAMP_SECCFGR_CNT2SEC		BIT(14)
#define _TAMP_SECCFGR_CNT1SEC		BIT(15)
#define _TAMP_SECCFGR_BKPWSEC_MASK	GENMASK_32(23, 16)
#define _TAMP_SECCFGR_BKPWSEC_SHIFT	16U
#define _TAMP_SECCFGR_BHKLOCK		BIT(30)
#define _TAMP_SECCFGR_TAMPSEC		BIT(31)

/* _TAMP_SMCR bit fields */
#define _TAMP_SMCR_BKPRWDPROT_MASK	GENMASK_32(7, 0)
#define _TAMP_SMCR_BKPRWDPROT_SHIFT	0U
#define _TAMP_SMCR_BKPWDPROT_MASK	GENMASK_32(23, 16)
#define _TAMP_SMCR_BKPWDPROT_SHIFT	16U
#define _TAMP_SMCR_DPROT		BIT(31)

/*
 * _TAMP_PRIVCFGR bit fields defined in stm32_tamp.h
 * See TAMP_.*PRIVILEGED
 */
#define _TAMP_PRIVCFGR_MASK		(GENMASK_32(31, 29) | \
					 GENMASK_32(15, 14))

/* _TAMP_ATCR2 bit fields */
#define _TAMP_ATCR2_ATOSEL_MASK(i)	GENMASK_32(((i) + 1) * 3U + 7U, \
						   (i) * 3U + 8U)
#define _TAMP_ATCR2_ATOSEL(i, o)	((o) << ((i) * 3U + 8U))

/* _TAMP_OR bit fields */
#define _TAMP_OR_IN1RMP_PF10		0U
#define _TAMP_OR_IN1RMP_PC13		BIT(0)
#define _TAMP_OR_IN2RMP_PA6		0U
#define _TAMP_OR_IN2RMP_PI1		BIT(1)
#define _TAMP_OR_IN3RMP_PC0		0U
#define _TAMP_OR_IN3RMP_PI2		BIT(2)
#define _TAMP_OR_IN4RMP_PG8		0U
#define _TAMP_OR_IN4RMP_PI3		BIT(3)

#define _TAMP_OR_OUT3RMP_PI8		0U
#define _TAMP_OR_OUT3RMP_PC13		BIT(0)

/* _TAMP_ERCFGR bit fields */
#define _TAMP_ERCFGR_ERCFG0		BIT(0)

/* _TAMP_HWCFGR2 bit fields */
#define _TAMP_HWCFGR2_TZ		GENMASK_32(11, 8)
#define _TAMP_HWCFGR2_OR		GENMASK_32(7, 0)

/* _TAMP_HWCFGR1 bit fields */
#define _TAMP_HWCFGR1_BKPREG		GENMASK_32(7, 0)
#define _TAMP_HWCFGR1_TAMPER		GENMASK_32(11, 8)
#define _TAMP_HWCFGR1_ACTIVE		GENMASK_32(15, 12)
#define _TAMP_HWCFGR1_INTERN		GENMASK_32(31, 16)
#define _TAMP_HWCFGR1_ITAMP_MAX_ID	16U
#define _TAMP_HWCFGR1_ITAMP(id)		BIT((id) + 16U)

/* _TAMP_VERR bit fields */
#define _TAMP_VERR_MINREV		GENMASK_32(3, 0)
#define _TAMP_VERR_MAJREV		GENMASK_32(7, 4)

#define SEED_TIMEOUT_US			1000

struct stm32_tamp_int {
	const uint32_t id;
	uint32_t mode;
	int (*func)(int id);
};

struct stm32_tamp_ext {
	const uint32_t id;
	uint32_t mode;
	uint8_t out_pin;
	int (*func)(int id);
};

struct stm32_tamp_instance {
	struct stm32_tamp_platdata pdata;
	uint32_t hwconf1;
	uint32_t hwconf2;
	uint32_t secret_list_conf;
	uint32_t privilege_conf;
	uint32_t secure_conf;
	uint32_t passive_conf;
	uint32_t active_conf;
};

struct stm32_tamp_int int_tamp_mp15[] = {
	{ .id = INT_TAMP1 }, { .id = INT_TAMP2 }, { .id = INT_TAMP3 },
	{ .id = INT_TAMP4 }, { .id = INT_TAMP5 }, { .id = INT_TAMP8 },
};

struct stm32_tamp_ext ext_tamp_mp15[] = {
	{ .id = EXT_TAMP1 }, { .id = EXT_TAMP2 }, { .id = EXT_TAMP3 },
};

/*
 * Only 1 instance of TAMP is expected per platform
 *
 * 0 is the expected initial values for all fields but .id
 */
static struct stm32_tamp_instance stm32_tamp = { };
DECLARE_KEEP_PAGER(stm32_tamp);

struct my_dt_device_match {
	const char *compatible;
	void *data;
};

static struct stm32_tamp_compat mp15_compat = {
		.nb_monotonic_counter = 1,
		.tags = 0,
		.int_tamp = int_tamp_mp15,
		.int_tamp_size = ARRAY_SIZE(int_tamp_mp15),
		.ext_tamp = ext_tamp_mp15,
		.ext_tamp_size = ARRAY_SIZE(ext_tamp_mp15),
};

static const struct my_dt_device_match tamp_match_table[] = {
	{ .compatible = "st,stm32-tamp", .data = &mp15_compat},
	{ 0 }
};

static void stm32_tamp_set_secret_list(struct stm32_tamp_instance *tamp)
{
	vaddr_t base = io_pa_or_va(&tamp->pdata.base);

	if (tamp->pdata.compat &&
	    (tamp->pdata.compat->tags & TAMP_HAS_CONF_SECRET_IPS)) {
		if (tamp->secret_list_conf & TAMP_CONF_SECRET_BACKUP_SRAM)
			io_setbits32(base + _TAMP_ERCFGR, _TAMP_ERCFGR_ERCFG0);
		else
			io_clrbits32(base + _TAMP_ERCFGR, _TAMP_ERCFGR_ERCFG0);
	}
}

static void stm32_tamp_set_secure(struct stm32_tamp_instance *tamp)
{
	vaddr_t base = io_pa_or_va(&tamp->pdata.base);
	uint32_t mode = 0;

	/* Force secure mode: within OP-TEE we only access TAMP from secure */
	tamp->secure_conf |= TAMP_REGS_IT_SECURE;
	mode = tamp->secure_conf;

	if (tamp->pdata.compat &&
	    (tamp->pdata.compat->tags & TAMP_HAS_CONF_SECURE)) {
		if (mode & TAMP_REGS_IT_SECURE)
			io_setbits32(base + _TAMP_SECCFGR,
				     _TAMP_SECCFGR_TAMPSEC);
		else
			io_clrbits32(base + _TAMP_SECCFGR,
				     _TAMP_SECCFGR_TAMPSEC);

		if (mode & TAMP_CNT1_SECURE)
			io_setbits32(base + _TAMP_SECCFGR,
				     _TAMP_SECCFGR_CNT1SEC);
		else
			io_clrbits32(base + _TAMP_SECCFGR,
				     _TAMP_SECCFGR_CNT1SEC);

		if (mode & TAMP_CNT2_SECURE)
			io_setbits32(base + _TAMP_SECCFGR,
				     _TAMP_SECCFGR_CNT2SEC);
		else
			io_clrbits32(base + _TAMP_SECCFGR,
				     _TAMP_SECCFGR_CNT2SEC);
	} else {
		/* Note: MP15 doesn't use SECCFG register
		 * and inverts the secure bit
		 */
		if (mode & TAMP_REGS_IT_SECURE)
			io_clrbits32(base + _TAMP_SMCR, _TAMP_SMCR_DPROT);
		else
			io_setbits32(base + _TAMP_SMCR, _TAMP_SMCR_DPROT);
	}
}

static void stm32_tamp_set_privilege(struct stm32_tamp_instance *tamp)
{
	vaddr_t base = io_pa_or_va(&tamp->pdata.base);
	uint32_t mode = tamp->privilege_conf;

	if (tamp->pdata.compat &&
	    (tamp->pdata.compat->tags & TAMP_HAS_CONF_PRIVILEGE))
		io_clrsetbits32(base + _TAMP_PRIVCFGR, _TAMP_PRIVCFGR_MASK,
				mode & _TAMP_PRIVCFGR_MASK);
}

static void stm32_tamp_set_pins(vaddr_t base, uint32_t mode)
{
	io_setbits32(base + _TAMP_OR, mode);
}

static TEE_Result stm32_tamp_set_seed(vaddr_t base)
{
	/* Need RNG access. */
	uint64_t timeout_ref = timeout_init_us(SEED_TIMEOUT_US);
	uint8_t idx = 0;

	for (idx = 0; idx < 4U; idx++) {
		uint32_t rnd = 0;

		if (crypto_rng_read((uint8_t *)&rnd, sizeof(uint32_t)) != 0)
			return TEE_ERROR_BAD_STATE;

		io_write32(base + _TAMP_ATSEEDR, rnd);
	}

	while ((io_read32(base + _TAMP_ATOR) & _TAMP_SEEDF) != 0U) {
		if (timeout_elapsed(timeout_ref))
			return TEE_ERROR_BAD_STATE;
	}

	return TEE_SUCCESS;
}

static bool is_int_tamp_id_valid(uint32_t id)
{
	return (id <= _TAMP_HWCFGR1_ITAMP_MAX_ID) &&
	       (stm32_tamp.hwconf1 & _TAMP_HWCFGR1_ITAMP(id));
}

static bool is_ext_tamp_id_valid(uint32_t id)
{
	return (stm32_tamp.pdata.compat) &&
	       (id <= stm32_tamp.pdata.compat->ext_tamp_size);
}

static TEE_Result stm32_tamp_set_int_config(struct stm32_tamp_compat *tcompat,
					    uint32_t itamp_index, uint32_t *cr1,
					    uint32_t *cr3, uint32_t *ier)
{
	uint32_t id = 0;
	struct stm32_tamp_int *tamp_int = NULL;

	if (!tcompat)
		return TEE_ERROR_BAD_PARAMETERS;

	tamp_int = &tcompat->int_tamp[itamp_index];
	id = tamp_int->id;

	if (!is_int_tamp_id_valid(id))
		return TEE_ERROR_BAD_PARAMETERS;

	/* If this internal tamper is disabled, reset its configuration. */
	if ((tamp_int->mode & TAMP_ENABLE) != TAMP_ENABLE) {
		*cr1 &= ~_TAMP_CR1_ITAMP(id);
		*ier &= ~_TAMP_IER_ITAMP(id);
		if (tcompat->tags & TAMP_HAS_CR3)
			*cr3 &= ~_TAMP_CR3_ITAMPNOER(id);

		return TEE_SUCCESS;
	}

	*cr1 |= _TAMP_CR1_ITAMP(id);
	*ier |= _TAMP_IER_ITAMP(id);

	if (tcompat->tags & TAMP_HAS_CR3) {
		if ((tamp_int->mode & TAMP_NOERASE) == TAMP_NOERASE)
			*cr3 |= _TAMP_CR3_ITAMPNOER(id);
		else
			*cr3 &= ~_TAMP_CR3_ITAMPNOER(id);
	}

	return TEE_SUCCESS;
}

static TEE_Result stm32_tamp_set_ext_config(struct stm32_tamp_compat *tcompat,
					    uint32_t etamp_index,
					    uint32_t *cr1, uint32_t *cr2,
					    uint32_t *atcr1, uint32_t *atcr2,
					    uint32_t *ier)
{
	uint32_t id = 0;
	struct stm32_tamp_ext *tamp_ext = NULL;

	if (!tcompat)
		return TEE_ERROR_BAD_PARAMETERS;

	tamp_ext = &tcompat->ext_tamp[etamp_index];
	id = tamp_ext->id;

	/* Exit if not a valid TAMP_ID */
	if (!is_ext_tamp_id_valid(id))
		return TEE_ERROR_BAD_PARAMETERS;

	/* If this external tamper is disabled, reset its configuration. */
	if ((tamp_ext->mode & TAMP_ENABLE) != TAMP_ENABLE) {
		*cr1 &= ~_TAMP_CR1_ETAMP(id);
		*cr2 &= ~_TAMP_CR2_ETAMPMSK(id);
		*cr2 &= ~_TAMP_CR2_ETAMPTRG(id);
		*cr2 &= ~_TAMP_CR2_ETAMPNOER(id);
		*ier &= ~_TAMP_IER_ETAMP(id);
		return TEE_SUCCESS;
	}

	*cr1 |= _TAMP_CR1_ETAMP(id);

	if ((tamp_ext->mode & TAMP_TRIG_ON) == TAMP_TRIG_ON)
		*cr2 |= _TAMP_CR2_ETAMPTRG(id);
	else
		*cr2 &= ~_TAMP_CR2_ETAMPTRG(id);

	if ((tamp_ext->mode & TAMP_ACTIVE) == TAMP_ACTIVE) {
		*atcr1 |= _TAMP_ATCR1_ETAMPAM(id);
		/* Configure output pin:
		 * For the case out_pin = 0, we select same output pin than the
		 * input one.
		 */
		if (tamp_ext->out_pin == TAMPOUTSEL_SAME_AS_INPUT)
			tamp_ext->out_pin = id + 1;

		if (tamp_ext->out_pin <= tcompat->ext_tamp_size) {
			if (tcompat->tags & TAMP_HAS_ATCR2)
				*atcr2 = (*atcr2 &
					  ~_TAMP_ATCR2_ATOSEL_MASK(id)) |
					_TAMP_ATCR2_ATOSEL(id,
							   tamp_ext->out_pin);
			else
				*atcr1 = (*atcr1 &
					  ~_TAMP_ATCR1_ATOSEL_MASK(id)) |
					_TAMP_ATCR1_ATOSEL(id,
							   tamp_ext->out_pin);
		}
	} else {
		*atcr1 &= ~_TAMP_ATCR1_ETAMPAM(id);
	}

	if ((tamp_ext->mode & TAMP_NOERASE) == TAMP_NOERASE)
		*cr2 |= _TAMP_CR2_ETAMPNOER(id);
	else
		*cr2 &= ~_TAMP_CR2_ETAMPNOER(id);

	if (id < _TAMP_CR2_ETAMPMSK_MAX_ID) {
		/*
		 * Only external TAMP 1, 2 and 3 can be masked
		 */
		if ((tamp_ext->mode & TAMP_EVT_MASK) == TAMP_EVT_MASK) {
			/*
			 * ETAMP(id) event generates a trigger event. This
			 * ETAMP(id) is masked and internally cleared by
			 * hardware.
			 * The secrets are not erased.
			 */
			*ier &= ~_TAMP_IER_ETAMP(id);
			*cr2 |= _TAMP_CR2_ETAMPMSK(id);
		} else {
			/*
			 * normal ETAMP interrupt:
			 * ETAMP(id) event generates a trigger event and
			 * TAMP(id) must be cleared by software to allow
			 * next tamper event detection.
			 */
			*cr2 &= ~_TAMP_CR2_ETAMPMSK(id);
			*ier |= _TAMP_IER_ETAMP(id);
		}
	} else {
		/* Other than 1,2,3 external TAMP, we want its interruption */
		*ier |= _TAMP_IER_ETAMP(id);
	}

	return TEE_SUCCESS;
}

TEE_Result stm32_tamp_set_secure_bkpregs(struct stm32_bkpregs_conf *bkr_conf)
{
	uint32_t first_z2 = 0;
	uint32_t first_z3 = 0;
	vaddr_t base = io_pa_or_va(&stm32_tamp.pdata.base);

	if (!bkr_conf)
		return TEE_ERROR_BAD_PARAMETERS;

	first_z2 = bkr_conf->nb_zone1_regs;
	first_z3 = bkr_conf->nb_zone1_regs + bkr_conf->nb_zone2_regs;

	if ((first_z2 > (stm32_tamp.hwconf1 & _TAMP_HWCFGR1_BKPREG)) ||
	    (first_z3 > (stm32_tamp.hwconf1 & _TAMP_HWCFGR1_BKPREG)))
		return TEE_ERROR_NOT_SUPPORTED;

	if (stm32_tamp.pdata.compat &&
	    (stm32_tamp.pdata.compat->tags & TAMP_HAS_CONF_SECURE)) {
		io_clrsetbits32(base + _TAMP_SECCFGR,
				_TAMP_SECCFGR_BKPRWSEC_MASK,
				(first_z2 << _TAMP_SECCFGR_BKPRWSEC_SHIFT) &
				_TAMP_SECCFGR_BKPRWSEC_MASK);

		io_clrsetbits32(base + _TAMP_SECCFGR,
				_TAMP_SECCFGR_BKPWSEC_MASK,
				(first_z3 << _TAMP_SECCFGR_BKPWSEC_SHIFT) &
				_TAMP_SECCFGR_BKPWSEC_MASK);
	} else {
		io_clrsetbits32(base + _TAMP_SMCR,
				_TAMP_SMCR_BKPRWDPROT_MASK,
				(first_z2 << _TAMP_SMCR_BKPRWDPROT_SHIFT) &
				_TAMP_SMCR_BKPRWDPROT_MASK);

		io_clrsetbits32(base + _TAMP_SMCR,
				_TAMP_SMCR_BKPWDPROT_MASK,
				(first_z3 << _TAMP_SMCR_BKPWDPROT_SHIFT) &
				_TAMP_SMCR_BKPWDPROT_MASK);
	}

	return TEE_SUCCESS;
}

TEE_Result stm32_tamp_set_config(void)
{
	int ret = TEE_SUCCESS;
	uint32_t i = 0;
	uint32_t cr1 = 0;
	uint32_t cr2 = 0;
	uint32_t cr3 = 0;
	uint32_t atcr1 = 0;
	uint32_t atcr2 = 0;
	uint32_t fltcr = 0;
	uint32_t ier = 0;

	vaddr_t base = io_pa_or_va(&stm32_tamp.pdata.base);

	if (!stm32_tamp.pdata.compat ||
	    !stm32_tamp.pdata.compat->int_tamp ||
	    !stm32_tamp.pdata.compat->ext_tamp)
		return TEE_ERROR_BAD_STATE;

	/* Select extra IP to add in the deleted/blocked IP in case of
	 * tamper event
	 */
	stm32_tamp_set_secret_list(&stm32_tamp);

	/* Select access in secure or unsecure */
	stm32_tamp_set_secure(&stm32_tamp);

	/* Select access in privileged mode or unprivileged mode */
	stm32_tamp_set_privilege(&stm32_tamp);

	/* Set passive filter configuration */
	if (stm32_tamp.passive_conf != 0U)
		fltcr = stm32_tamp.passive_conf;

	/* Set active mode configuration */
	if (stm32_tamp.active_conf != 0U)
		atcr1 = (atcr1 &  ~_TAMP_ATCR1_COMMON_MASK) |
			   (stm32_tamp.active_conf & _TAMP_ATCR1_COMMON_MASK);

	for (i = 0U; i < stm32_tamp.pdata.compat->int_tamp_size; i++) {
		ret = stm32_tamp_set_int_config(stm32_tamp.pdata.compat, i,
						&cr1, &cr3, &ier);
		if (ret != 0)
			return ret;
	}

	for (i = 0U; i < stm32_tamp.pdata.compat->ext_tamp_size; i++) {
		ret = stm32_tamp_set_ext_config(stm32_tamp.pdata.compat, i,
						&cr1, &cr2, &atcr1, &atcr2,
						&ier);
		if (ret != 0)
			return ret;
	}

	/*
	 * We apply configuration all in a row:
	 * As for active ext tamper "all the needed tampers must be enabled in
	 * the same write access".
	 */
	io_write32(base + _TAMP_FLTCR, fltcr);
	/* Active filter configuration applied only if not already done. */
	if (((io_read32(base + _TAMP_ATOR) & _TAMP_INITS) != _TAMP_INITS)) {
		io_write32(base + _TAMP_ATCR1, atcr1);
		if (stm32_tamp.pdata.compat->tags & TAMP_HAS_ATCR2)
			io_write32(base + _TAMP_ATCR2, atcr2);
	}

	io_write32(base + _TAMP_CR1, cr1);
	io_write32(base + _TAMP_CR2, cr2);
	if (stm32_tamp.pdata.compat->tags & TAMP_HAS_CR3)
		io_write32(base + _TAMP_CR3, cr3);

	/* If active tamper we reinit the seed. */
	if (stm32_tamp.active_conf != 0U) {
		if (stm32_tamp_set_seed(base) != TEE_SUCCESS) {
			EMSG("Active tamper: SEED not initialized");
			return TEE_ERROR_BAD_STATE;
		}
	}

	/* Enable interrupts. */
	io_write32(base + _TAMP_IER, ier);

	return TEE_SUCCESS;
}

TEE_Result stm32_tamp_write_mcounter(int cnt_idx)
{
	vaddr_t base = io_pa_or_va(&stm32_tamp.pdata.base);

	if (cnt_idx < 0 || !stm32_tamp.pdata.compat ||
	    cnt_idx > stm32_tamp.pdata.compat->nb_monotonic_counter)
		return TEE_ERROR_BAD_PARAMETERS;

	io_write32(base + _TAMP_COUNTR + cnt_idx * sizeof(uint32_t), 1U);

	return TEE_SUCCESS;
}

uint32_t stm32_tamp_read_mcounter(int cnt_idx)
{
	vaddr_t base = io_pa_or_va(&stm32_tamp.pdata.base);

	if (cnt_idx < 0 || !stm32_tamp.pdata.compat ||
	    cnt_idx > stm32_tamp.pdata.compat->nb_monotonic_counter)
		return 0U;

	return io_read32(base + _TAMP_COUNTR + cnt_idx * sizeof(uint32_t));
}

void stm32_tamp_configure_secret_list(uint32_t secret_list_conf)
{
	stm32_tamp.secret_list_conf = secret_list_conf;
}

void stm32_tamp_configure_privilege_access(uint32_t privilege_conf)
{
	stm32_tamp.privilege_conf = privilege_conf;
}

void stm32_tamp_configure_secure_access(uint32_t secure_conf)
{
	stm32_tamp.secure_conf = secure_conf;
}

void stm32_tamp_configure_passive(uint32_t passive_conf)
{
	stm32_tamp.passive_conf = passive_conf;
}

void stm32_tamp_configure_active(uint32_t active_conf)
{
	stm32_tamp.active_conf = active_conf;
}

TEE_Result stm32_tamp_configure_internal(enum stm32_tamp_int_id id,
					 uint32_t mode,
					 int (*callback)(int id))
{
	uint32_t i = 0;
	uint32_t itamp_id = id;
	struct stm32_tamp_int *tamp_int = NULL;

	if (!stm32_tamp.pdata.compat)
		return TEE_ERROR_BAD_STATE;

	/* Find internal Tamp struct*/
	for (i = 0U; i < stm32_tamp.pdata.compat->int_tamp_size; i++) {
		if (stm32_tamp.pdata.compat->int_tamp[i].id == itamp_id) {
			tamp_int = &stm32_tamp.pdata.compat->int_tamp[i];
			break;
		}
	}

	if (!tamp_int)
		return TEE_ERROR_BAD_PARAMETERS;

	tamp_int->mode = mode;
	tamp_int->func = callback;

	return TEE_SUCCESS;
}

TEE_Result stm32_tamp_configure_external(enum stm32_tamp_ext_id id,
					 uint32_t mode,
					 enum stm32_tamp_ext_out_id out_pin,
					 int (*callback)(int id))
{
	uint32_t i = 0;
	uint32_t etamp_id = id;
	struct stm32_tamp_ext *tamp_ext = NULL;

	if (!stm32_tamp.pdata.compat)
		return TEE_ERROR_BAD_STATE;

	/* Find external Tamp struct */
	for (i = 0U; i < stm32_tamp.pdata.compat->ext_tamp_size; i++) {
		if (stm32_tamp.pdata.compat->ext_tamp[i].id == etamp_id) {
			tamp_ext = &stm32_tamp.pdata.compat->ext_tamp[i];
			break;
		}
	}

	if (!tamp_ext)
		return TEE_ERROR_BAD_PARAMETERS;

	tamp_ext->mode = mode;
	tamp_ext->out_pin = out_pin;
	tamp_ext->func = callback;

	return TEE_SUCCESS;
}

bool stm32_tamp_are_secrets_blocked(void)
{
	if (stm32_tamp.pdata.compat &&
	    (stm32_tamp.pdata.compat->tags & TAMP_HAS_SECRET_STATUS)) {
		vaddr_t base = io_pa_or_va(&stm32_tamp.pdata.base);

		return (((io_read32(base + _TAMP_CR2) &
			  _TAMP_CR2_BKBLOCK) == _TAMP_CR2_BKBLOCK) ||
			(io_read32(base + _TAMP_SR) != 0));
	} else {
		return false;
	}
}

void stm32_tamp_block_secrets(void)
{
	vaddr_t base = io_pa_or_va(&stm32_tamp.pdata.base);

	if (stm32_tamp.pdata.compat &&
	    (stm32_tamp.pdata.compat->tags & TAMP_HAS_SECRET_STATUS))
		io_setbits32(base + _TAMP_CR2, _TAMP_CR2_BKBLOCK);
}

void stm32_tamp_unblock_secrets(void)
{
	vaddr_t base = io_pa_or_va(&stm32_tamp.pdata.base);

	if (stm32_tamp.pdata.compat &&
	    (stm32_tamp.pdata.compat->tags & TAMP_HAS_SECRET_STATUS))
		io_clrbits32(base + _TAMP_CR2, _TAMP_CR2_BKBLOCK);
}

void stm32_tamp_erase_secrets(void)
{
	vaddr_t base = io_pa_or_va(&stm32_tamp.pdata.base);

	if (stm32_tamp.pdata.compat &&
	    (stm32_tamp.pdata.compat->tags & TAMP_HAS_SECRET_STATUS))
		io_setbits32(base + _TAMP_CR2, _TAMP_CR2_BKERASE);
}

void stm32_tamp_lock_boot_hardware_key(void)
{
	vaddr_t base = io_pa_or_va(&stm32_tamp.pdata.base);

	if (stm32_tamp.pdata.compat &&
	    (stm32_tamp.pdata.compat->tags & TAMP_HAS_LOCK_KEY))
		io_setbits32(base + _TAMP_SECCFGR, _TAMP_SECCFGR_BHKLOCK);
}

static enum itr_return stm32_tamp_it_handler(struct itr_handler *h)
{
	struct stm32_tamp_instance *tamp = h->data;
	vaddr_t base = io_pa_or_va(&tamp->pdata.base);
	uint32_t it = io_read32(base + _TAMP_SR);
	uint32_t int_it = it & _TAMP_SR_ITAMPXF_MASK;
	uint32_t ext_it = it & _TAMP_SR_ETAMPXF_MASK;
	uint8_t i = 0;
	struct stm32_rtc_time tamp_ts = { };

	if (stm32_rtc_is_timestamp_enable()) {
		stm32_rtc_get_timestamp(&tamp_ts);
		FMSG("Tamper Event Occurred");
		FMSG("Date : %u/%u\n \t Time : %u:%u:%u",
		     tamp_ts.day, tamp_ts.month, tamp_ts.hour,
		     tamp_ts.min, tamp_ts.sec);
	}

	while ((int_it != 0U) && (i < stm32_tamp.pdata.compat->int_tamp_size)) {
		int ret = -1;
		uint32_t id = tamp->pdata.compat->int_tamp[i].id;

		if ((it & _TAMP_SR_ITAMP(id)) != 0U) {
			if (tamp->pdata.compat->int_tamp[i].func)
				ret = tamp->pdata.compat->int_tamp[i].func(id);

			if (ret >= 0) {
				io_setbits32(base + _TAMP_SCR,
					     _TAMP_SR_ITAMP(id));
				ext_it &= ~_TAMP_SR_ITAMP(id);

				if (ret > 0) {
#ifdef CFG_PM
					stm32_cores_reset();
#else
					psci_system_reset();
#endif
				}
			}
		}
		i++;
	}

	i = 0;
	/* External tamper interrupt */
	while ((ext_it != 0U) && (i < stm32_tamp.pdata.compat->ext_tamp_size)) {
		int ret = -1;
		uint32_t id = tamp->pdata.compat->ext_tamp[i].id;

		if ((ext_it & _TAMP_SR_ETAMP(id)) != 0U) {
			if (tamp->pdata.compat->ext_tamp[i].func)
				ret = tamp->pdata.compat->ext_tamp[i].func(id);

			if (ret >= 0) {
				io_setbits32(base + _TAMP_SCR,
					     _TAMP_SCR_ETAMP(id));
				ext_it &= ~_TAMP_SR_ETAMP(id);

				if (ret > 0) {
#ifdef CFG_PM
					stm32_cores_reset();
#else
					psci_system_reset();
#endif
				}
			}
		}
		i++;
	}

	return ITRR_HANDLED;
}

#ifdef CFG_DT
static int get_node_from_multiple_compatible(const void *fdt,
					     struct stm32_tamp_platdata *pdata)
{
	const struct my_dt_device_match *cur = tamp_match_table;

	while (cur->compatible) {
		int node = -1;

		node = fdt_node_offset_by_compatible(fdt, node,
						     cur->compatible);
		if (node >= 0) {
			pdata->compat = (struct stm32_tamp_compat *)cur->data;
			return node;
		}

		cur++;
	}

	/* Compatible string not found */
	return -1;
}

static int stm32_tamp_parse_fdt(struct stm32_tamp_platdata *pdata)
{
	int node = -1;
	struct dt_node_info dt_tamp = { };
	void *fdt = NULL;

	fdt = get_embedded_dt();
	if (!fdt)
		return -1;

	node = get_node_from_multiple_compatible(fdt, pdata);
	if (node < 0)
		return -1;

	_fdt_fill_device_info(fdt, &dt_tamp, node);

	if (dt_tamp.status == DT_STATUS_DISABLED ||
	    dt_tamp.clock == DT_INFO_INVALID_CLOCK ||
	    dt_tamp.reg == DT_INFO_INVALID_REG ||
	    dt_tamp.interrupt == DT_INFO_INVALID_INTERRUPT) {
		return -1;
	}

	pdata->it = dt_tamp.interrupt;
	pdata->base.pa = dt_tamp.reg;
	pdata->base.va = (vaddr_t)phys_to_virt(dt_tamp.reg, MEM_AREA_IO_SEC);

	pdata->clock = dt_tamp.clock;

	pdata->pins_conf = 0;
	if (stm32_pinctrl_fdt_get_pinctrl(fdt, node, NULL, 0) > 0) {
		if (pdata->compat == &mp15_compat) {
			if (fdt_getprop(fdt, node, "st,out3-pc13", NULL))
				pdata->pins_conf |= _TAMP_OR_OUT3RMP_PC13;
		}
	}

	if (fdt_getprop(fdt, node, "wakeup-source", NULL))
		pdata->is_wakeup_source = true;

	return 0;
}

__weak
int stm32_tamp_get_platdata(struct stm32_tamp_platdata *pdata __unused)
{
	/* In DT config, the platform data are filled by DT file */
	return 0;
}
#else /* CFG_DT */
static int stm32_tamp_parse_fdt(struct stm32_tamp_platdata *pdata)
{
	/* Do nothing, there is no fdt to parse in this case */
	return 0;
}

/* This function can be overridden by platform to define pdata of tamp driver */
__weak
int stm32_tamp_get_platdata(struct stm32_tamp_platdata *pdata __unused)
{
	return -1;
}
#endif /* CFG_DT */

static struct itr_handler stm32_tamp_itr_handler = {
	.handler =  stm32_tamp_it_handler,
};
DECLARE_KEEP_PAGER(stm32_tamp_itr_handler);

static TEE_Result stm32_tamp_init(void)
{
	int ret = TEE_SUCCESS;
	vaddr_t base = 0;
	uint32_t __unused rev = 0;

	ret = stm32_tamp_get_platdata(&stm32_tamp.pdata);
	if (ret)
		return TEE_ERROR_NOT_SUPPORTED;

	ret = stm32_tamp_parse_fdt(&stm32_tamp.pdata);
	if (ret)
		return TEE_ERROR_NOT_SUPPORTED;

	/* Init Tamp clock */
	clk_enable(stm32_tamp.pdata.clock);

	base = io_pa_or_va(&stm32_tamp.pdata.base);

	stm32_tamp.hwconf1 = io_read32(base + _TAMP_HWCFGR1);
	stm32_tamp.hwconf2 = io_read32(base + _TAMP_HWCFGR2);

#if TRACE_LEVEL >= TRACE_FLOW
	rev = io_read32(base + _TAMP_VERR);
	FMSG("STM32 TAMPER V%u.%u", (rev & _TAMP_VERR_MAJREV) >> 4,
	     rev & _TAMP_VERR_MINREV);
#endif

	if ((stm32_tamp.hwconf2 & _TAMP_HWCFGR2_TZ) == 0U) {
		EMSG("Tamper IP doesn't support trustzone");
		return TEE_ERROR_NOT_SUPPORTED;
	}

	stm32_tamp_set_pins(base, stm32_tamp.pdata.pins_conf);

	stm32_tamp_itr_handler.it = stm32_tamp.pdata.it;
	stm32_tamp_itr_handler.data = &stm32_tamp;
	itr_add(&stm32_tamp_itr_handler);
	itr_enable(stm32_tamp_itr_handler.it);

	/* Need a new API */
	if (stm32_tamp.pdata.is_wakeup_source)
		IMSG("TAMP event are not configured as wakeup source");

	return TEE_SUCCESS;
}

driver_init(stm32_tamp_init);
