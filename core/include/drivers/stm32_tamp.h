/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2021, STMicroelectronics
 */

#ifndef __STM32_TAMP_H__
#define __STM32_TAMP_H__

#include <drivers/clk.h>
#include <mm/core_memprot.h>
#include <stddef.h>
#include <stdint.h>
#include <tee_api_types.h>
#include <types_ext.h>

/* Tamper ident */
enum stm32_tamp_id {
	INT_TAMP1 = 0,
	INT_TAMP2,
	INT_TAMP3,
	INT_TAMP4,
	INT_TAMP5,
	INT_TAMP6,
	INT_TAMP7,
	INT_TAMP8,
	INT_TAMP9,
	INT_TAMP10,
	INT_TAMP11,
	INT_TAMP12,
	INT_TAMP13,
	INT_TAMP14,
	INT_TAMP15,
	INT_TAMP16,

	EXT_TAMP1,
	EXT_TAMP2,
	EXT_TAMP3,
	EXT_TAMP4,
	EXT_TAMP5,
	EXT_TAMP6,
	EXT_TAMP7,
	EXT_TAMP8,

	LAST_TAMP,
	INVALID_TAMP = 0xFFFF,
};

/* Callback return bitmask values */
#define TAMP_CB_ACK		BIT(0)
#define TAMP_CB_RESET		BIT(1)
#define TAMP_CB_ACK_AND_RESET	(TAMP_CB_RESET | TAMP_CB_ACK)

/*
 * Define number of backup registers in zone 1 and zone 2 (remaining are in
 * zone 3)
 *
 * backup registers in zone 1: read/write only in secure mode
 *                     zone 2: write only in secure mode, read in secure
 *                              and non-secure mode
 *                     zone 3: read/write in secure and non-secure mode
 *
 * Protection zone 1 if nb_zone1_regs == 0 no backup register are in zone 1
 *                   else backup registers from TAMP_BKP0R to TAMP_BKPxR
 *                   with x = nb_zone1_regs - 1 are in zone 1.
 * Protection zone 2 if nb_zone2_regs == 0 no backup register are in zone 2
 *                   else backup registers from
 *                   TAMP_BKPyR with y = nb_zone1_regs
 *                   to
 *                   TAMP_BKPzR with z = (nb_zone1_regs1 + nb_zone2_regs - 1)
 *                   are in zone 2.
 * Protection zone 3 backup registers from TAMP_BKPtR
 *                   with t = nb_zone1_regs1 + nb_zone2_regs to last backup
 *                   register are in zone 3.
 */
struct stm32_bkpregs_conf {
	uint32_t nb_zone1_regs;
	uint32_t nb_zone2_regs;
};

/* Define TAMPER modes */
#define TAMP_ERASE		0x0U
#define TAMP_NOERASE		BIT(1)
#define TAMP_NO_EVT_MASK	0x0U
#define TAMP_EVT_MASK		BIT(2)
#define TAMP_MODE_MASK		GENMASK_32(15, 0)

/*
 * stm32_tamp_write_mcounter: Increment monotonic counter[counter_idx].
 */
TEE_Result stm32_tamp_write_mcounter(int counter_idx);
uint32_t stm32_tamp_read_mcounter(int counter_idx);

bool stm32_tamp_are_secrets_blocked(void);
void stm32_tamp_block_secrets(void);
void stm32_tamp_unblock_secrets(void);
void stm32_tamp_erase_secrets(void);
void stm32_tamp_lock_boot_hardware_key(void);

/*
 * stm32_tamp_activate_tamp: Configure and activate one tamper (internal or
 * external).
 *
 * id: tamper id
 * mode: bitmask from TAMPER modes define:
 *       TAMP_ERASE/TAMP_NOERASE:
 *            TAMP_ERASE: when this tamp event raises secret will be erased.
 *            TAMP_NOERASE: when this event raises secured IPs will be locked
 *            until the acknowledge. If the callback confirms the TAMPER, it
 *            can manually erase secrets with stm32_tamp_erase_secrets().
 *       TAMP_NO_EVT_MASK/TAMP_EVT_MASK:
 *            TAMP_NO_EVT_MASK: normal behavior.
 *            TAMP_EVT_MASK: if the event is triggered, the event is masked and
 *            internally cleared by hardware. Secrets are not erased. Only
 *            applicable for EXT_TAMP1,2,3. This defines only the status at
 *            boot. To change mask while runtime stm32_tamp_set_mask() and
 *            stm32_tamp_unset_mask can be used.
 * callback: function to call when tamper is raised (cannot be NULL),
 *           called in interrupt context,
 *           Callback function returns a bitmask defining the action to take by
 *           the driver:
 *           TAMP_CB_RESET: will reset the board.
 *           TAMP_CB_ACK: this specific tamp is acknowledged (in case
 *           of no-erase tamper, blocked secret are unblocked).
 *
 * return: TEE_ERROR_BAD_PARAMETERS:
 *                   if 'id' is not a valid tamp id,
 *                   if callback is NULL,
 *                   if TAMP_EVT_MASK mode is set for a non supported 'id'.
 *         TEE_ERROR BAD_STATE
 *                   if driver wasn't previously initialized.
 *         TEE_ERROR ITEM_NOT_FOUND
 *                   if the activated external tamper wasn't previously
 *                   defined in the device tree.
 *         else TEE_SUCCESS.
 */
TEE_Result stm32_tamp_activate_tamp(enum stm32_tamp_id id, uint32_t mode,
				    uint32_t (*callback)(int id));

TEE_Result stm32_tamp_set_mask(enum stm32_tamp_id id);
TEE_Result stm32_tamp_unset_mask(enum stm32_tamp_id id);

/*
 * stm32_tamp_set_secure_bkprwregs: Configure backup registers zone.
 * registers in zone 1: read/write only in secure mode
 *              zone 2: write only in secure mode, read in secure and
 *                       non-secure mode
 *              zone 3: read/write in secure and non-secure mode
 *
 * bkpregs_conf: a pointer to struct bkpregs_conf that define the number of
 * registers in zone 1 and zone 2 (remaining backup registers will be in
 * zone 3).
 *
 * return TEE_ERROR_NOT_SUPPORTED:  if zone 1 and/or zone 2 definition are out
 *                                  of range.
 *        TEE_ERROR_BAD_PARAMETERS: if bkpregs_cond is NULL.
 *        TEE_SUCCESS             : if OK.
 */
TEE_Result stm32_tamp_set_secure_bkpregs(struct stm32_bkpregs_conf
					 *bkpregs_conf);

/*
 * stm32_tamp_set_config: Apply configuration.
 * Default one if no previous call to any of:
 * stm32_tamp_configure_passive()
 * stm32_tamp_configure_active()
 * stm32_tamp_configure_internal()
 * stm32_tamp_configure_external()
 * stm32_tamp_configure_secret_list()
 *
 */
TEE_Result stm32_tamp_set_config(void);

/* Compatibility tags */
#define TAMP_HAS_REGISTER_SECCFG	BIT(0)
#define TAMP_HAS_REGISTER_PRIVCFGR	BIT(1)
#define TAMP_HAS_REGISTER_ERCFGR	BIT(2)
#define TAMP_HAS_REGISTER_ATCR2		BIT(3)
#define TAMP_HAS_REGISTER_CR3		BIT(4)
#define TAMP_HAS_CR2_SECRET_STATUS	BIT(5)
#define TAMP_SIZE_ATCR1_ATCKSEL_IS_4	BIT(7)

struct stm32_tamp_compat {
	int nb_monotonic_counter;
	uint32_t tags;
	struct stm32_tamp_conf *int_tamp;
	uint32_t int_tamp_size;
	struct stm32_tamp_conf *ext_tamp;
	uint32_t ext_tamp_size;
	const struct stm32_tamp_pin_map *pin_map;
	uint32_t pin_map_size;
};

struct stm32_tamp_platdata {
	struct io_pa_va base;
	struct clk *clock;
	int it;
	uint32_t passive_conf;
	uint32_t active_conf;
	uint32_t pins_conf;
	uint32_t out_pins;
	bool is_wakeup_source;
	struct stm32_tamp_compat *compat;
};

TEE_Result stm32_tamp_get_platdata(struct stm32_tamp_platdata *pdata);

#endif /* __STM32_TAMP_H__ */
