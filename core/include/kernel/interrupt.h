/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2016-2019, Linaro Limited
 */
#ifndef __KERNEL_INTERRUPT_H
#define __KERNEL_INTERRUPT_H

#include <types_ext.h>
#include <sys/queue.h>
#include <util.h>

#define ITRF_TRIGGER_LEVEL	BIT(0)
#define ITRF_SHARED			BIT(1)

struct itr_chip {
	const struct itr_ops *ops;
	int (*dt_get_irq)(const uint32_t *properties, int len);
};

struct itr_ops {
	void (*add)(struct itr_chip *chip, size_t it, uint32_t flags);
	void (*enable)(struct itr_chip *chip, size_t it);
	void (*disable)(struct itr_chip *chip, size_t it);
	void (*raise_pi)(struct itr_chip *chip, size_t it);
	void (*raise_sgi)(struct itr_chip *chip, size_t it,
		uint8_t cpu_mask);
	void (*set_affinity)(struct itr_chip *chip, size_t it,
		uint8_t cpu_mask);
#if !defined(CFG_ARM_GICV3)
	uint8_t (*set_pmr)(struct itr_chip *chip, uint8_t mask);
	uint8_t (*set_ipriority)(struct itr_chip *chip, size_t it,
					uint8_t mask);
#endif
};

enum itr_return {
	ITRR_NONE,
	ITRR_HANDLED,
};

struct itr_handler {
	size_t it;
	uint32_t flags;
	enum itr_return (*handler)(struct itr_handler *h);
	void *data;
	SLIST_ENTRY(itr_handler) link;
};

void itr_init(struct itr_chip *data);
void itr_handle(size_t it);

#ifdef CFG_DT
/*
 * Get the DT interrupt property at @node. In the DT an interrupt
 *
 * @fdt reference to the Device Tree
 * @node is the node offset to read
 *
 * Returns the interrupt number if value >= 0
 * otherwise DT_INFO_INVALID_INTERRUPT
 */
int dt_get_irq(const void *fdt, int node);
#endif

void itr_add(struct itr_handler *handler);
void itr_enable(size_t it);
void itr_disable(size_t it);
/* raise the Peripheral Interrupt corresponding to the interrupt ID */
void itr_raise_pi(size_t it);
/*
 * raise the Software Generated Interrupt corresponding to the interrupt ID,
 * the cpu_mask represents which cpu interface to forward.
 */
void itr_raise_sgi(size_t it, uint8_t cpu_mask);
/*
 * let corresponding interrupt forward to the cpu interface
 * according to the cpu_mask.
 */
void itr_set_affinity(size_t it, uint8_t cpu_mask);

/*
 * __weak overridable function which is called when a secure interrupt is
 * received. The default function calls panic() immediately, platforms which
 * expects to receive secure interrupts should override this function.
 */
void itr_core_handler(void);

/*
 * Set the Priority Mask Regarding and return its previous value
 */
uint8_t itr_set_pmr(uint8_t mask);

/*
 * Set the targe tinterrupt priority mask and return its previous value
 */
uint8_t itr_set_ipriority(size_t it, uint8_t mask);

#endif /*__KERNEL_INTERRUPT_H*/
