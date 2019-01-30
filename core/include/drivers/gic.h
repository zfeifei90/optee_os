/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2016, Linaro Limited
 * Copyright (c) 2014, STMicroelectronics International N.V.
 */

#ifndef __DRIVERS_GIC_H
#define __DRIVERS_GIC_H
#include <types_ext.h>
#include <kernel/interrupt.h>

/* Constants to categorize priorities */
#define GIC_HIGHEST_SEC_PRIORITY	0x0U
#define GIC_LOWEST_SEC_PRIORITY		0x7fU
#define GIC_HIGHEST_NS_PRIORITY		0x80U
#define GIC_LOWEST_NS_PRIORITY		0xfeU
/* 0xff would disable all interrupts */

#define GIC_DIST_REG_SIZE	0x10000
#define GIC_CPU_REG_SIZE	0x10000

#if !defined(CFG_WITH_ARM_TRUSTED_FW) && defined(CFG_ARM_GICV3)
#error CFG_ARM_GICV3 is not supported without CFG_WITH_ARM_TRUSTED_FW
#endif

struct gic_it_pm;

/*
 * Save and restore some interrupts configuration during low power sequences.
 * This is used on platforms using OP-TEE secure monitor.
 */
struct gic_pm {
	struct gic_it_pm *pm_cfg;
	size_t count;
};

struct gic_data {
	vaddr_t gicc_base;
	vaddr_t gicd_base;
	size_t max_it;
	struct itr_chip chip;
#if !defined(CFG_WITH_ARM_TRUSTED_FW)
	struct gic_pm pm;
#endif
};

/*
 * The two gic_init_* functions initializes the struct gic_data which is
 * then used by the other functions.
 */

void gic_init(struct gic_data *gd, vaddr_t gicc_base, vaddr_t gicd_base);
/* initial base address only */
void gic_init_base_addr(struct gic_data *gd, vaddr_t gicc_base,
			vaddr_t gicd_base);
/* initial cpu if only, mainly use for secondary cpu setup cpu interface */
void gic_cpu_init(struct gic_data *gd);

void gic_it_handle(struct gic_data *gd);

void gic_dump_state(struct gic_data *gd);

void gic_suspend(struct gic_data *gd);
void gic_resume(struct gic_data *gd);

#endif /*__DRIVERS_GIC_H*/
