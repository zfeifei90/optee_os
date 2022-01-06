/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2018-2021, STMicroelectronics
 */

#ifndef __STM32_RNG_H__
#define __STM32_RNG_H__

#include <drivers/clk.h>
#include <mm/core_memprot.h>
#include <stdint.h>
#include <stddef.h>
#include <tee_api_types.h>
#include <types_ext.h>

struct stm32_rng_platdata {
	struct io_pa_va base;
	struct clk *clock;
	unsigned long reset;
	unsigned int lock;
	bool clock_error;
};

#ifdef CFG_WITH_SOFTWARE_PRNG
/*
 * Fill buffer with bytes from the STM32_RNG
 * @out: Output buffer
 * @size: Byte size of the output buffer
 * Return a TEE_Result compliant sttus
 */
TEE_Result stm32_rng_read(void *buf, size_t blen);
#endif

/* Set RNG platform data the driver shall initialize with */
TEE_Result stm32_rng_get_platdata(struct stm32_rng_platdata *pdata);
#endif /*__STM32_RNG_H__*/
