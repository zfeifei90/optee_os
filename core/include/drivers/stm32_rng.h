/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2018, STMicroelectronics - All Rights Reserved
 */

#ifndef __STM32_RNG_H__
#define __STM32_RNG_H__

#include <stdint.h>
#include <stddef.h>

int stm32_rng_read(uint8_t *out, size_t size);

#endif /*__STM32_RNG__*/
