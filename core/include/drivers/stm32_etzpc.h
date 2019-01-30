/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2014, ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2017-2018, STMicroelectronics
 */

#ifndef __STM32_ETZPC_H__
#define __STM32_ETZPC_H__

#include <io.h>
#include <stdint.h>

#define ETZPC_V1_0			0x10
#define ETZPC_V2_0			0x20

/*
 * Define security level for each peripheral (DECPROT)
 */
enum etzpc_decprot_attributes {
	TZPC_DECPROT_S_RW = 0,
	TZPC_DECPROT_NS_R_S_W = 1,
	TZPC_DECPROT_MCU_ISOLATION = 2,
	TZPC_DECPROT_NS_RW = 3,
	TZPC_DECPROT_MAX = 4,
};

/*
 * etzpc_configure_decprot : Load a DECPROT configuration
 * decprot_id : ID of the IP
 * decprot_attr : Restriction access attributes
 */
void etzpc_configure_decprot(uint32_t decprot_id,
			     enum etzpc_decprot_attributes decprot_attr);

/*
 * etzpc_get_decprot : Get the DECPROT attribute
 * decprot_id : ID of the IP
 * return : Attribute of this DECPROT
 */
enum etzpc_decprot_attributes etzpc_get_decprot(uint32_t decprot_id);

/*
 * etzpc_lock_decprot : Lock access to the DECPROT attributes
 * decprot_id : ID of the IP
 */
void etzpc_lock_decprot(uint32_t decprot_id);

/*
 * etzpc_configure_tzma : Configure the target TZMA read only size
 * tzma_id : ID of the memory
 * tzma_value : read-only size
 */
void etzpc_configure_tzma(uint32_t tzma_id, uint16_t tzma_value);

/*
 * etzpc_get_tzma : Get the target TZMA read only size
 * tzma_id : TZMA ID
 * return : Size of read only size
 */
uint16_t etzpc_get_tzma(uint32_t tzma_id);

/*
 * etzpc_lock_tzma : Lock the target TZMA
 * tzma_id : TZMA ID
 */
void etzpc_lock_tzma(uint32_t tzma_id);

/*
 * etzpc_get_lock_tzma : Return the lock status of the target TZMA
 * tzma_id : TZMA ID
 * return : True if TZMA is locked, false otherwise
 */
bool etzpc_get_lock_tzma(uint32_t tzma_id);

#endif /*__STM32_ETZPC_H__*/
