/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2015-2018, STMicroelectronics - All Rights Reserved
 */

#ifndef __STM32_GPIO_H__
#define __STM32_GPIO_H__

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define GPIO_MODE_OFFSET	0x00U
#define GPIO_TYPE_OFFSET	0x04U
#define GPIO_SPEED_OFFSET	0x08U
#define GPIO_PUPD_OFFSET	0x0CU
#define GPIO_ODR_OFFSET		0x14U
#define GPIO_BSRR_OFFSET	0x18U
#define GPIO_AFRL_OFFSET	0x20U
#define GPIO_AFRH_OFFSET	0x24U
#define GPIO_SECR_OFFSET	0x30U

#define GPIO_ALT_LOWER_LIMIT	0x08U

#define GPIO_PIN_0		0x00U
#define GPIO_PIN_1		0x01U
#define GPIO_PIN_2		0x02U
#define GPIO_PIN_3		0x03U
#define GPIO_PIN_4		0x04U
#define GPIO_PIN_5		0x05U
#define GPIO_PIN_6		0x06U
#define GPIO_PIN_7		0x07U
#define GPIO_PIN_8		0x08U
#define GPIO_PIN_9		0x09U
#define GPIO_PIN_10		0x0AU
#define GPIO_PIN_11		0x0BU
#define GPIO_PIN_12		0x0CU
#define GPIO_PIN_13		0x0DU
#define GPIO_PIN_14		0x0EU
#define GPIO_PIN_15		0x0FU
#define GPIO_PIN_MAX		GPIO_PIN_15

#define GPIO_ALTERNATE_0	0x00
#define GPIO_ALTERNATE_1	0x01
#define GPIO_ALTERNATE_2	0x02
#define GPIO_ALTERNATE_3	0x03
#define GPIO_ALTERNATE_4	0x04
#define GPIO_ALTERNATE_5	0x05
#define GPIO_ALTERNATE_6	0x06
#define GPIO_ALTERNATE_7	0x07
#define GPIO_ALTERNATE_8	0x08
#define GPIO_ALTERNATE_9	0x09
#define GPIO_ALTERNATE_10	0x0A
#define GPIO_ALTERNATE_11	0x0B
#define GPIO_ALTERNATE_12	0x0C
#define GPIO_ALTERNATE_13	0x0D
#define GPIO_ALTERNATE_14	0x0E
#define GPIO_ALTERNATE_15	0x0F
#define GPIO_ALTERNATE_MAX	0x0FU

#define GPIO_MODE_INPUT		0x00
#define GPIO_MODE_OUTPUT	0x01
#define GPIO_MODE_ALTERNATE	0x02
#define GPIO_MODE_ANALOG	0x03
#define GPIO_MODE_MAX		0x03U

#define GPIO_OPEN_DRAIN		0x10U

#define GPIO_SPEED_LOW		0x00
#define GPIO_SPEED_MEDIUM	0x01
#define GPIO_SPEED_HIGH		0x02
#define GPIO_SPEED_VERY_HIGH	0x03
#define GPIO_SPEED_MAX		0x03U

#define GPIO_NO_PULL		0x00
#define GPIO_PULL_UP		0x01
#define GPIO_PULL_DOWN		0x02
#define GPIO_PULL_MAX		0x03U

struct gpio_cfg {
	uint16_t moder:		2;
	uint16_t otyper:	1;
	uint16_t ospeedr:	2;
	uint16_t pupdr:		2;
	uint16_t odr:		1;
	uint16_t afr:		4;
};

struct stm32_pinctrl {
	uint8_t bank;
	uint8_t pin;
	struct gpio_cfg active_cfg;
	struct gpio_cfg standby_cfg;
};

void stm32_pinctrl_load_active_cfg(struct stm32_pinctrl *pinctrl, size_t cnt);
void stm32_pinctrl_load_standby_cfg(struct stm32_pinctrl *pinctrl, size_t cnt);
void stm32_pinctrl_store_standby_cfg(struct stm32_pinctrl *pinctrl, size_t cnt);

int stm32_pinctrl_fdt_get_pinctrl(void *fdt, int node,
				  struct stm32_pinctrl *cfg, size_t count);

void stm32_gpio_set_output_level(uint32_t bank, uint32_t pin, int high);

void stm32_gpio_set_secure_cfg(uint32_t bank, uint32_t pin, bool secure);

#endif /*__STM32_GPIO_H__*/
