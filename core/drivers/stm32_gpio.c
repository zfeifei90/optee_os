// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2016-2018, STMicroelectronics - All Rights Reserved
 */

#include <assert.h>
#include <drivers/stm32_gpio.h>
#include <io.h>
#include <kernel/dt.h>
#include <kernel/generic_boot.h>
#include <kernel/panic.h>
#include <stdbool.h>
#include <stm32_util.h>
#include <stm32mp_dt.h>
#include <trace.h>
#include <util.h>

#ifdef CFG_DT
#include <libfdt.h>
#endif

#define DT_GPIO_BANK_SHIFT	12
#define DT_GPIO_BANK_MASK	GENMASK_32(16, 12)
#define DT_GPIO_PIN_SHIFT	8
#define DT_GPIO_PIN_MASK	GENMASK_32(11, 8)
#define DT_GPIO_MODE_MASK	GENMASK_32(7, 0)

static void get_gpio_cfg(uint32_t bank, uint32_t pin, struct gpio_cfg *cfg)
{
	uintptr_t base = stm32_get_gpio_bank_base(bank);
	int clock = stm32_get_gpio_bank_clock(bank);

	stm32_clock_enable((unsigned long)clock);

	cfg->moder = (mmio_read_32(base + GPIO_MODE_OFFSET) >> (pin << 1)) &
			GPIO_MODE_MAX;

	cfg->otyper = (mmio_read_32(base + GPIO_TYPE_OFFSET) >> pin) & 1;
	cfg->ospeedr = (mmio_read_32(base +  GPIO_SPEED_OFFSET) >> (pin << 1)) &
			GPIO_SPEED_MAX;
	cfg->pupdr = (mmio_read_32(base +  GPIO_PUPD_OFFSET) >> (pin << 1)) &
			GPIO_PULL_MAX;

	cfg->odr = (mmio_read_32(base + GPIO_ODR_OFFSET) >> (pin << 1)) & 1;

	if (pin < GPIO_ALT_LOWER_LIMIT) {
		cfg->afr = (mmio_read_32(base + GPIO_AFRL_OFFSET) >>
				(pin << 2)) & GPIO_ALTERNATE_MAX;
	} else {
		cfg->afr = (mmio_read_32(base + GPIO_AFRH_OFFSET) >>
				((pin - GPIO_ALT_LOWER_LIMIT) << 2)) &
				GPIO_ALTERNATE_MAX;
	}

	stm32_clock_disable((unsigned long)clock);
}

static void set_gpio_cfg(uint32_t bank, uint32_t pin, struct gpio_cfg *cfg)
{
	uintptr_t base = stm32_get_gpio_bank_base(bank);
	int clock = stm32_get_gpio_bank_clock(bank);

	stm32_clock_enable((unsigned long)clock);

	mmio_clrsetbits_32(base + GPIO_MODE_OFFSET,
			   GPIO_MODE_MAX << (pin << 1),
			   cfg->moder << (pin << 1));

	mmio_clrsetbits_32(base + GPIO_TYPE_OFFSET,
			   BIT(pin),
			   cfg->otyper << pin);

	mmio_clrsetbits_32(base + GPIO_SPEED_OFFSET,
			   GPIO_SPEED_MAX << (pin << 1),
			   cfg->ospeedr << (pin << 1));

	mmio_clrsetbits_32(base + GPIO_PUPD_OFFSET,
			   BIT(pin),
			   cfg->pupdr << (pin << 1));

	if (pin < GPIO_ALT_LOWER_LIMIT) {
		mmio_clrsetbits_32(base + GPIO_AFRL_OFFSET,
				   GPIO_ALTERNATE_MAX << (pin << 2),
				   cfg->afr << (pin << 2));
	} else {
		size_t shift = (pin - GPIO_ALT_LOWER_LIMIT) << 2;

		mmio_clrsetbits_32(base + GPIO_AFRH_OFFSET,
				   GPIO_ALTERNATE_MAX << shift,
				   cfg->afr << shift);
	}

	mmio_clrsetbits_32(base + GPIO_ODR_OFFSET,
			   BIT(pin), cfg->odr << pin);

	stm32_clock_disable((unsigned long)clock);
}

/*
 * stm32_gpio_set_output_level - Set level of an output GPIO instance
 *
 * @bank: GPIO bank
 * @pin: GPIO pin position in bank
 * @level: target level, either 0 (level low) or non zero (level high)
 */
void stm32_gpio_set_output_level(uint32_t bank, uint32_t pin, int level)
{
	uintptr_t base = stm32_get_gpio_bank_base(bank);
	int clock = stm32_get_gpio_bank_clock(bank);

	assert(pin <= GPIO_PIN_MAX);

	stm32_clock_enable((unsigned long)clock);

	if (level) {
		write32(BIT(pin), base + GPIO_BSRR_OFFSET);
	} else {
		write32(BIT(pin + 16), base + GPIO_BSRR_OFFSET);
	}

	stm32_clock_disable((unsigned long)clock);
}

/*
 * stm32_pinctrl_load_active_cfg - Load the PINCTRLs active configuration
 *
 * @pinctrl: array of PINCTRL configurations to apply
 * @cnt: number of elements in array pinctrl
 */
void stm32_pinctrl_load_active_cfg(struct stm32_pinctrl *pinctrl, size_t cnt)
{
	size_t n;

	for (n = 0; n < cnt; n++) {
		set_gpio_cfg(pinctrl[n].bank, pinctrl[n].pin,
			     &pinctrl[n].active_cfg);
	}
}

/*
 * stm32_pinctrl_load_standby_cfg - Load the PINCTRLs standby configuration
 *
 * @pinctrl: array of PINCTRL configurations to apply
 * @cnt: number of elements in array pinctrl
 */
void stm32_pinctrl_load_standby_cfg(struct stm32_pinctrl *pinctrl, size_t cnt)
{
	size_t n;

	for (n = 0; n < cnt; n++) {
		set_gpio_cfg(pinctrl[n].bank, pinctrl[n].pin,
			     &pinctrl[n].standby_cfg);
	}
}

/*
 * stm32_pinctrl_store_standby_cfg - Save current PINCTRLs config as standby
 *
 * @pinctrl: array of PINCTRL configurations to store
 * @cnt: number of elements in array pinctrl
 */
void stm32_pinctrl_store_standby_cfg(struct stm32_pinctrl *pinctrl, size_t cnt)
{
	size_t n;

	for (n = 0; n < cnt; n++) {
		get_gpio_cfg(pinctrl[n].bank, pinctrl[n].pin,
			     &pinctrl[n].standby_cfg);
	}
}

#ifdef CFG_DT
/* Return GPIO bank node if status is okay in DT, else return 0 */
static int ckeck_gpio_bank(void *fdt, uint32_t bank, int pinctrl_node)
{
	int pinctrl_subnode;

	fdt_for_each_subnode(pinctrl_subnode, fdt, pinctrl_node) {
		uint32_t bank_offset;
		const fdt32_t *cuint;

		if (fdt_getprop(fdt, pinctrl_subnode,
				"gpio-controller", NULL) == NULL) {
			continue;
		}

		cuint = fdt_getprop(fdt, pinctrl_subnode, "reg", NULL);
		if (cuint == NULL) {
			continue;
		}

		bank_offset = stm32_get_gpio_bank_offset(bank);

		if ((fdt32_to_cpu(*cuint) == bank_offset) &&
		    (_fdt_get_status(fdt,pinctrl_subnode) !=
		     DT_STATUS_DISABLED)) {
			return pinctrl_subnode;
		}
	}

	return 0;
}

static int get_pinctrl_from_fdt(void *fdt, int node,
				struct stm32_pinctrl *pinctrl, size_t count)
{
	const fdt32_t *cuint, *slewrate;
	int len;
	int pinctrl_node;
	int bank_node;
	int clk;
	uint32_t i;
	uint32_t speed = GPIO_SPEED_LOW;
	uint32_t pull = GPIO_NO_PULL;
	size_t found = 0;

	cuint = fdt_getprop(fdt, node, "pinmux", &len);
	if (cuint == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	pinctrl_node = fdt_parent_offset(fdt, fdt_parent_offset(fdt, node));
	if (pinctrl_node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	slewrate = fdt_getprop(fdt, node, "slew-rate", NULL);
	if (slewrate != NULL) {
		speed = fdt32_to_cpu(*slewrate);
	}

	if (fdt_getprop(fdt, node, "bias-pull-up", NULL) != NULL) {
		pull = GPIO_PULL_UP;
	} else if (fdt_getprop(fdt, node, "bias-pull-down", NULL) != NULL) {
		pull = GPIO_PULL_DOWN;
	} else {
		DMSG("No bias configured in node %d\n", node);
	}

	for (i = 0; i < ((uint32_t)len / sizeof(uint32_t)); i++) {
		uint32_t pincfg;
		uint32_t bank;
		uint32_t pin;
		uint32_t mode;
		uint32_t alternate = GPIO_ALTERNATE_0;

		pincfg = fdt32_to_cpu(*cuint);
		cuint++;

		bank = (pincfg & DT_GPIO_BANK_MASK) >> DT_GPIO_BANK_SHIFT;

		pin = (pincfg & DT_GPIO_PIN_MASK) >> DT_GPIO_PIN_SHIFT;

		mode = pincfg & DT_GPIO_MODE_MASK;

		switch (mode) {
		case 0:
			mode = GPIO_MODE_INPUT;
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		case 15:
		case 16:
			alternate = mode - 1U;
			mode = GPIO_MODE_ALTERNATE;
			break;
		case 17:
			mode = GPIO_MODE_ANALOG;
			break;
		default:
			mode = GPIO_MODE_OUTPUT;
			break;
		}

		if (fdt_getprop(fdt, node, "drive-open-drain", NULL) != NULL) {
			mode |= GPIO_OPEN_DRAIN;
		}

		bank_node = ckeck_gpio_bank(fdt, bank, pinctrl_node);
		if (bank_node == 0) {
			panic("PINCTRL inconsistent in DT");
		}

		clk = fdt_get_clock_id(fdt, bank_node);
		if (clk < 0) {
			return -FDT_ERR_NOTFOUND;
		}

		/* Platform knows the clock: assert it is okay */
		assert(clk == stm32_get_gpio_bank_clock(bank));
		stm32_clock_enable((unsigned int)clk);
		stm32_clock_disable((unsigned int)clk);

		if (found < count) {
			struct stm32_pinctrl *cfg = &pinctrl[found];

			cfg->bank = (uint8_t)bank;
			cfg->pin = (uint8_t)pin;
			cfg->active_cfg.moder = mode & ~GPIO_OPEN_DRAIN;
			cfg->active_cfg.otyper = mode & GPIO_OPEN_DRAIN ? 1 : 0;
			cfg->active_cfg.ospeedr = speed;
			cfg->active_cfg.pupdr = pull;
			cfg->active_cfg.odr = 0;
			cfg->active_cfg.afr = alternate;
			cfg->standby_cfg.moder = GPIO_MODE_ANALOG;
			cfg->standby_cfg.pupdr = GPIO_NO_PULL;
		}

		found++;
	}

	return (int)found;
}

/*
 * stm32_pinctrl_fdt_get_pinctrl - get PINCTRL config from a DT device node
 *
 * Argument cfg can be set to NULL or count to 0: the function will return only
 * the number of PINCTRL instances found in the device tree for the target
 * device.
 *
 * If more instances than count are found then the function returns the
 * effective number of PINCTRL found but will fill array cfg only according
 * to the value of count.
 *
 * @fdt: device tree
 * @node: device node in the device tree
 * @cfg: NULL or pointer to array of struct stm32_pinctrl
 * @count: number of elements pointed by argument cfg
 *
 * Return the number of PINCTRL instances found and a negative value on error
 */
int stm32_pinctrl_fdt_get_pinctrl(void *fdt, int device_node,
				  struct stm32_pinctrl *cfg, size_t count)
{
	const fdt32_t *cuint;
	int lenp;
	uint32_t i;
	size_t found = 0;

	cuint = fdt_getprop(fdt, device_node, "pinctrl-0", &lenp);
	if (cuint == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	for (i = 0; i < ((uint32_t)lenp / 4U); i++) {
		int node;
		int subnode;

		node = fdt_node_offset_by_phandle(fdt, fdt32_to_cpu(*cuint));
		if (node < 0) {
			return -FDT_ERR_NOTFOUND;
		}

		fdt_for_each_subnode(subnode, fdt, node) {
			size_t n;
			int rc;

			if (count > found) {
				n = count - found;
			} else {
				n = 0;
			}

			rc = get_pinctrl_from_fdt(fdt, subnode, &cfg[found], n);
			if (rc < 0) {
				return rc;
			}

			found += (size_t)rc;
		}

		cuint++;
	}

	return (int)found;
}
#endif /*CFG_DT*/

/*
 * stm32_gpio_set_secure_cfg - Set secure field of an output GPIO instance
 *
 * @bank: GPIO bank
 * @pin: GPIO pin position in bank
 * @secure: Secure field, either false (non secure) or true (secure)
 */
void stm32_gpio_set_secure_cfg(uint32_t bank, uint32_t pin, bool secure)
{
	uintptr_t base = stm32_get_gpio_bank_base(bank);
	int clock = stm32_get_gpio_bank_clock(bank);

	assert(pin <= GPIO_PIN_MAX);

	assert(!(secure && stm32mp_gpio_bank_is_non_secure(bank)));

	stm32_clock_enable((unsigned long)clock);

	if (secure) {
		mmio_setbits_32(base + GPIO_SECR_OFFSET, BIT(pin));
	} else {
		mmio_clrbits_32(base + GPIO_SECR_OFFSET, BIT(pin));
	}

	stm32_clock_disable((unsigned long)clock);
}
