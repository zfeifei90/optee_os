// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2017-2019, STMicroelectronics
 * Copyright (c) 2016-2018, Linaro Limited
 */

#include <arm.h>
#include <boot_api.h>
#include <console.h>
#include <drivers/gic.h>
#include <drivers/stm32_bsec.h>
#include <drivers/stm32_etzpc.h>
#include <drivers/stm32_iwdg.h>
#include <drivers/stm32_uart.h>
#include <drivers/stm32mp1_clk.h>
#include <drivers/stm32mp1_pmic.h>
#include <drivers/stm32mp1_rcc.h>
#include <drivers/stpmic1.h>
#include <dt-bindings/etzpc/stm32-etzpc.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <io.h>
#include <keep.h>
#include <kernel/generic_boot.h>
#include <kernel/interrupt.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/pm_stubs.h>
#include <kernel/spinlock.h>
#include <mm/core_mmu.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <sm/optee_smc.h>
#include <sm/psci.h>
#include <tee/arch_svc.h>
#include <tee/entry_std.h>
#include <tee/entry_fast.h>
#include <stdint.h>
#include <stm32_util.h>
#include <stm32mp_dt.h>
#include <stm32mp_pm.h>
#include <string.h>
#include <trace.h>
#include <util.h>

#ifdef CFG_WITH_NSEC_GPIOS
register_phys_mem(MEM_AREA_IO_NSEC, GPIOS_NSEC_BASE, GPIOS_NSEC_SIZE);
#endif
register_phys_mem(MEM_AREA_IO_NSEC, RNG1_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_NSEC, IWDG1_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_NSEC, IWDG2_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_NSEC, RTC_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_NSEC, SYSCFG_BASE, SMALL_PAGE_SIZE);
#ifdef CFG_WITH_NSEC_UARTS
register_phys_mem(MEM_AREA_IO_NSEC, USART1_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_NSEC, USART2_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_NSEC, USART3_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_NSEC, UART4_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_NSEC, UART5_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_NSEC, USART6_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_NSEC, UART7_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_NSEC, UART8_BASE, SMALL_PAGE_SIZE);
#endif

register_phys_mem(MEM_AREA_IO_SEC, GIC_IOMEM_BASE, GIC_IOMEM_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, TAMP_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, RCC_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, PWR_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, ETZPC_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, BSEC_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, I2C4_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, STGEN_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, GPIOZ_BASE,
		  STM32MP1_GPIOZ_MAX_COUNT * SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, RNG1_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, IWDG1_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, RTC_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, DDRCTRL_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, DDRPHYC_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, BKPSRAM_BASE, SMALL_PAGE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, USART1_BASE, SMALL_PAGE_SIZE);

register_phys_mem(MEM_AREA_ROM_SEC, TEE_RAM_START, TEE_RAM_PH_SIZE);

register_ddr(DDR_BASE, STM32MP1_DDR_SIZE_DFLT);

#ifdef CFG_STM32MP_MAP_NSEC_LOW_DDR
register_phys_mem(MEM_AREA_RAM_NSEC, DDR_BASE, SMALL_PAGE_SIZE);
#endif

#define _ID2STR(id)		(#id)
#define ID2STR(id)		_ID2STR(id)

static TEE_Result platform_banner(void)
{
#ifdef CFG_DT
	IMSG("Platform stm32mp1: flavor %s - device tree %s",
		ID2STR(PLATFORM_FLAVOR), ID2STR(CFG_SECURE_DT));
	IMSG("Model: %s", fdt_get_board_model(get_dt_blob()));
#else
	IMSG("Platform stm32mp1: flavor %s - no device tree",
		ID2STR(PLATFORM_FLAVOR));
#endif

	return TEE_SUCCESS;
}
service_init(platform_banner);

static void main_fiq(void);

static const struct thread_handlers handlers = {
	.std_smc = tee_entry_std,
	.fast_smc = tee_entry_fast,
	.nintr = main_fiq,
	.cpu_on = pm_panic,
	.cpu_off = pm_panic,
	.cpu_suspend = pm_panic,
	.cpu_resume = pm_panic,
	.system_off = pm_panic,
	.system_reset = pm_panic,
};

const struct thread_handlers *generic_boot_get_handlers(void)
{
	return &handlers;
}

/*
 * Console
 *
 * We cannot use the generic serial_console support since probing
 * the console requires the platform clock driver to be already
 * up and ready which is done only once service_init are completed.
 *
 * If the console uses a non secure UART, its clock might have been
 * disabled by the non secure world, in which case secure traces must
 * be dropped unless what the driver may stall since the UART FIFO is
 * never emptied.
 */
#define CONSOLE_WITHOUT_CLOCK_MAGIC	~0UL

static struct stm32_uart_pdata console_data;
static struct serial_chip *serial_console __maybe_unused;

void console_init(void)
{
	/* Early console initialization before MMU setup */
	struct uart {
		uintptr_t pa;
		bool secure;
	} uarts[] = {
		[0] = { .pa = 0 },
		[1] = { .pa = USART1_BASE, .secure = true, },
		[2] = { .pa = USART2_BASE, .secure = false, },
		[3] = { .pa = USART3_BASE, .secure = false, },
		[4] = { .pa = UART4_BASE, .secure = false, },
		[5] = { .pa = UART5_BASE, .secure = false, },
		[6] = { .pa = USART6_BASE, .secure = false, },
		[7] = { .pa = UART7_BASE, .secure = false, },
		[8] = { .pa = UART8_BASE, .secure = false, },
	};

	COMPILE_TIME_ASSERT(CFG_STM32_EARLY_CONSOLE_UART < ARRAY_SIZE(uarts));
	assert(!cpu_mmu_enabled());

	if (!uarts[CFG_STM32_EARLY_CONSOLE_UART].pa) {
		return;
	}

	/* No clock/PINCTRL yet bound to the UART console */
	console_data.clock = CONSOLE_WITHOUT_CLOCK_MAGIC;

	console_data.secure = uarts[CFG_STM32_EARLY_CONSOLE_UART].secure;
	stm32_uart_init(&console_data, uarts[CFG_STM32_EARLY_CONSOLE_UART].pa);

#ifdef CFG_FORCE_CONSOLE_ON_SUSPEND
	serial_console = &console_data.chip;
#else
	register_serial_console(&console_data.chip);
#endif

	IMSG("Early console on UART#%u", CFG_STM32_EARLY_CONSOLE_UART);
}

#ifdef CFG_FORCE_CONSOLE_ON_SUSPEND
void console_putc(int ch)
{
	if (!serial_console)
		return;

	stm32_pinctrl_load_active_cfg(console_data.pinctrl,
				      console_data.pinctrl_count);

        if (ch == '\n')
                serial_console->ops->putc(serial_console, '\r');

	serial_console->ops->putc(serial_console, ch);
}

void console_flush(void)
{
	if (!serial_console)
		return;

	serial_console->ops->flush(serial_console);
}
#endif

#ifdef CFG_DT
/* Probe console once clocks inits (service_init level) are completed */
static TEE_Result stm32_uart_console_probe(void)
{
	void *fdt;
	int node;
	struct stm32_uart_pdata *pd = NULL;

	fdt = get_dt_blob();
	if (fdt == NULL) {
		panic();
	}

	node = fdt_get_stdout_node_offset(fdt);
	if (node >= 0) {
		pd = probe_uart_from_dt_node(fdt, node);
	}

	if (pd) {
		serial_console = NULL;
		dmb();
		memcpy(&console_data, pd, sizeof(*pd));
		dmb();
		serial_console = &console_data.chip;
		free(pd);
		IMSG("UART console probed from DT (%ssecure)",
			pd->secure ? "" : "non ");
	} else {
		IMSG("No UART console probed from DT");
		serial_console = NULL;
	}

	return TEE_SUCCESS;
}
service_init_late(stm32_uart_console_probe);

/* Compute PLL1 settings once PMIC init is completed */
static TEE_Result initialize_pll1_settings(void)
{
	uint32_t vddcore_voltage = 0U;
	int ret;

	if (stm32mp1_clk_pll1_settings_are_valid()) {
		return TEE_SUCCESS;
	}

	if (stm32mp_dt_pmic_status() > 0) {
		stm32mp_get_pmic();

		ret = stpmic1_regulator_voltage_get("buck1");
		if (ret < 0) {
			panic();
		}

		vddcore_voltage = (uint32_t)ret;

		stm32mp_put_pmic();
	}

	if (stm32mp1_clk_compute_all_pll1_settings(vddcore_voltage) != 0) {
		panic();
	}

	return TEE_SUCCESS;
}
driver_init_late(initialize_pll1_settings);

#endif

/*
 * GIC init, used also for primary/secondary boot core wake completion
 */
static struct gic_data gic_data;

static void main_fiq(void)
{
	gic_it_handle(&gic_data);
}

void main_init_gic(void)
{
	void *gicc_base;
	void *gicd_base;

	gicc_base = phys_to_virt(GIC_IOMEM_BASE + GICC_OFFSET, MEM_AREA_IO_SEC);
	gicd_base = phys_to_virt(GIC_IOMEM_BASE + GICD_OFFSET, MEM_AREA_IO_SEC);
	if (!gicc_base || !gicd_base)
		panic();

	gic_init(&gic_data, (vaddr_t)gicc_base, (vaddr_t)gicd_base);
	itr_init(&gic_data.chip);

#ifdef CFG_PSCI_ARM32
	stm32mp_register_online_cpu();
#endif
}

void main_secondary_init_gic(void)
{
	gic_cpu_init(&gic_data);

#if CFG_TEE_CORE_NB_CORE == 2
	/* Secondary core release constraint on APB5 clock */
	write32(BOOT_API_A7_RESET_MAGIC_NUMBER,
		stm32mp_bkpreg(BCKR_CORE1_MAGIC_NUMBER));
	stm32mp1_clk_disable_secure(RTCAPB);
#endif

#ifdef CFG_PSCI_ARM32
	stm32mp_register_online_cpu();
#endif
}

/* Specific GIC suspend/resume function called aside registered PM handlers */
void stm32mp_gic_suspend_resume(enum pm_op op)
{
	switch (op) {
	case PM_OP_SUSPEND:
		gic_suspend(&gic_data);
		break;
	case PM_OP_RESUME:
		gic_resume(&gic_data);
		break;
	default:
		panic();
	}
}
KEEP_PAGER(stm32mp_gic_suspend_resume);

/* stm32mp1 low power sequence needs straight access to GIC */
uintptr_t get_gicc_base(void)
{
	uintptr_t pbase = GIC_IOMEM_BASE + GICC_OFFSET;

	if (cpu_mmu_enabled())
		return (uintptr_t)phys_to_virt(pbase, MEM_AREA_IO_SEC);

	return pbase;
}

uintptr_t get_gicd_base(void)
{
	uintptr_t pbase = GIC_IOMEM_BASE + GICD_OFFSET;

	if (cpu_mmu_enabled())
		return (uintptr_t)phys_to_virt(pbase, MEM_AREA_IO_SEC);

	return pbase;
}

/*
 * Various platform specific functions
 */
unsigned int stm32mp_get_otp_max(void)
{
	return STM32MP1_OTP_MAX_ID;
}
unsigned int stm32mp_get_otp_upper_start(void)
{
	return STM32MP1_UPPER_OTP_START;
}

bool __weak stm32mp_with_pmic(void)
{
	return false;
}

/*
 * SIP and other platform specific services
 */
bool sm_platform_handler(struct sm_ctx *ctx)
{
	uint32_t *a0 = (uint32_t *)(&ctx->nsec.r0);
	uint32_t *a1 = (uint32_t *)(&ctx->nsec.r1);
	uint32_t *a2 = (uint32_t *)(&ctx->nsec.r2);
	uint32_t *a3 = (uint32_t *)(&ctx->nsec.r3);

	if (!OPTEE_SMC_IS_FAST_CALL(*a0))
		return true;

	switch (OPTEE_SMC_OWNER_NUM(*a0)) {
	case OPTEE_SMC_OWNER_SIP:
		return stm32_sip_service(ctx, a0, a1, a2, a3);
	case OPTEE_SMC_OWNER_OEM:
		return stm32_oem_service(ctx, a0, a1, a2, a3);
	default:
		return true;
	}
}

static uintptr_t stm32_dbgmcu_base(void)
{
	static void *va;

	if (!cpu_mmu_enabled())
		return DBGMCU_BASE;

	if (!va)
		va = phys_to_virt(DBGMCU_BASE, MEM_AREA_IO_SEC);

	return (uintptr_t)va;
}

/* SoC versioning util */
int stm32mp1_dbgmcu_get_chip_version(uint32_t *chip_version)
{
	assert(chip_version != NULL);

	*chip_version = (read32(stm32_dbgmcu_base() + DBGMCU_IDC) &
			 DBGMCU_IDC_REV_ID_MASK) >> DBGMCU_IDC_REV_ID_SHIFT;

	return 0;
}

/* SoC device ID util */
int stm32mp1_dbgmcu_get_chip_dev_id(uint32_t *chip_dev_id)
{
	assert(chip_dev_id != NULL);

	*chip_dev_id = read32(stm32_dbgmcu_base() + DBGMCU_IDC) &
		       DBGMCU_IDC_DEV_ID_MASK;

	return 0;
}

static uintptr_t stm32_tamp_base(void)
{
	static void *va;

	if (!cpu_mmu_enabled())
		return TAMP_BASE;

	if (!va)
		va = phys_to_virt(TAMP_BASE, MEM_AREA_IO_SEC);

	return (uintptr_t)va;
}

static uintptr_t bkpreg_base(void)
{
	return stm32_tamp_base() + TAMP_BKP_REGISTER_OFF;
}

uintptr_t stm32mp_bkpreg(unsigned int idx)
{
	return bkpreg_base() + (idx * sizeof(uint32_t));
}

uintptr_t stm32mp1_bkpsram_base(void)
{
	static void *va;

	if (!cpu_mmu_enabled())
		return BKPSRAM_BASE;

	if (!va)
		va = phys_to_virt(BKPSRAM_BASE, MEM_AREA_IO_SEC);

	return (uintptr_t)va;
}

uintptr_t stm32_get_stgen_base(void)
{
	static uintptr_t va;

	if (!cpu_mmu_enabled())
		return STGEN_BASE;

	if (!va)
		va = (uintptr_t)phys_to_virt(STGEN_BASE, MEM_AREA_IO_SEC);

	return va;
}

uintptr_t stm32_get_syscfg_base(void)
{
	static uintptr_t va;

	if (!cpu_mmu_enabled())
		return SYSCFG_BASE;

	if (!va)
		va = (uintptr_t)phys_to_virt(SYSCFG_BASE, MEM_AREA_IO_NSEC);

	return va;
}

uintptr_t stm32_get_gpio_bank_base(unsigned int bank)
{
	/* Non secure banks and mapped together, same for secure banks */
	static uintptr_t gpiox_va;
	static uintptr_t gpioz_va;

	switch (bank) {
	case GPIO_BANK_A:
	case GPIO_BANK_B:
	case GPIO_BANK_C:
	case GPIO_BANK_D:
	case GPIO_BANK_E:
	case GPIO_BANK_F:
	case GPIO_BANK_G:
	case GPIO_BANK_H:
	case GPIO_BANK_I:
	case GPIO_BANK_J:
	case GPIO_BANK_K:
		if (!gpiox_va && cpu_mmu_enabled())
			gpiox_va = (uintptr_t)phys_to_virt(GPIOS_NSEC_BASE,
							   MEM_AREA_IO_NSEC);

		if (cpu_mmu_enabled())
			return gpiox_va + (bank * GPIO_BANK_OFFSET);

		return GPIOS_NSEC_BASE + (bank * GPIO_BANK_OFFSET);

	case GPIO_BANK_Z:
		if (!gpioz_va && cpu_mmu_enabled())
			gpioz_va = (uintptr_t)phys_to_virt(GPIOZ_BASE,
							   MEM_AREA_IO_SEC);

		if (cpu_mmu_enabled())
			return gpioz_va;

		return GPIOZ_BASE;
	default:
		panic();
	}
}

uint32_t stm32_get_gpio_bank_offset(unsigned int bank)
{
	if (bank == GPIO_BANK_Z) {
		return 0;
	} else {
		return bank * GPIO_BANK_OFFSET;
	}
}

/* Return clock ID on success, negative value on error */
int stm32_get_gpio_bank_clock(unsigned int bank)
{
	switch (bank) {
	case GPIO_BANK_A:
	case GPIO_BANK_B:
	case GPIO_BANK_C:
	case GPIO_BANK_D:
	case GPIO_BANK_E:
	case GPIO_BANK_F:
	case GPIO_BANK_G:
	case GPIO_BANK_H:
	case GPIO_BANK_I:
	case GPIO_BANK_J:
	case GPIO_BANK_K:
		return (int)GPIOA + (bank - GPIO_BANK_A);
	case GPIO_BANK_Z:
		return (int)GPIOZ;
	default:
		panic();
	}
}

int stm32mp_iwdg_irq2instance(size_t irq)
{
	int instance = (int)irq - STM32MP1_IRQ_IWDG1;

	assert((instance >= IWDG1_INST) && (instance <= IWDG2_INST));
	return instance;
}

size_t stm32mp_iwdg_instance2irq(int instance)
{
	return (size_t)(instance + STM32MP1_IRQ_IWDG1);
}

unsigned int stm32mp_iwdg_iomem2instance(uintptr_t pbase)
{
	switch (pbase) {
	case IWDG1_BASE:
		return IWDG1_INST;
	case IWDG2_BASE:
		return IWDG2_INST;
	default:
		panic();
	}
}

unsigned long stm32_get_iwdg_otp_config(uintptr_t pbase)
{
	unsigned int idx;
	unsigned long iwdg_cfg = 0;
	uint32_t otp;
	uint32_t otp_value;

	idx = stm32mp_iwdg_iomem2instance(pbase);

	if (bsec_find_otp_name_in_nvmem_layout(HW2_OTP, &otp, NULL))
		panic();

	if (bsec_read_otp(&otp_value, otp))
		panic();

	if (otp_value & BIT(idx + HW2_OTP_IWDG_HW_ENABLE_SHIFT))
		iwdg_cfg |= IWDG_HW_ENABLED;

	if (otp_value & BIT(idx + HW2_OTP_IWDG_FZ_STOP_SHIFT))
		iwdg_cfg |= IWDG_DISABLE_ON_STOP;

	if (otp_value & BIT(idx + HW2_OTP_IWDG_FZ_STANDBY_SHIFT))
		iwdg_cfg |= IWDG_DISABLE_ON_STANDBY;

	return iwdg_cfg;
}

static int get_part_number(uint32_t *part_nb)
{
	uint32_t part_number;
	uint32_t dev_id;
	uint32_t otp;
	uint32_t otp_value;
	size_t bit_len;

	assert(part_nb != NULL);

	if (stm32mp1_dbgmcu_get_chip_dev_id(&dev_id) != 0) {
		return -1;
	}

	if (bsec_find_otp_name_in_nvmem_layout(PART_NUMBER_OTP, &otp,
					       &bit_len) != BSEC_OK) {
		return -1;
	}

	assert(bit_len == 8);

	if (bsec_read_otp(&part_number, otp) != BSEC_OK) {
		return -1;
	}

	part_number = (part_number & PART_NUMBER_OTP_PART_MASK) >>
		      PART_NUMBER_OTP_PART_SHIFT;

	*part_nb = part_number | (dev_id << 16);

	return 0;
}

bool stm32mp_supports_cpu_opp(uint32_t opp_id)
{
	uint32_t part_number;
	uint32_t id;

	if (get_part_number(&part_number) != 0) {
		DMSG("Cannot get part number");
		panic();
	}

	switch (opp_id) {
	case PLAT_OPP_ID1:
	case PLAT_OPP_ID2:
		id = opp_id;
		break;
	default:
		return false;
	}

	switch (part_number) {
	case STM32MP157F_PART_NB:
	case STM32MP157D_PART_NB:
	case STM32MP153F_PART_NB:
	case STM32MP153D_PART_NB:
	case STM32MP151F_PART_NB:
	case STM32MP151D_PART_NB:
		return true;
	default:
		return id == PLAT_OPP_ID1;
	}
}

uintptr_t stm32mp_get_etzpc_base(void)
{
	return ETZPC_BASE;
}

/*
 * This function allows to split bindings between platform and ETZPC
 * HW mapping. If this conversion was done at driver level, the driver
 * should include all supported platform bindings. ETZPC may be used on
 * other platforms.
 */
enum etzpc_decprot_attributes stm32mp_etzpc_binding2decprot(uint32_t mode)
{
	switch (mode) {
	case DECPROT_S_RW:
		return TZPC_DECPROT_S_RW;
	case DECPROT_NS_R_S_W:
		return TZPC_DECPROT_NS_R_S_W;
	case DECPROT_MCU_ISOLATION:
		return TZPC_DECPROT_MCU_ISOLATION;
	case DECPROT_NS_RW:
		return TZPC_DECPROT_NS_RW;
	default:
		panic();
	}
}

#ifdef CFG_STM32_RTC
/*******************************************************************************
 * This function determines the number of needed RTC calendar read operations
 * to get consistent values (1 or 2 depending of clock frequencies).
 * If APB1 frequency is less than 7 times the RTC one, the software has to
 * read the calendar time and date register twice.
 * This function computes each of them and does the comparison.
 * Returns true if read twice is needed, false else.
 ******************************************************************************/
bool stm32_rtc_get_read_twice(void)
{
	unsigned long apb1_freq;
	unsigned long rtc_freq = 0;
	uint32_t apb1_div;
	uintptr_t rcc_base = stm32_rcc_base();

	/* Compute RTC frequency */
	switch ((mmio_read_32(rcc_base + RCC_BDCR) &
		 RCC_BDCR_RTCSRC_MASK) >> RCC_BDCR_RTCSRC_SHIFT) {
	case 1:
		rtc_freq = stm32mp1_clk_get_rate(CK_LSE);
		break;
	case 2:
		rtc_freq = stm32mp1_clk_get_rate(CK_LSI);
		break;
	case 3:
		rtc_freq = stm32mp1_clk_get_rate(CK_HSE);
		rtc_freq /= (mmio_read_32(rcc_base + RCC_RTCDIVR) &
			     RCC_DIVR_DIV_MASK) + 1U;
		break;
	default:
		/* Forbidden values: consider only one needed read here */
		panic();
	}

	/* Compute APB1 frequency */
	apb1_div = mmio_read_32(rcc_base + RCC_APB1DIVR) & RCC_APBXDIV_MASK;
	apb1_freq = stm32mp1_clk_get_rate(CK_MCU) >> apb1_div;

	/* Compare APB1 and 7*RTC frequencies */
	return apb1_freq < (rtc_freq * 7U);
}
#endif

uint32_t may_spin_lock(unsigned int *lock)
{
	if (!lock || !cpu_mmu_enabled()) {
		return 0;
	}

	return cpu_spin_lock_xsave(lock);
}

void may_spin_unlock(unsigned int *lock, uint32_t exceptions)
{
	if (!lock || !cpu_mmu_enabled()) {
		return;
	}

	cpu_spin_unlock_xrestore(lock, exceptions);
}
