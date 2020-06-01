PLATFORM_FLAVOR ?= stm32mp157c
STM32_BOARD ?= ev1

# 1GB and 512MB DDR target do not locate secure DDR at the same place.
#
flavorlist-1G = stm32mp157c-ev1 stm32mp157c-ed1
flavorlist-512M = stm32mp157a-dk1 stm32mp157c-dk2

# Generic stm32mp1 configuration directives
#
include core/arch/arm/cpu/cortex-a7.mk
ta-targets = ta_arm32

$(call force,CFG_ARM32_core,y)
$(call force,CFG_BOOT_SECONDARY_REQUEST,y)
$(call force,CFG_GENERIC_BOOT,y)
$(call force,CFG_GIC,y)
$(call force,CFG_INIT_CNTVOFF,y)
$(call force,CFG_PM_STUBS,y)
$(call force,CFG_PSCI_ARM32,y)
$(call force,CFG_PM_ARM32,y)
$(call force,CFG_SECONDARY_INIT_CNTFRQ,y)
$(call force,CFG_SECURE_TIME_SOURCE_CNTPCT,y)
$(call force,CFG_WITH_SOFTWARE_PRNG,y)
$(call force,CFG_SM_PLATFORM_HANDLER,y)

ifneq ($(CFG_SECURE_DT),)
$(call force,CFG_DT,y)
$(call force,CFG_STATIC_SECURE_DT,y)
endif

CFG_TEE_CORE_NB_CORE ?= 2

ifneq (,$(filter $(CFG_SECURE_DT),$(flavorlist-512M)))
CFG_TZDRAM_START ?= 0xde000000
CFG_SHMEM_START  ?= 0xdfe00000
endif

CFG_TZSRAM_START ?= 0x2ffc0000
CFG_TZSRAM_SIZE  ?= 0x00040000
CFG_TZDRAM_START ?= 0xfe000000
CFG_TZDRAM_SIZE  ?= 0x01e00000
CFG_SHMEM_START  ?= 0xffe00000
CFG_SHMEM_SIZE   ?= 0x00200000

CFG_CORE_HEAP_SIZE ?= 49152
CFG_WITH_PAGER ?= y
CFG_WITH_LPAE ?= y
CFG_WITH_STACK_CANARIES ?= y
CFG_MMAP_REGIONS ?= 23

ifneq ($(CFG_DT),y)
# Some drivers mandate DT support
$(call force,CFG_STPMIC1,n)
$(call force,CFG_STM32_I2C,n)
$(call force,CFG_STM32_IWDG,n)
$(call force,CFG_STM32_RNG,n)
$(call force,CFG_STM32_TIMER,n)
$(call force,CFG_STM32_CLOCKSRC_CALIB,n)
endif

$(call force,CFG_STM32_BSEC,y)
$(call force,CFG_STM32_CRYP,y)
$(call force,CFG_STM32_ETZPC,y)
CFG_STM32_GPIO ?= y
CFG_STM32_I2C ?= y
CFG_STM32_IWDG ?= y
CFG_STM32_UART ?= y
CFG_STM32_RNG ?= y
$(call force,CFG_STM32_RTC,y)
CFG_STM32_TIMER ?= y

CFG_STPMIC1 ?= y

$(call force,CFG_STM32_BSEC_SIP,y)
$(call force,CFG_STM32_RCC_SIP,y)
CFG_STM32_PWR_SIP ?= y
CFG_STM32_POWER_SERVICES ?= y
CFG_STM32_CLOCKSRC_CALIB ?= y

ifeq ($(CFG_STPMIC1),y)
$(call force,CFG_STM32_I2C,y)
$(call force,CFG_STM32_GPIO,y)
endif

ifeq ($(CFG_STM32_CLOCKSRC_CALIB),y)
$(call force,CFG_STM32_TIMER,y)
endif

# Get a static mapping for the non secure low DDR (save 4kB of unpaged memory)
CFG_STM32MP_MAP_NSEC_LOW_DDR ?= y

# Default use stm32mp1 PM mailbox context version 2
# Use CFG_STM32MP15_PM_CONTEX_VERSION=1 to force version 0 when dealing with
# a TF-A firmware that supports version 1 of the context mailbox.
CFG_STM32MP15_PM_CONTEX_VERSION ?= 2

# Default enable some test facitilites
CFG_TEE_CORE_EMBED_INTERNAL_TESTS ?= y
CFG_WITH_STATS ?= y
CFG_UNWIND ?= n
# Non secure UART and GPIO/pinctrl for the output console
CFG_WITH_NSEC_GPIOS ?= y
CFG_WITH_NSEC_UARTS ?= y
CFG_FORCE_CONSOLE_ON_SUSPEND ?= n
# UART instance used for early console (0 disables early console)
CFG_STM32_EARLY_CONSOLE_UART ?= 4
