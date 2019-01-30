global-incdirs-y += .

srcs-y += main.c
srcs-y += reset.S
srcs-y += shared_resources.c
srcs-$(CFG_DT) += stm32mp1_dt.c

subdirs-y += service
subdirs-y += drivers
subdirs-y += pm
