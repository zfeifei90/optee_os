/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2018, STMicroelectronics - All Rights Reserved
 */

#ifndef STM32_TIMER_H
#define STM32_TIMER_H

enum timer_cal {
	HSI_CAL = 0,
	CSI_CAL
};

unsigned long stm32_timer_hsi_freq(void);
unsigned long stm32_timer_csi_freq(void);

/*
 * Get the timer frequence callback function for a target clock calibration
 * @timer_freq_cb - Output callback function
 * @type - Target clock calibration ID
 */
void stm32_timer_freq_func(unsigned long (**timer_freq_cb)(void),
			   enum timer_cal type);

#endif /* STM32_TIMER_H */
