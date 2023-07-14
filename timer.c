/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "timer.h"
#include "ch.h"
#include "hal.h"
#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_bus.h"

// Settings
#define TIMER_HZ					14000000

void timer_init(void) {
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	uint16_t PrescalerValue = (uint16_t) (STM32_TIMCLK1 / TIMER_HZ) - 1;

	LL_TIM_DeInit(TIM2);

	LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetPrescaler(TIM2, PrescalerValue);
	LL_TIM_SetAutoReload(TIM2, 0xFFFFFFFF);

	TIM2->CNT = 0;
	LL_TIM_EnableCounter(TIM2);
	LL_TIM_GenerateEvent_UPDATE(TIM2);
}

uint32_t timer_time_now(void) {
	return TIM2->CNT;
}

float timer_seconds_elapsed_since(uint32_t time) {
	uint32_t diff = TIM2->CNT - time;
	return (float)diff / (float)TIMER_HZ;
}

/**
 * Blocking sleep based on timer.
 *
 * @param seconds
 * Seconds to sleep.
 */
void timer_sleep(float seconds) {
	uint32_t start_t = TIM2->CNT;

	for (;;) {
		if (timer_seconds_elapsed_since(start_t) >= seconds) {
			return;
		}
	}
}
