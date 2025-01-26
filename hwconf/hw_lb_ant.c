/*
	Copyright 2019 - 2020 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC DCDC firmware.

	The VESC DCDC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC DCDC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ch.h"
#include "hal.h"

#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_rcc.h"

#include "crc.h"
#include "terminal.h"
#include "commands.h"
#include "utils.h"

#include <string.h>

#define TIMER_MAX 			0xFFFF
#define TIMER_FREQ			168000000

#define HIGH				(TIMER_FREQ / 105000)
#define MID					(TIMER_FREQ / 100000)
#define LOW					(TIMER_FREQ / 95000)

#define CUT_LOW				(LOW + 50)
#define CUT_HIGH			(HIGH - 50)
#define IN_RANGE(X)			((X) < CUT_LOW && (X) > CUT_HIGH)

#define WINDOW_SIZE			40
#define WINDOW_VALID		15
#define THRS				(((MID - HIGH) * 2) / 3)

#define BUFFER_SIZE			512
#define DATA_SIZE			64

// Capture timer
static stm32_tim_t *tim = (stm32_tim_t*)TIM2;

// Types
typedef struct {
	int c_low;
	int c_mid;
	int c_high;
	int last_window;
	int8_t bit_buffer[BUFFER_SIZE];
	int bit_buffer_write;
	int bit_buffer_read;
} raw_decoder_t;

typedef enum {
	decode_state_sync = 1,
	decode_state_len,
	decode_state_data,
	decode_state_crc1,
	decode_state_crc2,
} decode_state;

typedef struct {
	decode_state state;
	int id;
	uint16_t sync;
	uint8_t data[DATA_SIZE];
	int data_ind;
	uint8_t curr_byte;
	int byte_ind;
	int len;
	uint8_t crc1;
	uint8_t crc2;
} bit_decoder_t;

// State
static bit_decoder_t bit_decoder[3];
static raw_decoder_t raw_decoder[3];

static volatile uint16_t raw_buffer[3][BUFFER_SIZE];
static volatile int raw_buffer_write[3] = { 0, 0, 0};
static volatile int raw_buffer_read[3]  = { 0, 0, 0};

// Debug
static volatile uint32_t crc_fails  = 0;
static volatile uint32_t isr_cnt[3] = {0, 0, 0};
static volatile uint32_t isr_freqs[3] = {0, 0, 0};
static volatile float isr_freqs_lp[3] = {0, 0, 0};

// Decoder thread
static THD_WORKING_AREA(thd_decoder_wa, 1024);
static THD_FUNCTION(thd_decoder, arg);

static void decoder_add_bit(raw_decoder_t *decoder,  int bit) {
	decoder->bit_buffer[decoder->bit_buffer_write] = bit;
	decoder->bit_buffer_write = (decoder->bit_buffer_write + 1) % BUFFER_SIZE;
}

int decoder_get_bit(raw_decoder_t *state) {
	int bit = -1;
	if (state->bit_buffer_read != state->bit_buffer_write) {
		bit = state->bit_buffer[state->bit_buffer_read];
		state->bit_buffer_read = (state->bit_buffer_read + 1) % 4096;
	}
	return bit;
}

static inline void raw_buffer_add(int buf, uint16_t value) {
	raw_buffer[buf][raw_buffer_write[buf]] = value;
	raw_buffer_write[buf] = (raw_buffer_write[buf] + 1) % BUFFER_SIZE;
}

static int raw_buffer_get(int buf) {
	int res = -1;
	if (raw_buffer_write[buf] != raw_buffer_read[buf]) {
		res = (int)raw_buffer[buf][raw_buffer_read[buf]];
		raw_buffer_read[buf] = (raw_buffer_read[buf] + 1) % BUFFER_SIZE;
	}
	return res;
}

static void process_raw_sample(int sample, raw_decoder_t *state) {
	if ((sample <= (MID + THRS)) &&
			(sample >= (MID - THRS))) {
		state->c_mid += (state->c_mid < WINDOW_SIZE) ? 2 : 0;
		state->c_low -= (state->c_low > 0) ? 1 : 0;
		state->c_high -= (state->c_high > 0) ? 1 : 0;
	} else if ((sample <= (HIGH + THRS)) &&
			(sample >= (HIGH - THRS))) {
		state->c_high += (state->c_high < WINDOW_SIZE) ? 2 : 0;
		state->c_mid -= (state->c_mid > 0) ? 1 : 0;
		state->c_low -= (state->c_low > 0) ? 1 : 0;
	} else if ((sample <= (LOW + THRS)) &&
			(sample >= (LOW - THRS))) {
		state->c_low += (state->c_low < WINDOW_SIZE) ? 2 : 0;
		state->c_mid -= (state->c_mid > 0) ? 1 : 0;
		state->c_low -= (state->c_high > 0) ? 1 : 0;
	}

	int wc = 0;
	if (state->c_mid > WINDOW_VALID) wc = MID;
	else if (state->c_low > WINDOW_VALID) wc = LOW;
	else if (state->c_high > WINDOW_VALID) wc = HIGH;

	if (state->last_window == MID) {
		if (wc == LOW) {
			decoder_add_bit(state, 0);
		} else if (wc == HIGH) {
			decoder_add_bit(state, 1);
		}
	}
	if (wc > 0) {
		state->last_window = wc;
	}
}

static void process_cmd(bit_decoder_t *state, char *cmd) {
	cmd[state->len] = '\0';
	commands_printf("CMD RX (ch %d): %s\n", state->id, cmd);
}

bool process_bit(bit_decoder_t *state, int bit) {

	if (bit < 0) return false;

	switch(state->state) {
	case decode_state_sync:
		state->sync <<= 1;
		state->sync += bit;
		if (state->sync == 0xBEEF) {
			state->byte_ind = 0;
			state->state = decode_state_len;
			memset(state->data, 0 , DATA_SIZE);
		}
		break;
	case decode_state_len:
		state->curr_byte <<= 1;
		state->curr_byte += bit;
		state->byte_ind ++;
		if (state->byte_ind == 8) {
			state->byte_ind = 0;
			state->len = state->curr_byte;
			if (state->len == 0 || state->len > DATA_SIZE) {
				state->state = decode_state_sync;
			} else {
				state->state = decode_state_data;
			}
			state->data_ind = 0;
		}
		break;
	case decode_state_data:
		state->curr_byte <<= 1;
		state->curr_byte += bit;
		state->byte_ind ++;
		if (state->byte_ind == 8) {
			state->byte_ind = 0;
			state->data[state->data_ind++] = state->curr_byte;
			if (state->data_ind == state->len) {
				state->state = decode_state_crc1;
			}
		}
		break;
	case decode_state_crc1:
		state->curr_byte <<= 1;
		state->curr_byte += bit;
		state->byte_ind ++;
		if (state->byte_ind == 8) {
			state->byte_ind = 0;
			state->crc1 = state->curr_byte;
			state->state = decode_state_crc2;
		}
		break;
	case decode_state_crc2:
		state->curr_byte <<= 1;
		state->curr_byte += bit;
		state->byte_ind ++;
		if (state->byte_ind == 8) {
			state->byte_ind = 0;
			state->crc2 = state->curr_byte;
			state->state = decode_state_sync;
			uint16_t crc_calc = crc16(state->data, state->len);
			if ((state->crc1 == (crc_calc >> 8)) &&
					(state->crc2 == (crc_calc & 0xFF))) {
				process_cmd(state, (char*)state->data);
			} else {
				crc_fails ++;
			}
			state->sync = 0;
		}
		break;
	default:

		break;
	}
	return true;
}

static void init_capture(void) {
	rccEnableTIM2(false);
	rccResetTIM2();

	nvicEnableVector(STM32_TIM2_NUMBER, 5);
	tim->PSC = 0;         // prescaler
	tim->ARR = TIMER_MAX; // counter maximum

	// Input capture mode
	tim->CCMR1 |= 0x1;
	tim->CCMR1 |= (0x1 << 8);
	tim->CCMR2 |= 0x1;

	// Enable channels
	tim->CCER |= 0x1 | (0x1 << 4) | (0x1 << 8);
//	tim->CCER |= (0x1 << 8);

	tim->CNT = 0;
	tim->EGR = 0x1;
	tim->CR1 = 0x1;

	// activate interrupt on 3 channels
	tim->DIER |= (0x2 | 0x4 | 0x8);
}

OSAL_IRQ_HANDLER(STM32_TIM2_HANDLER) {
	OSAL_IRQ_PROLOGUE();

	static uint32_t last_t0 = 0;
	static uint32_t last_t1 = 0;
	static uint32_t last_t2 = 0;

	uint32_t sr = tim->SR;
	tim->SR = ~sr;

	if ( sr & 0x2 ) { // channel 0
		int32_t ch0 = tim->CCR[0];
		int32_t t = ch0 - last_t0;
		if (t < 0) t += TIMER_MAX;
		if (IN_RANGE(t)) {
			raw_buffer_add(0,(uint16_t)t);
		}
		isr_freqs[0] = t;
		isr_cnt[0]++;
		last_t0 = ch0;
	}

	if ( sr & 0x4 ) { // channel 1
		int32_t ch1 = tim->CCR[1];
		int32_t t = ch1 - last_t1;
		if (t < 0) t += TIMER_MAX;
		if (IN_RANGE(t)) {
			raw_buffer_add(1,(uint16_t)t);
		}
		isr_freqs[1] = t;
		isr_cnt[1]++;
		last_t1 = ch1;
	}

	if ( sr & 0x8 ) { // channel 2
		int32_t ch2 = tim->CCR[2];
		int32_t t = ch2 - last_t2;
		if (t < 0) t += TIMER_MAX;
		if (IN_RANGE(t)) {
			raw_buffer_add(2,(uint16_t)t);
		}
		isr_freqs[2] = t;
		isr_cnt[2]++;
		last_t2 = ch2;
	}

	OSAL_IRQ_EPILOGUE();
}

static THD_FUNCTION(thd_decoder, arg) {
	(void)arg;

	chRegSetThreadName("Decoder");

	memset(&raw_decoder[0],0,sizeof(raw_decoder_t));
	memset(&raw_decoder[1],0,sizeof(raw_decoder_t));
	memset(&raw_decoder[2],0,sizeof(raw_decoder_t));

	memset(&bit_decoder[0], 0, sizeof(bit_decoder_t));
	bit_decoder[0].state = decode_state_sync;
	bit_decoder[0].id = 0;

	memset(&bit_decoder[1], 0, sizeof(bit_decoder_t));
	bit_decoder[1].state = decode_state_sync;
	bit_decoder[1].id = 1;

	memset(&bit_decoder[2], 0, sizeof(bit_decoder_t));
	bit_decoder[2].state = decode_state_sync;
	bit_decoder[2].id = 2;

	bool last_ok0 = false;
	bool last_ok1 = false;
	bool last_ok2 = false;

	for(;;) {
		int sample = raw_buffer_get(0);
		while (sample > 0) {
			process_raw_sample(sample, &raw_decoder[0]);
			sample = raw_buffer_get(0);
		}
		sample = raw_buffer_get(1);
		while (sample > 0) {
			process_raw_sample(sample, &raw_decoder[1]);
			sample = raw_buffer_get(1);
		}
		sample = raw_buffer_get(2);
		while (sample > 0) {
			process_raw_sample(sample, &raw_decoder[2]);
			sample = raw_buffer_get(2);
		}

		bool ok = process_bit(&bit_decoder[0], decoder_get_bit(&raw_decoder[0]));
		if (!ok && !last_ok0) {
			bit_decoder[0].state = decode_state_sync;
		}
		while (ok) {
			ok = process_bit(&bit_decoder[0], decoder_get_bit(&raw_decoder[0]));
		}
		last_ok0 = false;

		ok = process_bit(&bit_decoder[1], decoder_get_bit(&raw_decoder[1]));
		if (!ok && !last_ok1) bit_decoder[1].state = decode_state_sync;
		while (ok) {
			ok = process_bit(&bit_decoder[1], decoder_get_bit(&raw_decoder[1]));
		}
		last_ok1 = false;

		ok = process_bit(&bit_decoder[2], decoder_get_bit(&raw_decoder[2]));
		if (!ok && !last_ok2) bit_decoder[2].state = decode_state_sync;
		while (ok) {
			ok = process_bit(&bit_decoder[2], decoder_get_bit(&raw_decoder[2]));
		}
		last_ok2 = false;

		UTILS_LP_FAST(isr_freqs_lp[0], (float)TIMER_FREQ / (float)isr_freqs[0] / 1000.0, 0.01);
		UTILS_LP_FAST(isr_freqs_lp[1], (float)TIMER_FREQ / (float)isr_freqs[1] / 1000.0, 0.01);
		UTILS_LP_FAST(isr_freqs_lp[2], (float)TIMER_FREQ / (float)isr_freqs[2] / 1000.0, 0.01);

		chThdSleepMilliseconds(1);
	}
}

static void terminal_decoder_info(int argc, const char **argv) {
	(void)argc; (void)argv;
	commands_printf("ISR CNT      : %u  %u  %u", isr_cnt[0], isr_cnt[1], isr_cnt[2]);
	commands_printf("ISR Freq     : %.2fk  %.2fk  %.2fk",
			isr_freqs_lp[0], isr_freqs_lp[1], isr_freqs_lp[2]);
	commands_printf("CRC Failures : %u", crc_fails);
	commands_printf(" ");
}

void hw_init(void) {
	palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(1));
	palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(1));
	palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(1));

//	init_capture();
//	chThdCreateStatic(thd_decoder_wa, sizeof(thd_decoder_wa), NORMALPRIO, thd_decoder, NULL);

	terminal_register_command_callback(
			"decoder_info",
			"Print state information about the decoder",
			NULL,
			terminal_decoder_info);
}
