/*
	Copyright 2019 - 2023 Benjamin Vedder	benjamin@vedder.se

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

#include "commands.h"
#include "buffer.h"
#include "main.h"
#include "comm_can.h"
#include "confparser.h"
#include "packet.h"
#include "terminal.h"
#include "confxml.h"
#include "flash_helper.h"
#include "utils.h"
#include "lwprintf.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// Private variables
static uint8_t send_buffer_global[PACKET_MAX_PL_LEN];
static mutex_t send_buffer_mutex;
static mutex_t print_mutex;
static mutex_t terminal_mutex;
static uint8_t blocking_thread_cmd_buffer[PACKET_MAX_PL_LEN];
static volatile unsigned int blocking_thread_cmd_len = 0;
static volatile bool is_blocking = false;

// Threads
static THD_FUNCTION(blocking_thread, arg);
static THD_WORKING_AREA(blocking_thread_wa, 2048);
static thread_t *blocking_tp;

// Function pointers
static void(* volatile send_func)(unsigned char *data, unsigned int len) = 0;
static void(* volatile send_func_blocking)(unsigned char *data, unsigned int len) = 0;
static void(* volatile appdata_func)(unsigned char *data, unsigned int len) = 0;

void commands_init(void) {
	chMtxObjectInit(&send_buffer_mutex);
	chMtxObjectInit(&print_mutex);
	chMtxObjectInit(&terminal_mutex);
	chThdCreateStatic(blocking_thread_wa, sizeof(blocking_thread_wa), NORMALPRIO, blocking_thread, NULL);
}

static void reply_func_dummy(unsigned char *data, unsigned int len) {
	(void)data; (void)len;
}

void commands_process_packet(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len)) {
	if (len < 1) {
		return;
	}

	if (reply_func == 0) {
		reply_func = reply_func_dummy;
	}

	send_func = reply_func;

	COMM_PACKET_ID packet_id = data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_FW_VERSION: {
		int32_t ind = 0;
		uint8_t send_buffer[50];
		uint8_t num_name_chars;
		send_buffer[ind++] = COMM_FW_VERSION;
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;

		num_name_chars =  strlen(HW_NAME);
		if (num_name_chars >= HW_NAME_MAX_CHARS) {
			num_name_chars =  HW_NAME_MAX_CHARS;
		}
		strncpy((char*)(send_buffer + ind), HW_NAME, num_name_chars);
		ind += num_name_chars;
#ifdef HW_REV_CHAR
		if (num_name_chars <= (HW_NAME_MAX_CHARS - 2)) {
			send_buffer[ind++] = '_';
			send_buffer[ind++] = HW_REV_CHAR;
		}
#endif
		send_buffer[ind++] = '\0';
		memcpy(send_buffer + ind, STM32_UUID_8, 12);
		ind += 12;

		send_buffer[ind++] = 0;
		send_buffer[ind++] = FW_TEST_VERSION_NUMBER;
		send_buffer[ind++] = HW_TYPE_CUSTOM_MODULE;
		send_buffer[ind++] = 1; // One custom config

		reply_func(send_buffer, ind);
	} break;

	case COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW:
		data[-1] = COMM_JUMP_TO_BOOTLOADER_HW;
		comm_can_send_buffer(255, data - 1, len + 1, 2);
		chThdSleepMilliseconds(100);
		/* Falls through. */
		/* no break */
	case COMM_JUMP_TO_BOOTLOADER_HW: {
		int32_t ind = 0;
		HW_TYPE hw = data[ind++];
		char *hw_name = (char*)data + ind;
		ind += strlen(hw_name) + 1;

		if (hw != HW_TYPE_VESC_BMS || strcmp(hw_name, HW_NAME) != 0) {
			break;
		}

		data += ind;
		len -= ind;
	}
	/* Falls through. */
	/* no break */
	case COMM_JUMP_TO_BOOTLOADER:
		flash_helper_jump_to_bootloader();
		break;

	case COMM_ERASE_NEW_APP_ALL_CAN_HW:
		data[-1] = COMM_ERASE_NEW_APP_HW;
		comm_can_send_buffer(255, data - 1, len + 1, 2);
		chThdSleepMilliseconds(1500);
		/* Falls through. */
		/* no break */
	case COMM_ERASE_NEW_APP_HW: {
		int32_t ind = 0;
		HW_TYPE hw = data[ind++];
		char *hw_name = (char*)data + ind;
		ind += strlen(hw_name) + 1;

		if (hw != HW_TYPE_VESC_BMS || strcmp(hw_name, HW_NAME) != 0) {
			break;
		}

		data += ind;
		len -= ind;
	}
	/* Falls through. */
	/* no break */
	case COMM_ERASE_NEW_APP: {
		int32_t ind = 0;

		uint16_t flash_res = flash_helper_erase_new_app(buffer_get_uint32(data, &ind));

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_ERASE_NEW_APP;
		send_buffer[ind++] = flash_res == HAL_OK ? 1 : 0;
		reply_func(send_buffer, ind);
	} break;

	case COMM_ERASE_BOOTLOADER_ALL_CAN_HW:
		data[-1] = COMM_ERASE_BOOTLOADER_HW;
		comm_can_send_buffer(255, data - 1, len + 1, 2);
		chThdSleepMilliseconds(1500);
		/* Falls through. */
		/* no break */
	case COMM_ERASE_BOOTLOADER_HW: {
		int32_t ind = 0;
		HW_TYPE hw = data[ind++];
		char *hw_name = (char*)data + ind;
		ind += strlen(hw_name) + 1;

		if (hw != HW_TYPE_VESC_BMS || strcmp(hw_name, HW_NAME) != 0) {
			break;
		}

		data += ind;
		len -= ind;
	}
	/* Falls through. */
	/* no break */
	case COMM_ERASE_BOOTLOADER: {
		int32_t ind = 0;

		uint16_t flash_res = flash_helper_erase_bootloader();

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_ERASE_BOOTLOADER;
		send_buffer[ind++] = flash_res == HAL_OK ? 1 : 0;
		reply_func(send_buffer, ind);
	} break;

	case COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW:
		data[-1] = COMM_WRITE_NEW_APP_DATA_HW;

		comm_can_send_buffer(255, data - 1, len + 1, 2);
		/* Falls through. */
		/* no break */
	case COMM_WRITE_NEW_APP_DATA_HW: {
		int32_t ind = 0;
		HW_TYPE hw = data[ind++];
		char *hw_name = (char*)data + ind;
		ind += strlen(hw_name) + 1;

		if (hw != HW_TYPE_CUSTOM_MODULE || strcmp(hw_name, HW_NAME) != 0) {
			break;
		}

		data += ind;
		len -= ind;
	}
	/* Falls through. */
	/* no break */
	case COMM_WRITE_NEW_APP_DATA: {
		int32_t ind = 0;
		uint32_t new_app_offset = buffer_get_uint32(data, &ind);

		// Pad to multiple of 8 bytes
		while (((len - ind) % 8) != 0) {
			data[len++] = 0;
		}

		uint16_t flash_res = flash_helper_write_new_app_data(new_app_offset, data + ind, len - ind);

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_WRITE_NEW_APP_DATA;
		send_buffer[ind++] = flash_res == HAL_OK ? 1 : 0;
		buffer_append_uint32(send_buffer, new_app_offset, &ind);
		reply_func(send_buffer, ind);
	} break;

	case COMM_FORWARD_CAN:
		comm_can_send_buffer(data[0], data + 1, len - 1, 0);
		break;

	case COMM_GET_CUSTOM_CONFIG:
	case COMM_GET_CUSTOM_CONFIG_DEFAULT: {
		main_config_t conf = conf = backup.config;

		int conf_ind = data[0];

		if (conf_ind != 0) {
			break;
		}

		if (packet_id == COMM_GET_CUSTOM_CONFIG) {
			conf = backup.config;
		} else {
			confparser_set_defaults_main_config_t(&conf);
		}

		chMtxLock(&send_buffer_mutex);
		int32_t ind = 0;
		send_buffer_global[ind++] = packet_id;
		send_buffer_global[ind++] = conf_ind;
		int32_t len = confparser_serialize_main_config_t(send_buffer_global + ind, &conf);
		commands_send_packet(send_buffer_global, len + ind);
		chMtxUnlock(&send_buffer_mutex);
	} break;

	case COMM_SET_CUSTOM_CONFIG: {
		main_config_t conf = conf = backup.config;

		int conf_ind = data[0];

		if (conf_ind == 0 && confparser_deserialize_main_config_t(data + 1, &conf)) {
			conf_general_apply_hw_limits(&conf);
			backup.config = conf;
			flash_helper_store_backup_data();
			comm_can_set_baud(backup.config.can_baud_rate, 0);

			int32_t ind = 0;
			uint8_t send_buffer[50];
			send_buffer[ind++] = packet_id;
			reply_func(send_buffer, ind);
		} else {
			commands_printf("Warning: Could not set configuration");
		}
	} break;

	case COMM_GET_CUSTOM_CONFIG_XML: {
		int32_t ind = 0;

		int conf_ind = data[ind++];

		if (conf_ind != 0) {
			break;
		}

		int32_t len_conf = buffer_get_int32(data, &ind);
		int32_t ofs_conf = buffer_get_int32(data, &ind);

		if ((len_conf + ofs_conf) > DATA_MAIN_CONFIG_T__SIZE || len_conf > (PACKET_MAX_PL_LEN - 10)) {
			break;
		}

		chMtxLock(&send_buffer_mutex);
		ind = 0;
		send_buffer_global[ind++] = packet_id;
		send_buffer_global[ind++] = conf_ind;
		buffer_append_int32(send_buffer_global, DATA_MAIN_CONFIG_T__SIZE, &ind);
		buffer_append_int32(send_buffer_global, ofs_conf, &ind);
		memcpy(send_buffer_global + ind, data_main_config_t_ + ofs_conf, len_conf);
		ind += len_conf;
		reply_func(send_buffer_global, ind);

		chMtxUnlock(&send_buffer_mutex);
	} break;

	case COMM_TERMINAL_CMD_SYNC:
		data[len] = '\0';
		chMtxLock(&terminal_mutex);
		terminal_process_string((char*)data);
		chMtxUnlock(&terminal_mutex);
		break;

	case COMM_CUSTOM_APP_DATA: {
		if (appdata_func) {
			appdata_func(data, len);
		}
	} break;

	case COMM_FW_INFO: {
		// Write at most the first max_len characters of str into buffer,
		// followed by a null byte.
		void buffer_append_str_max_len(uint8_t *buffer, char *str, size_t max_len, int32_t *index) {
			size_t str_len = strlen(str);
			if (str_len > max_len) {
				str_len = max_len;
				return;
			}
			
			memcpy(&buffer[*index], str, str_len);
			*index += str_len;
			buffer[(*index)++] = '\0';
		}
		
		int32_t ind = 0;
		uint8_t send_buffer[98];
		
		send_buffer[ind++] = COMM_FW_INFO;
		
		// This information is technically duplicated with COMM_FW_VERSION, but
		// I don't care.
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;
		send_buffer[ind++] = FW_TEST_VERSION_NUMBER;
		
		// We don't include the branch name unfortunately
		buffer_append_str_max_len(send_buffer, GIT_COMMIT_HASH, 46, &ind);
#ifdef USER_GIT_COMMIT_HASH
		char *user_commit_hash = USER_GIT_COMMIT_HASH;
#else
		char *user_commit_hash = "";
#endif
		buffer_append_str_max_len(send_buffer, user_commit_hash, 46, &ind);

		reply_func(send_buffer, ind);
	} break;

	// Blocking commands. Only one of them runs at any given time, in their
	// own thread. If other blocking commands come before the previous one has
	// finished, they are discarded.
	case COMM_TERMINAL_CMD:
	case COMM_PING_CAN:
	case COMM_CAN_UPDATE_BAUD_ALL:
		if (!is_blocking) {
			memcpy(blocking_thread_cmd_buffer, data - 1, len + 1);
			blocking_thread_cmd_len = len + 1;
			is_blocking = true;
			send_func_blocking = reply_func;
			chEvtSignal(blocking_tp, (eventmask_t)1);
		}
		break;

	default:
		break;
	}
}

void commands_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

void commands_printf(const char* format, ...) {
	chMtxLock(&print_mutex);

	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[255];

	print_buffer[0] = COMM_PRINT;
	len = lwprintf_vsnprintf(print_buffer + 1, 254, format, arg);
	va_end (arg);

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer,
				(len < 254) ? len + 1 : 255);
	}

	chMtxUnlock(&print_mutex);
}

void commands_set_app_data_handler(void(*func)(unsigned char *data, unsigned int len)) {
	appdata_func = func;
}

void commands_send_app_data(unsigned char *data, unsigned int len) {
	int32_t index = 0;

	if (len > PACKET_MAX_PL_LEN)
		return;

	chMtxLock(&send_buffer_mutex);
	send_buffer_global[index++] = COMM_CUSTOM_APP_DATA;
	memcpy(send_buffer_global + index, data, len);
	index += len;
	commands_send_packet(send_buffer_global, index);
	chMtxUnlock(&send_buffer_mutex);
}

static THD_FUNCTION(blocking_thread, arg) {
	(void)arg;

	chRegSetThreadName("comm_block");

	blocking_tp = chThdGetSelfX();

	for(;;) {
		is_blocking = false;

		chEvtWaitAny((eventmask_t) 1);

		uint8_t *data = blocking_thread_cmd_buffer;
		unsigned int len = blocking_thread_cmd_len;

		COMM_PACKET_ID packet_id;
		static uint8_t send_buffer[512];

		packet_id = data[0];
		data++;
		len--;

		switch (packet_id) {
		case COMM_TERMINAL_CMD:
			data[len] = '\0';
			chMtxLock(&terminal_mutex);
			terminal_process_string((char*)data);
			chMtxUnlock(&terminal_mutex);
			break;

		case COMM_PING_CAN: {
			int32_t ind = 0;
			send_buffer[ind++] = COMM_PING_CAN;

			for (uint8_t i = 0;i < 255;i++) {
				HW_TYPE hw_type;
				if (comm_can_ping(i, &hw_type)) {
					send_buffer[ind++] = i;
				}
			}

			if (send_func_blocking) {
				send_func_blocking(send_buffer, ind);
			}
		} break;

		case COMM_CAN_UPDATE_BAUD_ALL: {
			int32_t ind = 0;
			uint32_t kbits = buffer_get_int16(data, &ind);
			uint32_t delay_msec = buffer_get_int16(data, &ind);
			CAN_BAUD baud = comm_can_kbits_to_baud(kbits);
			if (baud != CAN_BAUD_INVALID) {
				for (int i = 0;i < 10;i++) {
					comm_can_send_update_baud(kbits, delay_msec);
					chThdSleepMilliseconds(50);
				}
				comm_can_set_baud(baud, delay_msec);
				backup.config.can_baud_rate = baud;
				flash_helper_store_backup_data();
			}
			ind = 0;
			send_buffer[ind++] = packet_id;
			send_buffer[ind++] = baud != CAN_BAUD_INVALID;
			if (send_func_blocking) {
				send_func_blocking(send_buffer, ind);
			}
		} break;

		default:
			break;
		}
	}
}
