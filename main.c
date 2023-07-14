/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se

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

#include "conf_general.h"
#include "comm_can.h"
#include "utils.h"
#include "confparser.h"
#include "commands.h"
#include "flash_helper.h"
#include "hw.h"
#include "timer.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

//__attribute__((section(".ram4"))) volatile backup_data backup;
volatile backup_data backup;

#ifndef VAR_INIT_CODE_HW_CONF
#define VAR_INIT_CODE_HW_CONF		VAR_INIT_CODE
#endif

int main(void) {
	halInit();
	chSysInit();

	// Stop debug mode in case no power cycle has been done after upload. This
	// saves power.
	DBGMCU->CR = DBGMCU_CR_DBG_STOP;

	// If there is no backup data in RAM try to load it from flash. This can
	// happen if power was lost.
	if (backup.controller_id_init_flag != VAR_INIT_CODE ||
			backup.send_can_status_rate_hz_init_flag != VAR_INIT_CODE ||
			backup.can_baud_rate_init_flag != VAR_INIT_CODE ||
			backup.conf_flash_write_cnt_init_flag != VAR_INIT_CODE ||
			backup.hw_config_init_flag != VAR_INIT_CODE_HW_CONF) {
		flash_helper_load_backup_data();
	}

	// Reset backup counters that haven't been set. Should work across firmware uploads.
	if (backup.controller_id_init_flag != VAR_INIT_CODE) {
		backup.controller_id = HW_DEFAULT_ID;
		backup.controller_id_init_flag = VAR_INIT_CODE;
	}

	if (backup.send_can_status_rate_hz_init_flag != VAR_INIT_CODE) {
		backup.send_can_status_rate_hz = CONF_SEND_CAN_STATUS_RATE_HZ;
		backup.send_can_status_rate_hz_init_flag = VAR_INIT_CODE;
	}

	if (backup.can_baud_rate_init_flag != VAR_INIT_CODE) {
		backup.can_baud_rate = CONF_CAN_BAUD_RATE;
		backup.can_baud_rate_init_flag = VAR_INIT_CODE;
	}

	if (backup.conf_flash_write_cnt_init_flag != VAR_INIT_CODE) {
		backup.conf_flash_write_cnt = 0;
		backup.conf_flash_write_cnt_init_flag = VAR_INIT_CODE;
	}

	if (backup.hw_config_init_flag != VAR_INIT_CODE_HW_CONF) {
		memset((void*)backup.hw_config, 0, sizeof(backup.hw_config));
		backup.hw_config_init_flag = VAR_INIT_CODE_HW_CONF;
	}

	if (backup.config_init_flag != MAIN_CONFIG_T_SIGNATURE) {
		confparser_set_defaults_main_config_t((main_config_t*)(&backup.config));
		backup.config_init_flag = MAIN_CONFIG_T_SIGNATURE;
		backup.config.controller_id = backup.controller_id;
		backup.config.send_can_status_rate_hz = backup.send_can_status_rate_hz;
		backup.config.can_baud_rate = backup.can_baud_rate;

	}

	conf_general_apply_hw_limits((main_config_t*)&backup.config);

	timer_init();

	// USB needs some time to detect if a cable is connected, so start it before powering the regulators
	// to not waste too much power.
	commands_init();

	comm_can_init();
	comm_can_set_baud(backup.config.can_baud_rate);

	timeout_init();

	HW_INIT_HOOK();

	for(;;) {
		backup.controller_id = backup.config.controller_id;
		backup.send_can_status_rate_hz = backup.config.send_can_status_rate_hz;
		backup.can_baud_rate = backup.config.can_baud_rate;
		chThdSleepMilliseconds(1);
	}

	return 0;
}
