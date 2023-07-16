/*
	Copyright 2019 - 2021 Benjamin Vedder	benjamin@vedder.se

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
#include "datatypes.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

// Firmware version
#define FW_VERSION_MAJOR			6
#define FW_VERSION_MINOR			05
// Set to 0 for building a release and iterate during beta test builds
#define FW_TEST_VERSION_NUMBER		0

// Init codes for the persistent storage. Change the config code when updating the config struct
// in a way that is not backwards compatible.
#define VAR_INIT_CODE				59763223

#define HW_NAME_MAX_CHARS			16

#if !defined(HW_SOURCE) && !defined(HW_HEADER)
#define HW_HEADER					"hw_lb_ant.h"
#define HW_SOURCE					"hw_lb_ant.c"
#endif

/*
 * MCU
 */
#define STM32_UUID					((uint32_t*)0x1FFF7590)
#define STM32_UUID_8				((uint8_t*)0x1FFF7590)
#define STM32_FLASH_SIZE			((uint16_t*)0x1FFF75E0)

#ifndef HW_SOURCE
#error "No hardware source file set"
#endif

#ifndef HW_HEADER
#error "No hardware header file set"
#endif

#include "hw.h"
#include "conf_default.h"

// Functions
void conf_general_apply_hw_limits(main_config_t *config);

#endif /* CONF_GENERAL_H_ */
