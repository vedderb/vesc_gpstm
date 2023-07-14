/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

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

#ifndef HWCONF_HW_LB_ANT_H_
#define HWCONF_HW_LB_ANT_H_

#define HW_NAME							"LB-ANT-STM"
#define HW_DEFAULT_ID					30

// HW-specific
#define HW_INIT_HOOK()					hw_init()

#define CURR_MEASURE_ON()
#define CURR_MEASURE_OFF()

// Settings
#define V_REG							3.3

// CAN
#define LINE_CAN_RX						PAL_LINE(GPIOA, 11)
#define LINE_CAN_TX						PAL_LINE(GPIOA, 12)
#define HW_CAN_DEV						CAND1
#define HW_CAN_AF						9

// Functions
void hw_init(void);

#endif /* HWCONF_HW_LB_ANT_H_ */
