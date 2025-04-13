/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main.c
 *
 * Copyright (C) 2025  Marc CAPDEVILLE F4JMZ
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include "ulp_riscv_utils.h"
#include "ulp_riscv_adc_ulp_core.h"

uint16_t ulp_battery_threshold = 3886;	// change for battery voltage > 3.3V
uint16_t ulp_battery_voltage; 

int main (void) {
    if ((ulp_battery_voltrage = ulp_riscv_adc_read_channel(ADC_UNIT_2, ADC_CHANNEL_0 )) > adc_threshold)
        ulp_riscv_wakeup_main_processor();

    return 0;
}
