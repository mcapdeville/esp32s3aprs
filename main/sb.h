/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/sb.h
 *
 * Copyright (C) 2025  Marc CAPDEVILLE (F4JMZ)
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

#ifndef _SB_H_
#define _SB_H_

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <nvs.h>

typedef struct SB_S {
	// Smart Beaconning parameters
	uint16_t fast_speed;
	uint16_t fast_rate;
	uint16_t slow_speed;
	uint16_t slow_rate;
	uint16_t min_turn_time;
	uint16_t min_turn_angle;
	uint16_t min_turn_speed;
	uint16_t turn_slope;

	// Smart Beaconning state
	time_t last_time;
	uint16_t last_heading;
} SB_t;

void SB_Init(SB_t * Sb, nvs_handle_t nvs);
bool SB_Update(SB_t *Sb , time_t Time, uint16_t Speed, uint16_t Heading, bool force);

#endif
