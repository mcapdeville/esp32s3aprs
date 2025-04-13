/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/sb.c
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

#include "sb.h"

#include <esp_log.h>
#include <nvs.h>

#include "gps.h"

#define TAG	"SB"

#define SB_MIN_SLOW_RATE 120
#define SB_MIN_FAST_RATE 30
#define SB_MIN_DISTANCE	10
#define SB_MIN_TURN_SPEED 1
#define SB_MIN_TURN_ANGLE 1
#define SB_MIN_TURN_TIME 10

void SB_Init(SB_t * Sb, nvs_handle_t nvs) {
	uint32_t distance;

	if (nvs_get_u16(nvs,"SlowRate",&Sb->slow_rate)) {
		Sb->slow_rate = 1800; // 1800 s
	} else if (Sb->slow_rate < SB_MIN_SLOW_RATE)
		Sb->slow_rate = SB_MIN_SLOW_RATE;

	if (nvs_get_u16(nvs,"FastRate",&Sb->fast_rate)) {
		Sb->fast_rate = 120; // 120 s
	} else if (Sb->fast_rate < SB_MIN_FAST_RATE)
		Sb->fast_rate = SB_MIN_FAST_RATE;

	if (nvs_get_u32(nvs,"Distance",&distance)) {
		distance = 2500; // 2500 m
	} else if (distance < SB_MIN_DISTANCE)
		distance = SB_MIN_DISTANCE;

	Sb->slow_speed = (distance*3600.0f)/(Sb->slow_rate*1852.0f) + 0.5f;
	Sb->fast_speed = (distance*3600.0f)/(Sb->fast_rate*1852.0f) + 0.5f;

	ESP_LOGD(TAG,"sr=%u s fr=%u s d=%lu m ss=%u kn fs=%u kn",
			Sb->slow_rate,
			Sb->fast_rate,
			distance,
			Sb->slow_speed,
			Sb->fast_speed
		);

	if (nvs_get_u16(nvs,"MinTurnTime",&Sb->min_turn_time)) {
		Sb->min_turn_time = 10; // 10 s
	} else if (Sb->min_turn_time < SB_MIN_TURN_TIME)
		Sb->min_turn_time = SB_MIN_TURN_TIME;

	if (nvs_get_u16(nvs,"MinTurnAngle",&Sb->min_turn_angle)) {
		Sb->min_turn_angle = 15;
	} else if (Sb->min_turn_angle < SB_MIN_TURN_ANGLE)
		Sb->min_turn_angle = SB_MIN_TURN_ANGLE;

	if (nvs_get_u16(nvs,"TurnSlope",&Sb->turn_slope)) {
		Sb->turn_slope = 30;// Angle to be added to MinTurnAngle for 1 knot speed (add half for 2 knot)
	}

	if (nvs_get_u16(nvs,"MinTurnSpeed",&Sb->min_turn_speed)) {
		Sb->min_turn_speed = 1;
	} else if (Sb->min_turn_speed < SB_MIN_TURN_SPEED)
		Sb->min_turn_speed = SB_MIN_TURN_SPEED;
}

bool SB_Update(SB_t *Sb , time_t Time, uint16_t Speed, uint16_t Heading, bool force) {
	time_t elapsed = Time - Sb->last_time;
	int16_t turned = Heading - Sb->last_heading;
	uint32_t time_th;
	uint16_t turned_th;

	if (force) {
		ESP_LOGD(TAG,"Forced beaconing");
		goto now;
	}

	ESP_LOGD(TAG,"Elapsed time = %llds, Turrned = %d°, Speed = %u knot, Heading = %u°",
			elapsed,
			turned,
			Speed,
			Heading);

	if (Speed <= Sb->slow_speed) {
	       if (elapsed >= Sb->slow_rate) {
		       ESP_LOGD(TAG,"Beaconing at slow rate");
		       goto now;
	       }
	} else if (Speed >= Sb->fast_speed) {
		if (elapsed >= Sb->fast_rate) {
		       ESP_LOGD(TAG,"Beaconing at fast rate");
			goto now;
		}
	} else {
		time_th = (((((uint32_t)Sb->fast_rate * (uint32_t)Sb->fast_speed)<<GPS_FIXED_POINT)/Speed + (1L<<(GPS_FIXED_POINT-1)))>>GPS_FIXED_POINT);
		if (elapsed >= time_th ) {
		       ESP_LOGD(TAG,"Beaconing at epalsed >= %lus", time_th);
		       goto now;
		}
	}

	if (Speed >= Sb->min_turn_speed && elapsed >= Sb->min_turn_time) {
		if (turned < 0)
			turned = -turned;
		if (turned > 180)
			turned = 360-turned;

		turned_th = Sb->min_turn_angle + (((((uint32_t)Sb->turn_slope)<<GPS_FIXED_POINT)/Speed + (1L<<(GPS_FIXED_POINT-1)))>>GPS_FIXED_POINT);
		if (turned >= turned_th) {
			ESP_LOGD(TAG,"Beaconing at turned angle >= %d", turned_th);
			goto now;
		}
	}

	return false;

now:
	Sb->last_time = Time;
	Sb->last_heading = Heading;
	
	return true;	
}
