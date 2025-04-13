/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * mp_aprs.h
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

#ifndef _MP_APRS_H_
#define _MP_APRS_H_

#include "../main/aprs.h"
#include "../main/ax25.h"

#include "mp_ax25_addr.h"

// Main Aprs object
extern APRS_t * Aprs;

typedef struct _mp_obj_aprs_stations_db_t {
    mp_obj_base_t base;
	APRS_t * aprs;
	AX25_Addr_t filter;
	uint8_t filter_type;	// b0 : wildcard on ssid
							// b1 : wildcard on callsign
	// Put your data here
} mp_obj_aprs_stations_db_t;

// Position obj type
typedef struct _mp_obj_aprs_position_t {
    mp_obj_base_t base;
	struct APRS_Position position;
} mp_obj_aprs_position_t;

// Station obj type
typedef struct _mp_obj_aprs_station_t {
    mp_obj_base_t base;
	APRS_Station_t station;
} mp_obj_aprs_station_t;

// Aprs obj type
typedef struct _mp_obj_aprs_t {
    mp_obj_base_t base;
	APRS_t * aprs;
} mp_obj_aprs_t;

extern const mp_obj_type_t mp_type_aprs;
extern const mp_obj_type_t mp_type_aprs_position;
extern const mp_obj_type_t mp_type_aprs_station;
extern const mp_obj_type_t mp_type_aprs_stations_db;

#endif
