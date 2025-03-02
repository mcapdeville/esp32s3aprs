/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * esp32s3aprs.c
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

#include <py/runtime.h>
#include "mp_ax25_addr.h"
#include "mp_aprs.h"
#include "mp_radio.h"
#include "mp_templ.h"

#include <esp_log.h>

extern int Battery;
extern uint8_t Rssi;
extern uint8_t Rssi_max;

static mp_obj_t master_log(const mp_obj_t in) {
    esp_log_level_t level = LOG_LOCAL_LEVEL;
    level = mp_obj_get_int(in);

	esp_log_set_level_master(level);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(master_log_obj, master_log);

static mp_obj_t tagged_log(const mp_obj_t tag_in, const mp_obj_t level_in ) {
    esp_log_level_t level = LOG_LOCAL_LEVEL;
    const char *tag = mp_obj_str_get_str(tag_in);
    level = mp_obj_get_int(level_in);

	esp_log_level_set(tag, level);

	return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(tagged_log_obj, tagged_log);

static mp_obj_t battery(void) {
	return mp_obj_new_int(Battery);
}
static MP_DEFINE_CONST_FUN_OBJ_0(battery_obj, battery);

static mp_obj_t rssi(void) {
	return mp_obj_new_int(Rssi);
}
static MP_DEFINE_CONST_FUN_OBJ_0(rssi_obj, rssi);

static mp_obj_t rssi_max(void) {
	return mp_obj_new_int(Rssi);
}
static MP_DEFINE_CONST_FUN_OBJ_0(rssi_max_obj, rssi_max);

static const mp_rom_map_elem_t esp32s3aprs_globals_table[] = {
	{ MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_esp32s3aprs) },
	{ MP_ROM_QSTR(MP_QSTR_aprs),     MP_ROM_PTR(&mp_type_aprs) },
//	{ MP_ROM_QSTR(MP_QSTR_aprs_position),     MP_ROM_PTR(&mp_type_aprs_position) },
//	{ MP_ROM_QSTR(MP_QSTR_aprs_station),     MP_ROM_PTR(&mp_type_aprs_station) },
	{ MP_ROM_QSTR(MP_QSTR_master_log),     MP_ROM_PTR(&master_log_obj) },
	{ MP_ROM_QSTR(MP_QSTR_tagged_log),     MP_ROM_PTR(&tagged_log_obj) },
	{ MP_ROM_QSTR(MP_QSTR_battery),     MP_ROM_PTR(&battery_obj) },
	{ MP_ROM_QSTR(MP_QSTR_rssi),     MP_ROM_PTR(&rssi_obj) },
	{ MP_ROM_QSTR(MP_QSTR_rssi_max),     MP_ROM_PTR(&rssi_max_obj) },
	{ MP_ROM_QSTR(MP_QSTR_radio),     MP_ROM_PTR(&mp_type_radio) },
	{ MP_ROM_QSTR(MP_QSTR_ax25_addr),     MP_ROM_PTR(&mp_type_ax25_addr) },
//	{ MP_ROM_QSTR(MP_QSTR_templ),     MP_ROM_PTR(&mp_type_templ) },
//	{ MP_ROM_QSTR(MP_QSTR_aprs_stations_db),     MP_ROM_PTR(&mp_type_aprs_stations_db) },
};

static MP_DEFINE_CONST_DICT(esp32s3aprs_globals, esp32s3aprs_globals_table);

const mp_obj_module_t esp32s3aprs = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&esp32s3aprs_globals,
};

// Register the module to make it available in Python.
MP_REGISTER_MODULE(MP_QSTR_esp32s3aprs, esp32s3aprs);
