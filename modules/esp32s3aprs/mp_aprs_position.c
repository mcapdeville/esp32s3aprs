/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * mp_aprs_position.c
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
#include "mp_aprs.h"
#include <time.h>

// position class methode
/*
static void *aprs_position_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_obj_aprs_position_t *o = mp_obj_malloc(mp_obj_aprs_position_t, type);
	bzero(&o->position, sizeof(struct APRS_Position));
    return MP_OBJ_FROM_PTR(o);
}
*/

static void aprs_position_print(const mp_print_t *print, mp_obj_t self, mp_print_kind_t kind) {
    (void)kind;
    mp_obj_aprs_position_t *o = MP_OBJ_TO_PTR(self);
	char buff[64];
	float lat,lon;

	lon = ((float)o->position.longitude)/((float)(1L<<22));
	lat = ((float)o->position.latitude)/((float)(1L<<22));

	snprintf(buff,sizeof(buff), "%02f° %c %03f° %c Alt = %dft",
			lat<0?-lat:lat,
			lat<0?'S':'N',
			lon<0?-lon:lon,
			lon<0?'W':'E',
			o->position.altitude
			);

    mp_printf(print, buff);
}

static mp_obj_t aprs_position_longitude(mp_obj_t self) {
    mp_obj_aprs_position_t *o = MP_OBJ_TO_PTR(self);

	return mp_obj_new_float(((float)o->position.longitude)/((float)(1L<<22)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(aprs_position_longitude_obj, aprs_position_longitude);

static mp_obj_t aprs_position_latitude(mp_obj_t self) {
    mp_obj_aprs_position_t *o = MP_OBJ_TO_PTR(self);

	return mp_obj_new_float(((float)o->position.latitude)/((float)(1L<<22)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(aprs_position_latitude_obj, aprs_position_latitude);

static mp_obj_t aprs_position_altitude(mp_obj_t self) {
    mp_obj_aprs_position_t *o = MP_OBJ_TO_PTR(self);

	return mp_obj_new_float(o->position.altitude);
}
static MP_DEFINE_CONST_FUN_OBJ_1(aprs_position_altitude_obj, aprs_position_altitude);

static mp_obj_t aprs_position_ambiguity(mp_obj_t self) {
    mp_obj_aprs_position_t *o = MP_OBJ_TO_PTR(self);

	return MP_OBJ_NEW_SMALL_INT(o->position.ambiguity);
}
static MP_DEFINE_CONST_FUN_OBJ_1(aprs_position_ambiguity_obj, aprs_position_ambiguity);

// Position local dictionay
static const mp_rom_map_elem_t aprs_position_locals_dict_table[] = {
	{MP_ROM_QSTR(MP_QSTR_longitude), MP_ROM_PTR(&aprs_position_longitude_obj)},
	{MP_ROM_QSTR(MP_QSTR_latitude), MP_ROM_PTR(&aprs_position_latitude_obj)},
	{MP_ROM_QSTR(MP_QSTR_altitude), MP_ROM_PTR(&aprs_position_altitude_obj)},
	{MP_ROM_QSTR(MP_QSTR_ambiguity), MP_ROM_PTR(&aprs_position_ambiguity_obj)},
};
static MP_DEFINE_CONST_DICT(aprs_position_locals_dict, aprs_position_locals_dict_table);

// Apts_Position class
MP_DEFINE_CONST_OBJ_TYPE(
	mp_type_aprs_position,
    MP_QSTR_Aprs_position,
    MP_TYPE_FLAG_NONE,
    print, aprs_position_print,
    locals_dict, &aprs_position_locals_dict
);
