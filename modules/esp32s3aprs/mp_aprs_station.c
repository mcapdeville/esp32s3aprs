/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * mp_aprs_station.c
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

// Aprs station methode
static mp_obj_t aprs_station_callid(size_t nargs, const mp_obj_t *args ) {
    mp_obj_aprs_station_t *o = MP_OBJ_TO_PTR(args[0]);
#if 0
	if (nargs == 2) {
	   if (mp_obj_is_str(args[1]))
		   return MP_OBJ_NEW_SMALL_INT(AX25_Str_To_Addr(mp_obj_str_get_str(args[1]), &o->station.callid));
	   else if (mp_obj_is_type(args[1], &mp_type_ax25_addr)) {
		   memcpy(&o->station.callid, &((mp_obj_ax25_addr_t*)args[1])->addr, sizeof(AX25_Addr_t));
		   return 0;
	   }
	}
#endif
	mp_obj_ax25_addr_t *callid = mp_obj_malloc(mp_obj_ax25_addr_t, &mp_type_ax25_addr);
	if (callid)
		memcpy(&callid->addr, &o->station.callid, sizeof(AX25_Addr_t));

	return MP_OBJ_FROM_PTR(callid);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aprs_station_callid_obj, 1, 2, aprs_station_callid);

static mp_obj_t aprs_station_symbol(mp_obj_t self) {
    mp_obj_aprs_station_t *o = MP_OBJ_TO_PTR(self);

	return MP_OBJ_FROM_PTR(mp_obj_new_str(o->station.symbol, 2));
}
static MP_DEFINE_CONST_FUN_OBJ_1(aprs_station_symbol_obj, aprs_station_symbol);

static mp_obj_t aprs_station_timestamp(mp_obj_t self) {
    mp_obj_aprs_station_t *o = MP_OBJ_TO_PTR(self);
	mp_obj_aprs_position_t *ts = mp_obj_new_int(o->station.timestamp);

	return MP_OBJ_FROM_PTR(ts);
}
static MP_DEFINE_CONST_FUN_OBJ_1(aprs_station_timestamp_obj, aprs_station_timestamp);

static mp_obj_t aprs_station_status(mp_obj_t self) {
    mp_obj_aprs_station_t *o = MP_OBJ_TO_PTR(self);

	return MP_OBJ_FROM_PTR(mp_obj_new_str(o->station.status, strlen(o->station.status)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(aprs_station_status_obj, aprs_station_status);

static mp_obj_t aprs_station_position(mp_obj_t self) {
    mp_obj_aprs_station_t *o = MP_OBJ_TO_PTR(self);
	mp_obj_aprs_position_t *pos = mp_obj_malloc(mp_obj_aprs_position_t, &mp_type_aprs_position);

	if (pos)
		memcpy(&pos->position, &o->station.position, sizeof(struct APRS_Position));

	return MP_OBJ_FROM_PTR(pos);
}
static MP_DEFINE_CONST_FUN_OBJ_1(aprs_station_position_obj, aprs_station_position);

static mp_obj_t aprs_station_course(mp_obj_t self) {
    mp_obj_aprs_station_t *o = MP_OBJ_TO_PTR(self);
	mp_obj_t item[2] = {
		MP_OBJ_NEW_SMALL_INT(o->station.course.speed),
		MP_OBJ_NEW_SMALL_INT(o->station.course.dir)
	};

	return MP_OBJ_FROM_PTR(mp_obj_new_tuple(2, item));
}
static MP_DEFINE_CONST_FUN_OBJ_1(aprs_station_course_obj, aprs_station_course);


// Station local dictionay
static const mp_rom_map_elem_t aprs_station_locals_dict_table[] = {
	{MP_ROM_QSTR(MP_QSTR_callid), MP_ROM_PTR(&aprs_station_callid_obj)},
	{MP_ROM_QSTR(MP_QSTR_symbol), MP_ROM_PTR(&aprs_station_symbol_obj)},
	{MP_ROM_QSTR(MP_QSTR_timestamp), MP_ROM_PTR(&aprs_station_timestamp_obj)},
	{MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&aprs_station_status_obj)},
	{MP_ROM_QSTR(MP_QSTR_position), MP_ROM_PTR(&aprs_station_position_obj)},
	{MP_ROM_QSTR(MP_QSTR_course), MP_ROM_PTR(&aprs_station_course_obj)},
};
static MP_DEFINE_CONST_DICT(aprs_station_locals_dict, aprs_station_locals_dict_table);

// Station class methode
/*
static void *aprs_station_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_obj_aprs_station_t *o = mp_obj_malloc(mp_obj_aprs_station_t, type);
	bzero(&o->station, sizeof(APRS_Station_t));
    return MP_OBJ_FROM_PTR(o);
}
*/

static void aprs_station_print(const mp_print_t *print, mp_obj_t self, mp_print_kind_t kind) {
    (void)kind;
    mp_obj_aprs_station_t *o = MP_OBJ_TO_PTR(self);
	char buff[64];
	int pos;
	float lon, lat;
	struct tm tm;

	pos=0;
	gmtime_r(&o->station.timestamp, &tm);
	pos += strftime(buff + pos,sizeof(buff) - pos, "%d/%m/%y %H:%M : ",&tm);
	
	if (o->station.symbol[0] && o->station.symbol[1]) {
		buff[pos++] = o->station.symbol[0];
		buff[pos++] = o->station.symbol[1];
		buff[pos++] = ' ';
	}
	pos += AX25_Addr_To_Str(&o->station.callid, buff + pos , sizeof(buff)-pos);
	buff[pos++] = '\n';
	buff[pos] = '\0';
	mp_print_str(print, buff);


	if (o->station.position.latitude || o->station.position.longitude || o->station.position.altitude) {
		pos = 0;
		lon = ((float)o->station.position.longitude)/((float)(1L<<22));
		lat = ((float)o->station.position.latitude)/((float)(1L<<22));

		pos += snprintf(buff + pos,sizeof(buff)-pos, " %02f° %c %03f° %c",
				lat<0?-lat:lat,
				lat<0?'S':'N',
				lon<0?-lon:lon,
				lon<0?'W':'E'
				);

		if (o->station.position.altitude) {
			pos += snprintf(buff+pos, sizeof(buff)-pos, " Alt = %dft\n",
					o->station.position.altitude);
		} else
			buff[pos++] = '\n';

		buff[pos] = '\0';

		mp_print_str(print, buff);
	}

	if (o->station.course.dir || o->station.course.speed) {
		pos = 0;
		if (o->station.symbol[1] != '_')
			pos += snprintf(buff + pos, sizeof(buff)-pos, "Course %03d° / %03dkn\n",
					o->station.course.dir,
					o->station.course.speed);
		else
			pos += snprintf(buff + pos, sizeof(buff)-pos, "Wind : %03d° / %03d°\n",
					o->station.course.dir,
					o->station.course.speed);
	
		buff[pos] = '\0';
		mp_print_str(print, buff);
	}

	if (o->station.status[0]) {
		mp_print_str(print, o->station.status);
	}

}

// Station class
MP_DEFINE_CONST_OBJ_TYPE(
	mp_type_aprs_station,
    MP_QSTR_aprs_station,
    MP_TYPE_FLAG_NONE,
    print, aprs_station_print,
    locals_dict, &aprs_station_locals_dict
);
