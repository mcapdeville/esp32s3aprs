/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * mp_aprs_stations_db.c
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
#include "berkeley-db/db.h"

// Aprs stations db methods
static mp_obj_t aprs_stations_db_filter(size_t nargs, const mp_obj_t * args) {
    mp_obj_aprs_stations_db_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		mp_obj_ax25_addr_t *addr = mp_obj_malloc(mp_obj_ax25_addr_t, &mp_type_ax25_addr);
		memcpy(&addr->addr, &o->filter, sizeof(AX25_Addr_t));
		return MP_OBJ_FROM_PTR(addr);
	} else if (nargs == 2 && mp_obj_is_type(args[1], &mp_type_ax25_addr)) {
		mp_obj_ax25_addr_t *addr = MP_OBJ_TO_PTR(args[1]);
		memcpy(&o->filter, &addr->addr, sizeof(AX25_Addr_t));
	} else if (nargs == 2 && mp_obj_is_str(args[1])) {
			const char * str = mp_obj_str_get_str(args[1]);
			AX25_Str_To_Addr(str, &o->filter);
	}

	o->filter_type = 0;
	int i=0;
	while (i < 6 && o->filter.addr[i])
		i++;
	if (i<6)
		o->filter_type |= 2;

	if (!o->filter.ssid)
		o->filter_type |= 1;

	return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aprs_stations_db_filter_obj, 1, 2, aprs_stations_db_filter);

// Aprs stations database local dictionary
static const mp_rom_map_elem_t aprs_stations_db_locals_dict_table[] = {
	{MP_ROM_QSTR(MP_QSTR_filter), MP_ROM_PTR(&aprs_stations_db_filter_obj)},
};

static MP_DEFINE_CONST_DICT(aprs_stations_db_locals_dict, aprs_stations_db_locals_dict_table);

// Aprs stations database class methode
/*
static void *aprs_stations_db_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_obj_aprs_stations_db_t *o = mp_obj_malloc(mp_obj_aprs_stations_db_t, type);
	// Initialise here
	o->aprs = Aprs;
	
    return MP_OBJ_FROM_PTR(o);
}
*/

static void aprs_stations_db_print(const mp_print_t *print, mp_obj_t self, mp_print_kind_t kind) {
    (void)kind;
    mp_obj_aprs_stations_db_t *o = MP_OBJ_TO_PTR(self);

    mp_printf(print, "Aprs stations database object @%p", o);
}

static mp_obj_t aprs_stations_db_subscr(mp_obj_t self, mp_obj_t callid, mp_obj_t value) {
    mp_obj_aprs_stations_db_t *o = MP_OBJ_TO_PTR(self);
	AX25_Addr_t addr;

    if (mp_obj_is_str(callid))
			AX25_Str_To_Addr(mp_obj_str_get_str(callid), &addr);
	else if (mp_obj_is_type(callid, &mp_type_ax25_addr)) {
		mp_obj_ax25_addr_t *mp_addr = MP_OBJ_TO_PTR(callid);
		memcpy(&addr, &mp_addr->addr, sizeof(AX25_Addr_t));
	} else
		return mp_const_none;

	if (value == MP_OBJ_SENTINEL) { // Get
		mp_obj_aprs_station_t *sta = mp_obj_malloc(mp_obj_aprs_station_t, &mp_type_aprs_station);
		if (sta) {
			if (!APRS_Get_Station(o->aprs, &addr, &sta->station)) {
				sta->station.status[sizeof(sta->station.status)-1] = '\0';
				return MP_OBJ_FROM_PTR(sta);
			}
		}
	} else if (mp_obj_is_type(value, &mp_type_aprs_station)) { // Set
	}		

	return mp_const_none;
}

// Aprs station db iterator
typedef struct _mp_obj_aprs_stations_db_it_t {
    mp_obj_base_t base;
    mp_fun_1_t iternext;
    mp_obj_aprs_stations_db_t * aprs_stations_db;
	uint8_t next_flag;
} mp_obj_aprs_stations_db_it_t;

mp_obj_t aprs_stations_db_it_iternext(mp_obj_t self) {
	int ret = -1;
    mp_obj_aprs_stations_db_it_t *o = MP_OBJ_TO_PTR(self);
	mp_obj_aprs_station_t *sta = mp_obj_malloc(mp_obj_aprs_station_t, &mp_type_aprs_station);

	if (sta) {
		ret = APRS_Stations_Seq(o->aprs_stations_db->aprs, &o->aprs_stations_db->filter, o->next_flag, &sta->station);
		o->next_flag = R_NEXT;
		// for case where filter is in form 'CALL*-10'
		if (o->aprs_stations_db->filter_type == 2) {
			// so walk db until the end
			while (!ret && o->aprs_stations_db->filter.ssid && AX25_Addr_Cmp(&o->aprs_stations_db->filter, &sta->station.callid)) 
				ret = APRS_Stations_Seq(o->aprs_stations_db->aprs, &o->aprs_stations_db->filter, o->next_flag, &sta->station);
		}

	   if (!ret) {
			return MP_OBJ_FROM_PTR(sta);
	   }
	}
	o->next_flag = 0;
	return MP_OBJ_STOP_ITERATION;
}

static mp_obj_t mp_obj_new_aprs_stations_db_iterator(mp_obj_t self, AX25_Addr_t *First, mp_obj_iter_buf_t *iter_buf) {
    assert(sizeof(mp_obj_aprs_stations_db_it_t) <= sizeof(mp_obj_iter_buf_t));
    mp_obj_aprs_stations_db_it_t *o = (mp_obj_aprs_stations_db_it_t*)iter_buf;
    o->base.type = &mp_type_polymorph_iter;
	o->iternext = aprs_stations_db_it_iternext,
    o->aprs_stations_db = MP_OBJ_TO_PTR(self);
	if (First) {
		memcpy(&o->aprs_stations_db->filter, First, sizeof(AX25_Addr_t));
		o->next_flag = R_CURSOR;
	}
	else {
		bzero(&o->aprs_stations_db->filter, sizeof(AX25_Addr_t));
		o->next_flag = R_FIRST;
	}
    return MP_OBJ_FROM_PTR(o);
}

static mp_obj_t aprs_stations_db_getiter(mp_obj_t o_in, mp_obj_iter_buf_t *iter_buf) {
    return mp_obj_new_aprs_stations_db_iterator(o_in, &((mp_obj_aprs_stations_db_t*)o_in)->filter, iter_buf);
}

// Aprs stations database class
MP_DEFINE_CONST_OBJ_TYPE(
	mp_type_aprs_stations_db,
    MP_QSTR_aprs_stations_db,
    MP_TYPE_FLAG_ITER_IS_GETITER,
    print, aprs_stations_db_print,
	subscr, aprs_stations_db_subscr,
	iter, aprs_stations_db_getiter,
    locals_dict, &aprs_stations_db_locals_dict
);
