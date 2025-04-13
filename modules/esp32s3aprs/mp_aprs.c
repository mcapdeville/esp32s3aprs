/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * mp_aprs.c
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
#include <esp_log.h>

#define TAG "MP_APRS"

// Aprs obj methode
static mp_obj_t aprs_status(size_t nargs, const mp_obj_t *args ) {
    mp_obj_aprs_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		char buff[64];
		APRS_Get_Status(o->aprs, buff, sizeof(buff));
		return mp_obj_new_str(buff, strnlen(buff, sizeof(buff)));
	}
	if (nargs == 2 && mp_obj_is_str(args[1]))
		return MP_OBJ_NEW_SMALL_INT(APRS_Send_Status(o->aprs, mp_obj_str_get_str(args[1])));

	return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aprs_status_obj, 1, 2, aprs_status);

static mp_obj_t aprs_position(size_t nargs, const mp_obj_t *args) {
	int ret;
    mp_obj_aprs_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		mp_obj_aprs_position_t *pos = mp_obj_malloc(mp_obj_aprs_position_t, &mp_type_aprs_position);
		if (pos) {
			APRS_Get_Position(o->aprs, &pos->position);
			return MP_OBJ_FROM_PTR(pos);
		}
		return mp_const_none;
	} else if (nargs == 2 && mp_obj_is_str(args[1])) {
		ret = APRS_Send_Position(o->aprs, mp_obj_str_get_str(args[1]));
		return MP_OBJ_NEW_SMALL_INT(ret);
	}

	return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aprs_position_obj, 1, 2, aprs_position);

static mp_obj_t aprs_local(size_t nargs, const mp_obj_t *args) {
    mp_obj_aprs_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		mp_obj_aprs_station_t *sta = mp_obj_malloc(mp_obj_aprs_station_t, &mp_type_aprs_station);
		if (sta) {
			APRS_Get_Local(o->aprs, &sta->station);
			sta->station.status[sizeof(sta->station.status)-1] = '\0';
		}
		return MP_OBJ_FROM_PTR(sta);
	}
	return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aprs_local_obj, 1, 1, aprs_local);

static mp_obj_t aprs_stations(size_t nargs, const mp_obj_t *args) {
    mp_obj_aprs_t *o = MP_OBJ_TO_PTR(args[0]);
	mp_obj_aprs_stations_db_t * stations_db = mp_obj_malloc(mp_obj_aprs_stations_db_t, &mp_type_aprs_stations_db);
	stations_db->aprs = o->aprs;

	if (nargs == 2 && mp_obj_is_type(args[1], &mp_type_ax25_addr)) {
		mp_obj_ax25_addr_t *addr = MP_OBJ_TO_PTR(args[0]);
		memcpy(&stations_db->filter, &addr->addr, sizeof(AX25_Addr_t));
	} else
		bzero(&stations_db->filter, sizeof(AX25_Addr_t));

	return MP_OBJ_FROM_PTR(stations_db);
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aprs_stations_obj, 1, 2, aprs_stations);

static mp_obj_t aprs_callid(size_t nargs, const mp_obj_t * args) {
    mp_obj_aprs_t *o = MP_OBJ_TO_PTR(args[0]);
	int ret;

	if (nargs == 1) {
		mp_obj_ax25_addr_t *addr = mp_obj_malloc(mp_obj_ax25_addr_t, &mp_type_ax25_addr);
		if (!APRS_Get_Callid(o->aprs, &addr->addr))
			return MP_OBJ_FROM_PTR(addr);
	} else if (nargs == 2 && mp_obj_is_str(args[1])) {
		AX25_Addr_t addr;
		const char * str = mp_obj_str_get_str(args[1]);
		if (str) {
			AX25_Str_To_Addr(str, &addr);
			ret = APRS_Set_Callid(o->aprs, &addr);
		} else
			ret = -1;
		return MP_OBJ_NEW_SMALL_INT(ret);
	} else if (nargs == 2 && mp_obj_is_type(args[2], &mp_type_ax25_addr)) {
			mp_obj_ax25_addr_t *addr = MP_OBJ_TO_PTR(args[2]);
			ret = APRS_Set_Callid(o->aprs, &addr->addr);
			return MP_OBJ_NEW_SMALL_INT(ret);
	}

	return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aprs_callid_obj, 1, 2, aprs_callid);

static mp_obj_t aprs_path(size_t nargs, const mp_obj_t * args) {
    mp_obj_aprs_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		int n = 0;
		mp_obj_list_t *lst = mp_obj_malloc(mp_obj_list_t, &mp_type_list);
		mp_obj_list_init(lst, 8);
		lst->len = 0;
		mp_obj_ax25_addr_t *addr = mp_obj_malloc(mp_obj_ax25_addr_t, &mp_type_ax25_addr);
		while (n < APRS_MAX_DIGI && !APRS_Get_Digi(o->aprs, n, &addr->addr)) {
			mp_obj_list_append(lst, addr);
			addr = mp_obj_malloc(mp_obj_ax25_addr_t, &mp_type_ax25_addr);
			n++;
		}
		m_free(addr);
		return MP_OBJ_FROM_PTR(lst);
	} else if (nargs == 2 && mp_obj_is_type(args[1], &mp_type_list)) {
		mp_obj_list_t *lst = MP_OBJ_TO_PTR(args[1]);
		int n = 0,i = 0;
		AX25_Addr_t path[8];
		while (i < lst->len && n < 8) {
			if (mp_obj_is_type(lst->items[i], &mp_type_ax25_addr)) {
				mp_obj_ax25_addr_t *addr = MP_OBJ_TO_PTR(lst->items[i]);
				memcpy(&path[n], &addr->addr, sizeof(AX25_Addr_t));
				n++;
			} else if (mp_obj_is_str(lst->items[i])) {
				const char * str = mp_obj_str_get_str(lst->items[i]);
				AX25_Str_To_Addr(str, &path[n]);
				n++;
			}
			i++;
		}
		return MP_OBJ_NEW_SMALL_INT(APRS_Set_Digis(o->aprs, path, n));
	} if (nargs > 1) {
		int i = 1, n =0;
		AX25_Addr_t path[8];

		while (i < nargs && n < 8) {
			if (mp_obj_is_type(args[i], &mp_type_ax25_addr)) {
				mp_obj_ax25_addr_t *addr = MP_OBJ_TO_PTR(args[i]);
				memcpy(&path[n], &addr->addr, sizeof(AX25_Addr_t));
				n++;
			} else if (mp_obj_is_str(args[i])) {
				const char * str = mp_obj_str_get_str(args[i]);
				AX25_Str_To_Addr(str, &path[n]);
				n++;
			}
			i++;
		}
		return MP_OBJ_NEW_SMALL_INT(APRS_Set_Digis(o->aprs, path, n));
	}
	return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aprs_path_obj, 1, 9, aprs_path);

static mp_obj_t aprs_symbol(size_t nargs, const mp_obj_t *args) {
    mp_obj_aprs_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		char str[2];
		APRS_Get_Symbol(o->aprs, str);
		return MP_OBJ_FROM_PTR(mp_obj_new_str(str, 2));
	} else if (nargs == 2 && mp_obj_is_str(args[1])) {
		size_t len;
		const char *str = mp_obj_str_get_data(args[1], &len);
		if (len == 2)
			return MP_OBJ_NEW_SMALL_INT(APRS_Set_Symbol(o->aprs, str));
	}

	return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aprs_symbol_obj, 1, 2, aprs_symbol) ;

static mp_obj_t aprs_ambiguity(size_t nargs, const mp_obj_t *args) {
    mp_obj_aprs_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		uint8_t ambiguity;
		APRS_Get_Ambiguity(o->aprs, &ambiguity);
		return MP_OBJ_NEW_SMALL_INT(ambiguity);
	} else if (nargs == 2 && mp_obj_is_int(args[1])) {
		uint8_t ambiguity = mp_obj_get_int(args[1]);
		return MP_OBJ_NEW_SMALL_INT(APRS_Set_Ambiguity(o->aprs, ambiguity));
	}

	return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aprs_ambiguity_obj, 1, 2, aprs_ambiguity) ;

// Aprs local dictionary
static const mp_rom_map_elem_t aprs_locals_dict_table[] = {
	{MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&aprs_status_obj)},
	{MP_ROM_QSTR(MP_QSTR_position), MP_ROM_PTR(&aprs_position_obj)},
	{MP_ROM_QSTR(MP_QSTR_local), MP_ROM_PTR(&aprs_local_obj)},
	{MP_ROM_QSTR(MP_QSTR_stations), MP_ROM_PTR(&aprs_stations_obj)},
	{MP_ROM_QSTR(MP_QSTR_callid), MP_ROM_PTR(&aprs_callid_obj)},
	{MP_ROM_QSTR(MP_QSTR_path), MP_ROM_PTR(&aprs_path_obj)},
	{MP_ROM_QSTR(MP_QSTR_symbol), MP_ROM_PTR(&aprs_symbol_obj)},
	{MP_ROM_QSTR(MP_QSTR_ambiguity), MP_ROM_PTR(&aprs_ambiguity_obj)},
};

static MP_DEFINE_CONST_DICT(aprs_locals_dict, aprs_locals_dict_table);

// Aprs class methode
static void *aprs_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_obj_aprs_t *o = mp_obj_malloc(mp_obj_aprs_t, type);
    if (Aprs)
		o->aprs = Aprs;
	else
		return mp_const_none;

    return MP_OBJ_FROM_PTR(o);
}

static void aprs_print(const mp_print_t *print, mp_obj_t self, mp_print_kind_t kind) {
    (void)kind;
	int pos;
	char * ptr;
	char buff[64];

    mp_obj_aprs_t *o = MP_OBJ_TO_PTR(self);

	pos = 0;

	pos += APRS_Get_Symbol_Str(o->aprs, buff + pos, sizeof(buff)-pos);

	buff[pos++] = ' ';
	pos += APRS_Get_Callid_Str(o->aprs, buff + pos, sizeof(buff)-pos);

	buff[pos++] = ' ';
	buff[pos++] = '>';
	buff[pos++] = ' ';
	pos += APRS_Get_Appid_Str(o->aprs, buff + pos, sizeof(buff)-pos);
	ptr = buff + pos;
	pos++;
	pos++;
	pos +=  APRS_Get_Digis_Str(o->aprs, buff + pos, sizeof(buff)-pos);
	if ( ptr[2] ) {
		*(ptr++) = ',';
		*(ptr++) = ' ';
	}

    mp_printf(print, buff);
}

// Aprs class
MP_DEFINE_CONST_OBJ_TYPE(
	mp_type_aprs,
    MP_QSTR_aprs,
    MP_TYPE_FLAG_NONE,
	make_new, aprs_new,
    print, aprs_print,
    locals_dict, &aprs_locals_dict
);
