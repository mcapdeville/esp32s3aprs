/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * mp_ax25_addr.c
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

// Ax25 address obj methode

// Ax25 address local dictionary
static const mp_rom_map_elem_t ax25_addr_locals_dict_table[] = {
};

static MP_DEFINE_CONST_DICT(ax25_addr_locals_dict, ax25_addr_locals_dict_table);

// Ax25 address class methode
static void *ax25_addr_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_obj_ax25_addr_t *o = mp_obj_malloc(mp_obj_ax25_addr_t, type);
	// Initialise here
	if (n_args == 1) {
	   if (mp_obj_is_str(args[0])) {
			const char * str = mp_obj_str_get_str(args[0]);
			AX25_Str_To_Addr(str, &o->addr);
	   } else if (mp_obj_is_type(args[0], &mp_type_ax25_addr)) {
		   mp_obj_ax25_addr_t *src = MP_OBJ_TO_PTR(args[0]);
		   memcpy(&o->addr, &src->addr, sizeof(AX25_Addr_t));
	   }
	} else if (n_args == 2) {
		if (mp_obj_is_str(args[0]) && mp_obj_is_int(args[1])) {
			const char * name = mp_obj_str_get_str(args[0]);
			AX25_Make_Addr(name, mp_obj_get_int(args[1]), &o->addr);
		}
	}
	
    return MP_OBJ_FROM_PTR(o);
}

static void ax25_addr_print(const mp_print_t *print, mp_obj_t self, mp_print_kind_t kind) {
    (void)kind;
	char buff[11];
    mp_obj_ax25_addr_t *o = MP_OBJ_TO_PTR(self);

	AX25_Addr_To_Str(&o->addr, buff, sizeof(buff));
    mp_printf(print, buff);
}

// Ax25 address class
MP_DEFINE_CONST_OBJ_TYPE(
	mp_type_ax25_addr,
    MP_QSTR_ax25_addr,
    MP_TYPE_FLAG_NONE,
	make_new, ax25_addr_new,
    print, ax25_addr_print,
    locals_dict, &ax25_addr_locals_dict
);

