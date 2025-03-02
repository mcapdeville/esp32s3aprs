/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * mp_templ.c
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
#include "mp_templ.h"

// Template obj methode
static mp_obj_t templ_method_var(size_t nargs, const mp_obj_t *args ) {
    mp_obj_templ_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		// get
		return MP_OBJ_FROM_PTR(o);
	}
	if (nargs == 2 && mp_obj_is_type(args[1], &mp_type_templ)) {
		// set
		return MP_OBJ_FROM_PTR(o);
	}

	return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(templ_method_var_obj, 1, 2, templ_method_var);

static mp_obj_t templ_method_1(mp_obj_t self) {
    mp_obj_templ_t *o = MP_OBJ_TO_PTR(self);

	return MP_OBJ_FROM_PTR(o);
}
static MP_DEFINE_CONST_FUN_OBJ_1(templ_method_1_obj, templ_method_1);

// Template local dictionary
static const mp_rom_map_elem_t templ_locals_dict_table[] = {
	{MP_ROM_QSTR(MP_QSTR_method_var), MP_ROM_PTR(&templ_method_var_obj)},
	{MP_ROM_QSTR(MP_QSTR_method_1), MP_ROM_PTR(&templ_method_1_obj)},
};

static MP_DEFINE_CONST_DICT(templ_locals_dict, templ_locals_dict_table);

// Template class methode
static void *templ_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_obj_templ_t *o = mp_obj_malloc(mp_obj_templ_t, type);
	// Initialise here
	
    return MP_OBJ_FROM_PTR(o);
}

static void templ_print(const mp_print_t *print, mp_obj_t self, mp_print_kind_t kind) {
    (void)kind;
    mp_obj_templ_t *o = MP_OBJ_TO_PTR(self);

    mp_printf(print, "Template object @%p", o);
}

// Template class
MP_DEFINE_CONST_OBJ_TYPE(
	mp_type_templ,
    MP_QSTR_templ,
    MP_TYPE_FLAG_NONE,
	make_new, templ_new,
    print, templ_print,
    locals_dict, &templ_locals_dict
);
