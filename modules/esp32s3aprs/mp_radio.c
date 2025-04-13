/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * mp_radio.c
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
#include "mp_radio.h"
#include <string.h>

// Main radio object
extern SA8x8_t *SA8x8;

// Radio obj methode
static mp_obj_t radio_power(size_t nargs, const mp_obj_t *args ) {
	if (!nargs)
		return MP_OBJ_NULL;

	mp_obj_radio_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Get_Power(o->sa8x8));
	}

	if (nargs == 2 && mp_obj_is_int(args[1])) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Set_Power(o->sa8x8, mp_obj_get_int(args[1])));
	}

	return MP_OBJ_NULL;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(radio_power_obj, 1, 2, radio_power);

static mp_obj_t radio_freq(size_t nargs, const mp_obj_t *args ) {
	if (!nargs)
		return MP_OBJ_NULL;

	mp_obj_radio_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		return mp_obj_new_int(SA8x8_Get_RxFreq(o->sa8x8));
	}

	if (nargs == 2 && mp_obj_is_int(args[1])) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Set_Freq(o->sa8x8, mp_obj_get_int(args[1])));
	}

	return MP_OBJ_NULL;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(radio_freq_obj, 1, 2, radio_freq);

static mp_obj_t radio_sub(size_t nargs, const mp_obj_t *args ) {
	if (!nargs)
		return MP_OBJ_NULL;

	mp_obj_radio_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		char sub[5];
		SA8x8_Get_RxSub(o->sa8x8, sub, sizeof(sub));
		return mp_obj_new_str(sub,strlen(sub));
	}

	if (nargs == 2 && mp_obj_is_str(args[1])) {
		const char * sub = mp_obj_str_get_str(args[1]);
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Set_Sub(o->sa8x8, sub));
	}

	return MP_OBJ_NULL;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(radio_sub_obj, 1, 2, radio_sub);

static mp_obj_t radio_bandwidth(size_t nargs, const mp_obj_t *args ) {
	if (!nargs)
		return MP_OBJ_NULL;

	mp_obj_radio_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Get_Bandwidth(o->sa8x8));
	}

	if (nargs == 2 && mp_obj_is_int(args[1])) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Set_Bandwidth(o->sa8x8, mp_obj_get_int(args[1])));
	}

	return MP_OBJ_NULL;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(radio_bandwidth_obj, 1, 2, radio_bandwidth);

static mp_obj_t radio_squelch(size_t nargs, const mp_obj_t *args ) {
	if (!nargs)
		return MP_OBJ_NULL;

	mp_obj_radio_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Get_Squelch(o->sa8x8));
	}

	if (nargs == 2 && mp_obj_is_int(args[1])) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Set_Squelch(o->sa8x8, mp_obj_get_int(args[1])));
	}

	return MP_OBJ_NULL;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(radio_squelch_obj, 1, 2, radio_squelch);

static mp_obj_t radio_volume(size_t nargs, const mp_obj_t *args ) {
	if (!nargs)
		return MP_OBJ_NULL;

	mp_obj_radio_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Get_Volume(o->sa8x8));
	}

	if (nargs == 2 && mp_obj_is_int(args[1])) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Set_Volume(o->sa8x8, mp_obj_get_int(args[1])));
	}

	return MP_OBJ_NULL;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(radio_volume_obj, 1, 2, radio_volume);

static mp_obj_t radio_tail(size_t nargs, const mp_obj_t *args ) {
	if (!nargs)
		return MP_OBJ_NULL;

	mp_obj_radio_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Get_Tail(o->sa8x8));
	}

	if (nargs == 2 && mp_obj_is_int(args[1])) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Set_Tail(o->sa8x8, mp_obj_get_int(args[1])));
	}

	return MP_OBJ_NULL;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(radio_tail_obj, 1, 2, radio_tail);

static mp_obj_t radio_emphasis(size_t nargs, const mp_obj_t *args ) {
	if (!nargs)
		return MP_OBJ_NULL;

	mp_obj_radio_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Get_Emphasis(o->sa8x8));
	}

	if (nargs == 2 && mp_obj_is_int(args[1])) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Set_Emphasis(o->sa8x8, mp_obj_get_int(args[1])));
	}

	return MP_OBJ_NULL;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(radio_emphasis_obj, 1, 2, radio_emphasis);

static mp_obj_t radio_hipass(size_t nargs, const mp_obj_t *args ) {
	if (!nargs)
		return MP_OBJ_NULL;

	mp_obj_radio_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Get_Hipass(o->sa8x8));
	}

	if (nargs == 2 && mp_obj_is_int(args[1])) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Set_Hipass(o->sa8x8, mp_obj_get_int(args[1])));
	}

	return MP_OBJ_NULL;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(radio_hipass_obj, 1, 2, radio_hipass);

static mp_obj_t radio_lowpass(size_t nargs, const mp_obj_t *args ) {
	if (!nargs)
		return MP_OBJ_NULL;

	mp_obj_radio_t *o = MP_OBJ_TO_PTR(args[0]);

	if (nargs == 1) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Get_Lowpass(o->sa8x8));
	}

	if (nargs == 2 && mp_obj_is_int(args[1])) {
		return MP_OBJ_NEW_SMALL_INT(SA8x8_Set_Lowpass(o->sa8x8, mp_obj_get_int(args[1])));
	}

	return MP_OBJ_NULL;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(radio_lowpass_obj, 1, 2, radio_lowpass);

// Radio local dictionary
static const mp_rom_map_elem_t radio_locals_dict_table[] = {
	{MP_ROM_QSTR(MP_QSTR_power), MP_ROM_PTR(&radio_power_obj)},
	{MP_ROM_QSTR(MP_QSTR_freq), MP_ROM_PTR(&radio_freq_obj)},
	{MP_ROM_QSTR(MP_QSTR_sub), MP_ROM_PTR(&radio_sub_obj)},
	{MP_ROM_QSTR(MP_QSTR_bandwidth), MP_ROM_PTR(&radio_bandwidth_obj)},
	{MP_ROM_QSTR(MP_QSTR_squelch), MP_ROM_PTR(&radio_squelch_obj)},
	{MP_ROM_QSTR(MP_QSTR_volume), MP_ROM_PTR(&radio_volume_obj)},
	{MP_ROM_QSTR(MP_QSTR_tail), MP_ROM_PTR(&radio_tail_obj)},
	{MP_ROM_QSTR(MP_QSTR_emphasis), MP_ROM_PTR(&radio_emphasis_obj)},
	{MP_ROM_QSTR(MP_QSTR_hipass), MP_ROM_PTR(&radio_hipass_obj)},
	{MP_ROM_QSTR(MP_QSTR_lowpass), MP_ROM_PTR(&radio_lowpass_obj)},
};
static MP_DEFINE_CONST_DICT(radio_locals_dict, radio_locals_dict_table);


// Radio class methode
static void *radio_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_obj_radio_t *o = mp_obj_malloc(mp_obj_radio_t, type);
    o->sa8x8 = SA8x8;
    return MP_OBJ_FROM_PTR(o);
}

static void radio_print(const mp_print_t *print, mp_obj_t self, mp_print_kind_t kind) {
    (void)kind;
	char buff[64];
    mp_obj_radio_t *o = MP_OBJ_TO_PTR(self);

	SA8x8_GetVersion(o->sa8x8, buff, sizeof(buff));
    mp_printf(print, buff);
}

// Radio class
MP_DEFINE_CONST_OBJ_TYPE(
	mp_type_radio,
    MP_QSTR_radio,
    MP_TYPE_FLAG_NONE,
	make_new, radio_new,
    print, radio_print,
    locals_dict, &radio_locals_dict
);

