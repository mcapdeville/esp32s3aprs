/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * mp_ax25_addr.h
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

#ifndef _MP_AX25_ADDR_H_
#define _MP_AX25_ADDR_H_

#include "../../main/ax25.h"

// Ax25 address obj type
typedef struct _mp_obj_ax25_addr_t {
    mp_obj_base_t base;
	// Put your data herea
	AX25_Addr_t addr;
} mp_obj_ax25_addr_t;

extern const mp_obj_type_t mp_type_ax25_addr;

#endif
