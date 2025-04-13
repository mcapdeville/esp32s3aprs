/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/ax25.h
 *
 * Copyright (C) 2025  Marc CAPDEVILLE (F4JMZ)
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

#ifndef _AX25_H_
#define _AX25_H_

#include <stdint.h>
#include "hdlc_enc.h"
#include "framebuff.h"

#define AX25_MAX_FRAME_LEN	HDLC_MAX_FRAME_LEN
#define AX25_MIN_FRAME_LEN	HDLC_MIN_FRAME_LEN

#define AX25_MAX_ADDR (10)	

typedef	union __attribute__((packed)) AX25_Addr_U {
	uint8_t addr[7];
	struct __attribute__((packed)) {
		uint8_t callid[6];
		uint8_t ssid;
	};
} AX25_Addr_t;

int AX25_Make_Addr(const char *Str, uint8_t Ssid, AX25_Addr_t * Addr);
int AX25_Str_To_Addr(const char *Str, AX25_Addr_t * Addr);
void AX25_Norm_Addr(AX25_Addr_t * Addr);
int AX25_Addr_To_Str(const AX25_Addr_t *Addr, char *Str, int Len);
int AX25_Addr_Cmp(const AX25_Addr_t *Addr1, const AX25_Addr_t *Addr2);
int AX25_Addr_Count(AX25_Addr_t *Addr);
int AX25_Addr_Filter(AX25_Addr_t *Addr, AX25_Addr_t *Filter);
uint8_t AX25_Get_Addr(int pos, const Frame_t * Frame, AX25_Addr_t *Addr);

#endif
