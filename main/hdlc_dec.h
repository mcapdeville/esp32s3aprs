/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/hdlc_dec.h
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

#ifndef _HDLC_DEC_H_
#define _HDLC_DEC_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <framebuff.h>

#ifndef HDLC_MAX_FRAMEçLEN
#define HDLC_MAX_FRAME_LEN	(330)	// 2*7 address + 8*7 relay + 1 control + 1 pid + 256 info + 2 fcs
#endif

#ifndef HDLC_MIN_FRAME_LEN
#define HDLC_MIN_FRAME_LEN	(17)	// 2*7 address + 1 control + 2 fcs
#endif

typedef struct Hdlc_Dec_S Hdlc_Dec_t;
typedef void (*Hdlc_Dec_Cb_t)(void * arg, Frame_t *Frame);

Hdlc_Dec_t * Hdlc_Dec_Init(Hdlc_Dec_Cb_t Cb, void * arg);
void Hdlc_Dec_Reset(Hdlc_Dec_t * Hdlc);
int Hdlc_Dec_Input(Hdlc_Dec_t * Hdlc,uint8_t * Bitstream, int BitLen);
void Hdlc_Dec_Add_Frame(Hdlc_Dec_t * Hdlc, Frame_t *Frame);
bool HDLC_Dec_Get_Sync(Hdlc_Dec_t * Hdlc);

#endif
