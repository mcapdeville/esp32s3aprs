/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/hdlc_enc.h
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

#ifndef _HDLC_ENC_H_
#define _HDLC_ENC_H_

#include <stdint.h>
#include <stddef.h>
#include "framebuff.h"

#ifndef HDLC_MAX_FRAMEçLEN
#define HDLC_MAX_FRAME_LEN	(330)	// 2*7 address + 8*7 relay + 1 control + 1 pid + 256 info + 2 fcs
#endif

#ifndef HDLC_MIN_FRAME_LEN
#define HDLC_MIN_FRAME_LEN	(17)	// 2*7 address + 1 control + 2 fcs
#endif

typedef struct Hdlc_Enc_S Hdlc_Enc_t;
typedef void (*Hdlc_Enc_Cb_t)(void * Arg, Frame_t *Frame);

Hdlc_Enc_t * Hdlc_Enc_Init(Hdlc_Enc_Cb_t Cb, void * Arg);
void Hdlc_Enc_Reset(Hdlc_Enc_t * Hdlc);
size_t Hdlc_Enc_Output(Hdlc_Enc_t * Hdlc,uint8_t * Bitstream, size_t Len);
int Hdlc_Enc_Add_Frame(Hdlc_Enc_t * Hdlc, Frame_t * Frame);

#endif
