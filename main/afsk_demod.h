/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/afsk_demod.h
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

#ifndef _AFSK_DEMOD_H_
#define _AFSK_DEMOD_H_

#include <stdint.h>
#include "afsk_config.h"

typedef struct AFSK_Demod_S AFSK_Demod_t;

AFSK_Demod_t* AFSK_Demod_Init(AFSK_Config_t const *Config);
uint16_t AFSK_Demod_Input(AFSK_Demod_t * Demod,int16_t *Samples,uint16_t Len,uint8_t * Out_buff, int16_t buff_size, uint16_t *Out_len);
void AFSK_Demod_Reset(AFSK_Demod_t * Demod);
void AFSK_Demod_Get_Buffs(AFSK_Demod_t * Demod,float ** Input,uint16_t *Input_len,float ** Mark, float ** Space,uint16_t * Decim_len);
bool AFSK_Demod_Get_DCD(AFSK_Demod_t * Demod);

#endif
