/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/afsk_mod.h
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

#ifndef _AFSK_MOD_H_
#define _AFSK_MOD_H_

#include <stdint.h>
#include <stddef.h>
#include "afsk_config.h"

typedef struct AFSK_Mod_S AFSK_Mod_t;

AFSK_Mod_t* AFSK_Mod_Init(AFSK_Config_t const *Config);
uint16_t AFSK_Mod_Output(AFSK_Mod_t * Mod,uint8_t ** Out_bit, uint16_t *Bit_len,int16_t * Buff,uint16_t Buff_len);
void AFSK_Mod_Reset(AFSK_Mod_t * Mod);

#endif
