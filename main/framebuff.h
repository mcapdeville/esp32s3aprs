/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/framebuff.h
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

#ifndef _FRAMEBUFF_H_
#define _FRAMEBUFF_H_

#include <stdint.h>
#include <stddef.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

typedef struct Framebuff_Frame_S Frame_t;
typedef struct Framebuff_S Framebuff_t;

typedef void (*Framebuff_Free_Func_t)(Frame_t * Frame);

struct Framebuff_Frame_S {
	Framebuff_t * parent;
	SemaphoreHandle_t usage;
	StaticSemaphore_t usage_buff;
	size_t frame_size;
	size_t frame_len;
	uint8_t frame[];
};

Framebuff_t * Framebuff_Init(int MaxFrame,size_t Frame_len);
int Framebuff_Put_Frame(Framebuff_t * Framebuff,Frame_t *Frame);
Frame_t * Framebuff_Get_Frame(Framebuff_t * Framebuff);
int Framebuff_Count_Frame(Framebuff_t * Framebuff);

void Framebuff_Inc_Frame_Usage(Frame_t *Frame);
void Framebuff_Free_Frame(Frame_t * Frame);

static inline void Framebuff_Copy_Frame(Frame_t * dst, Frame_t * src) {
	memcpy(dst,src,sizeof(Frame_t)+src->frame_len);
}

#endif
