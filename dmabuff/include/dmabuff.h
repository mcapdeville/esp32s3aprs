/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * dmabuff/dmabuff.h
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

#ifndef _DMABUFF_H_
#define _DMABUFF_H_

#define DMABUFF_MAX_ACCESSORS 4
#define DMABUFF_MAX_BLOCKS  (CONFIG_ADC_CONTINUOUS_NUM_DMA-2)


struct Dmabuff_Accessor_S {
	size_t len;	// Total remaining size
	int current; // current block index
	size_t pos;	// next byte index
};

struct Dmabuff_Block_S {
	size_t len;	// len of the block
	void * ptr;	// pointer to the block
};

typedef struct Dmabuff_S Dmabuff_t;

struct Dmabuff_S {
	SemaphoreHandle_t sem;
	StaticSemaphore_t sem_data;
	size_t capacity;	// total capacity in bytes
	int first_block;	// oldest block index
	int last_block;		// newest block index
	struct Dmabuff_Accessor_S accessors[DMABUFF_MAX_ACCESSORS];
	struct Dmabuff_Block_S blocks[DMABUFF_MAX_BLOCKS];
};

int Dmabuff_Init(struct Dmabuff_S * buffer);
void Dmabuff_Clear(struct Dmabuff_S * buffer);
size_t Dmabuff_Add_Block(struct Dmabuff_S * Buffer, void * Block, size_t Len);
size_t Dmabuff_Get_Capacity(struct Dmabuff_S * Buffer);
size_t Dmabuff_Get_Len(struct Dmabuff_S * Buffer, int Accessor);
size_t Dmabuff_Get_Ptr(struct Dmabuff_S * Buffer, int Accessor, void ** pPtr, size_t * pLen);
size_t Dmabuff_Advance_Ptr(struct Dmabuff_S * Buffer,int Accessor, size_t Len);

#endif
