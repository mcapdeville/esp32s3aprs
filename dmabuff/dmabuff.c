/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * dmabuff/dmabuff.c
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

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include "dmabuff.h"

#define TAG "DMABUFF"

int Dmabuff_Init(struct Dmabuff_S * Buffer) {
	int i;

	if (!Buffer)
		return -1;

	Buffer->sem = xSemaphoreCreateMutexStatic(&Buffer->sem_data);

	Buffer->capacity = 0;
	Buffer->first_block=-1;
	Buffer->last_block=-1;

	for (i=0 ; i<DMABUFF_MAX_ACCESSORS; i++) {
		Buffer->accessors[i].current = -1;
	}

	for (i=0; i<DMABUFF_MAX_BLOCKS; i++) {
		Buffer->blocks[i].len = 0;
		Buffer->blocks[i].ptr = NULL;
	}

	return 0;
}

void Dmabuff_Clear(struct Dmabuff_S * Buffer) {
	int i;
	if (!Buffer)
		return;
#ifndef DMABUFF_NO_LOCK
if( portCHECK_IF_IN_ISR() == pdFALSE )
	xSemaphoreTake(Buffer->sem,portMAX_DELAY);
else
	ESP_LOGE(TAG,"IN_ISR_CONTEXT");
#endif

	Buffer->capacity = 0;
	Buffer->first_block=-1;
	Buffer->last_block=-1;

	for (i=0 ; i<DMABUFF_MAX_ACCESSORS; i++) {
		Buffer->accessors[i].current = -1;
		Buffer->accessors[i].len = 0;
	}

	for (i=0; i<DMABUFF_MAX_BLOCKS; i++) {
		Buffer->blocks[i].len = 0;
		Buffer->blocks[i].ptr = NULL;
	}

#ifndef DMABUFF_NO_LOCK
if( portCHECK_IF_IN_ISR() == pdFALSE )
	xSemaphoreGive(Buffer->sem);
#endif
}

size_t Dmabuff_Add_Block(struct Dmabuff_S * Buffer, void * Block, size_t Len) {
	int i;
	size_t ret;

	if (!Buffer)
		return 0;

	if (!Block || !Len)
		return Buffer->capacity;

#ifndef DMABUFF_NO_LOCK
if( portCHECK_IF_IN_ISR() == pdFALSE )
	xSemaphoreTake(Buffer->sem,portMAX_DELAY);
else
	ESP_LOGE(TAG,"IN_ISR_CONTEXT");
#endif

	// Advance last_block index
	if (Buffer->last_block < (DMABUFF_MAX_BLOCKS-1))
		Buffer->last_block = Buffer->last_block+1;
	else
		Buffer->last_block = 0;

	if (Buffer->first_block != -1) {
		if (Buffer->last_block == Buffer->first_block || Block == Buffer->blocks[Buffer->first_block].ptr) {	// overwriting oldest block
			int current;

			// Delete old first_block
			Buffer->capacity -= Buffer->blocks[Buffer->first_block].len;
			Buffer->blocks[Buffer->first_block].len = 0;
			Buffer->blocks[Buffer->first_block].ptr = NULL;

			current = Buffer->first_block;

			// Advance first_block index
			if (Buffer->first_block < (DMABUFF_MAX_BLOCKS-1)) 
				Buffer->first_block++;
			else Buffer->first_block = 0;

			// Discard late accessors
			for (i=0;i<DMABUFF_MAX_ACCESSORS;i++)
				if (Buffer->accessors[i].current == current && Buffer->accessors[i].len)
					Buffer->accessors[i].current = -1;
		}
	} else {
		Buffer->first_block = Buffer->last_block;
	}

	// Add new block
	Buffer->capacity += Len;
	Buffer->blocks[Buffer->last_block].len = Len;
	Buffer->blocks[Buffer->last_block].ptr = Block;

	// update accessors len
	for (i=0;i<DMABUFF_MAX_ACCESSORS;i++)
		if (Buffer->accessors[i].current != -1)
			Buffer->accessors[i].len += Len;

	ret = Buffer->capacity;

#ifndef DMABUFF_NO_LOCK
if( portCHECK_IF_IN_ISR() == pdFALSE )
	xSemaphoreGive(Buffer->sem);
#endif

	return ret;
}

size_t Dmabuff_Get_Capacity(struct Dmabuff_S * Buffer) {
	size_t ret;
	if (!Buffer)
		return 0;

#ifndef DMABUFF_NO_LOCK
if( portCHECK_IF_IN_ISR() == pdFALSE )
	xSemaphoreTake(Buffer->sem,portMAX_DELAY);
else
	ESP_LOGE(TAG,"IN_ISR_CONTEXT");
#endif

	ret = Buffer->capacity;

#ifndef DMABUFF_NO_LOCK
if( portCHECK_IF_IN_ISR() == pdFALSE )
	xSemaphoreGive(Buffer->sem);
#endif

	return ret;

}

size_t Dmabuff_Get_Len(struct Dmabuff_S * Buffer, int Accessor) {
	int ret;
	
	if (!Buffer)
		return 0;

#ifndef DMABUFF_NO_LOCK
if( portCHECK_IF_IN_ISR() == pdFALSE )
	xSemaphoreTake(Buffer->sem,portMAX_DELAY);
else
	ESP_LOGE(TAG,"IN_ISR_CONTEXT");
#endif

	if (Buffer->accessors[Accessor].current == -1) {	// initialize accessor
		if (!Buffer->capacity)
			Buffer->accessors[Accessor].len = 0;
		else {
			Buffer->accessors[Accessor].current = Buffer->first_block;
			Buffer->accessors[Accessor].len = Buffer->capacity;
			Buffer->accessors[Accessor].pos = 0;
		}
	}

	ret = Buffer->accessors[Accessor].len;

#ifndef DMABUFF_NO_LOCK
if( portCHECK_IF_IN_ISR() == pdFALSE )
	xSemaphoreGive(Buffer->sem);
#endif

	return ret;
}

size_t Dmabuff_Get_Ptr(struct Dmabuff_S * Buffer, int Accessor, void ** pPtr, size_t * pLen) {
	int ret;
	
	if (!Buffer) {
		*pLen = 0;
		*pPtr = NULL;
		return 0;
	}

#ifndef DMABUFF_NO_LOCK
if( portCHECK_IF_IN_ISR() == pdFALSE )
	xSemaphoreTake(Buffer->sem,portMAX_DELAY);
else
	ESP_LOGE(TAG,"IN_ISR_CONTEXT");
#endif

	if (Buffer->accessors[Accessor].current == -1) {	// initialize accessor
		if (!Buffer->capacity)
			Buffer->accessors[Accessor].len = 0;
		else {
			Buffer->accessors[Accessor].current = Buffer->first_block;
			Buffer->accessors[Accessor].len = Buffer->capacity;
			Buffer->accessors[Accessor].pos = 0;
		}
	}

	if (Buffer->accessors[Accessor].len) {
		if (pPtr)
			*pPtr = ((char*)Buffer->blocks[Buffer->accessors[Accessor].current].ptr)+Buffer->accessors[Accessor].pos;
		if (pLen)
			*pLen = Buffer->blocks[Buffer->accessors[Accessor].current].len-Buffer->accessors[Accessor].pos;
	} else {
		if (pPtr)
			*pPtr = NULL;
		if (pLen)
			*pLen = 0;
	}

	ret = Buffer->accessors[Accessor].len;

#ifndef DMABUFF_NO_LOCK
if( portCHECK_IF_IN_ISR() == pdFALSE )
	xSemaphoreGive(Buffer->sem);
#endif

	return ret;
}

size_t Dmabuff_Advance_Ptr(struct Dmabuff_S * Buffer,int Accessor, size_t Len) {
	int ret;
	size_t current,apos,blen;
	
	if (!Buffer)
		return 0;

#ifndef DMABUFF_NO_LOCK
if( portCHECK_IF_IN_ISR() == pdFALSE )
	xSemaphoreTake(Buffer->sem,portMAX_DELAY);
else
	ESP_LOGE(TAG,"IN_ISR_CONTEXT");
#endif

	if (Buffer->accessors[Accessor].current == -1) {	// initialize accessor
		if (!Buffer->capacity)
			Buffer->accessors[Accessor].len = 0;
		else {
			Buffer->accessors[Accessor].current = Buffer->first_block;
			Buffer->accessors[Accessor].len = Buffer->capacity;
			Buffer->accessors[Accessor].pos = 0;
		}
	}

	if (Len > Buffer->accessors[Accessor].len)
		Len = Buffer->accessors[Accessor].len;

	current = Buffer->accessors[Accessor].current;
	while (Len) {
		apos = Buffer->accessors[Accessor].pos;
		blen = Buffer->blocks[current].len;
		if (Len < (blen-apos)) {
			Buffer->accessors[Accessor].pos += Len;
			Buffer->accessors[Accessor].len -= Len;
			Len =0;
		} else {
			Buffer->accessors[Accessor].pos = 0;
			Buffer->accessors[Accessor].len -= blen - apos;
			Len -= blen - apos;
			if (current < (DMABUFF_MAX_BLOCKS-1))
				current++;
			else
				current = 0;

			Buffer->accessors[Accessor].current = current;
		}
	}

	ret = Buffer->accessors[Accessor].len;

#ifndef DMABUFF_NO_LOCK
if( portCHECK_IF_IN_ISR() == pdFALSE )
	xSemaphoreGive(Buffer->sem);
#endif

	return ret;
}
