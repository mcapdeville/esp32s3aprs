/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/framebuff.c
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

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include "framebuff.h"

#define TAG "Framebuff"

#define USE_MUTEX 1

struct Framebuff_S {
	int max_frames;
	int nb_frames;	// Added - freed
	int first_frame; // next to be get 
	int last_frame;	 // next to be put
	SemaphoreHandle_t sem;
	StaticSemaphore_t sem_data;
	struct Framebuff_Frame_S *frames[];
}; 

Framebuff_t * Framebuff_Init(int MaxFrames,size_t Frame_len) {
	Framebuff_t * framebuff;

	if (!MaxFrames)
		return NULL;

	if (!(framebuff=malloc(sizeof(struct Framebuff_S)+sizeof(struct Framebuff_Frame_S*)*MaxFrames))) {
		ESP_LOGE(TAG,"Error allocating framebuff struct");
		return NULL;
	}

	bzero(framebuff,sizeof(struct Framebuff_S)+sizeof(struct Framebuff_Frame_S*)*MaxFrames);

	framebuff->sem = xSemaphoreCreateMutexStatic(&framebuff->sem_data);

	framebuff->max_frames = MaxFrames;

	if (Frame_len) {
		size_t frame_size = ((sizeof(struct Framebuff_Frame_S)+Frame_len+3)&~3);
		// Fill the buffer with empty frame
		if (!(framebuff->frames[0] = malloc(MaxFrames*frame_size))) {
			ESP_LOGE(TAG,"Error allocating framebuffer pool");
		} else {
			int i;
			for (i=0;i<MaxFrames;i++) {
				framebuff->frames[i] = (struct Framebuff_Frame_S*)(((uint8_t*)(framebuff->frames[0]))+(i*frame_size));
				framebuff->frames[i]->parent = framebuff;
				framebuff->frames[i]->frame_size = Frame_len;
				framebuff->frames[i]->frame_len = 0;
				if (!(framebuff->frames[i]->usage = xSemaphoreCreateCountingStatic(255,0,&(framebuff->frames[i]->usage_buff))))
					*((uint8_t*)&framebuff->frames[i]->usage_buff) = 0;
			}
			framebuff->nb_frames = MaxFrames;
		}

	}

	return framebuff;
}

int Framebuff_Put_Frame(Framebuff_t * Framebuff,Frame_t *Frame) {
	if (!Framebuff)
		return -ENODEV;

	ESP_LOGD(TAG,"Putting frame %p in buffer %p",Frame,Framebuff);

	xSemaphoreTake(Framebuff->sem,portMAX_DELAY);

	if (Framebuff->nb_frames == Framebuff->max_frames) {
		xSemaphoreGive(Framebuff->sem);
		ESP_LOGE(TAG,"Framebuff full putting frame %p in buffer %p",Frame,Framebuff);
		return -ENOMEM;
	}

	Framebuff->frames[Framebuff->last_frame] = Frame;

	if ((++Framebuff->last_frame) == Framebuff->max_frames)
		Framebuff->last_frame = 0;

	Framebuff->nb_frames++;

	xSemaphoreGive(Framebuff->sem);

	
	return 0;
}

Frame_t * Framebuff_Get_Frame(Framebuff_t * Framebuff) {
	Frame_t * frame;

	if (!Framebuff)
		return NULL;

	xSemaphoreTake(Framebuff->sem,portMAX_DELAY);

	if (!Framebuff->nb_frames) {
		xSemaphoreGive(Framebuff->sem);
		ESP_LOGV(TAG,"Buffer empty getting frame from buffer %p",Framebuff);
		return NULL;		
	}

	frame = Framebuff->frames[Framebuff->first_frame];

	if ((++Framebuff->first_frame) == Framebuff->max_frames)
		Framebuff->first_frame = 0;

	Framebuff->nb_frames--;

	xSemaphoreGive(Framebuff->sem);

	ESP_LOGD(TAG,"Getting frame %p from buffer %p",frame,Framebuff);

	return frame;
}

int Framebuff_Count_Frame(Framebuff_t * Framebuff) {
	int cnt;

	if (!Framebuff)
		return -ENODEV;

	xSemaphoreTake(Framebuff->sem,portMAX_DELAY);

	cnt = Framebuff->nb_frames;

	xSemaphoreGive(Framebuff->sem);

	return cnt;
}

	
void Framebuff_Inc_Frame_Usage(Frame_t *Frame) {
	if (!Frame) {
		ESP_LOGE(TAG,"NULL frame in call to Inc_Frame_Usage");
		return;
	}

	xSemaphoreGive(Frame->usage);
	ESP_LOGD(TAG,"Increment usage for Frame %p",Frame);
}

void Framebuff_Free_Frame(Frame_t *Frame) {

	if (!Frame) {
		ESP_LOGE(TAG,"NULL frame in call to Free_Frame");
		return;
	}

	if (xSemaphoreTake(Frame->usage,0) != pdPASS) {
		ESP_LOGD(TAG,"Freeing frame %p",Frame);
		if (Frame->parent) {
			Frame->frame_len = 0;
			Framebuff_Put_Frame(Frame->parent,Frame);
		}
	} else
		ESP_LOGD(TAG,"Decrement usage for frame %p",Frame);
}
