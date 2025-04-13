/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/kiss.c
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

#include "kiss.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <esp_log.h>
#include <esp_rom_crc.h>

#include "hdlc_dec.h"
#include "framebuff.h"
#include "aprs.h"
#include "ax25_lm.h"

#define TAG "KISS"

#define KISS_NUM_STATIC_TASK	1
#ifndef NDEBUG
#define KISS_TASK_STACK_SIZE	3072+1024
#else
#define KISS_TASK_STACK_SIZE	3072
#endif
#define KISS_TASK_PRIORITY	2
#define KISS_TASK_DELAY		100
#define KISS_MAX_FRAME_LEN	(HDLC_MAX_FRAME_LEN*2+3)
#define KISS_MAX_OUT_FRAMES		3
#define KISS_MAX_IN_FRAMES		10

#define KISS_FEND		0xC0
#define KISS_FESC		0XDB
#define KISS_TFEND		0xDC
#define KISS_TFESC		0xDD

#define KISS_WAIT_MS	100
#define KISS_FRAME_NUM	5

struct Kiss_S {
	const char * path;
	int uart_fd;
	AX25_Lm_t * ax25_lm;
	TaskHandle_t task;
	uint8_t in_buff[KISS_MAX_FRAME_LEN];
	size_t in_len, in_pos;
	uint8_t out_buff[KISS_MAX_FRAME_LEN];
	size_t out_pos;
	uint8_t out_last;
	Frame_t * out_frame;
	Frame_t * in_frame;
	bool out_enable;
	Frame_t * last_sent;
	SemaphoreHandle_t sem;
	StaticSemaphore_t sem_buff;
	Framebuff_t * out_framebuff;
	Framebuff_t * in_framebuff;
	uint8_t *pool;
};

extern APRS_t * Aprs;

struct {
	TaskHandle_t handle;
	StackType_t	stack[(KISS_TASK_STACK_SIZE)/sizeof(StackType_t)];
	StaticTask_t buffer;
} Kiss_Tasks[KISS_NUM_STATIC_TASK];

static void Kiss_Task(void * arg);

Kiss_t * Kiss_Init(const char *path, AX25_Lm_t * Ax25_Lm) {

	Kiss_t *kiss;
	AX25_Lm_Cbs_t cbs = {
		.seize_confirm = NULL,
		.data_indication = (typeof(cbs.data_indication))Kiss_Frame_Received_Cb,
		.busy_indication = NULL,
		.quiet_indication = NULL
	};

	if (!path || !Ax25_Lm)
		return NULL;

	if (!(kiss = malloc(sizeof(struct Kiss_S)))) {
		ESP_LOGE(TAG,"Error allocating Kiss structure");
		return NULL;
	}


	memset(kiss,0,sizeof(struct Kiss_S));
	kiss->path = path;
	kiss->uart_fd = -1;

	kiss->ax25_lm = Ax25_Lm;
	AX25_Lm_Register_Dl(kiss->ax25_lm, kiss, &cbs, NULL); // Get All frames from phy

	kiss->sem = xSemaphoreCreateBinaryStatic(&kiss->sem_buff);

	kiss->out_framebuff = Framebuff_Init(KISS_MAX_OUT_FRAMES,HDLC_MAX_FRAME_LEN);	
	ESP_LOGD(TAG,"Init out buffer %p",kiss->out_framebuff);
	kiss->in_framebuff = Framebuff_Init(KISS_MAX_IN_FRAMES,0);
	ESP_LOGD(TAG,"Init in buffer %p",kiss->in_framebuff);

	int i= 0;
	while (Kiss_Tasks[i].handle && i < KISS_NUM_STATIC_TASK) i++;
	if (i == KISS_NUM_STATIC_TASK) {
		ESP_LOGE(TAG,"Error creating task\n");
		AX25_Lm_Unregister_Dl(kiss->ax25_lm, kiss);
		free(kiss);
		return NULL;
	}

	Kiss_Tasks[i].handle = xTaskCreateStaticPinnedToCore(Kiss_Task,"Kiss", KISS_TASK_STACK_SIZE, kiss, KISS_TASK_PRIORITY, Kiss_Tasks[i].stack, &Kiss_Tasks[i].buffer, APP_CPU_NUM);
	
	/*if (pdPASS != xTaskCreate(Kiss_Task,"Kiss",KISS_TASK_STACK_SIZE,kiss,KISS_TASK_PRIORITY,&kiss->task)) {
		ESP_LOGE(TAG,"Error creating task\n");
		AX25_Lm_Unregister_Dl(kiss->ax25_lm, kiss);
		close(fd);
		free(kiss);
		return NULL;
	}*/

	return kiss;
}

// IN means from radio to pc
static void Kiss_Task(void * arg) {
	Kiss_t * kiss = (Kiss_t *)arg;
	int i, ret;
	size_t in_frame_len,out_len;
	uint8_t * in, * out;
	char c;

	kiss->out_frame = NULL;
	kiss->out_enable = false,
	kiss->in_len = 0;
	kiss->in_pos = 0;
	kiss->out_pos = 0;

	do {

		if (kiss->uart_fd < 0)
			kiss->uart_fd = open(kiss->path,O_RDWR);

		if (kiss->in_len == kiss->in_pos) {
			kiss->in_frame = Framebuff_Get_Frame(kiss->in_framebuff);
			if (kiss->in_frame) {
				ESP_LOGD(TAG,"Received frame : %p",kiss->in_frame);
				in_frame_len = kiss->in_frame->frame_len-2;
				if (in_frame_len>0 && in_frame_len <= HDLC_MAX_FRAME_LEN-2) {
					in = kiss->in_buff;
					*(in++) = KISS_FEND;
					*(in++) = 0;
					kiss->in_len = 2;
					for (i=0;i<in_frame_len;i++) {
						c=kiss->in_frame->frame[i];
						switch (c) {
							case KISS_FEND:
								*(in++) = KISS_FESC;
								*(in++) = KISS_TFEND;
								kiss->in_len += 2;
								break;
							case KISS_FESC:
								*(in++) = KISS_FESC;
								*(in++) = KISS_TFESC;
								kiss->in_len += 2;
								break;
							default:
								*(in++) = c;
								kiss->in_len ++;
								break;
						}

					}
					*(in++) = KISS_FEND;
					kiss->in_len ++;
					kiss->in_pos = 0;
			       }
			       Framebuff_Free_Frame(kiss->in_frame);
			}
		}

		if (kiss->in_len > kiss->in_pos) {
			i = write(kiss->uart_fd,kiss->in_buff+kiss->in_pos,kiss->in_len-kiss->in_pos);
			if (i>0) {
				kiss->in_pos += i;
				if (kiss->in_len != kiss->in_pos)
					continue;
			} else {
				ESP_LOGE(TAG, "Error writing kiss frame");
				kiss->in_pos = kiss->in_len;
			}
		}

		ret = read(kiss->uart_fd,kiss->out_buff,KISS_MAX_FRAME_LEN);
		out = kiss->out_buff;
		if (ret <= 0) {
			ESP_LOGD(TAG,"Read unlock");
			out_len = 0;
		}
		else
			out_len = ret;
		
		if (kiss->out_enable) {
			if (kiss->out_frame)
				in = kiss->out_frame->frame + kiss->out_pos;
			else {
				in = NULL;
				kiss->out_enable = false;
			}
		} else 
			in = NULL;

		for (i=0;i<out_len;i++) {
			if (kiss->out_enable) {
				if (kiss->out_last == KISS_FESC) {
					switch (*out) {
						case KISS_TFESC:
							*(in++) = KISS_FESC;
							kiss->out_pos++;
							break;
						case KISS_TFEND:
							*(in++) = KISS_FEND;
							kiss->out_pos++;
							break;
						default:
							// Protocol error
							kiss->out_pos = 0;
							in = kiss->out_frame->frame;
							kiss->out_enable = false;
							AX25_Lm_Release_Request(kiss->ax25_lm,  kiss);
							break;
					}
				}
				else {
					switch (*out) {
						case KISS_FESC:
							// Look next char
							break;
						case KISS_FEND:
/*							if (kiss->out_last == KISS_FEND) {
								// sync
								kiss->out_pos = 0;
								in = kiss->out_frame->frame;
							} else 
*/							{							
								// end of frame
								if (kiss->out_pos < HDLC_MAX_FRAME_LEN-2 && kiss->out_pos >= HDLC_MIN_FRAME_LEN) {
									uint16_t crc;
									// Add crc
									crc = esp_rom_crc16_le(0,kiss->out_frame->frame,kiss->out_pos);

									*(in++) = (crc)&0xff;
									*(in++) = (crc>>8)&0xff;
									kiss->out_pos += 2;

									kiss->out_frame->frame_len = kiss->out_pos;

									// Send it to transmiter
									ESP_LOGI(TAG,"Send frame : %p",kiss->out_frame);
									AX25_Lm_Data_Request(kiss->ax25_lm, kiss, kiss->out_frame);

									// Loop back to Aprs decoder
									APRS_Frame_Received_Cb(Aprs ,kiss->out_frame);

									Framebuff_Free_Frame(kiss->out_frame);
									kiss->out_frame = NULL;
								} else {
									kiss->out_pos = 0;
									in = kiss->out_frame->frame;
								}
							}
							break;
/*						case KISS_TFESC:
							// Protocol error
							kiss->out_pos = 0;
							in = kiss->out_frame->frame;
							kiss->out_enable = false;
							AX25_Lm_Release_Request(kiss->ax25_lm, kiss);
							break;
*/
						default:
							//receive char;
							*(in++) = *out;
							kiss->out_pos++;
							break;
					}
				}

				if (kiss->out_frame && kiss->out_pos > kiss->out_frame->frame_size) {
					ESP_LOGE(TAG,"OUT Frame too long");
					kiss->out_enable = false;
					AX25_Lm_Release_Request(kiss->ax25_lm, kiss);
				}

			} else if (kiss->out_last == KISS_FEND) {
				switch (*out) {
					case 0: // Data frame
						// Start of frame
						kiss->out_pos = 0;
						if (kiss->out_frame == NULL)
							kiss->out_frame  = Framebuff_Get_Frame(kiss->out_framebuff);
						if (kiss->out_frame) {
							in = kiss->out_frame->frame;
							kiss->out_enable = true;
							// AX25_Lm_Seize_Request(kiss->ax25_lm, kiss);
						}
						break;
					case 1: // TX Delay
						break;
					case 2: // Persistence
						break;
					case 3: // SlotTime
						break;
					case 4: // Tx tail
						break;
					case 5: // Full duplex
						break;
					case 6: // Set Hardware
						break;
					case 0xff: // Exit kiss
						break;
					default:
				}		
			}
		kiss->out_last = *out;
		out++;
		}

	} while (1);

}

int Kiss_Frame_Received_Cb(Kiss_t * Kiss, Frame_t * Frame) {

	if (!Kiss || !Frame || Frame->frame_len < HDLC_MIN_FRAME_LEN)
		return 0;

	Framebuff_Inc_Frame_Usage(Frame);
	if (Framebuff_Put_Frame(Kiss->in_framebuff,Frame)) {
		Framebuff_Free_Frame(Frame);
		ESP_LOGE(TAG,"Input frame buffer full");
		return 0;
	}

	if (Kiss->uart_fd) {
		// Unlock read
		close(Kiss->uart_fd);
		Kiss->uart_fd = -1;
	}

	return 0;
}
