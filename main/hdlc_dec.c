/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/hdlc_dec.c
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

#include "hdlc_dec.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <esp_log.h>
#include <esp_rom_crc.h>
#include <esp_heap_caps.h>

#define TAG "HDLC Decoder"

#define HDLC_MIN_SYNC	2

struct Hdlc_Dec_S {
	uint8_t state, out;
	uint8_t bit;
	uint8_t sync;
	Hdlc_Dec_Cb_t cb;
	Frame_t * frame;
	size_t len;
	uint8_t *frame_ptr;
	void * arg;
};


Hdlc_Dec_t * Hdlc_Dec_Init(Hdlc_Dec_Cb_t Cb, void * arg) {
	Hdlc_Dec_t * hdlc;

	if (!(hdlc = heap_caps_malloc(sizeof(struct Hdlc_Dec_S),MALLOC_CAP_INTERNAL))) {
		ESP_LOGE(TAG,"Error allocating Hdlc_Dec struture");
		return NULL;
	}

	hdlc->cb = Cb;
	hdlc->frame = NULL;
	hdlc->arg = arg;

	Hdlc_Dec_Reset(hdlc);

	return hdlc;
}

void Hdlc_Dec_Add_Frame(Hdlc_Dec_t * Hdlc, Frame_t *Frame) {
	if (!Hdlc)
		return;

	Hdlc->frame = Frame;
}

void Hdlc_Dec_Reset(Hdlc_Dec_t * Hdlc) {
	if (!Hdlc)
		return;

	if (Hdlc->frame)
		Hdlc->frame_ptr = Hdlc->frame->frame;
	else
		Hdlc->frame_ptr = NULL;
	Hdlc->len = 0;
	Hdlc->bit = 0;
}

__attribute__((hot))
int Hdlc_Dec_Input(Hdlc_Dec_t * Hdlc,uint8_t * Bitstream, int BitLen) {

	int i;

	if (!Hdlc || !Bitstream)
		return 0;

	Bitstream--;
	for (i=0;i<BitLen;i++) {
		if (!(i&7)) {
				Bitstream++;
		}

		// feed the bit
		Hdlc->state >>=1;
		Hdlc->state |= (*Bitstream&1)?0x80:0;
		(*Bitstream)>>=1;

		if (Hdlc->state == 0x7e) {
			// Frame sync
			if (Hdlc->sync < 255)
				Hdlc->sync++;
			if (Hdlc->sync >= (HDLC_MIN_SYNC+1)) {
				if (Hdlc->cb) {
					if (Hdlc->frame) {
						if (Hdlc->len>=HDLC_MIN_FRAME_LEN) {
							Hdlc->frame->frame_len = Hdlc->len;
							Hdlc->cb(Hdlc->arg, Hdlc->frame );
						}
					}
					else {
						Hdlc->cb(Hdlc->arg, NULL);
					}
				}
				Hdlc_Dec_Reset(Hdlc);
			}
			continue;
		} else if (Hdlc->state == 0xfe) {
			// Lost of sync or abort
			Hdlc->sync = 0;
			Hdlc_Dec_Reset(Hdlc);
			continue;
		} else if ((Hdlc->state & 0xfc) == 0x7c) {
			// bit stuffing
			continue;
		}

		if (Hdlc->sync >= HDLC_MIN_SYNC) {
			Hdlc->out >>=1;
			Hdlc->out |= Hdlc->state&0x80;
			Hdlc->bit++;

			if (!(Hdlc->bit&7) && Hdlc->frame_ptr) {
				*Hdlc->frame_ptr = Hdlc->out;
				Hdlc->len++;
				Hdlc->frame_ptr++;
				if (Hdlc->len == Hdlc->frame->frame_size) {
					ESP_LOGE(TAG,"Frame too long");
					Hdlc_Dec_Reset(Hdlc);
						continue;
				}
			}
		}
	}

	return i;
}

bool HDLC_Dec_Get_Sync(Hdlc_Dec_t * Hdlc) {
	if (!Hdlc)
		return false;

	return Hdlc->sync >= HDLC_MIN_SYNC;
}
