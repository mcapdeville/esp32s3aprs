/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/hdlc_enc.c
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

#include "hdlc_enc.h"

#include <errno.h>
#include <stdbool.h>
#include <esp_log.h>
#include <esp_rom_crc.h>
#include <esp_heap_caps.h>

#define TAG "HDLC Encoder"

struct Hdlc_Enc_S {
	Hdlc_Enc_Cb_t cb;
	void * arg;
	Frame_t *frame;
	uint8_t *frame_ptr;
	size_t len;
	bool bit_stuff;
	uint8_t in;
	uint16_t in_pos;
	uint8_t out;
	bool out_bit;
};

Hdlc_Enc_t * Hdlc_Enc_Init(Hdlc_Enc_Cb_t Cb, void * Arg) {
	Hdlc_Enc_t * hdlc;

	if (!(hdlc = heap_caps_malloc(sizeof(struct Hdlc_Enc_S),MALLOC_CAP_INTERNAL))) {
		ESP_LOGE(TAG,"Error allocating Hdlc_Enc struct");
		return NULL;
	}
	memset(hdlc,0,sizeof(struct Hdlc_Enc_S));

	hdlc->cb = Cb;
	hdlc->arg = Arg;
	hdlc->frame = NULL;
	hdlc->len = 0;

	Hdlc_Enc_Reset(hdlc);

	return hdlc;
}

void Hdlc_Enc_Reset(Hdlc_Enc_t * Hdlc) {
	if (!Hdlc->frame) {
		Hdlc->frame_ptr = NULL;
		Hdlc->len = 0;
	} else {
		Hdlc->frame_ptr = Hdlc->frame->frame;
		Hdlc->len = Hdlc->frame->frame_len;
	}

	Hdlc->in_pos = 0;
	Hdlc->in = 0;
	Hdlc->bit_stuff = false;
}

//static char bs[32];

// return number of bit generated
__attribute__((hot))
size_t Hdlc_Enc_Output(Hdlc_Enc_t * Hdlc,uint8_t * Bitstream, size_t Len) {

	bool bit;
	uint16_t out_pos = 0;
//     	uint16_t last_inpos;

	if (!Hdlc)
		return 0;

//	bs[0]='\0';
//	last_inpos = Hdlc->in_pos;

	while(Len) {
		if (Hdlc->bit_stuff && ((Hdlc->out & 0xFC) == 0xF8))
			// Bit stuffing
			bit = false;
		else {
			if (!(Hdlc->in_pos&7)) {
				if (Hdlc->in_pos && Hdlc->len && Hdlc->frame_ptr) {
					Hdlc->in = *Hdlc->frame_ptr;
//					sprintf(bs+strlen(bs),"%02x ",Hdlc->in);
					Hdlc->frame_ptr++;
					Hdlc->bit_stuff = true;
					Hdlc->len--;
				} else {
					if (Hdlc->in_pos == 8) {
						Frame_t * frame = Hdlc->frame;
						Hdlc->frame = NULL;
						Hdlc->frame_ptr = NULL;
						Hdlc->len = 0;
						if (Hdlc->cb) {
							Hdlc->cb(Hdlc->arg,frame);
						}
						if (Hdlc->frame) {
							Hdlc->frame_ptr = Hdlc->frame->frame;
							Hdlc->len = Hdlc->frame->frame_len;
							continue;
						}
					}
					Hdlc->in_pos = 0;
					Hdlc->in = 0x7e;
					Hdlc->bit_stuff = false;
				}
			}

			// Get bit
			bit = (Hdlc->in&1);
			Hdlc->in >>= 1;
			Hdlc->in_pos++;
		}

		Hdlc->out >>= 1;
		Hdlc->out |=bit?0x80:0;

		out_pos++;
		if (!(out_pos&7)) {
			*Bitstream = Hdlc->out;
			Bitstream++;
			Len--;
		}
	}

	//ESP_LOGD(TAG,"%d : %s",Hdlc->in_pos-last_inpos,bs);

	return out_pos;
}

int Hdlc_Enc_Add_Frame(Hdlc_Enc_t * Hdlc, Frame_t * Frame) {

	if (!Hdlc)
		return -1;

	Hdlc->frame = Frame;

	return 0;
}
