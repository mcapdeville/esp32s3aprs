/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/modem_afsk1200.c
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

#define _MODEM_PRIV_INCLUDE_
#include "modem.h"
#include "modem_afsk1200.h"

#include <string.h>
#include <esp_log.h>
#include <SA8x8.h>
#include "afsk_demod.h"
#include "afsk_mod.h"
#include "dmabuff.h"
#include "framebuff.h"
#include "afsk_demod.h"
#include "hdlc_dec.h"
#include "hdlc_enc.h"
#include "nrzi.h"
#include "config.h"

#define TAG "MODEM_AFSK1200"
#define MODEM_AFSK1200_OPS_TO		30

#define MODEM_AFSK1200_RECEIVE_BUFF_LEN	10	
#define MODEM_AFSK1200_TRANSMIT_BUFF_LEN 10

#define MODEM_DECODE_WATERMARK		(FRAME_LEN*5)
#define MODEM_ENCODE_WATERMARK		(FRAME_LEN*0)

enum Modem_AFSK1200_State_E {
	MODEM_AFSK1200_STATE_STOPPED = 0,
	MODEM_AFSK1200_STATE_RECEIVING,
	MODEM_AFSK1200_STATE_TRANSMITTING,
};

struct Modem_AFSK1200_S {
	// Interface
	struct Modem_S modem;

	// AFSK1200 Modem
	enum Modem_AFSK1200_State_E state;
	enum Modem_AFSK1200_State_E last_state;	// state before transmiting
	SA8x8_t *sa8x8;
	Dmabuff_t *sample_buff;	// dmabuff from SA8x8

	// AFSK1200 demodulation
	AFSK_Demod_t * afsk_demod;
	// Receiver HDLC Framing decoder
	Hdlc_Dec_t *hdlc_dec;
	// HDLC decoder sync state
	bool sync;
	// Received frames buffer
	Framebuff_t *receive_buff;
	uint32_t rx_frame_count;

	// Transmiter frames buffer
	Framebuff_t *transmit_buff;
	// HDLC Framing encoder
	Hdlc_Enc_t * hdlc_enc;
	// AFSK1200 modulation
	AFSK_Mod_t * afsk_mod;
	uint32_t tx_frame_count;

	// Bitstream buffer (Rx/Tx)
	uint8_t bitstream[1];
	uint8_t * bitstream_ptr;
	uint16_t bitstream_len;
	bool nrzi; // NRZI state

};

// Modem Ops
static int Modem_AFSK1200_Start_Receiver(struct Modem_AFSK1200_S * Modem);
static int Modem_AFSK1200_Stop_Receiver(struct Modem_AFSK1200_S * Modem);
static int Modem_AFSK1200_Start_Transmiter(struct Modem_AFSK1200_S * Modem);
static int Modem_AFSK1200_Stop_Transmiter(struct Modem_AFSK1200_S * Modem);
static int Modem_AFSK1200_Send_Frame(struct Modem_AFSK1200_S * Modem, Frame_t * Frame);

static const Modem_Ops_t Modem_AFSK1200_Ops = {
	.start_receiver = (typeof(Modem_AFSK1200_Ops.start_receiver))Modem_AFSK1200_Start_Receiver,
	.stop_receiver = (typeof(Modem_AFSK1200_Ops.stop_receiver))Modem_AFSK1200_Stop_Receiver,
	.start_transmiter = (typeof(Modem_AFSK1200_Ops.start_transmiter))Modem_AFSK1200_Start_Transmiter,
	.stop_transmiter = (typeof(Modem_AFSK1200_Ops.stop_transmiter))Modem_AFSK1200_Stop_Transmiter,
	.send_frame = (typeof(Modem_AFSK1200_Ops.send_frame))Modem_AFSK1200_Send_Frame
};

// Radio callback
static void Modem_AFSK1200_Radio_Cb(struct Modem_AFSK1200_S * Modem, struct SA8x8_Msg_S * Msg);

// Hdlc Callback
static void Modem_AFSK1200_Hdlc_Dec_Cb(struct Modem_AFSK1200_S * Modem, Frame_t * Frame);
static void Modem_AFSK1200_Hdlc_Enc_Cb(struct Modem_AFSK1200_S * Modem, Frame_t * Frame);

Modem_t * Modem_AFSK1200_Init(SA8x8_t *SA8x8, const AFSK_Config_t * Afsk_Config) {

	struct Modem_AFSK1200_S * modem;

	if (!SA8x8 || !Afsk_Config)
		return NULL;

	if (!(modem = malloc(sizeof(struct Modem_AFSK1200_S)))) {
		ESP_LOGE(TAG,"Error allocating modem struct");
		return NULL;
	}
	bzero(modem,sizeof(struct Modem_AFSK1200_S));
	modem->modem.ops = &Modem_AFSK1200_Ops;
	modem->sa8x8 = SA8x8;


	modem->sample_buff = SA8x8_Get_Buff(SA8x8);

	// AFSK1200 demodulator
	modem->afsk_demod = AFSK_Demod_Init(Afsk_Config);
	if (!modem->afsk_demod) {
		ESP_LOGE(TAG,"Error in initialisation of AFSK demodulator");
		// TODO : Cleanup
		return NULL;
	}

	// HDLC decoder
	modem->hdlc_dec = Hdlc_Dec_Init((Hdlc_Dec_Cb_t)Modem_AFSK1200_Hdlc_Dec_Cb,(void*)modem);
	if (!modem->hdlc_dec) {
		ESP_LOGE(TAG,"Error in initialisation of HDLC decoder");
		// TODO : Cleanup
		return NULL;
	}

	// AFSK1200 receiver frames buffer
	modem->receive_buff = Framebuff_Init(MODEM_AFSK1200_RECEIVE_BUFF_LEN, HDLC_MAX_FRAME_LEN);
	if (!modem->receive_buff) {
		ESP_LOGE(TAG,"Error Allocating receiver frames buffer");
		// TODO : Cleanup
		return NULL;
	}
	ESP_LOGD(TAG,"receiver frames buffer : %p", modem->receive_buff);

	// AFSK1200 transmiter frames buffer
	modem->transmit_buff = Framebuff_Init(MODEM_AFSK1200_TRANSMIT_BUFF_LEN, 0);
	if (!modem->transmit_buff) {
		ESP_LOGE(TAG,"Error Allocating transmiter frames buffer");
		// TODO : Cleanup
		return NULL;
	}
	ESP_LOGD(TAG,"transmiter frames buffer : %p", modem->receive_buff);

	// HDLC encoder
	modem->hdlc_enc = Hdlc_Enc_Init((Hdlc_Dec_Cb_t)Modem_AFSK1200_Hdlc_Enc_Cb,(void*)modem);
	if (!modem->hdlc_enc) {
	ESP_LOGE(TAG,"Error in initialisation of HDLC encoder");
		// TODO : Cleanup
		return NULL;
	}

	// AFSK1200 modulator
	modem->afsk_mod = AFSK_Mod_Init(Afsk_Config);
	if (!modem->afsk_mod) {
		ESP_LOGE(TAG,"Error in initialisation of AFSK modulator");
		// TODO : Cleanup
		return NULL;
	}

	SA8x8_Register_Cb(SA8x8,(SA8x8_Cb_t)Modem_AFSK1200_Radio_Cb,(void*)modem);

	return (Modem_t*)modem;
}

__attribute__((hot))
static void Modem_AFSK1200_Radio_Cb(struct Modem_AFSK1200_S * Modem, struct SA8x8_Msg_S * Msg) {
	void * samples;
	size_t len;
	int size;
	bool sync;

/*
	switch (Msg->type) {
		case SA8X8_SQUELCH_OPEN:
			Modem_Receiver_Started_Cb((Modem_t*)Modem);
			break;
		case SA8X8_SQUELCH_CLOSED:
			Modem_Receiver_Stopped_Cb((Modem_t*)Modem);
			break;
		case SA8X8_PTT_PUSHED:
			Modem_Transmiter_Started_Cb((Modem_t*)Modem);
			Modem->bitstream_len = 0;
			break;
		case SA8X8_PTT_RELEASED:
			Modem_Transmiter_Stopped_Cb((Modem_t*)Modem);
			break;
		default:
	}
*/
	switch (Modem->state) {
		case MODEM_AFSK1200_STATE_RECEIVING:
			switch (Msg->type) {
				case SA8X8_RECEIVER_DATA:
				case SA8X8_TRANSMITER_DATA:
					// Decode until watermark
					while ((size=(Dmabuff_Get_Ptr(Modem->sample_buff, 0, &samples, &len)>>1))>MODEM_DECODE_WATERMARK) {
						size -= MODEM_DECODE_WATERMARK;
						len >>=1;
						len = AFSK_Demod_Input(Modem->afsk_demod, samples, len,
								Modem->bitstream, sizeof(Modem->bitstream), &Modem->bitstream_len);
						NRZI_Decode(&Modem->nrzi, Modem->bitstream, Modem->bitstream_len);
						Hdlc_Dec_Input(Modem->hdlc_dec, Modem->bitstream, Modem->bitstream_len);
						Modem->bitstream_ptr = Modem->bitstream + (Modem->bitstream_len>>3);
						Dmabuff_Advance_Ptr(Modem->sample_buff, 0, len<<1);
					}

					sync = HDLC_Dec_Get_Sync(Modem->hdlc_dec);
					if (sync != Modem->sync){
						ESP_LOGD(TAG,"(Radio) %s of signal",sync?"Acquisition":"Lost");
						Modem->sync = sync;
						Modem_Dcd_Changed_Cb((Modem_t*)Modem, sync);
					}
					break;
			}
			break;
		case MODEM_AFSK1200_STATE_TRANSMITTING:
			switch (Msg->type) {
				case SA8X8_RECEIVER_DATA:
				case SA8X8_TRANSMITER_DATA:
					while ((size = (Dmabuff_Get_Ptr(Modem->sample_buff, 1, &samples, &len)>>1)) > MODEM_ENCODE_WATERMARK) {
						size -= MODEM_ENCODE_WATERMARK;
						len >>=1;
						if (!Modem->bitstream_len) {
							Modem->bitstream_len = Hdlc_Enc_Output(Modem->hdlc_enc, Modem->bitstream, sizeof(Modem->bitstream));
							NRZI_Encode(&Modem->nrzi, Modem->bitstream, Modem->bitstream_len);
							Modem->bitstream_ptr = Modem->bitstream;
						}

						len = AFSK_Mod_Output(Modem->afsk_mod, &Modem->bitstream_ptr, &Modem->bitstream_len, samples, len);
						Dmabuff_Advance_Ptr(Modem->sample_buff, 1, len<<1);
					};
					
					break;
			}
			break;
		default:
	}
}

__attribute__((hot))
static void Modem_AFSK1200_Hdlc_Dec_Cb(struct Modem_AFSK1200_S * Modem, Frame_t *Frame) {
	if (Frame) {
		Modem->rx_frame_count++;
		ESP_LOGD(TAG,"%ld frame received", Modem->rx_frame_count);
		Modem_Frame_Received_Cb((Modem_t*)Modem,Frame);
		Framebuff_Free_Frame(Frame);
	}

	Frame = Framebuff_Get_Frame(Modem->receive_buff);
	if (!Frame)
		ESP_LOGW(TAG,"Receiver frames buffer empty !");

	Hdlc_Dec_Add_Frame(Modem->hdlc_dec,Frame);
}

__attribute__((hot))
static void Modem_AFSK1200_Hdlc_Enc_Cb(struct Modem_AFSK1200_S * Modem, Frame_t * Frame) {

	if (Frame) {
		Modem->tx_frame_count++;
		ESP_LOGD(TAG,"%ld frame sent", Modem->tx_frame_count);
		ESP_LOGD(TAG,"Frame sent %p",Frame);
		Modem_Frame_Sent_Cb((Modem_t*)Modem, Frame);
		Framebuff_Free_Frame(Frame);
	}

	Frame = Framebuff_Get_Frame(Modem->transmit_buff);
	if (Frame)
		ESP_LOGD(TAG,"Sending frame %p", Frame);

	Hdlc_Enc_Add_Frame(Modem->hdlc_enc, Frame);
}

static int Modem_AFSK1200_Start_Receiver(struct Modem_AFSK1200_S * Modem) {
	int ret=-1;
	int cnt;

	switch (Modem->state) {
		case MODEM_AFSK1200_STATE_STOPPED:
			ESP_LOGD(TAG,"Starting Receiver");
			cnt = 10;
			do {
				if (!SA8x8_Start_Receiver(Modem->sa8x8))
					break;
				ESP_LOGD(TAG, "Waiting radio to start");
				vTaskDelay(200/portTICK_PERIOD_MS);
				cnt--;
			} while (cnt);
			if (!cnt) {
				ESP_LOGE(TAG, "Radio don't start");
				return 1;
			}
			Modem->state = MODEM_AFSK1200_STATE_RECEIVING;
			ret = 0;
			break;
		case MODEM_AFSK1200_STATE_RECEIVING:
			ret = 1;
			break;
		case MODEM_AFSK1200_STATE_TRANSMITTING:
			if (Modem->last_state == MODEM_AFSK1200_STATE_RECEIVING)
				ret = 1;
			else {
				ret = 0;
				Modem->last_state = MODEM_AFSK1200_STATE_RECEIVING;
			}
			break;
	}
	return ret;
}

static int Modem_AFSK1200_Stop_Receiver(struct Modem_AFSK1200_S * Modem) {
	int ret=-1;

	switch (Modem->state) {
		case MODEM_AFSK1200_STATE_STOPPED:
			ret = 1;
			break;
		case MODEM_AFSK1200_STATE_RECEIVING:
			ESP_LOGD(TAG,"Stopping Receiver");
			Modem->state = MODEM_AFSK1200_STATE_STOPPED;
			SA8x8_Stop_Receiver(Modem->sa8x8);
			ret = 0;
			break;
		case MODEM_AFSK1200_STATE_TRANSMITTING:
			if (Modem->last_state == MODEM_AFSK1200_STATE_STOPPED)
				ret = 1;
			else {
				Modem->last_state = MODEM_AFSK1200_STATE_STOPPED;
				ret = 0;
			}
			break;
	}
	return ret;
}

static int Modem_AFSK1200_Start_Transmiter(struct Modem_AFSK1200_S * Modem) {
	int ret=-1;

	switch (Modem->state) {
		case MODEM_AFSK1200_STATE_STOPPED:
		case MODEM_AFSK1200_STATE_RECEIVING:
			ESP_LOGD(TAG,"Starting Transmiter");
			Modem->last_state = Modem->state;
			Modem->state = MODEM_AFSK1200_STATE_TRANSMITTING;
			SA8x8_Start_Transmiter(Modem->sa8x8);
			ret = 0;
			break;
		case MODEM_AFSK1200_STATE_TRANSMITTING:
			ret = 1;
			break;
	}
	return ret;
}

static int Modem_AFSK1200_Stop_Transmiter(struct Modem_AFSK1200_S * Modem) {
	int ret=-1;

	switch (Modem->state) {
		case MODEM_AFSK1200_STATE_STOPPED:
		case MODEM_AFSK1200_STATE_RECEIVING:
			ret = 1;
			break;
		case MODEM_AFSK1200_STATE_TRANSMITTING:
			vTaskDelay(20/portTICK_PERIOD_MS);
			ESP_LOGD(TAG,"Stopping Transmiter");
			SA8x8_Stop_Transmiter(Modem->sa8x8);
			Modem->state = Modem->last_state;
			ret = 0;
			break;
	}
	return ret;
}

static int Modem_AFSK1200_Send_Frame(struct Modem_AFSK1200_S * Modem, Frame_t * Frame) {

	Framebuff_Inc_Frame_Usage(Frame);
	ESP_LOGD(TAG,"Queueing frame %p",Frame);
	if (Framebuff_Put_Frame(Modem->transmit_buff, Frame)) {
		Framebuff_Free_Frame(Frame);
		ESP_LOGE(TAG,"Transmit frames buffer full");
		return -1;
	}
	return 0;
}
