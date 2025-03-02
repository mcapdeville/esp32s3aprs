/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * SA8x8.h
 *
 * Copyright (C) 2025  Marc CAPDEVILLE F4JMZ
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

#ifndef _SA8X8_H_
#define _SA8X8_H_

#include <freertos/FreeRTOS.h>

typedef struct SA8x8_S SA8x8_t;
typedef struct Dmabuff_S Dmabuff_t;

typedef enum SA8X8_Bw_E {
	SA8X8_BW_12K5=0,
	SA8X8_BW_25K=1,
} SA8x8_Bw_t;

typedef enum SA8X8_Power_E {
	SA8X8_POWER_DOWN,
	SA8X8_POWER_LOW,
	SA8X8_POWER_HI,
} SA8x8_Power_t;

typedef struct SA8x8_Config_S {
	int uart_num;
	int tx_pin,rx_pin;
	int adc_pin,sq_pin;
	int dac_pin;
	int hl_pin,pd_pin,ptt_pin;
	int sample_rate;
	int frame_len;
} SA8x8_config_t;

typedef struct SA8x8_Group_S {
	SA8x8_Bw_t bw;
	uint32_t tx_freq;
	uint32_t rx_freq;
	char tx_sub[5];
	char rx_sub[5];
	uint8_t squelch;
} SA8x8_Group_t;

enum SA8x8_Event_E {
	SA8X8_TRANSMITER_DATA = 0,
	SA8X8_PTT_PUSHED,
	SA8X8_PTT_RELEASED,

	SA8X8_RECEIVER_DATA,
	SA8X8_SQUELCH_OPEN,
	SA8X8_SQUELCH_CLOSED,

	SA8X8_USER_EVENT_0	// First user event number
};

typedef struct SA8x8_Msg_S {
	uint8_t type;
	int32_t size;
	void * data;
} SA8x8_Msg_t;

typedef void (*SA8x8_Cb_t)(void * Ctx, SA8x8_Msg_t * Msg);

// Default value for NVS variable
#define SA8X8_DEFAULT_FREQ	144800000
#define SA8X8_DEFAULT_SQUELCH	1
#define SA8X8_DEFAULT_VOLUME	6
#define SA8X8_DEFAULT_SUBAUDIO	"0000"
#define SA8X8_DEFAULT_TAIL	0
#define SA8X8_DEFAULT_BANDWIDTH	SA8X8_BW_12K5
#define SA8X8_DEFAULT_EMPHASIS	false
#define SA8X8_DEFAULT_HIPASS	false
#define SA8X8_DEFAULT_LOWPASS	false
#define SA8X8_DEFAULT_POWER		SA8X8_POWER_HI

SA8x8_t * SA8x8_Init(const SA8x8_config_t * config);
Dmabuff_t * SA8x8_Get_Buff(SA8x8_t * SA8x8);
int SA8X8_Load_Config(SA8x8_t *SA8x8);

int SA8x8_Send_Event(SA8x8_t * SA8x8, struct SA8x8_Msg_S *msg, TickType_t to);
int SA8x8_Register_Cb(SA8x8_t * SA8x8, SA8x8_Cb_t Cb, void * Ctx);
int SA8x8_Unregister_Cb(SA8x8_t * SA8x8, void * Ctx);

int SA8x8_Start_Transmiter(SA8x8_t * SA8x8);
int SA8x8_Stop_Transmiter(SA8x8_t * SA8x8);
bool SA8x8_Transmiter_Is_Started(SA8x8_t * SA8x8);

int SA8x8_Start_Receiver(SA8x8_t * SA8x8);
int SA8x8_Stop_Receiver(SA8x8_t * SA8x8);
bool SA8x8_Receiver_Is_Started(SA8x8_t * SA8x8);

int SA8x8_Set_Power(struct SA8x8_S * SA8x8, SA8x8_Power_t Power);

int SA8x8_Handcheck(SA8x8_t * SA8x8);
int SA8x8_GetVersion(SA8x8_t * SA8x8,char * Verstr,int strlen);
int SA8x8_GetRssi(SA8x8_t * SA8x8,uint8_t * rssi);
int SA8x8_Scan(SA8x8_t * SA8x8,uint32_t Freq);

int SA8x8_Set_Freq(SA8x8_t * SA8x8,uint32_t Freq);
uint32_t SA8x8_Get_RxFreq(SA8x8_t * SA8x8);
uint32_t SA8x8_Get_TxFreq(SA8x8_t * SA8x8);

int SA8x8_Set_Sub(SA8x8_t * SA8x8, const char * Sub);
int SA8x8_Get_RxSub(SA8x8_t * SA8x8, char *Sub, int len);
int SA8x8_Get_TxSub(SA8x8_t * SA8x8, char *Sub, int len);

int SA8x8_Set_Bandwidth(SA8x8_t * SA8x8, SA8x8_Bw_t Bw);
SA8x8_Bw_t SA8x8_Get_Bandwidth(SA8x8_t * SA8x8);

int SA8x8_Set_Squelch(SA8x8_t * SA8x8,uint8_t Sq);
uint8_t SA8x8_Get_Squelch(SA8x8_t * SA8x8);

int SA8x8_Set_Emphasis(SA8x8_t * SA8x8, bool Em);
bool SA8x8_Get_Emphasis(SA8x8_t * SA8x8);

int SA8x8_Set_Hipass(SA8x8_t * SA8x8, bool Hp);
bool SA8x8_Get_Hipass(SA8x8_t * SA8x8);

int SA8x8_Set_Lowpass(SA8x8_t * SA8x8, bool Lp);
bool SA8x8_Get_Lowpass(SA8x8_t * SA8x8);

int SA8x8_Set_Volume(SA8x8_t * SA8x8, uint8_t Lp);
uint8_t SA8x8_Get_Volume(SA8x8_t * SA8x8);

int SA8x8_Set_Tail(SA8x8_t * SA8x8, uint8_t Lp);
uint8_t SA8x8_Get_Tail(SA8x8_t * SA8x8);

int SA8x8_Set_Power(SA8x8_t * SA8x8, SA8x8_Power_t Power);
SA8x8_Power_t SA8x8_Get_Power(SA8x8_t * SA8x8);

#endif
