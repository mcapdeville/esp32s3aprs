/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * SA8x8_priv.h
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

#ifndef _SA8X8_PRIV_H_
#define _SA8X8_PRIV_H_

#define TAG "SA8X8"

#include <esp_adc/adc_continuous.h>
#include <driver/i2s_common.h>
#include <dmabuff.h>
#include <nvs.h>
#include "SA8x8.h"

#define SA8X8_QUEUE_LEN 20

typedef struct SA8x8_Cb_List_S {
		struct SA8x8_Cb_List_S * next;
		SA8x8_Cb_t cb;
		void * cb_ctx;
	} SA8x8_Cb_List_t;

enum SA8x8_State_E {
	SA8X8_STATE_STOPPED = 0,
	SA8X8_STATE_STARTED,
	SA8X8_STATE_RECEIVING,
	SA8X8_STATE_TRANSMITTING
};

struct SA8x8_S {
	// Configuration
	struct {
		int uart_num;
		int sq_pin, ptt_pin, pd_pin, hl_pin;
		int dac_pin, adc_pin;
	};

	nvs_handle_t nvs;

	// Radio setup
	SA8x8_Group_t group;	
	uint8_t volume;
	bool emph;
	bool hi_pass;
	bool low_pass;
	uint8_t tail;
	SA8x8_Power_t power;
	SA8x8_Bw_t bw;


	// State
	struct {
		SemaphoreHandle_t state_sem;
		StaticSemaphore_t state_sem_buff;
		enum SA8x8_State_E state;
		uint8_t rx_cnt;
		uint8_t tx_cnt;
	};

	// Receiver ADC (ADC0)
	adc_continuous_handle_t adc;
	bool adc_started;

	// Trabnsmiter DAC (I2S PDM mode)
	i2s_chan_handle_t dac;
	bool dac_started;

	// sample buffer
	Dmabuff_t sample_buff;

	// uart buffer
	SemaphoreHandle_t uart_sem;
	StaticSemaphore_t uart_sem_buff;
	char uart_buff[64];
	char uart_command[64];

	// Task and queue
	TaskHandle_t task;
	QueueHandle_t queue;
	StaticQueue_t QueueBuffer;
	struct SA8x8_Msg_S QueueStore[SA8X8_QUEUE_LEN];

	// Callbacks list
	SA8x8_Cb_List_t * cb_list;
};

int SA8x8_Transmiter_Init(SA8x8_t * SA8x8, uint16_t sample_rate, uint16_t frame_len, int dac_pin);
void SA8x8_Transmiter_Deinit(SA8x8_t * SA8x8);

int SA8x8_Receiver_Init(SA8x8_t * SA8x8, int sample_rate, int32_t frame_len, int adc_pin);
void SA8x8_Receiver_Deinit(SA8x8_t * SA8x8);

int SA8x8_Uart_Init(SA8x8_t * SA8x8, int uart_num, int Baud, int tx_pin,int rx_pin);
void SA8x8_Uart_Deinit(SA8x8_t * SA8x8);
int SA8x8_Uart_Command(SA8x8_t * SA8x8, const char * Command,const char * Reply,int Len ,char * Buff);

int SA8x8_Gpio_Init(SA8x8_t *SA8x8, int sq_pin, int ptt_pin, int pd_pin, int hl_pin);
void SA8x8_Gpio_Deinit(SA8x8_t *SA8x8);

void SA8x8_Receiver_Start_Adc(SA8x8_t * SA8x8);
void SA8x8_Receiver_Stop_Adc(SA8x8_t * SA8x8);

void SA8x8_Transmiter_Start_Dac(SA8x8_t * SA8x8);
void SA8x8_Transmiter_Stop_Dac(SA8x8_t * SA8x8);

int SA8x8_Gpio_SetPtt(struct SA8x8_S * SA8x8, bool ptt);
int SA8x8_Gpio_GetSquelch(struct SA8x8_S * SA8x8);
int SA8x8_Gpio_SetPower(struct SA8x8_S * SA8x8, SA8x8_Power_t power);

int SA8x8_Uart_SetGroup(SA8x8_t * SA8x8,SA8x8_Group_t * Param);
int SA8x8_Uart_SetFilter(SA8x8_t * SA8x8, bool emph, bool hi, bool lo);
int SA8x8_Uart_SetVolume(SA8x8_t * SA8x8, uint8_t Volume);
int SA8x8_Uart_SetTail(SA8x8_t * SA8x8, uint8_t Tail);

#endif
