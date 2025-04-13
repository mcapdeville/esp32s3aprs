/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * gpio.c
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

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <errno.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_err.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>
#include "SA8x8_priv.h"

static void SA8x8_Gpio_sq_isr_handler(void * arg);
static void SA8x8_Gpio_ptt_isr_handler(void * arg);

int SA8x8_Gpio_Init(struct SA8x8_S * SA8x8, int sq_pin, int ptt_pin, int pd_pin, int hl_pin) {
	gpio_config_t in_pin_config = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_INPUT,
		.pin_bit_mask = 1ULL<<sq_pin,
		.pull_down_en = 0,
		.pull_up_en = 0,
	};

	gpio_config_t out_pins_config = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1ULL<<hl_pin) | (1ULL<<pd_pin) | (1ULL<<ptt_pin),
		.pull_down_en = 0,
		.pull_up_en = 0,
	};

	if (gpio_config(&in_pin_config) != ESP_OK) {
		ESP_LOGE(TAG,"Error configuring input pins");
		return -1;
	}

	gpio_set_level(pd_pin,1);	// Low power for init
	gpio_set_level(hl_pin,1);	// Low power
	gpio_set_level(ptt_pin,1);	// ptt release
	
	if (gpio_config(&out_pins_config) != ESP_OK) {
		ESP_LOGE(TAG,"Error configuring output pins");
		in_pin_config.mode = GPIO_MODE_DISABLE;
		in_pin_config.pull_down_en = 0;
		gpio_config(&in_pin_config);
		return -1;
	}

	if (gpio_set_direction(ptt_pin,GPIO_MODE_INPUT_OUTPUT_OD) != ESP_OK) {
		ESP_LOGE(TAG,"Error configuring output pins");
		out_pins_config.mode = GPIO_MODE_DISABLE;
		gpio_config(&out_pins_config);
		in_pin_config.mode = GPIO_MODE_DISABLE;
		in_pin_config.pull_down_en = 0;
		gpio_config(&in_pin_config);
		return -1;
	}

	SA8x8->pd_pin = pd_pin;
	SA8x8->hl_pin = hl_pin;
	SA8x8->sq_pin = sq_pin;
	SA8x8->ptt_pin = ptt_pin;

	if (gpio_isr_handler_add(sq_pin, SA8x8_Gpio_sq_isr_handler, (void*) SA8x8) != ESP_OK) {
		ESP_LOGE(TAG,"Error installing gpio isr handler for squelch pin");
		out_pins_config.mode = GPIO_MODE_DISABLE;
		gpio_config(&out_pins_config);
		in_pin_config.mode = GPIO_MODE_DISABLE;
		in_pin_config.pull_down_en = 0;
		gpio_config(&in_pin_config);
		return -1;
	}

	if (gpio_isr_handler_add(ptt_pin, SA8x8_Gpio_ptt_isr_handler, (void*) SA8x8) != ESP_OK) {
		ESP_LOGE(TAG,"Error installing gpio isr handler for ptt pin");
		gpio_isr_handler_remove(sq_pin);
		out_pins_config.mode = GPIO_MODE_DISABLE;
		gpio_config(&out_pins_config);
		in_pin_config.mode = GPIO_MODE_DISABLE;
		in_pin_config.pull_down_en = 0;
		gpio_config(&in_pin_config);
		return -1;
	}

	return 0;
}

void SA8x8_Gpio_Deinit(SA8x8_t * SA8x8) {
	gpio_config_t pins_config = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_DISABLE,
		.pin_bit_mask = (1ULL<<SA8x8->sq_pin) | (1ULL<<SA8x8->hl_pin) | (1ULL<<SA8x8->pd_pin) | (1ULL<<SA8x8->ptt_pin),
		.pull_down_en = 0,
		.pull_up_en = 0,
	};

	gpio_isr_handler_remove(SA8x8->ptt_pin);
	gpio_isr_handler_remove(SA8x8->sq_pin);
	gpio_config(&pins_config);
}

static void IRAM_ATTR SA8x8_Gpio_ptt_isr_handler(void * arg) {
	BaseType_t MustYield = pdFALSE;
	struct SA8x8_S * SA8x8 = arg;
	struct SA8x8_Msg_S msg;
	bool ptt_state = gpio_get_level(SA8x8->ptt_pin);

	msg.type = ptt_state ? SA8X8_PTT_RELEASED : SA8X8_PTT_PUSHED;
	xQueueSendFromISR(SA8x8->queue,(void*)&msg,&MustYield);

	portYIELD_FROM_ISR(MustYield);
}

static void IRAM_ATTR SA8x8_Gpio_sq_isr_handler(void * arg) {
	BaseType_t MustYield = pdFALSE;
	struct SA8x8_S * SA8x8 = arg;
	struct SA8x8_Msg_S msg;
	bool sq_state = gpio_get_level(SA8x8->sq_pin);

	msg.type = sq_state ? SA8X8_SQUELCH_OPEN : SA8X8_SQUELCH_CLOSED;
	xQueueSendFromISR(SA8x8->queue,(void*)&msg,&MustYield);

	portYIELD_FROM_ISR(MustYield);
	return;
}

int SA8x8_Gpio_SetPower(struct SA8x8_S * SA8x8, SA8x8_Power_t Power) {
	int ret;

	if (xSemaphoreTake(SA8x8->state_sem,portMAX_DELAY) != pdPASS)
		return -1;

	switch (Power) {
		case SA8X8_POWER_DOWN:
			switch (SA8x8->state) {
				case SA8X8_STATE_TRANSMITTING:
					SA8x8_Transmiter_Stop_Dac(SA8x8);
					break;
				case SA8X8_STATE_RECEIVING:
					SA8x8_Receiver_Stop_Adc(SA8x8);
					break;
				default:
			}
			gpio_set_intr_type(SA8x8->sq_pin,GPIO_INTR_DISABLE);
			gpio_set_intr_type(SA8x8->ptt_pin,GPIO_INTR_DISABLE);
			gpio_set_level(SA8x8->ptt_pin,1);
			
			gpio_set_level(SA8x8->hl_pin,0);
			gpio_set_level(SA8x8->pd_pin,0);

			ESP_LOGD(TAG,"Radio power off");
			SA8x8->state = SA8X8_STATE_STOPPED;
			SA8x8->tx_cnt = 0;
			SA8x8->rx_cnt = 0;
			ret = 0;
			break;
		case SA8X8_POWER_LOW:
			gpio_set_level(SA8x8->hl_pin,1);
			gpio_set_level(SA8x8->pd_pin,1);
			if (SA8x8->state == SA8X8_STATE_STOPPED) {
				gpio_set_intr_type(SA8x8->sq_pin,GPIO_INTR_ANYEDGE);
				gpio_set_intr_type(SA8x8->ptt_pin,GPIO_INTR_ANYEDGE);
			}
			ESP_LOGD(TAG,"Radio power low");
			SA8x8->state = SA8X8_STATE_STARTED;
			ret = 0;
			break;
		case SA8X8_POWER_HI:
			gpio_set_level(SA8x8->hl_pin,0);
			gpio_set_level(SA8x8->pd_pin,1);
			if (SA8x8->state == SA8X8_STATE_STOPPED) {
				SA8x8->state = SA8X8_STATE_STARTED;
				gpio_set_intr_type(SA8x8->sq_pin,GPIO_INTR_ANYEDGE);
				gpio_set_intr_type(SA8x8->ptt_pin,GPIO_INTR_ANYEDGE);
			}
			ESP_LOGD(TAG,"Radio power high");
			SA8x8->state = SA8X8_STATE_STARTED;
			ret = 0;
			break;
		default:
			ret = -1;
	}

	xSemaphoreGive(SA8x8->state_sem);

	return ret;
}

int SA8x8_Gpio_SetPtt(struct SA8x8_S * SA8x8, bool ptt) {
	ESP_LOGD(TAG,"Ptt %s",ptt?"Pushed":"released");
	gpio_set_level(SA8x8->ptt_pin,ptt?0:1);
	return 0;
}

int SA8x8_Gpio_GetSquelch(struct SA8x8_S * SA8x8) {
	return gpio_get_level(SA8x8->sq_pin);
}
