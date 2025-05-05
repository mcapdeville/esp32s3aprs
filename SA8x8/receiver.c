/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * receiver.c
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
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include "SA8x8_priv.h"

#define ADC_ATTEN	ADC_ATTEN_DB_6	// 0->1.750mv +-10mV error

#define ADC_NUM_DMA	CONFIG_ADC_CONTINUOUS_NUM_DMA

static bool SA8x8_Receiver_isr_handler(adc_continuous_handle_t adc, const adc_continuous_evt_data_t *event,void * arg);

int SA8x8_Receiver_Init(struct SA8x8_S * SA8x8, int sample_rate, int32_t frame_len, int adc_pin) {
/* ADC Initialisation */
	const	adc_continuous_handle_cfg_t adc_handle_config = {
		.max_store_buf_size = 0, // Don't use ringbuffer (hack on adc_countinuous component)
		.conv_frame_size = frame_len*SOC_ADC_DIGI_DATA_BYTES_PER_CONV,
	};

	adc_channel_t adc_channel;
	adc_unit_t adc_unit;

	adc_continuous_io_to_channel(adc_pin,&adc_unit,&adc_channel);

	adc_digi_pattern_config_t adc_pattern_config = {
		.atten = ADC_ATTEN,
		.channel = adc_channel,
		.unit = adc_unit,
		.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
	};

	const adc_continuous_config_t adc_config = {
		.pattern_num = 1,
		.adc_pattern = &adc_pattern_config,
		.sample_freq_hz = sample_rate,
		.conv_mode = (adc_unit==ADC_UNIT_1)?ADC_CONV_SINGLE_UNIT_1:ADC_CONV_SINGLE_UNIT_2,
		.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
	};

	if (adc_continuous_new_handle(&adc_handle_config, &SA8x8->adc) != ESP_OK) {
		ESP_LOGE(TAG,"Error creating adc handle");
		return -1;
	}

	if (adc_continuous_config(SA8x8->adc,&adc_config) != ESP_OK) {
		ESP_LOGE(TAG,"Error configuring adc");
		adc_continuous_deinit(SA8x8->adc);
		return -1;
	}
	
	const adc_continuous_evt_cbs_t adc_cbs = {
		.on_conv_done = SA8x8_Receiver_isr_handler,
	};

	if (adc_continuous_register_event_callbacks(SA8x8->adc,&adc_cbs,SA8x8) != ESP_OK) {
		adc_continuous_deinit(SA8x8->adc);
		ESP_LOGE(TAG,"Error registering adc callback");
		return -1;
	}

	SA8x8->adc_pin = adc_pin;

	return 0;
}

void SA8x8_Receiver_Deinit(SA8x8_t * SA8x8) {
	adc_continuous_deinit(SA8x8->adc);
}

static bool IRAM_ATTR SA8x8_Receiver_isr_handler(adc_continuous_handle_t adc, const adc_continuous_evt_data_t *event,void * arg) {
	BaseType_t MustYield = pdFALSE;
	struct SA8x8_S * SA8x8 = arg;
	struct SA8x8_Msg_S msg;

	msg.type = SA8X8_RECEIVER_DATA;
	msg.size = event->size;
	msg.data = (void*)event->conv_frame_buffer;

	xQueueSendFromISR(SA8x8->queue,(void*)&msg,&MustYield);

	return (MustYield == pdTRUE);
}

void SA8x8_Receiver_Start_Adc(SA8x8_t * SA8x8) {
	if (SA8x8->adc_started) {
		ESP_LOGW(TAG,"adc allready started");
		return;
	}

	adc_continuous_start(SA8x8->adc);
	SA8x8->adc_started = true;

	ESP_LOGD(TAG,"Adc started");
}

void SA8x8_Receiver_Stop_Adc(SA8x8_t * SA8x8) {
	if (!SA8x8->adc_started) {
		ESP_LOGW(TAG,"adc not started");
		return;
	}

	adc_continuous_stop(SA8x8->adc);
	SA8x8->adc_started = false;

	ESP_LOGD(TAG,"Adc stopped");
}

int SA8x8_Start_Receiver(SA8x8_t * SA8x8) {
	int ret;

	if (xSemaphoreTake(SA8x8->state_sem, portMAX_DELAY) != pdPASS)
		return -1;

	switch (SA8x8->state) {
		case SA8X8_STATE_STARTED:
			ESP_LOGD(TAG,"Start receiver");
			SA8x8->state = SA8X8_STATE_RECEIVING;
			if (SA8x8_Gpio_GetSquelch(SA8x8)) {
				struct SA8x8_Msg_S msg = {
					.type = SA8X8_SQUELCH_OPEN
				};
				SA8x8_Send_Event(SA8x8, &msg, portMAX_DELAY);
			}
			/* FALLTHRU */
		case SA8X8_STATE_RECEIVING:
		case SA8X8_STATE_TRANSMITTING:
			if (SA8x8->rx_cnt < 255)
				SA8x8->rx_cnt++;
			ret = 0;
			break;
		default:
			ESP_LOGI(TAG, "Radio not started");
			ret = 1;
			break;
	}
	
	xSemaphoreGive(SA8x8->state_sem);

	return ret;
}

int SA8x8_Stop_Receiver(SA8x8_t * SA8x8) {
	int ret;

	if (xSemaphoreTake(SA8x8->state_sem, portMAX_DELAY) != pdPASS)
		return -1;

	switch (SA8x8->state) {
		case SA8X8_STATE_TRANSMITTING:
			if (SA8x8->rx_cnt)
				SA8x8->rx_cnt--;
			ret = 0;
			break;
		case SA8X8_STATE_RECEIVING:
			SA8x8->rx_cnt--;
			if (!SA8x8->rx_cnt) {
				ESP_LOGD(TAG,"Stop receiver");
				SA8x8->state = SA8X8_STATE_STARTED;
				if (SA8x8_Gpio_GetSquelch(SA8x8)) {
					struct SA8x8_Msg_S msg = {
						.type = SA8X8_SQUELCH_CLOSED
					};
					SA8x8_Send_Event(SA8x8, &msg, portMAX_DELAY);
				}
			}
			/* FALLTHRU */
		case SA8X8_STATE_STARTED:
			ret = 0;
			break;
		default:
			ret = 1;
			break;
	}

	xSemaphoreGive(SA8x8->state_sem);

	return ret;
}

bool SA8x8_Receiver_Is_Started(struct SA8x8_S * SA8x8) {
	return SA8x8->state == SA8X8_STATE_RECEIVING;
}
