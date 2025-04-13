/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * transmiter.c
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
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>
#include <driver/i2s_pdm.h>
#include <soc/gpio_sig_map.h>
#include "SA8x8_priv.h"

#define I2S_NUM_DMA	CONFIG_ADC_CONTINUOUS_NUM_DMA	

static bool SA8x8_Transmiter_isr_handler(i2s_chan_handle_t dac, i2s_event_data_t *event,void * arg);

int SA8x8_Transmiter_Init(struct SA8x8_S * SA8x8, uint16_t sample_rate, uint16_t frame_len, int dac_pin) {
	/* dac i2s pdm config */	
	i2s_chan_config_t dac_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0,I2S_ROLE_MASTER);
	dac_chan_cfg.dma_desc_num = I2S_NUM_DMA;
	dac_chan_cfg.dma_frame_num = frame_len;
	dac_chan_cfg.auto_clear = false;

	if (i2s_new_channel(&dac_chan_cfg, &SA8x8->dac, NULL) != ESP_OK) {
		ESP_LOGE(TAG,"Error creating I2S TX channel");
		return -1;
	}

	i2s_pdm_tx_config_t pdm_tx_cfg = {
		.slot_cfg = I2S_PDM_TX_SLOT_DAC_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
		.gpio_cfg = {
			.clk = -1,
			.dout = -1,
		},
	};

	pdm_tx_cfg.clk_cfg.sample_rate_hz = sample_rate;
    	pdm_tx_cfg.clk_cfg.clk_src = I2S_CLK_SRC_PLL_D2;
	pdm_tx_cfg.clk_cfg.up_sample_fp = (sample_rate) / 55;
	pdm_tx_cfg.clk_cfg.up_sample_fs = (sample_rate) / 110;
	pdm_tx_cfg.clk_cfg.bclk_div = 16;

	if (i2s_channel_init_pdm_tx_mode(SA8x8->dac, &pdm_tx_cfg) != ESP_OK) {
		ESP_LOGE(TAG,"Error configuring I2S TX channel");
		i2s_del_channel(SA8x8->dac);
		return -1;
	}

	const i2s_event_callbacks_t i2s_cbs = {
		.on_sent = SA8x8_Transmiter_isr_handler,
	};
	if (i2s_channel_register_event_callback(SA8x8->dac, &i2s_cbs, SA8x8) != ESP_OK) { 
		ESP_LOGE(TAG,"Error installing I2S TX callback");
		i2s_del_channel(SA8x8->dac);
		return -1;
	}

	gpio_set_direction(dac_pin,GPIO_MODE_DISABLE);
	gpio_set_pull_mode(dac_pin,GPIO_PULLUP_PULLDOWN);

	SA8x8->dac_pin = dac_pin;

	return 0;
}

void SA8x8_Transmiter_Deinit(SA8x8_t * SA8x8) {
	i2s_del_channel(SA8x8->dac);
	gpio_set_pull_mode(SA8x8->dac_pin,GPIO_FLOATING);
	gpio_set_direction(SA8x8->dac_pin,GPIO_MODE_DISABLE);
}

static bool IRAM_ATTR SA8x8_Transmiter_isr_handler(i2s_chan_handle_t dac, i2s_event_data_t *event,void * arg) {
	BaseType_t MustYield = pdFALSE;
	struct SA8x8_S * SA8x8 = arg;
	struct SA8x8_Msg_S msg;

	msg.type = SA8X8_TRANSMITER_DATA;
	msg.data = event->dma_buf;
	msg.size = event->size;

	xQueueSendFromISR(SA8x8->queue,(void*)&msg,&MustYield);

	return MustYield;
}

void SA8x8_Transmiter_Start_Dac(SA8x8_t * SA8x8) {
	i2s_pdm_tx_gpio_config_t tx_gpio = {
		.dout = SA8x8->dac_pin,
		.clk = -1,
	};

	if (SA8x8->dac_started) {
		ESP_LOGW(TAG,"Dac allready started");
		return;
	}

	// start dac
	gpio_set_pull_mode(SA8x8->dac_pin,GPIO_FLOATING);
	i2s_channel_reconfig_pdm_tx_gpio(SA8x8->dac,&tx_gpio);
	i2s_channel_enable(SA8x8->dac);
	SA8x8->dac_started = true;

	ESP_LOGD(TAG,"Dac started");
}

void SA8x8_Transmiter_Stop_Dac(SA8x8_t * SA8x8) {
	i2s_pdm_tx_gpio_config_t tx_gpio = {
		.dout = -1,
		.clk = -1,
	};

	if (!SA8x8->dac_started) {
		ESP_LOGW(TAG,"Dac not started");
		return;
	}

	// Stop dac
	// let pin at mid-level when dac is off
	i2s_channel_disable(SA8x8->dac);
	i2s_channel_reconfig_pdm_tx_gpio(SA8x8->dac,&tx_gpio);
	gpio_set_pull_mode(SA8x8->dac_pin,GPIO_PULLUP_PULLDOWN);
	gpio_set_direction(SA8x8->dac_pin,GPIO_MODE_DISABLE);
	SA8x8->dac_started = false;

	ESP_LOGD(TAG,"Dac stopped");
}

int SA8x8_Start_Transmiter(SA8x8_t * SA8x8) {
	int ret;

	if (xSemaphoreTake(SA8x8->state_sem, portMAX_DELAY) != pdPASS)
		return -1;

	switch (SA8x8->state) {
		case SA8X8_STATE_RECEIVING:
		case SA8X8_STATE_STARTED:
			ESP_LOGD(TAG,"Starting transmiter");
			SA8x8->state = SA8X8_STATE_TRANSMITTING;
			SA8x8_Gpio_SetPtt(SA8x8,true);
			/* FALLTHRU */
		case SA8X8_STATE_TRANSMITTING:
			if (SA8x8->tx_cnt<255)
				SA8x8->tx_cnt++;
			ret = 0;
			break;
		default:
			ret = 1;
			break;
	}
	
	xSemaphoreGive(SA8x8->state_sem);

	return ret;
}

int SA8x8_Stop_Transmiter(SA8x8_t * SA8x8) {
	int ret;

	if (xSemaphoreTake(SA8x8->state_sem, portMAX_DELAY) != pdPASS)
		return -1;

	switch (SA8x8->state) {
		case SA8X8_STATE_TRANSMITTING:
			SA8x8->tx_cnt--;
			if (!SA8x8->tx_cnt) {
				if (SA8x8->rx_cnt) {
					SA8x8->state = SA8X8_STATE_RECEIVING;
				} else
					SA8x8->state = SA8X8_STATE_STARTED;
				ESP_LOGD(TAG,"Stoping transmiter");
				SA8x8_Gpio_SetPtt(SA8x8,false);
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

bool SA8x8_Transmiter_Is_Started(struct SA8x8_S * SA8x8) {
	return (SA8x8->state == SA8X8_STATE_TRANSMITTING);
}
