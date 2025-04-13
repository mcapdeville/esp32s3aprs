/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * SA8x8.c
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
#include <nvs_flash.h>
#include "SA8x8_priv.h"

#ifndef NDEBUG
#define TASK_STACK_SIZE		3072+1024
#else
#define TASK_STACK_SIZE		3072
#endif
#define TASK_PRIORITY		5
#define NUM_STATIC_TASK		1

struct {
	TaskHandle_t handle;
	StackType_t	stack[(TASK_STACK_SIZE)/sizeof(StackType_t)];
	StaticTask_t buffer;
} SA8X8_Tasks[NUM_STATIC_TASK];

static void SA8x8_Task(void * arg);
static void SA8x8_Event_Cb(SA8x8_t * SA8x8, SA8x8_Msg_t * Msg);

SA8x8_t * SA8x8_Init(const SA8x8_config_t * config) {
	SA8x8_t * SA8x8;
	int i=0;

	if (!(SA8x8 = heap_caps_malloc(sizeof(SA8x8_t),MALLOC_CAP_INTERNAL))) {
		ESP_LOGE(TAG,"Error allocating SA8x8 struct");
		return NULL;
	}
	bzero(SA8x8,sizeof(SA8x8_t));

	SA8x8->state_sem = xSemaphoreCreateMutexStatic(&SA8x8->state_sem_buff);

	if (SA8x8_Gpio_Init(SA8x8,config->sq_pin,config->ptt_pin,config->pd_pin,config->hl_pin)) {
		ESP_LOGE(TAG,"Error initializing gpio");
		free(SA8x8);
		return NULL;
	}

	if (SA8x8_Uart_Init(SA8x8,config->uart_num,9600,config->tx_pin,config->rx_pin)) {
		ESP_LOGE(TAG,"Error initializing uart");
		SA8x8_Gpio_Deinit(SA8x8);
		free(SA8x8);
		return NULL;
	}

	if (SA8x8_Receiver_Init(SA8x8,config->sample_rate,config->frame_len,config->adc_pin)) {
		ESP_LOGE(TAG,"Error initializing adc");
		SA8x8_Gpio_Deinit(SA8x8);
		SA8x8_Uart_Deinit(SA8x8);
		free(SA8x8);
		return NULL;
	}

	if (SA8x8_Transmiter_Init(SA8x8,config->sample_rate,config->frame_len,config->dac_pin)) {
		ESP_LOGE(TAG,"Error initializing dac");
		SA8x8_Receiver_Deinit(SA8x8);
		SA8x8_Gpio_Deinit(SA8x8);
		SA8x8_Uart_Deinit(SA8x8);
		free(SA8x8);
		return NULL;
	}

	if (!(SA8x8->queue = xQueueCreateStatic(SA8X8_QUEUE_LEN,sizeof(struct SA8x8_Msg_S),(uint8_t*)&SA8x8->QueueStore,&SA8x8->QueueBuffer))) {
		ESP_LOGE(TAG,"Error creating receiver message queue");
		SA8x8_Transmiter_Deinit(SA8x8);
		SA8x8_Receiver_Deinit(SA8x8);
		SA8x8_Gpio_Deinit(SA8x8);
		SA8x8_Uart_Deinit(SA8x8);
		free(SA8x8);
		return NULL;
	}

	Dmabuff_Init(&SA8x8->sample_buff);

	i= 0;
	while (SA8X8_Tasks[i].handle && i < NUM_STATIC_TASK) i++;
	if (i == NUM_STATIC_TASK) {
		ESP_LOGE(TAG,"Error creating SA8x8 task\n");
		vQueueDelete(SA8x8->queue);
		SA8x8_Transmiter_Deinit(SA8x8);
		SA8x8_Receiver_Deinit(SA8x8);
		SA8x8_Gpio_Deinit(SA8x8);
		SA8x8_Uart_Deinit(SA8x8);
		free(SA8x8);
		return NULL;
	}
	SA8X8_Tasks[i].handle = xTaskCreateStaticPinnedToCore(SA8x8_Task,"SA8X8", TASK_STACK_SIZE, SA8x8, TASK_PRIORITY, SA8X8_Tasks[i].stack, &SA8X8_Tasks[i].buffer, PRO_CPU_NUM);

	SA8x8->task = SA8X8_Tasks[i].handle;

	return SA8x8;
}

int SA8X8_Load_Config(SA8x8_t *SA8x8) {
	char version[16];
	uint8_t val;
	size_t len;

	nvs_open("Radio",NVS_READWRITE,&SA8x8->nvs);

	// Get SA8x8->nvs parameters
	if (nvs_get_u8(SA8x8->nvs, "Bandwidth", (uint8_t*)&SA8x8->group.bw))
		SA8x8->group.bw = SA8X8_DEFAULT_BANDWIDTH;

	if (!nvs_get_u32(SA8x8->nvs, "Freq", &SA8x8->group.rx_freq))
		SA8x8->group.tx_freq = SA8x8->group.rx_freq;
	else {
		if (nvs_get_u32(SA8x8->nvs, "RxFreq", &SA8x8->group.rx_freq))
			SA8x8->group.rx_freq = SA8X8_DEFAULT_FREQ;
		if (nvs_get_u32(SA8x8->nvs, "TxFreq", &SA8x8->group.rx_freq))
			SA8x8->group.tx_freq = SA8X8_DEFAULT_FREQ;
	}
	len = sizeof(SA8x8->group.rx_sub);
	if (!nvs_get_str(SA8x8->nvs, "Sub", SA8x8->group.rx_sub, &len))
		strncpy(SA8x8->group.tx_sub, SA8x8->group.rx_sub, sizeof(SA8x8->group.tx_sub));
	else {
		len = sizeof(SA8x8->group.rx_sub);
		if (nvs_get_str(SA8x8->nvs, "RxSub", SA8x8->group.rx_sub, &len))
			strncpy(SA8x8->group.rx_sub, SA8X8_DEFAULT_SUBAUDIO, sizeof(SA8x8->group.rx_sub));
		len = sizeof(SA8x8->group.tx_sub);
		if (nvs_get_str(SA8x8->nvs, "TxSub", SA8x8->group.tx_sub, &len))
			strncpy(SA8x8->group.tx_sub, SA8X8_DEFAULT_SUBAUDIO, sizeof(SA8x8->group.tx_sub));
	}

	if (nvs_get_u8(SA8x8->nvs, "Squelch", &SA8x8->group.squelch))
		SA8x8->group.squelch = SA8X8_DEFAULT_SQUELCH;

	if (nvs_get_u8(SA8x8->nvs, "Volume", &SA8x8->volume))
		SA8x8->volume = SA8X8_DEFAULT_VOLUME;

	if (nvs_get_u8(SA8x8->nvs, "Emphasis", (uint8_t*)&SA8x8->emph))
		SA8x8->emph = SA8X8_DEFAULT_EMPHASIS;

	if (nvs_get_u8(SA8x8->nvs, "Hipass", (uint8_t*)&SA8x8->hi_pass))
		SA8x8->hi_pass = SA8X8_DEFAULT_HIPASS;

	if (nvs_get_u8(SA8x8->nvs, "Lowpass", (uint8_t*)&SA8x8->low_pass))
		SA8x8->low_pass = SA8X8_DEFAULT_LOWPASS;

	if (nvs_get_u8(SA8x8->nvs, "Tail", &SA8x8->tail))
		SA8x8->tail = SA8X8_DEFAULT_TAIL;

	if (nvs_get_u8(SA8x8->nvs, "Power", (uint8_t*)&SA8x8->power))
		SA8x8->power = SA8X8_DEFAULT_POWER;


	if (SA8x8->power)
		SA8x8_Gpio_SetPower(SA8x8, SA8x8->power);

	val = 10;
	do {
		if (!SA8x8_Handcheck(SA8x8))
			break;
		vTaskDelay(100/portTICK_PERIOD_MS);
		val--;
	} while (val);

	if (!val) {
		ESP_LOGE(TAG, "Error initializing SA8X8 radio");
	} else {

		if (SA8x8_GetVersion(SA8x8,version,sizeof(version)-1))
			SA8x8_GetVersion(SA8x8,version,sizeof(version)-1);
		ESP_LOGI(TAG,"SA8x8 version : %s\n", version);

		val=0;
		if (nvs_get_u8(SA8x8->nvs, "RadioReset" , &val) || val) {
			SA8x8_Uart_SetGroup(SA8x8, &SA8x8->group);
			SA8x8_Uart_SetFilter(SA8x8, SA8x8->emph, SA8x8->hi_pass, SA8x8->low_pass);
			SA8x8_Uart_SetVolume(SA8x8, SA8x8->volume);
			SA8x8_Uart_SetTail(SA8x8, SA8x8->tail);

			nvs_set_u8(SA8x8->nvs, "RadioReset", 0);
		}
	}

	SA8x8_Gpio_SetPower(SA8x8, SA8x8->power);

	ESP_LOGD(TAG, "Radio Configured");

	return 0;
}

Dmabuff_t * SA8x8_Get_Buff(SA8x8_t * SA8x8) {
	return &SA8x8->sample_buff;
}
__attribute__((hot)) static void SA8x8_Task(void * arg) {
		struct SA8x8_S * SA8x8 = arg;
		struct SA8x8_Msg_S msg;
		bool cb = true;

		do {
			if (!(xQueueReceive(SA8x8->queue, &msg, portMAX_DELAY)))
				ESP_LOGE(TAG,"xQueueReceive return FALSE");

			switch (msg.type) {
				case SA8X8_TRANSMITER_DATA:
					// Buffer is not reset by i2s driver (auto_clear = false);
					bzero(msg.data,msg.size);
					Dmabuff_Add_Block(&SA8x8->sample_buff,msg.data,msg.size);
					break;
				case SA8X8_RECEIVER_DATA:
					adc_digi_output_data_t* src = msg.data;
					int16_t * dst = (int16_t*)src;

					// convert inplace to int16_t
					msg.size>>=1;
					for (int i=msg.size>>1;i;i--)
						*(dst++) = (((int16_t)((src++)->type2.data))-(1<<(SOC_ADC_DIGI_MAX_BITWIDTH-1)))<<(16-SOC_ADC_DIGI_MAX_BITWIDTH);
					Dmabuff_Add_Block(&SA8x8->sample_buff,msg.data,msg.size);
					break;
				case SA8X8_SQUELCH_OPEN:
					ESP_LOGD(TAG,"Squelch open");
					if (SA8x8->state == SA8X8_STATE_RECEIVING && !SA8x8->adc_started) {
						SA8x8_Receiver_Start_Adc(SA8x8);
					}
					break;
				case SA8X8_SQUELCH_CLOSED:
					ESP_LOGD(TAG,"Squelch close");
					if (SA8x8->adc_started) {
						SA8x8_Receiver_Stop_Adc(SA8x8);
						Dmabuff_Clear(&SA8x8->sample_buff);
					}
					else if (SA8x8->state == SA8X8_STATE_TRANSMITTING)
						cb = false;
					break;
				case SA8X8_PTT_PUSHED:
					ESP_LOGD(TAG,"Ptt pushed");
					if (SA8x8->state == SA8X8_STATE_TRANSMITTING) {
						if (SA8x8->adc_started) {
							struct SA8x8_Msg_S msg = {
								.type = SA8X8_SQUELCH_CLOSED
							};
							SA8x8_Receiver_Stop_Adc(SA8x8);
							Dmabuff_Clear(&SA8x8->sample_buff);
							SA8x8_Event_Cb(SA8x8,&msg);
						}

						SA8x8_Transmiter_Start_Dac(SA8x8);
					}
					break;
				case SA8X8_PTT_RELEASED:
					ESP_LOGD(TAG,"Ptt released");
					if (SA8x8->dac_started) {
						SA8x8_Transmiter_Stop_Dac(SA8x8);
						Dmabuff_Clear(&SA8x8->sample_buff);
					}
					break;
			}
			if (cb)
				SA8x8_Event_Cb(SA8x8,&msg);
			else 
				cb = true;

		} while (1);
	}

int IRAM_ATTR SA8x8_Send_Event(SA8x8_t * SA8x8, struct SA8x8_Msg_S *msg, TickType_t to) {
	if (xQueueSend(SA8x8->queue,(void*)msg,to) != pdPASS)
		return -1;
	return 0;
}

int SA8x8_Register_Cb(SA8x8_t * SA8x8, SA8x8_Cb_t Cb, void * Ctx) {
	struct SA8x8_Cb_List_S * cb_list;
	if (Cb && Ctx) {
		if (!(cb_list = malloc(sizeof(struct SA8x8_Cb_List_S)))) {
			ESP_LOGE(TAG,"Error allocating cb_list");
			return -1;
		}
		bzero(cb_list,sizeof(struct SA8x8_Cb_List_S));

		cb_list->cb = Cb;
		cb_list->cb_ctx = Ctx;
		cb_list->next = SA8x8->cb_list;
		SA8x8->cb_list = cb_list;
		return 0;
	}

	return -1;
}

int SA8x8_Unregister_Cb(SA8x8_t * SA8x8, void * Ctx) {
	struct SA8x8_Cb_List_S * cb_list, **prev;
	if (Ctx) {
		prev = &SA8x8->cb_list;
		cb_list = *prev;
		while (cb_list) {
			if (cb_list->cb_ctx == Ctx) {
				*prev = cb_list->next;
				free(cb_list);
			} else
				prev = &cb_list->next;
			cb_list = *prev;
		}
		return 0;
	}
	return -1;
}

void IRAM_ATTR SA8x8_Event_Cb(SA8x8_t * SA8x8, SA8x8_Msg_t * Msg) {
	struct SA8x8_Cb_List_S * cb_list = SA8x8->cb_list;

	while (cb_list) {
		cb_list->cb(cb_list->cb_ctx, Msg);
		cb_list = cb_list->next;
	}
}

// Radio interface
int SA8x8_Set_Freq(SA8x8_t * SA8x8, uint32_t Freq) {
	int ret = 0;
	SA8x8_Group_t group;

	if (SA8x8->group.rx_freq != Freq || SA8x8->group.tx_freq != Freq) {
		memcpy(&group, &SA8x8->group, sizeof(SA8x8_Group_t));
		group.rx_freq = Freq;
		group.tx_freq = Freq;
		ret = SA8x8_Uart_SetGroup(SA8x8, &group);
		if (!ret) {
			SA8x8->group.tx_freq = Freq;
			SA8x8->group.rx_freq = Freq;
			nvs_erase_key(SA8x8->nvs, "RxFreq");
			nvs_erase_key(SA8x8->nvs, "TxFreq");
			nvs_set_u32(SA8x8->nvs, "Freq", Freq);
			nvs_commit(SA8x8->nvs);
		}
	}

	return ret;
}

uint32_t SA8x8_Get_RxFreq(SA8x8_t * SA8x8) {
	return SA8x8->group.rx_freq;
}

uint32_t SA8x8_Get_TxFreq(SA8x8_t * SA8x8) {
	return SA8x8->group.tx_freq;
}

int SA8x8_Set_Sub(SA8x8_t * SA8x8, const char * Sub) {
	int ret = 0;
	SA8x8_Group_t group;

	if (strncmp(SA8x8->group.rx_sub, Sub, 4) || strncmp(SA8x8->group.tx_sub, Sub, 4)) {
		memcpy(&group, &SA8x8->group, sizeof(SA8x8_Group_t));
		strncpy(group.rx_sub, Sub, sizeof(SA8x8->group.rx_sub));
		strncpy(group.tx_sub, Sub, sizeof(SA8x8->group.tx_sub));

		ret = SA8x8_Uart_SetGroup(SA8x8, &group);

		if (!ret) {
			strncpy(SA8x8->group.rx_sub, Sub, sizeof(SA8x8->group.rx_sub));
			strncpy(SA8x8->group.tx_sub, Sub, sizeof(SA8x8->group.tx_sub));
			nvs_erase_key(SA8x8->nvs, "RxSub");
			nvs_erase_key(SA8x8->nvs, "TxSub");
			nvs_set_str(SA8x8->nvs, "Sub", Sub);
			nvs_commit(SA8x8->nvs);
		}
	}

	return ret;
}
int SA8x8_Get_RxSub(SA8x8_t * SA8x8, char *Sub, int len) {
	strncpy(Sub, SA8x8->group.rx_sub, len);
	return 0;
}

int SA8x8_Get_TxSub(SA8x8_t * SA8x8, char *Sub, int len) {
	strncpy(Sub, SA8x8->group.tx_sub, len);
	return 0;
}

int SA8x8_Set_Squelch(SA8x8_t * SA8x8, uint8_t Sq) {
	int ret = 0;
	SA8x8_Group_t group;

	if (SA8x8->group.squelch != Sq) {
		memcpy(&group, &SA8x8->group, sizeof(SA8x8_Group_t));
		group.squelch = Sq;
		ret = SA8x8_Uart_SetGroup(SA8x8, &group);
		if (!ret) {
			SA8x8->group.squelch = Sq;
			nvs_set_u8(SA8x8->nvs, "Squelch", Sq);
			nvs_commit(SA8x8->nvs);
		}
	}

	return ret;
}

uint8_t SA8x8_Get_Squelch(SA8x8_t * SA8x8) {
	return SA8x8->group.squelch;
}

int SA8x8_Set_Bandwidth(SA8x8_t * SA8x8, SA8x8_Bw_t Bw) {
	int ret = 0;
	SA8x8_Group_t group;

	if (SA8x8->group.bw != Bw) {
		memcpy(&group, &SA8x8->group, sizeof(SA8x8_Group_t));
		group.bw = Bw;
		ret = SA8x8_Uart_SetGroup(SA8x8, &group);
		if (!ret) {
			SA8x8->group.bw = Bw;
			nvs_set_u8(SA8x8->nvs, "Bandwidth", Bw);
			nvs_commit(SA8x8->nvs);
		}
	}

	return ret;
}

SA8x8_Bw_t SA8x8_Get_Bandwidth(SA8x8_t * SA8x8) {
	return SA8x8->group.bw;
}

int SA8x8_Set_Emphasis(SA8x8_t * SA8x8, bool Em) {
	int ret = 0;

	if (SA8x8->emph != Em) {
		ret =  SA8x8_Uart_SetFilter(SA8x8, Em, SA8x8->hi_pass, SA8x8->low_pass);
		if (!ret) {
			SA8x8->emph = Em;
			nvs_set_u8(SA8x8->nvs, "Emphasis", (uint8_t)Em);
			nvs_commit(SA8x8->nvs);
		}
	}

	return ret;
}

bool SA8x8_Get_Emphasis(SA8x8_t * SA8x8) {
	return SA8x8->emph;
}

int SA8x8_Set_Hipass(SA8x8_t * SA8x8, bool Hp) {
	int ret = 0;

	if (SA8x8->hi_pass != Hp) {
		SA8x8->hi_pass = Hp;
		ret =  SA8x8_Uart_SetFilter(SA8x8,SA8x8->emph, Hp, SA8x8->low_pass);
		if (!ret) {
			SA8x8->hi_pass = Hp;
			nvs_set_u8(SA8x8->nvs, "Hipass", (uint8_t)Hp);
			nvs_commit(SA8x8->nvs);
		}
	}

	return ret;
}

bool SA8x8_Get_Hipass(SA8x8_t * SA8x8) {
	return SA8x8->hi_pass;
}

int SA8x8_Set_Lowpass(SA8x8_t * SA8x8, bool Lp) {
	int ret = 0;

	if (SA8x8->low_pass != Lp) {
		ret =  SA8x8_Uart_SetFilter(SA8x8,SA8x8->emph, SA8x8->hi_pass, Lp);
		if (!ret) {
			SA8x8->low_pass = Lp;
			nvs_set_u8(SA8x8->nvs, "Lowpass", (uint8_t)Lp);
			nvs_commit(SA8x8->nvs);
		}
	}

	return ret;
}

bool SA8x8_Get_Lowpass(SA8x8_t * SA8x8) {
	return SA8x8->low_pass;
}

int SA8x8_Set_Volume(SA8x8_t * SA8x8, uint8_t V) {
	int ret = 0;

	if (SA8x8->volume != V) {
		ret = SA8x8_Uart_SetVolume(SA8x8, V);
		if (!ret) {
			SA8x8->volume = V;
			nvs_set_u8(SA8x8->nvs, "Volume", V);
			nvs_commit(SA8x8->nvs);
		}
		return ret;
	} 

	return ret;
}

uint8_t SA8x8_Get_Volume(SA8x8_t * SA8x8) {
	return SA8x8->volume;
}

int SA8x8_Set_Tail(SA8x8_t * SA8x8, uint8_t Tail) {
	int ret = 0;

	if (SA8x8->tail != Tail) {
		ret = SA8x8_Uart_SetTail(SA8x8, Tail);
		if (!ret) {
			SA8x8->tail = Tail;
			nvs_set_u8(SA8x8->nvs, "Tail", Tail);
			nvs_commit(SA8x8->nvs);
		}
	}

	return ret;
}

uint8_t SA8x8_Get_Tail(SA8x8_t * SA8x8) {
	return SA8x8->tail;
}

int SA8x8_Set_Power(SA8x8_t * SA8x8,  SA8x8_Power_t Power) {
	int ret = 0;

	if (SA8x8->power != Power) {
		ret = SA8x8_Gpio_SetPower(SA8x8, Power);
		if (!ret) {
			SA8x8->power = Power;
			nvs_set_u8(SA8x8->nvs, "Power", (uint8_t)Power);
			nvs_commit(SA8x8->nvs);
		}
	}

	return ret;
}

SA8x8_Power_t SA8x8_Get_Power(SA8x8_t * SA8x8) {
	return SA8x8->power;
}

