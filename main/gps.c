/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/gps.c
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

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_err.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <nvs_flash.h>
#include <nvs.h>
#include "gps.h"
#include "gps_parsers.h"
#include <ssd1680.h>

#define TAG	"GPS"

#define GPS_DEFAULT_INTERVAL	10

#define GPS_EVENT_QUEUE_SIZE 10
#define GPS_PATTERN_QUEUE_SIZE 2
#define GPS_COMMAND_QUEUE_SIZE 10
#define GPS_RX_BUFF_LEN NMEA_MAX_LENGTH*2
#define GPS_TX_BUFF_LEN NMEA_MAX_LENGTH*2

#ifndef NDEBUG
#define GPS_TASK_STACK_SIZE 3072+1024
#else
#define GPS_TASK_STACK_SIZE 3072
#endif
#define GPS_TASK_PRIORITY 4
#define GPS_MAX_COMMAND_TRY 3
#define GPS_COMMAND_RETRY_PERIOD_MS 500
#define GPS_EVENT_POST_TO	30


struct GPS_S {
	int8_t uart_num;
	bool ready;
	bool standby;
	QueueHandle_t uart_queue;
	QueueHandle_t command_queue;
	StaticQueue_t command_queue_data;
	uint8_t command_queue_buff[NMEA_MAX_LENGTH*GPS_COMMAND_QUEUE_SIZE];
	uint32_t baud;
	int reset_pin;
	TaskHandle_t task_handle;
	char buffer[NMEA_MAX_LENGTH+1];
	char command[NMEA_MAX_LENGTH+1];
	int pos;
	GPS_Data_t data;
};

ESP_EVENT_DEFINE_BASE(GPS_EVENT);

extern SSD1680_t * Epd;
extern nvs_handle_t Nvs;

static const char HexTab[16] = "0123456789ABCDEF";

static int GPS_Get_Sentence(GPS_t * Gps) {
	int pat_pos = uart_pattern_pop_pos(Gps->uart_num);
	int ret;
	
	ESP_LOGV(TAG,"Buffer pos : %d, Pattern pos : %d",Gps->pos,pat_pos);

	if (pat_pos == -1 ) {
		ESP_LOGE(TAG,"uart buffer or queue overflow.");
		Gps->pos=0;
		Gps->buffer[0] = '\0';
		uart_flush(Gps->uart_num);
		xQueueReset(Gps->uart_queue);
		return -1;
	}

	if ((Gps->pos + (pat_pos+1)) > NMEA_MAX_LENGTH) {
		ESP_LOGE(TAG,"Sentence too long");
		Gps->pos=0;
		Gps->buffer[0] = '\0';
		uart_flush(Gps->uart_num);
		xQueueReset(Gps->uart_queue);
		return -1;
	}

	ret = uart_read_bytes(Gps->uart_num,&Gps->buffer[Gps->pos],pat_pos+1,0);
	if (ret == -1) {
		ESP_LOGE(TAG,"Error reading sentence");
		Gps->pos=0;
		Gps->buffer[0] = '\0';
		uart_flush(Gps->uart_num);
		xQueueReset(Gps->uart_queue);
		return -1;
	}

	ESP_LOGV(TAG,"Read %d bytes",ret);
	Gps->pos += ret;

	if (Gps->pos >=2) {
		if ((Gps->buffer[Gps->pos-1] != '\n' || Gps->buffer[Gps->pos-2] != '\r')) {
			Gps->buffer[Gps->pos] = '\0';
			ESP_LOGE(TAG,"Bad terminated sentence : %s",Gps->buffer);
			Gps->buffer[0] = '\0';
			Gps->pos = 0;
			uart_flush(Gps->uart_num);
			xQueueReset(Gps->uart_queue);
			return -1;
		}
		// suppress end of line chars
		Gps->buffer[Gps->pos-2] = '\0';
		ret = Gps->pos-2;
	} else {
		Gps->buffer[0] = '\0';
		ret = 0;
	}

	Gps->pos = 0;

	return ret;
}

static void GPS_Task(void * arg) {
	uart_event_t event;
	GPS_t * Gps = (GPS_t*)arg;
	bool gps_started = false;
	int command_id;
	TickType_t command_time;
	int command_try;
	int ret;
	uint8_t gps_reset;
	uint16_t gps_interval;

	ESP_LOGD(TAG,"Task started");

	if (nvs_get_u8(Nvs,"GpsReset",&gps_reset))
		gps_reset = 1;

	if (nvs_get_u16(Nvs,"GpsInterval",&gps_interval))
		gps_interval = GPS_DEFAULT_INTERVAL;

	// Full cold tart
	if (gps_reset) {
		ESP_LOGI(TAG,"Reseting GPS to factory default");
		GPS_Reset(Gps);
		SSD1680_PowerUp(Epd);
		vTaskDelay(500/portTICK_PERIOD_MS);
		GPS_Send(Gps,"PMTK104");
		vTaskDelay(100/portTICK_PERIOD_MS);
		SSD1680_PowerDown(Epd);

		gps_reset = 0;
		if (nvs_set_u8(Nvs,"GpsReset",gps_reset))
			ESP_LOGE(TAG,"Error writing GpsReset=%u to nvs",gps_reset);

		if (nvs_commit(Nvs))
			ESP_LOGE(TAG,"Error will writing nvs to flash");
		else
			ESP_LOGI(TAG,"Gps reset done");

	}

	// Enable GPS, GLONASS, GALILEO
	GPS_QueueCommand(Gps,"PMTK353,1,1,1");
	// Set RMC,GGA position interval in fix number
	GPS_QueueCommand(Gps,"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
	// Set Fix interval
	snprintf(Gps->buffer,sizeof(Gps->buffer),"PMTK220,%lu",((uint32_t)gps_interval)*1000);
	GPS_QueueCommand(Gps,Gps->buffer);
	GPS_QueueCommand(Gps,"PMTK225,8");


	while (1) {
		if (xQueueReceive(Gps->uart_queue,&event,portMAX_DELAY) == pdPASS) {
			assert(Gps->uart_num == 1);
			switch (event.type) {
				case UART_DATA:
					ESP_LOGV(TAG,"Data event %d : %s ",event.size,Gps->buffer);
					if (event.size) {
						if ((Gps->pos + event.size) > NMEA_MAX_LENGTH ) {
							ESP_LOGE(TAG,"NMEA buffer overflow");
							Gps->pos = 0;
							Gps->buffer[0] = '\0';
							uart_flush(Gps->uart_num);
							xQueueReset(Gps->uart_queue);
							break;
						}
						ret = uart_read_bytes(Gps->uart_num,&Gps->buffer+Gps->pos,event.size,0);
						if (ret == -1) {
							ESP_LOGE(TAG,"Error reading data");
							Gps->pos=0;
							Gps->buffer[0] = '\0';
							uart_flush(Gps->uart_num);
							xQueueReset(Gps->uart_queue);
							break;
						}
						Gps->pos+=ret;
						Gps->buffer[Gps->pos] = '\0';
					}
					break;
				case UART_FIFO_OVF:
					ESP_LOGW(TAG, "HW FIFO Overflow");
					Gps->pos = 0;
					Gps->buffer[0] = '\0';
					uart_flush(Gps->uart_num);
					xQueueReset(Gps->uart_queue);
					break;
				case UART_BUFFER_FULL:
					ESP_LOGW(TAG, "Ring buffer Full");
					Gps->pos = 0;
					Gps->buffer[0] = '\0';
					uart_flush(Gps->uart_num);
					xQueueReset(Gps->uart_queue);
					break;
				case UART_BREAK:
					ESP_LOGW(TAG, "Rx Break");
					Gps->pos = 0;
					Gps->buffer[0] = '\0';
					uart_flush(Gps->uart_num);
					//xQueueReset(Gps->uart_queue);
					break;
				case UART_PARITY_ERR:
					ESP_LOGE(TAG, "Parity Error");
					Gps->pos = 0;
					Gps->buffer[0] = '\0';
					uart_flush(Gps->uart_num);
					xQueueReset(Gps->uart_queue);
					break;
				case UART_FRAME_ERR:
					ESP_LOGE(TAG, "Frame Error");
					Gps->pos = 0;
					Gps->buffer[0] = '\0';
					uart_flush(Gps->uart_num);
					xQueueReset(Gps->uart_queue);
					break;
				case UART_PATTERN_DET:
					ESP_LOGV(TAG,"Pattern detection event");
					ret = GPS_Get_Sentence(Gps);
					if (ret>0) {
						ESP_LOGD(TAG,"Sentence : %s",Gps->buffer);
						ret = GPS_Parse(Gps->buffer, ret, &Gps->data);
					}
					else
						break;

					if (Gps->data.valid & GPS_DATA_VALID_FIX_STATUS 
						/*	&& Gps->data.fix_status != GPS_FIX_STATUS_NOFIX */
							&& Gps->data.id == GPS_PARSER_RMC
							&& GPS_IS_DATA_VALID(Gps->data.valid, GPS_DATA_VALID_TIME | GPS_DATA_VALID_DATE)) {
						struct tm tm = {
							.tm_sec = Gps->data.time.seconds,
							.tm_min = Gps->data.time.minutes,
							.tm_hour = Gps->data.time.hours,
							.tm_mday = Gps->data.date.day,
							.tm_mon = Gps->data.date.month,
							.tm_year = Gps->data.date.year+((Gps->data.date.year>=80)?1900:2000),
						};
						time_t time = mktime(&tm);
						struct timeval tv = {
							.tv_sec = time,
							// TODO : use 1PPS to sync tv_usec
							.tv_usec = Gps->data.time.milliseconds*1000,
						};
						settimeofday(&tv,NULL);
#if 0
						if (!Gps->standby) {
							if (Gps->data.fix_status) {
								// Enable Allwayslocate standby mode
								ESP_LOGD(TAG,"Enable Allways Locate standby mode");
								GPS_QueueCommand(Gps,"PMTK225,8");
								Gps->standby = true;
							}
						} else {
							if (!Gps->data.fix_status) {
								ESP_LOGD(TAG,"Enable full on mode");
								GPS_QueueCommand(Gps,"PMTK225,0");
								Gps->standby = false;
							}
						}
#endif

					}
					
					if (ret >=0 && ret < GPS_PARSER_MAX)
						esp_event_post(GPS_EVENT,ret,&Gps->data,sizeof(GPS_Data_t), GPS_EVENT_POST_TO);

					if (!gps_started) {
						if (Gps->data.id == GPS_PARSER_PMTK010 && Gps->data.sys_msg == 1) {
							// gps is starting
							gps_started = true;
							ESP_LOGI(TAG,"Started !");
							command_id = 0;
						}
					} else {
						if (command_id) {
							if (Gps->data.id == GPS_PARSER_PMTK001 && Gps->data.ack == command_id) {
								command_id = 0; // TODO : Test data.ack_flag
							} else {
								if (command_try < GPS_MAX_COMMAND_TRY) {
									if ((xTaskGetTickCount()-command_time) >= (GPS_COMMAND_RETRY_PERIOD_MS/portTICK_PERIOD_MS)) {
										command_try++;
										ESP_LOGW(TAG,"Retrying command (%d) : %s",command_try, Gps->command);
										GPS_Send(Gps,Gps->command);
										command_time = xTaskGetTickCount();
									}
								} else {
									command_id = 0;

									ESP_LOGE(TAG,"Command failed : %s", Gps->command);
#if 0
									if (command_id == 225)
										Gps->standby = !Gps->standby;
#endif
								}
							}
						}
						if (!command_id) {
							if (xQueueReceive(Gps->command_queue,Gps->command,0) == pdPASS) {
								Gps->command[sizeof(Gps->command)-1] = '\0';
								ESP_LOGI(TAG,"Sending command : %s",Gps->command);
								GPS_Send(Gps,Gps->command);
								command_try = 1;
								command_time = xTaskGetTickCount();
								if (!strncmp(Gps->command,"PMTK",4)) {
									command_id = atoi(Gps->command+4);
								} else
									command_id = 0;
							}
						}
					}
					break;
				default:
					ESP_LOGW(TAG, "unknown uart event type: %d", event.type);
					break;
			}
		} else {
			ESP_LOGE(TAG,"Error waiting uart event");
		}
	}
}

/* GPS Initialisation */
GPS_t * GPS_Init(int uart_num, int Baud, int tx_pin,int rx_pin, int reset_pin) {
	GPS_t *gps;

	if (!(gps = heap_caps_malloc(sizeof(struct GPS_S),MALLOC_CAP_INTERNAL))) {
		ESP_LOGE(TAG,"Error allocating GPS structure");
		return NULL;
	}
	bzero(gps,sizeof(GPS_t));

	const gpio_config_t out_pins_config = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT_OD,
		.pin_bit_mask = (1ULL<<reset_pin),
		.pull_down_en = 0,
		.pull_up_en = 0, // (1ULL<<reset_pin),
	};

	gpio_set_level(reset_pin,1);
	gpio_config(&out_pins_config);
	
	gps->reset_pin = reset_pin;

	const	uart_config_t uart_config = {
		.baud_rate = Baud,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_XTAL,
	};

#if CONFIG_UART_ISR_IN_IRAM	
	if (uart_driver_install(uart_num,GPS_RX_BUFF_LEN,GPS_TX_BUFF_LEN,GPS_EVENT_QUEUE_SIZE,&gps->uart_queue,ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_SHARED) != ESP_OK) {
		ESP_LOGE(TAG,"Error installing uart driver");
		free(gps);
		return NULL;
	}

#else
#error "You must set CONFIG_UART_ISR_IN_IRAM=y in sdkconfig"
#endif

	ESP_ERROR_CHECK(uart_param_config(uart_num,&uart_config));
	ESP_ERROR_CHECK(uart_set_pin(uart_num,tx_pin,rx_pin,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));

	uart_intr_config_t intr_conf = {
		.intr_enable_mask = 0,
		.rx_timeout_thresh = 0,
		.txfifo_empty_intr_thresh = 0,
		.rxfifo_full_thresh = 0
	};

	ESP_ERROR_CHECK(uart_intr_config(uart_num, &intr_conf));

	uart_enable_pattern_det_baud_intr(uart_num,'\n',1,1,0,0);
	uart_pattern_queue_reset(uart_num,GPS_PATTERN_QUEUE_SIZE);

	gps->uart_num = uart_num;
	gps->baud = Baud;

	gpio_set_level(gps->reset_pin,1);

	gps->command_queue = xQueueCreateStatic(GPS_COMMAND_QUEUE_SIZE,NMEA_MAX_LENGTH,gps->command_queue_buff,&gps->command_queue_data);

	if (pdTRUE != xTaskCreate(GPS_Task,"GPS",GPS_TASK_STACK_SIZE,(void*)gps,GPS_TASK_PRIORITY,&gps->task_handle)) {
		ESP_LOGE(TAG,"Error creating gps task");
		ESP_ERROR_CHECK(uart_driver_delete(gps->uart_num));
		free(gps);
		return NULL;
	}

	return gps;
}

void GPS_DeInit(GPS_t * Gps) {
	const gpio_config_t out_pins_config = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_DISABLE,
		.pin_bit_mask = (1ULL<<Gps->reset_pin),
		.pull_down_en = 0,
		.pull_up_en = 0,
	};

	gpio_config(&out_pins_config);

	uart_driver_delete(Gps->uart_num);
	vTaskDelete(Gps->task_handle);
	free(Gps);
}

void GPS_Reset(GPS_t * Gps) {
	gpio_set_level(Gps->reset_pin,0);
	vTaskDelay(10/portTICK_PERIOD_MS);
	gpio_set_level(Gps->reset_pin,1);
}


int GPS_Send(GPS_t * Gps,const char * Command) {
	int ret,i;
	char const * n;
	uint8_t cs;
	char buffer[5], *ptr;

	i = strlen(Command);
	if (i>(NMEA_MAX_LENGTH-6)) {
		ESP_LOGE(TAG,"Command too long (%d)",i);
		return pdFALSE;
	}

	// calc chechsum
	cs = 0;
	n = Command;
	while (*n) {
		cs ^= (uint8_t)*n;
		n++;
	}

	ptr = buffer;
	*(ptr++)='*';
	*(ptr++)=HexTab[((cs>>4)&0x0F)];
	*(ptr++)=HexTab[(cs&0x0F)];
	*(ptr++)='\r';
	*(ptr++)='\n';

	if ((ret = uart_write_bytes(Gps->uart_num,"$",1))!=1) {
		ESP_LOGE(TAG,"write error sending start char (%d)",ret);
		return pdFALSE;
	}

	if ((ret=uart_write_bytes(Gps->uart_num,Command,i))!=i) {
		ESP_LOGE(TAG,"write error sending command (%d)",ret);
		return pdFALSE;
	}

	if ((ret=uart_write_bytes(Gps->uart_num,buffer,5))!=5) {
		ESP_LOGE(TAG,"write error sending checksum (%d)",ret);
		return pdFALSE;
	}

	return uart_wait_tx_done(Gps->uart_num, (((i+6)*10*1000)/Gps->baud)/portTICK_PERIOD_MS);
}

int GPS_QueueCommand(GPS_t * Gps,const char * Command) {
	if (xQueueSend(Gps->command_queue,Command,10000/portTICK_PERIOD_MS) != pdPASS) {
		ESP_LOGE(TAG,"Timeout while queueing command");
		return -1;
	}

	return 0;
}

