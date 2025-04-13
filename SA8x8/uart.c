/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * uart.c
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
#include <errno.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/uart.h>
#include "SA8x8_priv.h"

#define UART_QUEUE_SIZE 2
#define SA8X8_UART_REPLY_TIMEOUT 500


/* UART Initialisation */
int SA8x8_Uart_Init(SA8x8_t * SA8x8, int uart_num, int Baud, int tx_pin,int rx_pin) {
	const	uart_config_t uart_config = {
		.baud_rate = Baud,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_XTAL,
	};

	SA8x8->uart_num = -1;

#if CONFIG_UART_ISR_IN_IRAM	
	if (uart_driver_install(uart_num,SOC_UART_FIFO_LEN*2,SOC_UART_FIFO_LEN*2,0,NULL,ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_SHARED) != ESP_OK) {
		ESP_LOGE(TAG,"Error installing uart driver");
		return -1;
	}
#else
#error "Your must set CONFIG_UART_ISR_IN_IRAM=y in sdkconfig"
#endif

	if (uart_param_config(uart_num,&uart_config) != ESP_OK) {
		ESP_LOGE(TAG,"Invalid uart parameters");
		uart_driver_delete(uart_num);
		return -1;
	}

	if (uart_set_pin(uart_num,tx_pin,rx_pin,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE) != ESP_OK) {
		ESP_LOGE(TAG,"Invalid uart gpio pins");
		uart_driver_delete(uart_num);
		return -1;
	}

	SA8x8->uart_num = uart_num;

	SA8x8->uart_sem = xSemaphoreCreateMutexStatic(&SA8x8->uart_sem_buff);

	return 0;
}

void SA8x8_Uart_Deinit(SA8x8_t * SA8x8) {
	uart_driver_delete(SA8x8->uart_num);
	SA8x8->uart_num = -1;
}

int SA8x8_Uart_Command(SA8x8_t * SA8x8,const char * Command,const char * Reply,int Len ,char * Data) {
	int ret = 1, i,pos;
	TimeOut_t to;
	TickType_t tick;

	ESP_LOGV(TAG,"%s : Command = %s",__func__,Command);

	i = strlen(Command);
	if (i>64) {
		ESP_LOGE(TAG,"Command too long!");
		return 1;
	}

	uart_flush(SA8x8->uart_num);

	if ((ret=uart_write_bytes(SA8x8->uart_num,Command,i))!=i) {
		ESP_LOGE(TAG,"write error sending command (%d)",ret);
		goto err;
	}

	SA8x8->uart_buff[0] = '\r';
	SA8x8->uart_buff[1] = '\n';
	if ((ret=uart_write_bytes(SA8x8->uart_num,SA8x8->uart_buff,2))!=2) {
		ESP_LOGE(TAG,"write error sending cr/lf");
		goto err;
	}
	uart_wait_tx_done(SA8x8->uart_num, 10/portTICK_PERIOD_MS);

	vTaskSetTimeOutState(&to);
	tick = SA8X8_UART_REPLY_TIMEOUT/portTICK_PERIOD_MS;
	pos=0;
	i=0;
	do {
		if (xTaskCheckForTimeOut(&to,&tick))
				break;

		ret = uart_read_bytes(SA8x8->uart_num, &SA8x8->uart_buff[pos], (sizeof(SA8x8->uart_buff)-1)-pos, 100 / portTICK_PERIOD_MS);
		if (ret<0) {
			SA8x8->uart_buff[pos] = '\0';
			ESP_LOGE(TAG,"read error waiting reply (%d-%d) : %s",ret,errno,SA8x8->uart_buff);
			goto err;
		}
		
		pos += ret;
		i++;
	} while (SA8x8->uart_buff[pos-1]!='\n' && SA8x8->uart_buff[pos-2]!='\r' && (pos < (sizeof(SA8x8->uart_buff)-1)));

	SA8x8->uart_buff[pos]='\0';

	if (pos>=2 && (SA8x8->uart_buff[pos-1]!='\n' || SA8x8->uart_buff[pos-2]!='\r')) {
		ESP_LOGW(TAG,"Incomplete reply (pos=%d, i=%d): %s", pos, i, SA8x8->uart_buff);
		goto err;
	}

	if (!Reply)
		goto end;

	i = strlen(Reply);
	if (i>(sizeof(SA8x8->uart_buff)-1)) {
		ESP_LOGE(TAG,"Reply too long");
		goto err;
	}

	if (strncmp(Reply,SA8x8->uart_buff,i)) {
		ESP_LOGE(TAG,"Incorrect reply to command %s:(pos = %d) %s ",Command,pos,SA8x8->uart_buff);
		goto err;
	}
	ESP_LOGV(TAG,"Reply to command %s : %s ",Command,SA8x8->uart_buff);

	if (!Len || !Data)
		goto end;

	Len--;
	pos-=(i+2);
	Len = Len<pos?Len:pos;
	strncpy(Data,SA8x8->uart_buff+i,Len);
	Data[Len]='\0';

end:
	ret = 0;
err:
	return ret;
}

int SA8x8_Handcheck(SA8x8_t * SA8x8) {
	int ret;
	
	xSemaphoreTake(SA8x8->uart_sem, portMAX_DELAY);
	ret = SA8x8_Uart_Command(SA8x8,"AT+DMOCONNECT","+DMOCONNECT:0",0,NULL);
	xSemaphoreGive(SA8x8->uart_sem);

	return ret;
}

int SA8x8_Uart_SetGroup(SA8x8_t * SA8x8,SA8x8_Group_t * Param) {
	int ret;
	snprintf(SA8x8->uart_command, sizeof(SA8x8->uart_command),"AT+DMOSETGROUP=%1d,%03.4f,%03.4f,%4s,%1d,%4s",
			Param->bw,
			Param->tx_freq/1000000.0f,
			Param->rx_freq/1000000.0f,
			Param->tx_sub,
			Param->squelch,
			Param->rx_sub);

	xSemaphoreTake(SA8x8->uart_sem, portMAX_DELAY);
	ret = SA8x8_Uart_Command(SA8x8,SA8x8->uart_command,"+DMOSETGROUP:0",0,NULL);
	xSemaphoreGive(SA8x8->uart_sem);

	return ret;
}

int SA8x8_Uart_SetVolume(SA8x8_t * SA8x8, uint8_t Vol) {
	int ret;

	snprintf(SA8x8->uart_command, sizeof(SA8x8->uart_command),"AT+DMOSETVOLUME=%1d",
			Vol);

	xSemaphoreTake(SA8x8->uart_sem, portMAX_DELAY);
	ret = SA8x8_Uart_Command(SA8x8,SA8x8->uart_command,"+DMOSETVOLUME:0",0,NULL);
	xSemaphoreGive(SA8x8->uart_sem);

	return ret;
}


int SA8x8_Uart_SetFilter(SA8x8_t * SA8x8, bool Emph, bool HighP, bool LowP) {
	int ret;

	snprintf(SA8x8->uart_command,sizeof(SA8x8->uart_command),"AT+SETFILTER=%1d,%1d,%1d",
			!Emph,HighP,LowP);

	xSemaphoreTake(SA8x8->uart_sem, portMAX_DELAY);
	ret = SA8x8_Uart_Command(SA8x8,SA8x8->uart_command,"+DMOSETFILTER:0",0,NULL);
	xSemaphoreGive(SA8x8->uart_sem);

	return ret;
}

int SA8x8_Uart_SetTail(SA8x8_t * SA8x8, uint8_t Tail) {
	int ret;

	snprintf(SA8x8->uart_command,sizeof(SA8x8->uart_command),"AT+SETTAIL=%1d",
				Tail);
	xSemaphoreTake(SA8x8->uart_sem, portMAX_DELAY);
	ret = SA8x8_Uart_Command(SA8x8,SA8x8->uart_command,"+DMOSETTAIL:0",0,NULL);
	xSemaphoreGive(SA8x8->uart_sem);

	return ret;
}

int SA8x8_GetVersion(SA8x8_t * SA8x8,char * Verstr,int Strlen) {
	int ret;

	xSemaphoreTake(SA8x8->uart_sem, portMAX_DELAY);
	ret = SA8x8_Uart_Command(SA8x8,"AT+VERSION:","+VERSION:",Strlen,Verstr);
	xSemaphoreGive(SA8x8->uart_sem);

	return ret;
}

int SA8x8_GetRssi(SA8x8_t * SA8x8,uint8_t * rssi) {
	int ret;

	xSemaphoreTake(SA8x8->uart_sem, portMAX_DELAY);
	ret = SA8x8_Uart_Command(SA8x8,"AT+RSSI?:","RSSI=",sizeof(SA8x8->uart_command),SA8x8->uart_command);
	xSemaphoreGive(SA8x8->uart_sem);

	if (ret)
		return 1;

	*rssi = atoi(SA8x8->uart_command);

	return 0;
}

int SA8x8_Scan(SA8x8_t *SA8x8,uint32_t Freq) {
	int ret;

	snprintf(SA8x8->uart_command,sizeof(SA8x8->uart_command),"S+%03.4f",
			Freq/1000000.0);

	xSemaphoreTake(SA8x8->uart_sem, portMAX_DELAY);
	ret = SA8x8_Uart_Command(SA8x8,SA8x8->uart_command,"S=0",0,NULL);
	xSemaphoreGive(SA8x8->uart_sem);

	return ret;
}
