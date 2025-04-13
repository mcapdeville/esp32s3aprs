/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * ssd1680.c
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

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <string.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_random.h>
#include <lvgl.h>
#include "ssd1680.h"

#define TAG "SSD1680"

#define BITBANG_CS_PIN	1
#define CMD_BUFFER_SIZE	32
#define USE_CB 1

#define TASK_STACK_SIZE		2560
#define TASK_PRIORITY		3

#define SSD1680_WAIT_TO		30

struct SSD1680_S {
	lv_disp_drv_t lvgl_driver;	// mut be first
	lv_disp_draw_buf_t lvgl_buffer;
	uint8_t *buff1, *buff2;
	int pwr_cnt;
	bool dirty;
	spi_host_device_t spi_host;
	spi_device_handle_t spi_device;
	SSD1680_Config_t config;
	uint32_t plane_size;
	uint32_t screen_area, redraw_area;
	int power_pin,cs_pin,dc_pin,busy_pin;
	SemaphoreHandle_t busy_sem;
	StaticSemaphore_t busy_sem_buff;
	SemaphoreHandle_t power_sem;
	StaticSemaphore_t power_sem_buff;
	SemaphoreHandle_t refresh_sem;
	StaticSemaphore_t refresh_sem_buff;
	SemaphoreHandle_t flush_sem;
	StaticSemaphore_t flush_sem_buff;
	TaskHandle_t refresh_task;
	bool full;
	uint8_t power_cnt;
	uint8_t *planes[2];
	uint8_t *CmdBuff;
	struct SSD1680_Set_Ram_X_Pos_S xwin;
	struct SSD1680_Set_Ram_Y_Pos_S ywin;
	struct SSD1680_Data_Entry_Mode_S entry_mode;

};

static void IRAM_ATTR SSD1680_Busy_isr_handler(void * arg) {
	SSD1680_t * Epd = (SSD1680_t*)arg;
	BaseType_t MustYield = pdFALSE;

	gpio_intr_disable(Epd->busy_pin);
	xSemaphoreGiveFromISR(Epd->busy_sem,&MustYield);

	portYIELD_FROM_ISR(MustYield);
	return;
};

#if USE_CB == 1
union SSD1680_PreTrans_Data_S {
	void * user;
	struct {
		uint8_t cs_pin;
		uint8_t dc_pin;
		uint8_t dc_level;
	};
};

static void IRAM_ATTR SSD1680_PreTrans_cb(spi_transaction_t *trans) {
	union SSD1680_PreTrans_Data_S data;
	data.user = trans->user;

	gpio_set_level(data.dc_pin,data.dc_level);

#if BITBANG_CS_PIN == 1
	gpio_set_level(data.cs_pin,0);
#endif

}

static void IRAM_ATTR SSD1680_PostTrans_cb(spi_transaction_t *trans) {
	union SSD1680_PreTrans_Data_S data;
	data.user = trans->user;
	(void) data;

#if BITBANG_CS_PIN == 1
	if (!(trans->flags & SPI_TRANS_CS_KEEP_ACTIVE)) {
		gpio_set_level(data.cs_pin,1);
	}
#endif
}
#endif // USE_CB

static void SSD1680_Refresh_task(SSD1680_t *Epd);

SSD1680_t * SSD1680_Init(spi_host_device_t Host, int Power_Pin, int Cs_Pin, int Dc_Pin, int Busy_Pin, const SSD1680_Config_t * Config,unsigned char *logo) {

	SSD1680_t * epd;
	if (!(epd = heap_caps_malloc(sizeof(SSD1680_t),MALLOC_CAP_INTERNAL))) {
		ESP_LOGE(TAG,"Error allocating memory for SSD1680");
			return NULL;
	}
	bzero(epd,sizeof(SSD1680_t));
	
	epd->spi_host = Host;
	epd->power_pin = Power_Pin;
	epd->cs_pin = Cs_Pin;
	epd->dc_pin = Dc_Pin;
	epd->busy_pin = Busy_Pin;
	memcpy(&epd->config,Config,sizeof(SSD1680_Config_t));

	epd->busy_sem = xSemaphoreCreateBinaryStatic(&epd->busy_sem_buff);
	epd->power_sem = xSemaphoreCreateMutexStatic(&epd->power_sem_buff);
	epd->refresh_sem = xSemaphoreCreateBinaryStatic(&epd->refresh_sem_buff);
	epd->flush_sem = xSemaphoreCreateBinaryStatic(&epd->flush_sem_buff);
	xSemaphoreGive(epd->flush_sem);
	epd->power_cnt = 0;

	epd->plane_size = ((epd->config.xres+7)>>3)*epd->config.yres;

	if (!(epd->planes[0] = heap_caps_malloc(epd->plane_size,MALLOC_CAP_DMA))) {
		ESP_LOGE(TAG,"Error allocating dma buffer for BW plane");
		free(epd);
		return NULL;
	}

	if (!(epd->planes[1] = heap_caps_malloc(epd->plane_size,MALLOC_CAP_DMA))) {
		ESP_LOGE(TAG,"Error allocating dma buffer for Red plane");
		free(epd->planes[0]);
		free(epd);
		return NULL;
	}
	
	if (!(epd->CmdBuff = heap_caps_malloc(CMD_BUFFER_SIZE,MALLOC_CAP_DMA))) {
		ESP_LOGE(TAG,"Error allocating dma buffer for Cmd/Data");
		free(epd->planes[1]);
		free(epd->planes[0]);
		free(epd);
		return NULL;
	}

	if (!logo) {
		memset(epd->planes[0],0xff,epd->plane_size);
	} else {
		memcpy(epd->planes[0],logo,epd->plane_size);
	}
	memset(epd->planes[1],0x00,epd->plane_size);

	epd->screen_area = epd->config.xres* epd->config.yres;

	const gpio_config_t in_pin_config = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_INPUT,
#if BITBANG_CS_PIN == 1
		.pin_bit_mask = (1ULL<<Busy_Pin) | (1ULL<<Dc_Pin) | (1ULL<<Cs_Pin),
		.pull_down_en = (1ULL<<Busy_Pin) | (1ULL<<Dc_Pin) | (1ULL<<Cs_Pin),
#else
		.pin_bit_mask = (1ULL<<Busy_Pin) | (1ULL<<Dc_Pin),
		.pull_down_en = (1ULL<<Busy_Pin) | (1ULL<<Dc_Pin),
#endif
		.pull_up_en = 0,
	};

	gpio_config(&in_pin_config);
	gpio_intr_disable(Busy_Pin);
	gpio_set_intr_type(Busy_Pin,GPIO_INTR_LOW_LEVEL);

	const gpio_config_t out_pin_config = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1ULL<<Power_Pin),
		.pull_down_en = 0,
		.pull_up_en = 0,
	};
	gpio_config(&out_pin_config);
	gpio_set_level(Power_Pin,0);

	if (ESP_OK != gpio_isr_handler_add(Busy_Pin, SSD1680_Busy_isr_handler, (void*) epd)) {
		ESP_LOGE(TAG,"Error registering busy intr");
		spi_bus_remove_device(epd->spi_device);
		free(epd->CmdBuff);
		free(epd->planes[1]);
		free(epd->planes[0]);
		free(epd);
	}

	spi_device_interface_config_t SSD1680_dev_config;
        memset(&SSD1680_dev_config,0,sizeof(SSD1680_dev_config));
	SSD1680_dev_config.mode = 0;
	SSD1680_dev_config.clock_speed_hz = 10000000;

#if BITBANG_CS_PIN == 1
	SSD1680_dev_config.spics_io_num = -1;
#else
	SSD1680_dev_config.spics_io_num = epd->cs_pin;
#endif
	SSD1680_dev_config.flags = SPI_DEVICE_3WIRE;
	SSD1680_dev_config.queue_size = 4;
#if USE_CB == 1
	SSD1680_dev_config.pre_cb = SSD1680_PreTrans_cb;
#if BITBANG_CS_PIN == 1
	SSD1680_dev_config.post_cb = SSD1680_PostTrans_cb;
#endif
#endif
	SSD1680_dev_config.cs_ena_pretrans = 15;

	if (spi_bus_add_device(epd->spi_host,&SSD1680_dev_config,&epd->spi_device)!=ESP_OK) {
		ESP_LOGE(TAG,"Error adding device to bus");
		free(epd->CmdBuff);
		free(epd->planes[1]);
		free(epd->planes[0]);
		free(epd);
		return NULL;
	}

	// Alloc lvgl buffers
	if (!(epd->buff1 = malloc((Config->xres*Config->yres)))) {
		ESP_LOGE(TAG,"Error allocating buffer 1 of %d bytes",(Config->xres*Config->yres));
		spi_bus_remove_device(epd->spi_device);
		free(epd->CmdBuff);
		free(epd->planes[1]);
		free(epd->planes[0]);
		free(epd);
		return NULL;
	}

	if (!(epd->buff2 = malloc((Config->xres*Config->yres)))) {
		ESP_LOGE(TAG,"Error allocating buffer 2 of %d bytes",(Config->xres*Config->yres));
		free(epd->buff1);
		spi_bus_remove_device(epd->spi_device);
		free(epd->CmdBuff);
		free(epd->planes[1]);
		free(epd->planes[0]);
		free(epd);
		return NULL;
	}
	lv_disp_draw_buf_init(&epd->lvgl_buffer, epd->buff1, epd->buff2, (Config->xres*Config->yres));

	// Setup lvgl_driver struct
	lv_disp_drv_init(&epd->lvgl_driver);
	epd->lvgl_driver.hor_res = epd->config.xres;
	epd->lvgl_driver.ver_res = epd->config.yres;
	epd->lvgl_driver.dpi = epd->config.dpi; 
	epd->lvgl_driver.rotated = LV_DISP_ROT_NONE;
	epd->lvgl_driver.flush_cb = (void(*)(lv_disp_drv_t*,const lv_area_t*,lv_color_t*))SSD1680_Flush_cb;
	epd->lvgl_driver.wait_cb = (void(*)(lv_disp_drv_t*))SSD1680_Wait_cb;
	epd->lvgl_driver.user_data = epd;
	epd->lvgl_driver.draw_buf = &epd->lvgl_buffer;
	epd->lvgl_driver.drv_update_cb = (void(*)(lv_disp_drv_t*))SSD1680_Drv_Update_cb;
	epd->pwr_cnt = 0;

	SSD1680_Set_Orientation(epd,SSD1680_ORIENTATION_0);

	if (pdPASS != xTaskCreate((TaskFunction_t)SSD1680_Refresh_task,"SSD1680",TASK_STACK_SIZE,epd,TASK_PRIORITY,&epd->refresh_task)) {
		ESP_LOGE(TAG,"Error creating refresh task\n");
		free(epd->buff2);
		free(epd->buff1);
		spi_bus_remove_device(epd->spi_device);
		free(epd->CmdBuff);
		free(epd->planes[1]);
		free(epd->planes[0]);
		free(epd);
		return NULL;
	}

	return epd;
}

int SSD1680_Wait_Busy(SSD1680_t *Epd, int To) {

	int ret = ESP_OK;

	vTaskDelay(20/portTICK_PERIOD_MS);
	if (To < 20)
		To = 20;
	else To-= 20;

	xSemaphoreTake(Epd->busy_sem,0);
	gpio_intr_enable(Epd->busy_pin);
	ret = xSemaphoreTake(Epd->busy_sem,To/portTICK_PERIOD_MS);

	if (ret == pdFAIL)
		return ESP_ERR_TIMEOUT;

	return ESP_OK;
}

int SSD1680_Transaction(SSD1680_t *Epd, uint8_t Cmd, const void * SendBuff, size_t SendLen, void * RecvBuff, size_t RecvLen) {
	int ret;
	spi_transaction_t trans[3];
#if USE_CB == 1
	union SSD1680_PreTrans_Data_S data;
#endif

	memset(&trans,0,sizeof(trans));

	spi_device_acquire_bus(Epd->spi_device,portMAX_DELAY);
	
	if (Cmd) {
		trans[0].length = sizeof(Cmd)<<3;
		trans[0].tx_data[0] = Cmd;
		trans[0].flags = SPI_TRANS_USE_TXDATA | (((SendLen && SendBuff) || (RecvLen && RecvBuff))?SPI_TRANS_CS_KEEP_ACTIVE:0);
#if USE_CB == 1
		data.dc_pin = Epd->dc_pin;
		data.dc_level = 0;
		data.cs_pin = Epd->cs_pin;
		trans[0].user = data.user;
#else
		gpio_set_level(Epd->dc_pin,0);
#if BITBANG_CS_PIN == 1
		gpio_set_level(Epd->cs_pin,0);
#endif
#endif

		ESP_LOGV(TAG,"Cmd : 0x%02x",Cmd);
		fflush(stdout);

		ret = spi_device_transmit(Epd->spi_device,&trans[0]);
		if (ret != ESP_OK) {
#if BITBANG_CS_PIN == 1
			gpio_set_level(Epd->cs_pin,1);
#endif
			spi_device_release_bus(Epd->spi_device);
			ESP_LOGE(TAG,"Error Queueing command byte");
			return ret;
		}
	}

	if (SendLen && SendBuff) {
		if (SendBuff != Epd->CmdBuff && SendBuff != Epd->planes[0] && SendBuff != Epd->planes[1]) {
			if (SendLen > CMD_BUFFER_SIZE) {
				ESP_LOGW(TAG,"CmdBuff to small (%d), SendLen = %d",CMD_BUFFER_SIZE,SendLen);
				SendLen = CMD_BUFFER_SIZE;
			}
			memcpy(Epd->CmdBuff,SendBuff,SendLen);
			trans[1].tx_buffer = Epd->CmdBuff;
		}
		else
			trans[1].tx_buffer = SendBuff;

		trans[1].length = SendLen << 3;
		trans[1].flags = ((RecvLen && RecvBuff)?SPI_TRANS_CS_KEEP_ACTIVE:0);
#if USE_CB == 1
		data.dc_pin = Epd->dc_pin;
		data.dc_level = 1;
		data.cs_pin = Epd->cs_pin;
		trans[1].user = data.user;
#else
		gpio_set_level(Epd->dc_pin,1);
#if BITBANG_CS_PIN == 1
		gpio_set_level(Epd->cs_pin,0);
#endif
#endif

		ESP_LOGV(TAG,"Send : %d",SendLen);
		fflush(stdout);

		ret = spi_device_transmit(Epd->spi_device,&trans[1]);
		if (ret != ESP_OK) {
#if BITBANG_CS_PIN == 1
			gpio_set_level(Epd->cs_pin,1);
#endif
			spi_device_release_bus(Epd->spi_device);
			ESP_LOGE(TAG,"Error sendding data");
			return ret;
		}
	}

	if (RecvLen && RecvBuff) {
		if (RecvBuff != Epd->CmdBuff && RecvBuff != Epd->planes[0] && RecvBuff != Epd->planes[1]) {
			if (RecvLen > CMD_BUFFER_SIZE) {
				ESP_LOGW(TAG,"CmdBuff to small (%d), RecvLen = %d",CMD_BUFFER_SIZE,RecvLen);
				RecvLen = CMD_BUFFER_SIZE;
			}
			memcpy(Epd->CmdBuff,RecvBuff,RecvLen);
			trans[2].rx_buffer = Epd->CmdBuff;
		} else
			trans[2].rx_buffer = RecvBuff;
		trans[2].length = RecvLen << 3;
		trans[2].rxlength = RecvLen << 3;
		trans[2].flags = 0;
#if USE_CB == 1
		data.dc_pin = Epd->dc_pin;
		data.dc_level = 1;
		data.cs_pin = Epd->cs_pin;
		trans[2].user = data.user;
#else
		gpio_set_level(Epd->dc_pin,1);
#if BITBANG_CS_PIN == 1
		gpio_set_level(Epd->cs_pin,0);
#endif
#endif

		ESP_LOGV(TAG,"Recv : %d",RecvLen);
		fflush(stdout);
		
		ret = spi_device_transmit(Epd->spi_device,&trans[2]);
		if (ret != ESP_OK) {
#if BITBANG_CS_PIN == 1
			gpio_set_level(Epd->cs_pin,1);
#endif
			spi_device_release_bus(Epd->spi_device);
			ESP_LOGE(TAG,"Error receiving data");
			return ret;
		}

	if (RecvBuff != Epd->CmdBuff && RecvBuff != Epd->planes[0] && RecvBuff != Epd->planes[1])
			memcpy(RecvBuff,Epd->CmdBuff,RecvLen);
	}

#if (BITBANG_CS_PIN == 1)
		gpio_set_level(Epd->cs_pin,1);
#endif

	spi_device_release_bus(Epd->spi_device);

	return ESP_OK;
}

int SSD1680_Reset(SSD1680_t *Epd) {
	int ret;

	ret = SSD1680_Wait_Busy(Epd,100);
	if (ret == ESP_ERR_TIMEOUT)
		ESP_LOGE(TAG,"pre-Reset timeout.");

	ret = SSD1680_Transaction(Epd,SSD1680_CMD_SW_RESET,NULL,0,NULL,0);

	if (ret != ESP_OK)
		return ret;

	ret = SSD1680_Wait_Busy(Epd,100);
	if (ret == ESP_ERR_TIMEOUT)
		ESP_LOGE(TAG,"Reset timeout.");

	return ret;
}

int SSD1680_Write_Temperature(SSD1680_t * Epd, struct SSD1680_Temp_Register_S * Temp) {
	return SSD1680_Transaction(Epd,SSD1680_CMD_TEMP_REGISTER_WRITE,Temp,sizeof(struct SSD1680_Temp_Register_S),NULL,0);
}

int SSD1680_Read_Temperature(SSD1680_t * Epd, struct SSD1680_Temp_Register_S * Temp) {
	return SSD1680_Transaction(Epd,SSD1680_CMD_TEMP_REGISTER_READ,NULL,0,Temp,sizeof(struct SSD1680_Temp_Register_S));
}

int SSD1680_OTP_Read_Display_Option(SSD1680_t * Epd,struct SSD1680_OTP_Display_Option_S *DisplayOption) {
	return SSD1680_Transaction(Epd,SSD1680_CMD_OTP_READ_DISPLAY_OPTION,NULL,0,DisplayOption,sizeof(struct SSD1680_OTP_Display_Option_S));
}

int SSD1680_Setup(SSD1680_t *Epd) {

	int ret;

	ret = SSD1680_Transaction(Epd,SSD1680_CMD_DRIVER_OUTPUT_CTRL,
			&Epd->config.driver_output_control,sizeof(struct SSD1680_Driver_Output_Ctrl_S),
			NULL,0);

	if (ret != ESP_OK)
		return ret;

	ret = SSD1680_Transaction(Epd,SSD1680_CMD_DATA_ENTRY_MODE,
			&Epd->entry_mode,sizeof(struct SSD1680_Data_Entry_Mode_S),
			NULL,0);

	if (ret != ESP_OK)
		return ret;

	ret = SSD1680_Transaction(Epd,SSD1680_CMD_BORDER_WAVEFORM,
			&Epd->config.border_waveform,sizeof(struct SSD1680_Border_Waveform_S),
			NULL,0);

	if (ret != ESP_OK)
		return ret;

	ret = SSD1680_Transaction(Epd,SSD1680_CMD_LUT_END_OPTION,
			&Epd->config.lut_end,sizeof(struct SSD1680_Lut_End_Option_S),
			NULL,0);

	if (ret != ESP_OK)
		return ret;

	ret = SSD1680_Transaction(Epd,SSD1680_CMD_TEMP_SENSOR_CTRL,
			&Epd->config.temp_sensor,sizeof(struct SSD1680_Temp_Sensor_S),
			NULL,0);

	if (ret != ESP_OK)
		return ret;

	return ret;
}

int SSD1680_Set_Windows(SSD1680_t * Epd) {
	int ret;

	ESP_LOGV(TAG,"x(%d,%d) y(%d,%d)",Epd->xwin.start,Epd->xwin.end,Epd->ywin.start,Epd->ywin.end);

	ret = SSD1680_Transaction(Epd,SSD1680_CMD_SET_RAM_X_POS,
			&Epd->xwin,sizeof(struct SSD1680_Set_Ram_X_Pos_S),
			NULL,0);

	if (ret != ESP_OK)
		return ret;

	ret = SSD1680_Transaction(Epd,SSD1680_CMD_SET_RAM_Y_POS,
			&Epd->ywin,sizeof(struct SSD1680_Set_Ram_Y_Pos_S),
			NULL,0);

	if (ret != ESP_OK)
		return ret;

	return ESP_OK;
}

int SSD1680_Write_Image(SSD1680_t * Epd) {
	int ret;
	struct SSD1680_Set_Ram_X_Counter_S ram_x_counter;
	struct SSD1680_Set_Ram_Y_Counter_S ram_y_counter;

	SSD1680_Set_Windows(Epd);

	ram_x_counter.counter = Epd->xwin.start;
	ret = SSD1680_Transaction(Epd,SSD1680_CMD_SET_RAM_X_COUNTER,
			&ram_x_counter,sizeof(struct SSD1680_Set_Ram_X_Counter_S),
			NULL,0);
	
	if (ret != ESP_OK)
		return ret;

	ram_y_counter.counter = Epd->ywin.start;
	ret = SSD1680_Transaction(Epd,SSD1680_CMD_SET_RAM_Y_COUNTER,
			&ram_y_counter,sizeof(struct SSD1680_Set_Ram_Y_Counter_S),
			NULL,0);

	if (ret != ESP_OK)
		return ret;

	/* Write Black/White image data */
	ret = SSD1680_Transaction(Epd,SSD1680_CMD_WRITE_BW_RAM,
			Epd->planes[0],((Epd->config.xres+7)>>3)*Epd->config.yres,
			NULL,0);

	if (ret != ESP_OK)
		return ret;

	/* Write/Red image data */
	ret = SSD1680_Transaction(Epd,SSD1680_CMD_WRITE_RED_RAM,
			Epd->planes[1],((Epd->config.xres+7)>>3)*Epd->config.yres,
			NULL,0);

	return ret;
}

int SSD1680_Update(SSD1680_t *Epd) {
	int ret;

	if (Epd->config.load_sequence.sequence & ~0x08) {
		ret = SSD1680_Transaction(Epd,SSD1680_CMD_DISPLAY_UPDATE_CTRL_2,
				&Epd->config.load_sequence,sizeof(struct SSD1680_Display_Update_Sequence_S),
				NULL,0);
		if (ret != ESP_OK)
			return ret;

		ret = SSD1680_Transaction(Epd,SSD1680_CMD_MASTER_ACTIVATION,NULL,0,NULL,0);
		if (ret != ESP_OK)
			return ret;

		ret = SSD1680_Wait_Busy(Epd,1000);
		if (ret == ESP_ERR_TIMEOUT)
			ESP_LOGE(TAG,"Load sequence timeout.");
	}

	if ((*(uint32_t*)&Epd->config.soft_start)) {
		ret = SSD1680_Transaction(Epd,SSD1680_CMD_BOOST_SS_CTRL,
				&Epd->config.soft_start,sizeof(struct SSD1680_Boost_SS_Ctrl_S),
				NULL,0);
		if (ret != ESP_OK)
			return ret;
	}

	ret = SSD1680_Transaction(Epd,SSD1680_CMD_DISPLAY_UPDATE_CTRL_2,
			&Epd->config.update_sequence,sizeof(struct SSD1680_Display_Update_Sequence_S),
			NULL,0);

	if (ret != ESP_OK)
		return ret;

	ret = SSD1680_Transaction(Epd,SSD1680_CMD_MASTER_ACTIVATION,NULL,0,NULL,0);

	if (ret != ESP_OK)
		return ret;

	ret = SSD1680_Wait_Busy(Epd,7000);
	if (ret == ESP_ERR_TIMEOUT)
		ESP_LOGE(TAG,"Update timeout.");

	return ret;
}

int SSD1680_Deep_Sleep(SSD1680_t * Epd) {
	int ret;

	struct SSD1680_Deep_Sleep_S sleep = {
		.mode = SSD1680_DEEP_SLEEP_MODE_1
	};

	ret = SSD1680_Transaction(Epd,SSD1680_CMD_DEEP_SLEEP,
			&sleep,sizeof(struct SSD1680_Deep_Sleep_S),
			NULL,0);

	SSD1680_Wait_Busy(Epd,100);
	if (ret == ESP_ERR_TIMEOUT)
		ESP_LOGE(TAG,"Sleep timeout.");

	return ret;
}

void SSD1680_PowerUp(SSD1680_t * Epd) {
	uint8_t cnt;

	if (!Epd)
		return ;

	xSemaphoreTake(Epd->power_sem,portMAX_DELAY);
	cnt = Epd->power_cnt++;

	if (!cnt) {
		ESP_LOGD(TAG,"Power Up");
		gpio_set_level(Epd->dc_pin,0);
		gpio_set_direction(Epd->dc_pin,GPIO_MODE_OUTPUT);
#if BITBANG_CS_PIN == 1
		gpio_set_level(Epd->cs_pin,1);
		gpio_set_direction(Epd->cs_pin,GPIO_MODE_OUTPUT);
#endif
		gpio_set_direction(Epd->busy_pin,GPIO_MODE_INPUT);
		gpio_set_pull_mode(Epd->busy_pin,GPIO_PULLUP_ONLY);
		gpio_set_level(Epd->power_pin,1);
	}

	xSemaphoreGive(Epd->power_sem);
}

void SSD1680_PowerDown(SSD1680_t * Epd) {
	uint8_t cnt;

	xSemaphoreTake(Epd->power_sem,portMAX_DELAY);
	if (Epd->power_cnt) {
		cnt = --Epd->power_cnt;

		if (!cnt) {
			gpio_set_level(Epd->dc_pin,0);
			gpio_set_direction(Epd->dc_pin,GPIO_MODE_OUTPUT);
			gpio_set_level(Epd->power_pin,0);
#if BITBANG_CS_PIN == 1
			gpio_set_level(Epd->cs_pin,0);
			gpio_set_direction(Epd->cs_pin,GPIO_MODE_OUTPUT);
#endif
			gpio_set_level(Epd->busy_pin,0);
			gpio_set_direction(Epd->busy_pin,GPIO_MODE_OUTPUT);
			ESP_LOGD(TAG,"Power Down");
		}
	}
	xSemaphoreGive(Epd->power_sem);
}

int SSD1680_Redraw(SSD1680_t * Epd) {
	int ret;

	if (Epd->full) {	
		Epd->config.load_sequence.sequence &= ~0x08;
		Epd->config.update_sequence.sequence &= ~0x08;
	}
	else {
		Epd->config.load_sequence.sequence |= 0x08;
		Epd->config.update_sequence.sequence |= 0x08;
	}

	if (!Epd->dirty)
		return ESP_OK;

	ret = SSD1680_Setup(Epd);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Error configuring EPD");
		return ret;
	}

	ret = SSD1680_Write_Image(Epd);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Error writing image to  EPD");
		return ret;
	}

	ret = SSD1680_Update(Epd);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Error updating EPD");
		return ret;
	}

	Epd->dirty = false;

	return ret;

}

int SSD1680_Power_Redraw(SSD1680_t *Epd,bool Full) {
	int ret=0;

	Epd->full = Full;

	if (Epd->dirty) {
		SSD1680_PowerUp(Epd);

		ret = SSD1680_Reset(Epd);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG,"Error resetting EPD");
			SSD1680_PowerDown(Epd);
			return ret;
		}

		ret = SSD1680_Redraw(Epd);

		SSD1680_Deep_Sleep(Epd);
		SSD1680_PowerDown(Epd);
	}

	return ret;
}

void SSD1680_Fill_Random(SSD1680_t * Epd) {
	esp_fill_random(Epd->planes[0],((Epd->config.xres+7)>>3)*Epd->config.yres);
	esp_fill_random(Epd->planes[1],((Epd->config.xres+7)>>3)*Epd->config.yres);
	Epd->dirty = true;
}

void SSD1680_Fill(SSD1680_t * Epd,uint8_t c) {
	memset(Epd->planes[0],((int8_t)((c&1)<<7))>>7,((Epd->config.xres+7)>>3)*Epd->config.yres);
	memset(Epd->planes[1],((int8_t)((c&2)<<6))>>7,((Epd->config.xres+7)>>3)*Epd->config.yres);
	Epd->dirty = true;
}

void SSD1680_Fill_Bitmap(SSD1680_t * Epd,uint8_t * plane_1,uint8_t * plane_2) {
	if (plane_1) {
		memcpy(Epd->planes[0],plane_1,((Epd->config.xres+7)>>3)*Epd->config.yres);
		Epd->dirty = true;
	}

	if (plane_2) {
		memcpy(Epd->planes[1],plane_2,((Epd->config.xres+7)>>3)*Epd->config.yres);
		Epd->dirty = true;
	}
}

void SSD1680_Set_Orientation(SSD1680_t * Epd,enum SSD1680_Orientation_E Orient) {

	Epd->dirty = true;
	Epd->full = true;

	switch (Orient) {
		case SSD1680_ORIENTATION_0:
			Epd->entry_mode.x = SSD1680_ENTRY_MODE_INC;
			Epd->entry_mode.y = SSD1680_ENTRY_MODE_INC;
			Epd->entry_mode.dir = SSD1680_ENTRY_MODE_DIR_X;
			Epd->xwin.start = 0;
			Epd->xwin.end = ((Epd->config.xres+7)>>3)-1;
			Epd->ywin.start = 0;
			Epd->ywin.end = Epd->config.yres-1;
			Epd->lvgl_driver.rotated = LV_DISP_ROT_NONE;
			break;
		case SSD1680_ORIENTATION_270:
			Epd->entry_mode.x = SSD1680_ENTRY_MODE_DEC;
			Epd->entry_mode.y = SSD1680_ENTRY_MODE_INC;
			Epd->entry_mode.dir = SSD1680_ENTRY_MODE_DIR_Y;
			Epd->xwin.start = ((Epd->config.xres+7)>>3)-1;
			Epd->xwin.end = 0;
			Epd->ywin.start = 0;
			Epd->ywin.end = Epd->config.yres-1;
			Epd->lvgl_driver.rotated = LV_DISP_ROT_270;
			break;
		case SSD1680_ORIENTATION_180:
			Epd->entry_mode.x = SSD1680_ENTRY_MODE_DEC;
			Epd->entry_mode.y = SSD1680_ENTRY_MODE_DEC;
			Epd->entry_mode.dir = SSD1680_ENTRY_MODE_DIR_X;
			Epd->xwin.start = ((Epd->config.xres+7)>>3)-1;
			Epd->xwin.end = 0;
			Epd->ywin.start = Epd->config.yres-1;
			Epd->ywin.end = 0;
			Epd->lvgl_driver.rotated = LV_DISP_ROT_180;
			break;
		case SSD1680_ORIENTATION_90:
			Epd->entry_mode.x = SSD1680_ENTRY_MODE_INC;
			Epd->entry_mode.y = SSD1680_ENTRY_MODE_DEC;
			Epd->entry_mode.dir = SSD1680_ENTRY_MODE_DIR_Y;
			Epd->xwin.start= 0;
			Epd->xwin.end = ((Epd->config.xres+7)>>3)-1;
			Epd->ywin.start = Epd->config.yres-1;
			Epd->ywin.end = 0;
			Epd->lvgl_driver.rotated = LV_DISP_ROT_90;
			break;
	}
	return;
}

void SSD1680_Set_Border(SSD1680_t * Epd,uint8_t Lut) {
	Epd->config.border_waveform.transition = Lut;
}

bool SSD1680_Is_Dirty(SSD1680_t * Epd) {
	return Epd->dirty;
}

static void SSD1680_Refresh_task(SSD1680_t *Epd) {
	int ret;
	bool logo = true;

	Epd->full = true;
	Epd->dirty = true;

	while (1) {
		if (Epd->dirty) {
			if (Epd->redraw_area >= Epd->screen_area) {
				Epd->full = true;
				ESP_LOGD(TAG,"Full refresh (%ld)",Epd->redraw_area);
			}

			SSD1680_PowerUp(Epd);

			ret = SSD1680_Reset(Epd);
			if (ret == ESP_OK) {
				ret = SSD1680_Redraw(Epd);
				if (!ret)
					Epd->dirty = false;
				else
					ESP_LOGE(TAG,"Error refreshing screen");
			} else
				ESP_LOGE(TAG,"Error resetting EPD");

			SSD1680_Deep_Sleep(Epd);
			SSD1680_PowerDown(Epd);

			if (logo) {
				logo = false;
				Epd->full = true;
			} else if (Epd->full && !Epd->dirty) {
				Epd->redraw_area = 0;
				Epd->full = false;
			}
		}

		xSemaphoreGive(Epd->flush_sem); // signal end of refresh

		vTaskDelay(100/portTICK_PERIOD_MS);	// 100 delay after power down
		xSemaphoreTake(Epd->refresh_sem,portMAX_DELAY); // Wait for refresh signal
	}
}

// LVGL glue func
void SSD1680_Flush_cb(SSD1680_t * Epd, lv_area_t *Area, uint8_t *Colormap) {
	int x,y,b,xc,yc;
	uint8_t *data1,*data2;
	uint8_t *start_ptr = Colormap;

	ESP_LOGD(TAG,"Flush area x : %d-%d y : %d-%d",Area->x1,Area->x2,Area->y1,Area->y2);
	for (y=Area->y1;y<=Area->y2;y++) {
		for (x=Area->x1;x<=Area->x2;x++) {
			if (Epd->entry_mode.dir) {
				if (!Epd->entry_mode.x)
					yc = y + (7-((Epd->config.xres-1)&3));
				else 
					yc = y;
				data1 = &Epd->planes[0][x+(yc>>3)*((Epd->config.yres))];
				data2 = &Epd->planes[1][x+(yc>>3)*((Epd->config.yres))];
				b = yc&7;
			} else {
				if (!Epd->entry_mode.x)
					xc = x + (7-((Epd->config.xres-1)&3));
				else 
					xc = x;
				data1 = &Epd->planes[0][((xc>>3)+(y*((Epd->config.xres+7)>>3)))];
				data2 = &Epd->planes[1][((xc>>3)+(y*((Epd->config.xres+7)>>3)))];
				b = xc&7;
			}
			*data1 = (*Colormap&1)?(*data1 | 0x80>>b):(*data1 & ~(0x80>>b));
			*data2 = (*Colormap&2)?(*data2 | 0x80>>b):(*data2 & ~(0x80>>b));

			Colormap++;
		}
	}

	Epd->dirty = true;

	Epd->redraw_area += (Colormap-start_ptr);
	if (lv_disp_flush_is_last(&Epd->lvgl_driver)) {
		// Sync screen
		xSemaphoreGive(Epd->refresh_sem); // Start refresh
		if (!Epd->lvgl_driver.wait_cb && xSemaphoreTake(Epd->flush_sem,portMAX_DELAY) == pdPASS) // Wait end or refresh
			lv_disp_flush_ready(&Epd->lvgl_driver);	// signal end refresh
	} else
		lv_disp_flush_ready(&Epd->lvgl_driver);	// signal end refresh
}

void SSD1680_Wait_cb(SSD1680_t * Epd) {
	if (xSemaphoreTake(Epd->flush_sem,SSD1680_WAIT_TO/portTICK_PERIOD_MS) == pdPASS) // wait end of refresh
		lv_disp_flush_ready(&Epd->lvgl_driver);	// signal end refresh
}

void SSD1680_Drv_Update_cb(SSD1680_t * Epd) {
	SSD1680_Set_Orientation(Epd,Epd->lvgl_driver.rotated);
}
