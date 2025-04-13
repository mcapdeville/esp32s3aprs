/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/main.c
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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <esp_log.h>
#include <esp_intr_alloc.h>
#include <esp_pm.h>
#include <esp_heap_caps.h>
#include "esp_vfs_eventfd.h"
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_sleep.h>
#include <esp_spiffs.h>
#include <driver/uart.h>
#include <driver/uart_vfs.h>
#include <soc/rtc.h>
#include <soc/rtc_cntl_reg.h>
#include <esp_private/rtc_ctrl.h>
#include "hal/brownout_hal.h"
#include "hal/brownout_ll.h"


#include <ssd1680.h>
#include "xbm/logo.xbm"
#include <lvgl.h>
#include <SA8x8.h>
#include <nvs_flash.h>
#include <nvs.h>
#include "usb.h"
#include "adc.h"
#include "hmi.h"
#include "gps.h"
#include "afsk_config.h"
#include "modem.h"
#include "modem_afsk1200.h"
#include "kiss.h"
#include "aprs.h"
#include "ax25_phy.h"
#include "ax25_phy_simplex.h"
#include "ax25_lm.h"
#include "framebuff.h"
// #include <lowpower.h>
#include "micropython.h"
#include "usb_cdc.h"

#include "config.h"

#define TAG "MAIN"

ESP_EVENT_DEFINE_BASE(MAIN_EVENT);
#define MAIN_EVENT_RSSI	0
#define MAIN_EVENT_ADC2	1

#define XBM_SWAP_BITS(bitmap) do {\
	for (int i = 0;i<sizeof(bitmap);i++) {\
		uint8_t c;\
		c = bitmap[i];\
		c = ((c&0xf0)>>4) | ((c&0x0f)<<4);\
		c = ((c&0xcc)>>2) | ((c&0x33)<<2);\
		c = ((c&0xaa)>>1) | ((c&0x55)<<1);\
		bitmap[i] = c;\
	}} while (0)

// UI test frame : "F4JMZ>APX218,RELAY:=4405.2 N/00349.4 E-PHG0804"
// Fcs : 0x38 0x12
static struct {
	Frame_t frame;
	uint8_t data[52];
} TestFrame = {

	.frame.parent = NULL,
	.frame.frame_size = 52,
	.frame.frame_len = 52,
	.data = { 
		0x82,0xa0,0xb0,0x64,0x62,0x40,0xe0,0x8c,0x68,0x94,0x9a,0xb4,0x40,0xe0,0xa4,0x8a,
		0x98,0x82,0xb2,0x40,0x61,0x03,0xf0,0x3d,0x34,0x34,0x30,0x35,0x2e,0x32,0x20,0x4e,
		0x2f,0x30,0x30,0x33,0x34,0x39,0x2e,0x34,0x20,0x45,0x2d,0x50,0x48,0x47,0x30,0x38,
		0x30,0x34,0x38,0x12
	}
};

SSD1680_t * Epd;
HMI_t * Hmi;
GPS_t * Gps;
nvs_handle_t Nvs;
SA8x8_t * SA8x8;
Modem_t * Modem_AFSK1200;
Kiss_t *Kiss;
APRS_t * Aprs;
AX25_Phy_t * Ax25_Phy;
AX25_Lm_t * Ax25_Lm;
uint8_t Rssi;
uint8_t Rssi_max;
int Battery;


/*
IRAM_ATTR void brownout_detect_isr(void*) {

	// Clear interrupt
	REG_WRITE(RTC_CNTL_INT_CLR_REG, RTC_CNTL_BROWN_OUT_INT_CLR);

	if (REG_READ(RTC_CNTL_BROWN_OUT_REG) & RTC_CNTL_BROWN_OUT_DET) {
		if (Nvs)
			nvs_commit(Nvs);

		REG_WRITE(RTC_CNTL_BROWN_OUT_REG,
				(0x3ffL << RTC_CNTL_BROWN_OUT_INT_WAIT_S)
				|RTC_CNTL_BROWN_OUT_CLOSE_FLASH_ENA
				| RTC_CNTL_BROWN_OUT_PD_RF_ENA
				| (0x1ffL << RTC_CNTL_BROWN_OUT_RST_WAIT_S)
				| RTC_CNTL_BROWN_OUT_RST_ENA
				//			| RTC_CNTL_BROWN_OUT_RST_SEL
				//			| RTC_CNTL_BROWN_OUT_CNT_CLR
				| RTC_CNTL_BROWN_OUT_ENA);
		REG_WRITE( RTC_CNTL_FIB_SEL_REG, ((CONFIG_BROWNOUT_DET_LVL) & 7));
	}	
	
}
*/

static int Adc2_Voltage[5];
# if CONFIG_FREERTOS_USE_TRACE_FACILITY
static TaskStatus_t Task_status[20];
static unsigned long Runtime;
#endif

void app_main(void)
{
	int i,ret;
	uint8_t rssi,max_rssi;
	TickType_t now;
	int64_t ready_time, start_time;

	start_time = esp_timer_get_time();
/*
	// lowpower_start()
	// brownout init
    REG_WRITE(RTC_CNTL_BROWN_OUT_REG,
			(15L << RTC_CNTL_BROWN_OUT_INT_WAIT_S)
			//		| RTC_CNTL_BROWN_OUT_CLOSE_FLASH_ENA
			//		| RTC_CNTL_BROWN_OUT_PD_RF_ENA 
			| (0x3ffL << RTC_CNTL_BROWN_OUT_RST_WAIT_S)
			//			| RTC_CNTL_BROWN_OUT_RST_ENA
			//			| RTC_CNTL_BROWN_OUT_RST_SEL
			//			| RTC_CNTL_BROWN_OUT_CNT_CLR
			| RTC_CNTL_BROWN_OUT_ENA);

	REG_WRITE( RTC_CNTL_FIB_SEL_REG, ((BROWNOUT_DET_LVL) & 7));

    ESP_ERROR_CHECK( rtc_isr_register(brownout_detect_isr, NULL, RTC_CNTL_BROWN_OUT_INT_ENA_M, RTC_INTR_FLAG_IRAM) );

    REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_BROWN_OUT_INT_ENA_M);
*/

	// GPIO Init
	gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_SHARED);

	// SA8x8 Radio
	SA8x8 = SA8x8_Init(&SA8x8_config);

	// AFSK1200 Modem
	Modem_AFSK1200 = Modem_AFSK1200_Init(SA8x8, &AFSK_Config);

	SA8x8_Set_Power(SA8x8, SA8X8_POWER_LOW);

	// AX25 stack
	Ax25_Phy = AX25_Phy_Simplex_Init(Modem_AFSK1200);
	Ax25_Lm = AX25_Lm_Init(Ax25_Phy);
	
	// System Event loop init
	esp_event_loop_create_default();

	// Aprs protocol ont top of AX25 link multiplexer
	Aprs = APRS_Init(Ax25_Lm);

	ready_time = esp_timer_get_time();

	// Setup UART Console
	if (!uart_is_driver_installed(CONFIG_ESP_CONSOLE_UART_NUM)) {
		ESP_LOGI(TAG,"Initializing uart0");
		setvbuf(stdin, NULL, _IONBF, 0);
		if (uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, SOC_UART_FIFO_LEN*4, SOC_UART_FIFO_LEN*4, 0, NULL,
					ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_SHARED) != ESP_OK) {
			ESP_LOGE(TAG, "Error initializing uart0 driver");
		} else {
			if (uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM,&console_config) != ESP_OK) {
				uart_driver_delete(CONFIG_ESP_CONSOLE_UART_NUM);
			}/* else {
				if (uart_set_pin(CONFIG_ESP_CONSOLE_UART_NUM,
							CONFIG_ESP32S3APRS_CONSOLE_TX_GPIO,
						   	CONFIG_ESP32S3APRS_CONSOLE_RX_GPIO,
						   	UART_PIN_NO_CHANGE,
							UART_PIN_NO_CHANGE) != ESP_OK) {
					uart_driver_delete(CONFIG_ESP_CONSOLE_UART_NUM);
				}
			}*/
		}
	}


	if (uart_is_driver_installed(CONFIG_ESP_CONSOLE_UART_NUM))
		uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

	// Set log level
	esp_log_set_level_master(ESP_LOG_INFO);

	esp_log_level_set("MAIN", ESP_LOG_INFO);
	esp_log_level_set("event", ESP_LOG_INFO);
	esp_log_level_set("nvs", ESP_LOG_INFO);

	esp_log_level_set("SA8X8", ESP_LOG_INFO);
	esp_log_level_set("MODEM_AFSK1200", ESP_LOG_INFO);
	esp_log_level_set("framebuff", ESP_LOG_INFO);
	esp_log_level_set("AX25_PHY", ESP_LOG_INFO);
	esp_log_level_set("AX25_LM", ESP_LOG_INFO);
	esp_log_level_set("AX25", ESP_LOG_INFO);

	esp_log_level_set("APRS", ESP_LOG_INFO);
	esp_log_level_set("APRS_PARSERS", ESP_LOG_INFO);
	esp_log_level_set("APRS_ENCODER", ESP_LOG_INFO);
	esp_log_level_set("APRS_LOG", ESP_LOG_INFO);
	esp_log_level_set("SB", ESP_LOG_INFO);

	esp_log_level_set("KISS", ESP_LOG_INFO);

	esp_log_level_set("GPS", ESP_LOG_INFO);
	esp_log_level_set("GPS_PARSER", ESP_LOG_INFO);

	esp_log_level_set("USB", ESP_LOG_INFO);
	esp_log_level_set("USB_AUDIO", ESP_LOG_INFO);
	esp_log_level_set("USB_CDC", ESP_LOG_INFO);

	esp_log_level_set("SSD1680", ESP_LOG_INFO);
	esp_log_level_set("HMI", ESP_LOG_INFO);

	esp_log_level_set("MPY", ESP_LOG_INFO);
	esp_log_level_set("MP_APRS", ESP_LOG_INFO);

	ESP_LOGI(TAG, "Aprs stack started at %ld us since boot (%ld µs)", (long)ready_time, (long)(ready_time-start_time));

	// SPI master Init
	if (ESP_OK != spi_bus_initialize(CONFIG_ESP32S3APRS_SPI_HOST,
				&spi_config,
				SPI_DMA_CH_AUTO)) {
		ESP_LOGE(TAG,"Error initializing spi bus %d",CONFIG_ESP32S3APRS_SPI_HOST);
	}

	// SSD1680 Init
	XBM_SWAP_BITS(logo_bits);
	Epd = SSD1680_Init(CONFIG_ESP32S3APRS_SPI_HOST,
			CONFIG_ESP32S3APRS_FORCE_ON_GPIO,
			CONFIG_ESP32S3APRS_EPD_CS_GPIO,
			CONFIG_ESP32S3APRS_EPD_DC_GPIO,
			CONFIG_ESP32S3APRS_EPD_BUSY_GPIO,
			&epd_config,
			logo_bits);
	
	// Initialise SPIFFS
	ret =esp_vfs_spiffs_register(&Spiffs_Config);
	switch (ret) {
		case ESP_FAIL:
			ESP_LOGE(TAG,"Failed to mount or format spiffs partition labeled \"%s\"",Spiffs_Config.partition_label);
			break;
		case ESP_ERR_NOT_FOUND:
			ESP_LOGE(TAG,"Failed to find spiffs parition labeled\"%s\"",Spiffs_Config.partition_label);
			break;
		case ESP_OK:
			size_t total = 0, used = 0;
			ret = esp_spiffs_info(Spiffs_Config.partition_label, &total, &used);
			if (ret || used > total) {
				ESP_LOGE(TAG,"Can't get spiffs info (%s), checking ...",esp_err_to_name(ret));
				ret = esp_spiffs_check(Spiffs_Config.partition_label);
				if (ret != ESP_OK) {
					ESP_LOGE(TAG, "SPIFFS_check() failed (%s), formating ...", esp_err_to_name(ret));
					esp_spiffs_format(Spiffs_Config.partition_label);
				} else {
					ESP_LOGI(TAG, "SPIFFS_check() successful");
				}

				ret = esp_spiffs_info(Spiffs_Config.partition_label, &total, &used);
				if (ret != ESP_OK) {
					ESP_LOGE(TAG,"Error Mounting spiffs partition labeled \"%s\", aborting.",Spiffs_Config.partition_label);
				}
			}
			
			if (!ret)
				ESP_LOGI(TAG, "\"%s\" partition mounted.\n\t\tsize: total: %d, used: %d",
						Spiffs_Config.partition_label,
						total,
						used);
			break;
		default:
			ESP_LOGE(TAG,"Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
	}
	
	nvs_flash_init();

   	APRS_Load_Config(Aprs);
	nvs_open("Global",NVS_READWRITE,&Nvs);

	// GPS Init
	Gps = GPS_Init(CONFIG_ESP32S3APRS_GPS_UART_NUM, CONFIG_ESP32S3APRS_GPS_BAUD_RATE,
			CONFIG_ESP32S3APRS_GPS_TX_GPIO, CONFIG_ESP32S3APRS_GPS_RX_GPIO, CONFIG_ESP32S3APRS_GPS_RESET_GPIO);

	SA8X8_Load_Config(SA8x8);

	// USB Init
	USB_Init();

	// Kiss protocol on top of AX25 link multiplexer
	Kiss = Kiss_Init(USB_CDC_VFS_PATH "/0", Ax25_Lm);

	// ADC Init
	ADC_Init(ADC_UNIT_2);
	ADC_Config_Channel(CONFIG_ESP32S3APRS_ADC2_BATTERY_GPIO, ADC_ATTEN_DB_12);
	if (CONFIG_ESP32S3APRS_ADC2_CH1_GPIO != -1)
		ADC_Config_Channel(CONFIG_ESP32S3APRS_ADC2_CH1_GPIO, ADC_ATTEN_DB_12);
	if (CONFIG_ESP32S3APRS_ADC2_CH2_GPIO != -1)
		ADC_Config_Channel(CONFIG_ESP32S3APRS_ADC2_CH2_GPIO, ADC_ATTEN_DB_12);
	if (CONFIG_ESP32S3APRS_ADC2_CH3_GPIO != -1)
		ADC_Config_Channel(CONFIG_ESP32S3APRS_ADC2_CH3_GPIO, ADC_ATTEN_DB_12);
	if (CONFIG_ESP32S3APRS_ADC2_CH4_GPIO != -1)
		ADC_Config_Channel(CONFIG_ESP32S3APRS_ADC2_CH4_GPIO, ADC_ATTEN_DB_12);

	// HMI Init
	SSD1680_Set_Orientation(Epd,SSD1680_ORIENTATION_270);
	Hmi = HMI_Init((lv_disp_drv_t*)Epd);
	if (!Hmi)
		ESP_LOGW(TAG,"Error creating HMI");

   	APRS_Open_Db(Aprs);
	APRS_Start(Aprs);

	TestFrame.frame.usage = xSemaphoreCreateCountingStatic(255,0,&(TestFrame.frame.usage_buff));

	esp_pm_configure(&pm_config);

	// Start micropython shell on second usb-cdc
	mp_start(USB_CDC_VFS_PATH "/1");

	// Start main loop
	max_rssi=0;
	i=0;
	now = xTaskGetTickCount();
	while (1) {

		// Every Ten second
		if (!(i%10)) {
			// Get RSSI
			SA8x8_GetRssi(SA8x8,&rssi);
			if (rssi>max_rssi)
				max_rssi=rssi;
			Rssi = rssi;
		}

		// Every minutes
		if (!(i%60)) {
			// Get Battery voltage
			if (Epd) {
				SSD1680_PowerUp(Epd);
				vTaskDelay(10/portTICK_PERIOD_MS);
			}

			if (CONFIG_ESP32S3APRS_ADC2_BATTERY_GPIO != -1) {
				ADC_Read(CONFIG_ESP32S3APRS_ADC2_BATTERY_GPIO, NULL, &Adc2_Voltage[0]);
				Adc2_Voltage[0] <<= 1;
			}
			if (CONFIG_ESP32S3APRS_ADC2_CH1_GPIO != -1)
				ADC_Read(CONFIG_ESP32S3APRS_ADC2_CH1_GPIO, NULL, &Adc2_Voltage[1]);
			if (CONFIG_ESP32S3APRS_ADC2_CH2_GPIO != -1)
				ADC_Read(CONFIG_ESP32S3APRS_ADC2_CH2_GPIO, NULL, &Adc2_Voltage[2]);
			if (CONFIG_ESP32S3APRS_ADC2_CH3_GPIO != -1)
				ADC_Read(CONFIG_ESP32S3APRS_ADC2_CH3_GPIO, NULL, &Adc2_Voltage[3]);
			if (CONFIG_ESP32S3APRS_ADC2_CH4_GPIO != -1)
				ADC_Read(CONFIG_ESP32S3APRS_ADC2_CH4_GPIO, NULL, &Adc2_Voltage[4]);

			if (Epd)
				SSD1680_PowerDown(Epd);

			// Send ADC2 event
			if (CONFIG_ESP32S3APRS_ADC2_BATTERY_GPIO != -1) {
				Battery = Adc2_Voltage[0];
				ESP_LOGI(TAG,"Battery : %d mv",Battery);
			}

			esp_event_post(MAIN_EVENT,MAIN_EVENT_ADC2,&Adc2_Voltage,sizeof(Adc2_Voltage),portMAX_DELAY);

			// Send rssi event
			if (SA8x8) {
				ESP_LOGI(TAG,"Rssi : %d",max_rssi);
				Rssi_max = max_rssi;
				esp_event_post(MAIN_EVENT,MAIN_EVENT_RSSI,&max_rssi,sizeof(rssi),portMAX_DELAY);
				max_rssi=0;
			}
		}

		//Every 10 min + 10s
		if (!((i-10)%(60*10))) {
			// Send status frame
			if (Aprs)
				APRS_Send_Status(Aprs,NULL); // Send current status
			else {
				Modem_Start_Transmiter(Modem_AFSK1200);
				vTaskDelay(50);
				Modem_Send_Frame(Modem_AFSK1200, &TestFrame.frame);
				vTaskDelay(50);
				Modem_Stop_Transmiter(Modem_AFSK1200);
			}

#ifndef NDEBUG
# if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
			// Show runtime stat
			char *buff = malloc(1024);
			if (buff) {
				vTaskGetRunTimeStats(buff);
				fwrite(buff,strlen(buff), 1, stderr);
				free(buff);
			} else
				ESP_LOGE(TAG,"Can't allocate 1K on head");
# endif
			// Check heap integrity
			if (!heap_caps_check_integrity_all(true)) {
				ESP_LOGE(TAG,"Heap integrity check fail !");
				abort();
			}
			heap_caps_print_heap_info(0);

			// Dumpp pm locks
			esp_pm_dump_locks(stderr);

# if CONFIG_FREERTOS_USE_TRACE_FACILITY
			// Dump task status
			uxTaskGetSystemState(Task_status, sizeof(Task_status)/sizeof(Task_status[0]), &Runtime);
			printf("Id \tName \t StackFree\n");
			for (i=0; i< (sizeof(Task_status)/sizeof(Task_status[0])); i++) {
				if (Task_status[i].xTaskNumber)
					printf("%d\t%s\t%lu\n",
							Task_status[i].xTaskNumber, 
							Task_status[i].pcTaskName,
							Task_status[i].usStackHighWaterMark);

			}
# endif
#endif
		}

		vTaskDelayUntil(&now, 10000/portTICK_PERIOD_MS);
		i+=10;
	}

}

void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
    esp_default_wake_deep_sleep();
    // Add additional functionality here
}

