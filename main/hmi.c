/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/hmi.c
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

#include "hmi.h"

#include <string.h>
#include <sys/time.h>

#include <esp_timer.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_event.h>

ESP_EVENT_DECLARE_BASE(MAIN_EVENT);
#define MAIN_EVENT_RSSI	0
#define MAIN_EVENT_BATTERY	1

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <lvgl.h>
#include "lv_theme/lv_theme_mono_epd.h"

#include "aprs.h"
#include "gps.h"

#define TAG	"HMI"
#define HMI_TICK_PERIOD_MS      10
#define HMI_POLL_PERIOD_MS	30

#ifndef NDEBUG
#define HMI_TASK_STACK_SIZE     4096+1024
#else
#define HMI_TASK_STACK_SIZE     4096
#endif

#define HMI_TASK_PRIORITY       1

#define HMI_QUEUE_LEN           5

#define HMI_KEY_LEN             16
#define HMI_KEY_SEND_TO		10
#define HMI_KEY_RIGHT           CONFIG_ESP32S3APRS_BUTTON_RIGHT_GPIO
#define HMI_KEY_CENTER          CONFIG_ESP32S3APRS_BUTTON_CENTER_GPIO
#define HMI_KEY_LEFT            CONFIG_ESP32S3APRS_BUTTON_LEFT_GPIO
#define HMI_KEY_PRESSED         128
#define HMI_KEY_RELEASED        0

#define HMI_RX_LIST_MAX_CNT	20

#define HMI_LOCK	do xSemaphoreTake(HMI_Mutex, portMAX_DELAY); while(0)
#define HMI_UNLOCK	do xSemaphoreGive(HMI_Mutex); while(0)

static esp_timer_handle_t HMI_Tick_Timer;
static uint64_t HMI_Last_Timer_Event;
static SemaphoreHandle_t HMI_Mutex;
static StaticSemaphore_t HMI_Mutex_Buffer;

struct HMI_S {
	lv_disp_t * disp;
	lv_disp_drv_t * disp_drv;
	lv_indev_drv_t indev_drv;
	lv_indev_t *indev;
	StaticQueue_t key_data;
	uint8_t key_buff[sizeof(uint8_t)*HMI_KEY_LEN];
	QueueHandle_t key;
	uint8_t last_key;
	bool key_left, key_center, key_right;

	// main group
	lv_group_t *g_main;

	// Theme to apply
	lv_theme_t *theme;

	// Menu Windows
	lv_obj_t *w_menu;
	lv_obj_t *menu_main, *menu_rx, *menu_tx, *menu_radio;
	lv_obj_t *menu_station;

	// Current page
	lv_obj_t *w_current;

	// Rx windows
	lv_obj_t *w_rx;
	lv_obj_t *rx_list;

	// Tx windows
	lv_obj_t *w_tx;
	lv_obj_t *tx_src_dst;
	lv_obj_t *tx_path;
	lv_obj_t *tx_message;

	// radioio page;
	lv_obj_t *w_radio;
	lv_obj_t *frequency;
	lv_obj_t *volume;
	lv_obj_t *squelch;
	lv_obj_t *emphasis;
	lv_obj_t *hi_pass;
	lv_obj_t *low_pass;

	// status bar
	lv_obj_t *w_status;
	lv_obj_t *time;
	lv_obj_t *date;
	lv_obj_t *locator;
	lv_obj_t *battery;
	lv_obj_t *rssi;

	// station window
	AX25_Addr_t trackid;
	AX25_Addr_t lastid;
	APRS_Station_t sta;
	lv_obj_t *w_station;
	lv_obj_t *sta_title;
	lv_obj_t *sta_position;
	lv_obj_t *sta_course;
	lv_obj_t *sta_status;
	lv_obj_t *sta_comment;
};

struct {
	TaskHandle_t handle;
	StackType_t	stack[(HMI_TASK_STACK_SIZE)/sizeof(StackType_t)];
	StaticTask_t buffer;
} HMI_Tasks;

static void HMI_Timer_cb(uint64_t *last);
static void HMI_Gpio_Isr_Handler(void * arg);
static void HMI_Button_Read(lv_indev_drv_t *Indev, lv_indev_data_t *Data);
static void HMI_Task(void *Arg);
void HMI_Prepare(HMI_t * Hmi);

HMI_t * HMI_Init(lv_disp_drv_t * Disp_drv) {
	HMI_t * hmi;

	if (!lv_is_initialized()) {

		// Initialise lvgl timer and task

		HMI_Mutex = xSemaphoreCreateMutexStatic(&HMI_Mutex_Buffer);

		if (!HMI_Mutex) {
			ESP_LOGE(TAG,"Error creating HMI Mutex");
			return NULL;
		}

		HMI_LOCK;	// Should not block

		lv_init();
		HMI_Last_Timer_Event = esp_timer_get_time();

		// Tick interface for LVGL (using esp_timer to generate periodic event)
		const esp_timer_create_args_t HMI_Tick_Timer_args = {
			.callback = (esp_timer_cb_t) HMI_Timer_cb,
			.arg = (void*)&HMI_Last_Timer_Event,
			.name = "lvgl_tick",
			.skip_unhandled_events = true,
		};

		if (esp_timer_create(&HMI_Tick_Timer_args, &HMI_Tick_Timer) != ESP_OK) {
			ESP_LOGE(TAG,"Error creating tick timer");
			lv_deinit();
			vSemaphoreDelete(HMI_Mutex);
			return NULL;
		}

		if (esp_timer_start_periodic(HMI_Tick_Timer, HMI_TICK_PERIOD_MS * 1000) != ESP_OK) {
			ESP_LOGE(TAG,"Error starting tick timer");
			esp_timer_delete(HMI_Tick_Timer);
			lv_deinit();
			vSemaphoreDelete(HMI_Mutex);
			return NULL;
		}

		HMI_Tasks.handle = xTaskCreateStaticPinnedToCore(HMI_Task,"HMI", HMI_TASK_STACK_SIZE, NULL, HMI_TASK_PRIORITY, HMI_Tasks.stack, &HMI_Tasks.buffer, APP_CPU_NUM);

	} else
		HMI_LOCK;

	if (!(hmi = malloc(sizeof(HMI_t)))) {
		ESP_LOGE(TAG,"Error allocating HMI struct");
		return NULL;
	}
	bzero(hmi, sizeof(HMI_t));

	// Initialise display
	hmi->disp_drv = Disp_drv;
	hmi->disp = lv_disp_drv_register(hmi->disp_drv);

	hmi->key = xQueueCreateStatic(HMI_KEY_LEN,sizeof(uint8_t),hmi->key_buff,&hmi->key_data);

	// Initialize GPIO keys
	const gpio_config_t in_pins_config = {
		.intr_type = GPIO_INTR_ANYEDGE,
		.mode = GPIO_MODE_INPUT,
		.pin_bit_mask = (1ULL<<HMI_KEY_LEFT) | (1ULL<<HMI_KEY_CENTER) | (1ULL<<HMI_KEY_RIGHT),
		.pull_down_en = 0,
		.pull_up_en =  (1ULL<<HMI_KEY_LEFT) | (1ULL<<HMI_KEY_CENTER) | (1ULL<<HMI_KEY_RIGHT),
	};
	gpio_config(&in_pins_config);
	gpio_isr_handler_add(HMI_KEY_LEFT, HMI_Gpio_Isr_Handler, (void*) hmi);
	gpio_isr_handler_add(HMI_KEY_CENTER, HMI_Gpio_Isr_Handler, (void*) hmi);
	gpio_isr_handler_add(HMI_KEY_RIGHT, HMI_Gpio_Isr_Handler, (void*) hmi);

	lv_indev_drv_init(&hmi->indev_drv);
	hmi->indev_drv.type = LV_INDEV_TYPE_ENCODER;
	hmi->indev_drv.user_data = hmi;
	hmi->indev_drv.read_cb = HMI_Button_Read;
	hmi->indev = lv_indev_drv_register(&hmi->indev_drv);

	hmi->g_main = lv_group_create();
	lv_indev_set_group(hmi->indev, hmi->g_main);
	lv_group_set_default(hmi->g_main);

	// Set theme
	hmi->theme = lv_theme_mono_epd_init(hmi->disp, false, LV_FONT_DEFAULT);
	lv_disp_set_theme(hmi->disp, hmi->theme);

	// Create Interface
	HMI_Prepare(hmi);


	HMI_UNLOCK;

	return hmi;
}

static void HMI_Timer_cb(uint64_t * last) {
	uint64_t timer_event = esp_timer_get_time();

	lv_tick_inc((timer_event-*last)/1000);
	*last = timer_event;
}

static void HMI_Task(void * Arg) {
	HMI_LOCK;
	ESP_LOGI(TAG, "HMI task started");

	while (1) {
		lv_timer_handler();
		HMI_UNLOCK;
		vTaskDelay(HMI_POLL_PERIOD_MS/portTICK_PERIOD_MS);
		HMI_LOCK;
	}
}

// GPIO Interrupt handler
static void IRAM_ATTR HMI_Gpio_Isr_Handler(void * arg) {
	BaseType_t MustYield = pdFALSE;
	HMI_t * Hmi= arg;
	bool key;
	uint8_t code;

	key = !(gpio_get_level(HMI_KEY_CENTER));
	if (key != Hmi->key_center) {
		Hmi->key_center = key;
		code = HMI_KEY_CENTER | (key?HMI_KEY_PRESSED:HMI_KEY_RELEASED);
		xQueueSendFromISR(Hmi->key,&code,&MustYield);
	}

	key = !(gpio_get_level(HMI_KEY_RIGHT));
	if (key != Hmi->key_right) {
		Hmi->key_right = key;
		code = HMI_KEY_RIGHT | (key?HMI_KEY_PRESSED:HMI_KEY_RELEASED);
		xQueueSendFromISR(Hmi->key,&code,&MustYield);
	}

	key = !(gpio_get_level(HMI_KEY_LEFT));
	if (key != Hmi->key_left) {
		Hmi->key_left = key;
		code = HMI_KEY_LEFT | (key?HMI_KEY_PRESSED:HMI_KEY_RELEASED);
		xQueueSendFromISR(Hmi->key,&code,&MustYield);
	}

	portYIELD_FROM_ISR(MustYield);
}

static void HMI_Button_Read(lv_indev_drv_t * Indev, lv_indev_data_t * Data) {
	HMI_t * hmi = Indev->user_data;
	uint8_t new;

	if (xQueueReceive(hmi->key,&new,0) == pdPASS) {
		hmi->last_key = new;
		ESP_LOGD(TAG,"Key %d %s",new&0x7f,new&0x80?"pressed":"released");
	}

	switch (hmi->last_key&0x7f) {
		case HMI_KEY_RIGHT:
			Data->key = LV_KEY_RIGHT;
			break;
		case HMI_KEY_CENTER:
			Data->key = LV_KEY_ENTER;
			break;
		case HMI_KEY_LEFT:
			Data->key = LV_KEY_LEFT;
			break;
		default:
			Data->key = 0;
	}
	Data->state = (hmi->last_key&HMI_KEY_PRESSED)?LV_INDEV_STATE_PRESSED:LV_INDEV_STATE_RELEASED;
}


/******************** Specific HMI code *******************************/

#include "aprs.h"
#include <SA8x8.h>
#include <nvs.h>

#include "xbm/antenna_0.xbm"
#include "xbm/antenna_1.xbm"
#include "xbm/antenna_2.xbm"
#include "xbm/antenna_3.xbm"
#include "xbm/antenna_4.xbm"
#include "xbm/antenna_5.xbm"

lv_img_dsc_t HMI_Img_Antenna[6] = {
	{
		.header.always_zero = 0,
		.header.w = antenna_0_width,
		.header.h = antenna_0_height,
		.header.cf = LV_IMG_CF_ALPHA_1BIT,
		.data_size = sizeof(antenna_0_bits),
		.data = antenna_0_bits
	}, {
		.header.always_zero = 0,
		.header.w = antenna_1_width,
		.header.h = antenna_1_height,
		.header.cf = LV_IMG_CF_ALPHA_1BIT,
		.data_size = sizeof(antenna_1_bits),
		.data = antenna_1_bits
	}, {
		.header.always_zero = 0,
		.header.w = antenna_2_width,
		.header.h = antenna_2_height,
		.header.cf = LV_IMG_CF_ALPHA_1BIT,
		.data_size = sizeof(antenna_2_bits),
		.data = antenna_2_bits
	}, {
		.header.always_zero = 0,
			.header.w = antenna_3_width,
			.header.h = antenna_3_height,
			.header.cf = LV_IMG_CF_ALPHA_1BIT,
			.data_size = sizeof(antenna_3_bits),
			.data = antenna_3_bits
	}, {
		.header.always_zero = 0,
			.header.w = antenna_4_width,
			.header.h = antenna_4_height,
			.header.cf = LV_IMG_CF_ALPHA_1BIT,
			.data_size = sizeof(antenna_4_bits),
			.data = antenna_4_bits
	}, {
		.header.always_zero = 0,
			.header.w = antenna_5_width,
			.header.h = antenna_5_height,
			.header.cf = LV_IMG_CF_ALPHA_1BIT,
			.data_size = sizeof(antenna_5_bits),
			.data = antenna_5_bits
	}
};

static char HMI_Frequency_List[] =
"144800000\n"
"144812500\n"
"144825000\n"
"144837500\n"
"144850000\n"
"144862500\n"
"144875000\n"
"144887500\n"
"144900000\n"
"144912500\n"
"144925000\n"
"144937500\n"
"144950000\n"
"144962500\n"
"144975000\0"
".........";	// Place holder for custom freq

extern APRS_t * Aprs;
extern nvs_handle_t Nvs;
extern SA8x8_t * SA8x8;

static void HMI_Update_Station(HMI_t *Hmi) {
	struct tm tm;
	char txt[32];
	int pos;

	if (Hmi->sta.timestamp) {
		gmtime_r(&Hmi->sta.timestamp, &tm);
		pos = strftime(txt,sizeof(txt),"%H:%M ",&tm);
	} else {
		 strcpy(txt, "--:-- ");
		 pos = 6;
	}

	if (Hmi->sta.symbol[0]) {
		txt[pos++] = Hmi->sta.symbol[0];
		txt[pos++] = Hmi->sta.symbol[1];
		txt[pos++] = ' ';
	} else {
		txt[pos++] = ' ';
		txt[pos++] = ' ';
		txt[pos++] = ' ';
	}

	pos += AX25_Addr_To_Str(&Hmi->sta.callid, txt+pos, sizeof(txt)-pos);

	lv_label_set_text(Hmi->sta_title, txt);

	if (Hmi->sta.position.latitude || Hmi->sta.position.longitude) {
		uint16_t ld,lm,ls, lld, llm, lls;
		int32_t tmp;
		char ns, ew;
		tmp = Hmi->sta.position.latitude;
		if (tmp < 0) {
			tmp = -tmp;
			ns = 'S';
		} else
			ns = 'N';
		ld = tmp>>GPS_FIXED_POINT_DEG;
		tmp = (tmp & ((1<<GPS_FIXED_POINT_DEG)-1)) * 60;
		lm = tmp>>GPS_FIXED_POINT_DEG;
		tmp = (tmp & ((1<<GPS_FIXED_POINT_DEG)-1)) * 60;
		ls = (tmp + (1<<(GPS_FIXED_POINT_DEG-1)))>>GPS_FIXED_POINT_DEG;

		tmp = Hmi->sta.position.longitude;
		if (tmp < 0) {
			tmp = -tmp;
			ew = 'W';
		} else
			ew = 'E';
		lld = tmp>>GPS_FIXED_POINT_DEG;
		tmp = (tmp & ((1<<GPS_FIXED_POINT_DEG)-1)) * 60;
		llm = tmp>>GPS_FIXED_POINT_DEG;
		tmp = (tmp & ((1<<GPS_FIXED_POINT_DEG)-1)) * 60;
		lls = (tmp + (1<<(GPS_FIXED_POINT_DEG-1)))>>GPS_FIXED_POINT_DEG;

		snprintf(txt, sizeof(txt), "%02u°%02u\"%02u\"%c %02u°%02u\'%02u\"%c",
				ld, lm, ls, ns,
				lld, llm, lls, ew);
		lv_label_set_text(Hmi->sta_position, txt);
		lv_obj_clear_flag(Hmi->sta_position, LV_OBJ_FLAG_HIDDEN);
	} else
		lv_obj_add_flag(Hmi->sta_position, LV_OBJ_FLAG_HIDDEN);

	if (Hmi->sta.course.speed || Hmi->sta.course.dir) {

		snprintf(txt, sizeof(txt), "%s : %03d° %03dkn",
				(Hmi->sta.symbol[1] == '_')?"Wind":"Course",
				Hmi->sta.course.dir, Hmi->sta.course.speed);
		lv_label_set_text(Hmi->sta_course, txt);
		lv_obj_clear_flag(Hmi->sta_course, LV_OBJ_FLAG_HIDDEN);
	} else
		lv_obj_add_flag(Hmi->sta_course, LV_OBJ_FLAG_HIDDEN);
	if (Hmi->sta.status[0]) {
		lv_label_set_text(Hmi->sta_status, Hmi->sta.status);
		lv_obj_clear_flag(Hmi->sta_status, LV_OBJ_FLAG_HIDDEN);
	} else
		lv_obj_add_flag(Hmi->sta_status, LV_OBJ_FLAG_HIDDEN);
}

static void HMI_Message_cb(lv_event_t * Event) {
	HMI_t * hmi = lv_event_get_user_data(Event);
	lv_obj_t * target = lv_event_get_target(Event);

	ESP_LOGD(TAG,"Message Clicked");
	APRS_Send_Status(Aprs,lv_list_get_btn_text(hmi->tx_message,target));
}

static void HMI_Frequency_cb(lv_event_t * Event) {
	lv_obj_t * target = lv_event_get_target(Event);
	char freq[10];
	uint32_t f;

	lv_roller_get_selected_str(target, freq, sizeof(freq));
	f = atol(freq);
	ESP_LOGI(TAG,"Seeting Frequency to %ld MHz",f);
	SA8x8_Set_Freq(SA8x8,f);
}

static void HMI_Volume_cb(lv_event_t * Event) {
	lv_obj_t * target = lv_event_get_target(Event);
	uint8_t v;

	v = lv_spinbox_get_value(target);
	ESP_LOGI(TAG,"Setting volume to %d",v);
	SA8x8_Set_Volume(SA8x8,v);
}

static void HMI_Squelch_cb(lv_event_t * Event) {
	lv_obj_t * target = lv_event_get_target(Event);
	uint8_t s;

	s = lv_spinbox_get_value(target);
	ESP_LOGI(TAG,"Setting Squelch to %d",s);
	SA8x8_Set_Squelch(SA8x8,s);
}

static void HMI_Emphasis_cb(lv_event_t * Event) {
	lv_obj_t * target = lv_event_get_target(Event);
	uint8_t e;

	e = !!(lv_obj_get_state(target)&LV_STATE_CHECKED);
	ESP_LOGI(TAG,"Setting Emphasis to %d",e);
	SA8x8_Set_Emphasis(SA8x8,e);
}

static void HMI_Hipass_cb(lv_event_t * Event) {
	lv_obj_t * target = lv_event_get_target(Event);
	uint8_t h;

	h = !!(lv_obj_get_state(target)&LV_STATE_CHECKED);
	ESP_LOGI(TAG,"Setting Hipass to %d",h);
	SA8x8_Set_Hipass(SA8x8,h);
}

static void HMI_Lowpass_cb(lv_event_t * Event) {
	lv_obj_t * target = lv_event_get_target(Event);
	uint8_t l;

	l = !!(lv_obj_get_state(target)&LV_STATE_CHECKED);
	ESP_LOGI(TAG,"Setting Lowpass to %d",l);
	SA8x8_Set_Lowpass(SA8x8,l);
}

static void HMI_Stations_Menu_cb(lv_event_t * Event) {
	HMI_t * hmi = lv_event_get_user_data(Event);

	bzero(&hmi->trackid, sizeof(AX25_Addr_t));
	bzero(&hmi->sta, sizeof(APRS_Station_t));
	memcpy(&hmi->sta.callid, &hmi->lastid, sizeof(AX25_Addr_t));

	if (hmi->lastid.addr[0])
		APRS_Get_Station(Aprs, &hmi->lastid, &hmi->sta);

	HMI_Update_Station(hmi);
}

static void HMI_Stations_List_cb(lv_event_t * Event) {
	HMI_t * hmi = lv_event_get_user_data(Event);
	lv_obj_t * target = lv_event_get_target(Event);
	AX25_Addr_t *addr = lv_obj_get_user_data(target);

	memcpy(&hmi->trackid, addr, sizeof(AX25_Addr_t));
	bzero(&hmi->sta, sizeof(APRS_Station_t));
	memcpy(&hmi->sta.callid, addr, sizeof(AX25_Addr_t));

	APRS_Get_Station(Aprs, addr, &hmi->sta);

	HMI_Update_Station(hmi);
}

static void HMI_Prepare_Menu(HMI_t * Hmi) {
	Hmi->w_menu = lv_menu_create(lv_scr_act());
	lv_obj_set_size(Hmi->w_menu,LV_PCT(100),LV_PCT(100-17));
	lv_obj_align(Hmi->w_menu, LV_ALIGN_BOTTOM_LEFT, 0, 0);

	Hmi->menu_main = lv_menu_page_create(Hmi->w_menu,NULL);
	// set sidebar width
	lv_menu_set_sidebar_page(Hmi->w_menu, Hmi->menu_main);
	lv_obj_set_width(((lv_menu_t*)Hmi->w_menu)->sidebar, LV_PCT(20));
}

static void HMI_Aprs_Event(HMI_t * Hmi, esp_event_base_t event_base, int32_t event_id, APRS_Data_t * Data);

static void HMI_Prepare_Rx(HMI_t *Hmi) {

	// Rx menu button
	lv_obj_t *cont = lv_menu_cont_create(Hmi->menu_main);
	Hmi->menu_rx = lv_btn_create(cont);
	lv_obj_t *label = lv_label_create(Hmi->menu_rx);
	lv_label_set_text(label,"Rx");

	// RX  window
	Hmi->w_rx = lv_menu_page_create(Hmi->w_menu,NULL);

	lv_menu_set_load_page_event(Hmi->w_menu, Hmi->menu_rx, Hmi->w_rx);

	// Rx list
	Hmi->rx_list = lv_list_create(Hmi->w_rx);
	lv_obj_set_size(Hmi->rx_list,LV_PCT(100),LV_PCT(100));
}

static void HMI_Prepare_Tx(HMI_t *Hmi) {
	int pos;
	char var[16];
	char str[64];
	int i;
	size_t len;

	lv_obj_t *cont = lv_menu_cont_create(Hmi->menu_main);
	Hmi->menu_tx = lv_btn_create(cont);
	lv_obj_t *label = lv_label_create(Hmi->menu_tx);
	lv_label_set_text(label,"Tx");

	// Tx window
	Hmi->w_tx = lv_menu_page_create(Hmi->w_menu,NULL);

	lv_menu_set_load_page_event(Hmi->w_menu, Hmi->menu_tx, Hmi->w_tx);

	pos = APRS_Get_Callid_Str(Aprs,str,sizeof(str));
	str[pos++] = ' ';
	str[pos++] = '>';
	str[pos++] = ' ';
	APRS_Get_Appid_Str(Aprs,str+pos,sizeof(str)-pos);

	Hmi->tx_src_dst = lv_label_create(Hmi->w_tx);
	lv_label_set_text(Hmi->tx_src_dst,str);
	lv_obj_align(Hmi->tx_src_dst,LV_ALIGN_TOP_MID,0,0);

	strcpy(str,"via ");
	pos = APRS_Get_Digis_Str(Aprs,str+4,sizeof(str)-4);
	Hmi->tx_path = lv_label_create(Hmi->w_tx);
	lv_label_set_text(Hmi->tx_path,str);
	lv_obj_align_to(Hmi->tx_path,Hmi->tx_src_dst,LV_ALIGN_OUT_BOTTOM_MID,0,0);

	lv_obj_update_layout(Hmi->tx_path);

	Hmi->tx_message = lv_list_create(Hmi->w_tx);
	lv_obj_add_flag(Hmi->tx_message,LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_scrollbar_mode(Hmi->tx_message, LV_SCROLLBAR_MODE_AUTO);
	lv_obj_set_scroll_dir(Hmi->tx_message, LV_DIR_VER);
	lv_obj_align_to(Hmi->tx_message, Hmi->tx_path, LV_ALIGN_OUT_BOTTOM_LEFT,0,0);
	lv_obj_set_x(Hmi->tx_message,0);

	for (i=0;i<100;i++) {
		snprintf(var,sizeof(var),"Message%u",i);
		str[0] = '\0';
		len = sizeof(str);
		if (!nvs_get_str(Nvs,var,str,&len)) {
			ESP_LOGD(TAG,"Found Message %u : %s",i,str);
			lv_obj_t * btn = lv_list_add_btn(Hmi->tx_message,NULL,str);
			lv_obj_add_event_cb(btn, HMI_Message_cb, LV_EVENT_CLICKED, Hmi);
			// Set long mode
			for(int i = 0; i < lv_obj_get_child_cnt(btn); i++) {
				lv_obj_t * child = lv_obj_get_child(btn, i);
				if(lv_obj_check_type(child, &lv_label_class)) {
					// Found label
					lv_label_set_long_mode(child,LV_LABEL_LONG_CLIP);
					break;
				}
			}
		} else
			break;
	}

	if (!i) {
		ESP_LOGD(TAG,"Set default Message %u : %s",i,HMI_DEFAULT_MESSAGE);
		lv_obj_t * btn = lv_list_add_btn(Hmi->tx_message,NULL,APRS_DEFAULT_STATUS);
		lv_obj_add_event_cb(btn, HMI_Message_cb, LV_EVENT_CLICKED, Hmi);
		// Set long mode
		for(int i = 0; i < lv_obj_get_child_cnt(btn); i++) {
			lv_obj_t * child = lv_obj_get_child(btn, i);
			if(lv_obj_check_type(child, &lv_label_class)) {
				// Found label
				lv_label_set_long_mode(child,LV_LABEL_LONG_CLIP);
				break;
			}
		}
	}
}

void HMI_Prepare_Radio(HMI_t * Hmi) {
	int i;
	char *start,*end;
	uint32_t f;
	lv_obj_t * tmp_obj;
	lv_obj_t * cont;

	// Radio menu button
	cont = lv_menu_cont_create(Hmi->menu_main);
	Hmi->menu_radio = lv_btn_create(cont);
	lv_obj_t *label = lv_label_create(Hmi->menu_radio);
	lv_label_set_text(label,"Ra");

	// Radio window
	Hmi->w_radio = lv_menu_page_create(Hmi->w_menu, NULL);
	lv_obj_add_flag(Hmi->w_radio,LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_flag(Hmi->w_radio,LV_OBJ_FLAG_SCROLL_ON_FOCUS);
	lv_obj_clear_flag(Hmi->w_radio, LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM);

	lv_menu_set_load_page_event(Hmi->w_menu, Hmi->menu_radio, Hmi->w_radio);

	// Frequency
	f = SA8x8_Get_RxFreq(SA8x8);
	ESP_LOGI(TAG,"Freq set to %lu",f);

	start = HMI_Frequency_List;
	i = 0;
	do {
		end = strchrnul(start,'\n');
		if (atol(start) == f) {
			break;
		}
		start = end+1;
		i++;
	} while (*end!='\0');
	if (!(*end) && (start-end)==1) {
		// Add Custom freq at end of list
		ESP_LOGD(TAG, "Adding custom freq %lu to list", f);
		*end='\n';
		end++;
		snprintf(end,10,"%09lu",f);
	}

	ESP_LOGD(TAG,"Freq list : %s", HMI_Frequency_List);

	cont = lv_menu_cont_create(Hmi->w_radio);
	Hmi->frequency = lv_roller_create(cont);
	lv_roller_set_options(Hmi->frequency,HMI_Frequency_List,LV_ROLLER_MODE_NORMAL);
	lv_roller_set_visible_row_count(Hmi->frequency,1);
	lv_obj_clear_flag(Hmi->frequency, LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM);
	ESP_LOGD(TAG, "Selecting freq n°%d", i);
	lv_roller_set_selected(Hmi->frequency,i,LV_ANIM_OFF);
	lv_obj_add_event_cb(Hmi->frequency, HMI_Frequency_cb, LV_EVENT_VALUE_CHANGED, Hmi);

	tmp_obj = lv_label_create(cont);
	lv_label_set_text(tmp_obj, " Hz");

	/* Volume */
	uint8_t v;
	cont = lv_menu_cont_create(Hmi->w_radio);
	Hmi->volume = lv_spinbox_create(cont);
	tmp_obj=lv_label_create(cont);
	lv_label_set_text(tmp_obj,"Volume");
	lv_spinbox_set_range(Hmi->volume, 1, 8);
	lv_spinbox_set_digit_format(Hmi->volume, 1, 0);
	v = SA8x8_Get_Volume(SA8x8);
	lv_spinbox_set_value(Hmi->volume,v);
	lv_obj_add_event_cb(Hmi->volume, HMI_Volume_cb, LV_EVENT_VALUE_CHANGED, Hmi);

	/* Squelch */
	uint8_t s;
	cont = lv_menu_cont_create(Hmi->w_radio);
	Hmi->squelch = lv_spinbox_create(cont);
	tmp_obj=lv_label_create(cont);
	lv_label_set_text(tmp_obj,"Squelch");
	lv_spinbox_set_range(Hmi->squelch, 0, 8);
	lv_spinbox_set_digit_format(Hmi->squelch, 1, 0);
	s = SA8x8_Get_Squelch(SA8x8);
	lv_spinbox_set_value(Hmi->squelch,s);
	lv_obj_add_event_cb(Hmi->squelch, HMI_Squelch_cb, LV_EVENT_VALUE_CHANGED, Hmi);

	/* Emphasis */
	uint8_t e;
	cont = lv_menu_cont_create(Hmi->w_radio);
	Hmi->emphasis = lv_checkbox_create(cont);
	lv_checkbox_set_text(Hmi->emphasis,"Emphasis");
	e = SA8x8_Get_Emphasis(SA8x8);
	lv_obj_add_state(Hmi->emphasis, e ? LV_STATE_CHECKED:0);
	lv_obj_add_event_cb(Hmi->emphasis, HMI_Emphasis_cb, LV_EVENT_VALUE_CHANGED, Hmi);

	/* Hi-pass */
	uint8_t h;
	cont = lv_menu_cont_create(Hmi->w_radio);
	Hmi->hi_pass = lv_checkbox_create(cont);
	lv_checkbox_set_text(Hmi->hi_pass,"Hipass");
	h = SA8x8_Get_Hipass(SA8x8);
	lv_obj_add_state(Hmi->hi_pass, h?LV_STATE_CHECKED:0);
	lv_obj_add_event_cb(Hmi->hi_pass, HMI_Hipass_cb, LV_EVENT_VALUE_CHANGED, Hmi);

	/* Low pass */
	uint8_t l;
	cont = lv_menu_cont_create(Hmi->w_radio);
	Hmi->low_pass = lv_checkbox_create(cont);
	lv_checkbox_set_text(Hmi->low_pass,"Lowpass");
	l = SA8x8_Get_Lowpass(SA8x8);
	lv_obj_add_state(Hmi->low_pass, l?LV_STATE_CHECKED:0);
	lv_obj_add_event_cb(Hmi->low_pass, HMI_Lowpass_cb, LV_EVENT_VALUE_CHANGED, Hmi);

}

static void HMI_Battery_Event(HMI_t * Hmi, esp_event_base_t event_base, int32_t event_id, int * Voltage);
static void HMI_Rssi_Event(HMI_t * Hmi, esp_event_base_t event_base, int32_t event_id, uint8_t * Rssi);
static void HMI_Gps_Event_RMC(HMI_t * Hmi, esp_event_base_t event_base, int32_t event_id, GPS_Data_t * Data);

static void HMI_Prepare_Status(HMI_t * Hmi) {

	Hmi->w_status = lv_obj_create(lv_scr_act());
	lv_obj_set_size(Hmi->w_status,LV_PCT(100),LV_PCT(17));
	lv_obj_align(Hmi->w_status, LV_ALIGN_TOP_MID, 0, 0);
	lv_obj_clear_flag(Hmi->w_status,LV_OBJ_FLAG_SCROLLABLE);

	Hmi->rssi = lv_img_create(Hmi->w_status);
	lv_obj_align(Hmi->rssi,LV_ALIGN_RIGHT_MID,0,0);
	lv_img_set_src(Hmi->rssi,&HMI_Img_Antenna[0]);
	lv_obj_set_user_data(Hmi->rssi,(void*)0);	// save last rssi status

	Hmi->battery = lv_label_create(Hmi->w_status);
	lv_label_set_text(Hmi->battery,LV_SYMBOL_BATTERY_EMPTY);
	lv_obj_align_to(Hmi->battery,Hmi->rssi,LV_ALIGN_OUT_LEFT_MID,0,0);
	lv_obj_set_user_data(Hmi->battery,(void*)0);   // save last battery status

	Hmi->locator = lv_label_create(Hmi->w_status);
	lv_label_set_text(Hmi->locator,"JJ00AA00");
	lv_obj_align_to(Hmi->locator,Hmi->battery, LV_ALIGN_OUT_LEFT_MID,-2,0);

	Hmi->date = lv_label_create(Hmi->w_status);
	lv_label_set_text(Hmi->date,"00/00/00");
	lv_obj_set_user_data(Hmi->date,0);
	lv_obj_align(Hmi->date,LV_ALIGN_LEFT_MID,0,0);

	Hmi->time = lv_label_create(Hmi->w_status);
	lv_label_set_text(Hmi->time,"00:00");
	lv_obj_set_user_data(Hmi->time,0);
	lv_obj_align_to(Hmi->time, Hmi->date, LV_ALIGN_OUT_RIGHT_MID, 2,0);

}

static void HMI_Prepare_Station(HMI_t * Hmi) {
	lv_obj_t *cont;

	cont = lv_menu_cont_create(Hmi->menu_main);
	Hmi->menu_station = lv_btn_create(cont);
	lv_obj_t *label = lv_label_create(Hmi->menu_station);
	lv_label_set_text(label,"Sta");

	// RX  window
	Hmi->w_station = lv_menu_page_create(Hmi->w_menu, NULL);
	lv_obj_add_flag(Hmi->w_station,LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_flag(Hmi->w_station,LV_OBJ_FLAG_SCROLL_ON_FOCUS);
	lv_obj_clear_flag(Hmi->w_station, LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM);

	lv_menu_set_load_page_event(Hmi->w_menu, Hmi->menu_station, Hmi->w_station);
	lv_obj_add_event_cb(Hmi->menu_station, HMI_Stations_Menu_cb, LV_EVENT_CLICKED, Hmi);

	cont = lv_menu_cont_create(Hmi->w_station);
	Hmi->sta_title = lv_label_create(cont);
	lv_label_set_text(Hmi->sta_title,"88:88 XX XXXXXX-15");

	cont = lv_menu_cont_create(Hmi->w_station);
	Hmi->sta_position = lv_label_create(cont);
	lv_label_set_text(Hmi->sta_position,"88°88\'88\"N 188°88\'88\"W");

	cont = lv_menu_cont_create(Hmi->w_station);
	Hmi->sta_course = lv_label_create(cont);
	lv_label_set_text(Hmi->sta_course,"Course : 360° 888Kn");

	cont = lv_menu_cont_create(Hmi->w_station);
	Hmi->sta_status = lv_label_create(cont);
	lv_obj_set_size(Hmi->sta_status, LV_PCT(100), LV_SIZE_CONTENT);
	lv_label_set_long_mode(Hmi->sta_status, LV_LABEL_LONG_WRAP);
}

#define HMI_XBM_SWAP_BITS(bitmap) do {\
	for (int i = 0;i<sizeof(bitmap);i++) {\
		uint8_t c;\
		c = bitmap[i];\
		c = ((c&0xf0)>>4) | ((c&0x0f)<<4);\
		c = ((c&0xcc)>>2) | ((c&0x33)<<2);\
		c = ((c&0xaa)>>1) | ((c&0x55)<<1);\
		bitmap[i] = c;\
	}} while (0)

static void HMI_Img_Load(void) {
	static bool img_loaded = false;
	if (!img_loaded) {
		HMI_XBM_SWAP_BITS(antenna_0_bits);
		HMI_XBM_SWAP_BITS(antenna_1_bits);
		HMI_XBM_SWAP_BITS(antenna_2_bits);
		HMI_XBM_SWAP_BITS(antenna_3_bits);
		HMI_XBM_SWAP_BITS(antenna_4_bits);
		HMI_XBM_SWAP_BITS(antenna_5_bits);
		img_loaded = true;
	}
}

void HMI_Prepare(HMI_t * Hmi) {
	// xmb need reversed bit order
	HMI_Img_Load();

	HMI_Prepare_Status(Hmi);
	HMI_Prepare_Menu(Hmi);
	HMI_Prepare_Rx(Hmi);
	HMI_Prepare_Tx(Hmi);
	HMI_Prepare_Radio(Hmi);
	HMI_Prepare_Station(Hmi);

	HMI_Update_Station(Hmi);

	lv_menu_set_page(Hmi->w_menu, Hmi->w_station);
//	lv_group_focus_obj(Hmi->menu_station);

	esp_event_handler_register(APRS_EVENT, ESP_EVENT_ANY_ID, (esp_event_handler_t)HMI_Aprs_Event, (void*)Hmi);
	esp_event_handler_register(GPS_EVENT, GPS_PARSER_RMC, (esp_event_handler_t)HMI_Gps_Event_RMC, (void*)Hmi);
	esp_event_handler_register(MAIN_EVENT, MAIN_EVENT_BATTERY, (esp_event_handler_t)HMI_Battery_Event, (void*)Hmi);
	esp_event_handler_register(MAIN_EVENT, MAIN_EVENT_RSSI, (esp_event_handler_t)HMI_Rssi_Event, (void*)Hmi);
}

// Event handler
static void HMI_Battery_Event(HMI_t * Hmi, esp_event_base_t event_base, int32_t event_id, int * voltage)  {
	int32_t new, old;
	const char * sym;

	switch (*voltage) {
		case 0 ... 3399:	// Battery empty (do go bellow 3V3)
			new = 0;
			sym = LV_SYMBOL_BATTERY_EMPTY;
			break;
		case 3400 ... 3599:
			new = 1;
			sym = LV_SYMBOL_BATTERY_1;
			break;
		case 3600 ... 3799:
			new = 2;
			sym = LV_SYMBOL_BATTERY_2;
			break;
		case 3800 ... 3999:
			new = 3;
			sym = LV_SYMBOL_BATTERY_3;
			break;
		default:
			new = 4;
			sym = LV_SYMBOL_BATTERY_FULL;
			break;
	}

	old = (int32_t) lv_obj_get_user_data(Hmi->battery);
	if (old != new) {
		HMI_LOCK;
		ESP_LOGD(TAG,"Set Battery %ld -> %ld", old, new);
		lv_label_set_text(Hmi->battery,sym);
		lv_obj_set_user_data(Hmi->battery, (void*)new);
		HMI_UNLOCK;
	}
}

static void HMI_Rssi_Event(HMI_t * Hmi, esp_event_base_t event_base, int32_t event_id, uint8_t * rssi) {
	int32_t new, old;

	switch (*rssi) {
		case 0 ... 29:	// Low signal
			new = 0;
			break;
		case 30 ... 49: 
			new = 1;
			break;
		case 50 ... 64:
			new = 2;
			break;
		case 65 ... 79:
			new = 3;
			break;
		case 80 ... 94:
			new = 4;
			break;
		default:	// High signal
			new = 5;
	}

	old = (int32_t) lv_obj_get_user_data(Hmi->rssi);
	if (old != new) {
		HMI_LOCK;
		ESP_LOGD(TAG,"Set Rssi %ld -> %ld", old, new);
		lv_img_set_src(Hmi->rssi,&HMI_Img_Antenna[new]);
		lv_obj_set_user_data(Hmi->rssi, (void*)new);
		HMI_UNLOCK;
	}
}

static void HMI_Aprs_Event(HMI_t * Hmi, esp_event_base_t event_base, int32_t event_id, APRS_Data_t * Data) {
	int i, cnt;
	lv_obj_t * btn, *found;
	AX25_Addr_t * addr;
	char txt[32];
	struct tm tm;
	lv_obj_t *focus = NULL;

	HMI_LOCK;
	ESP_LOGD(TAG,"Frame received");
	cnt=lv_obj_get_child_cnt(Hmi->rx_list);
	ESP_LOGD(TAG,"rx list have %d child",cnt);
	found = NULL;
	addr = NULL;
	for (i=0; i<cnt; i++) {
		btn = lv_obj_get_child(Hmi->rx_list,i);

		if (btn == lv_group_get_focused(Hmi->g_main))
			focus = btn;
		if (!btn) {
			ESP_LOGE(TAG,"Null button found (idx = %d)",i);
			continue;
		}
		ESP_LOGV(TAG,"rx list chlid %d : %s",i,lv_label_get_text(lv_obj_get_child(btn,0)));
		addr = (AX25_Addr_t*)lv_obj_get_user_data(btn);
		if (addr) {
			if (!AX25_Addr_Cmp(addr,&Data->address[1])) {
				ESP_LOGD(TAG,"Existing Button found");
				found = btn;
				break;
			}
		} else {
			ESP_LOGE(TAG,"Button found with NULL addr");
			lv_obj_del(btn);	// remove button for this station
			ESP_LOGD(TAG,"Button deleted");
		}
	}

	// Limit rx_list size
	if (cnt >= HMI_RX_LIST_MAX_CNT && i == cnt && !found && addr) {
		ESP_LOGD(TAG,"Recycling button %d",cnt-1);
		found = btn;
		memcpy(addr,&Data->address[1],sizeof(AX25_Addr_t));
	}

	// Create button text
	gmtime_r(&Data->timestamp,&tm);
	i = strftime(txt,sizeof(txt),"%H:%M ",&tm);
	i+= AX25_Addr_To_Str(&Data->address[1],txt+i,sizeof(txt)-i);
	if (i && txt[i-1] == '*')
		i--;
	if (Data->from >1) {
		txt[i] = ' ';
		i++;
		i+= AX25_Addr_To_Str(&Data->address[Data->from],txt+i,sizeof(txt)-i);
		if (i && txt[i-1] == '*')
			i--;
	}
	txt[i] = '\0';

	if (!found) {
		// Add new button
		if (!(addr = malloc(sizeof(AX25_Addr_t)))) {
			ESP_LOGE(TAG,"Error allocating address for new button");
			HMI_UNLOCK;
			return;
		}
		memcpy(addr,&Data->address[1],sizeof(AX25_Addr_t));
		// TODO : Display APRS symbol
		btn = lv_list_add_btn(Hmi->rx_list,NULL,txt);
		if (!btn) {
			ESP_LOGE(TAG,"Error creating new button");
			HMI_UNLOCK;
			return;
		}
		lv_obj_set_user_data(btn,(void*)addr);
		// set label long mod
		for(int i = 0; i < lv_obj_get_child_cnt(btn); i++) {
			lv_obj_t * child = lv_obj_get_child(btn, i);
			if(lv_obj_check_type(child, &lv_label_class)) {
				// Found label
				lv_label_set_long_mode(child,LV_LABEL_LONG_CLIP);
				break;
			}
		}

		lv_menu_set_load_page_event(Hmi->w_menu, btn, Hmi->w_station);
		lv_obj_add_event_cb(btn, HMI_Stations_List_cb, LV_EVENT_CLICKED, Hmi);

		ESP_LOGV(TAG,"Button added with text %s",txt);
	} else {
		// Modify button
		btn = found;
		uint32_t i;
		for(i = 0; i < lv_obj_get_child_cnt(btn); i++) {
			lv_obj_t * child = lv_obj_get_child(btn, i);
			// modify label
			if(lv_obj_check_type(child, &lv_label_class)) {
				lv_label_set_text(child, txt);
				ESP_LOGD(TAG,"Button text changed to %s",txt);
				break;
			}
			// TODO : modify APRS symbol
		}
		lv_obj_set_user_data(btn,(void*)addr);
	}

	// move button at list head
	lv_obj_move_to_index(btn,0);

	if (focus)
		lv_obj_scroll_to_view(focus, LV_ANIM_OFF);

	if (!AX25_Addr_Cmp(&Hmi->trackid, addr)) {
		// Update station page

		if (AX25_Addr_Cmp(&Hmi->sta.callid, addr)) {
				bzero(&Hmi->sta, sizeof(APRS_Station_t));
		}

		if (APRS_Get_Station(Aprs, addr, &Hmi->sta)) {
			// Fallback if no stations_db
			memcpy(&Hmi->sta.callid, addr, sizeof(AX25_Addr_t));

			Hmi->sta.timestamp = Data->timestamp;
			if (Data->symbol[0]) {
				Hmi->sta.symbol[0] = Data->symbol[0];
				Hmi->sta.symbol[1] = Data->symbol[1];
			}

			if (Data->type == APRS_DTI_POS || Data->type == APRS_DTI_POS_W_TS ||
					Data->type == APRS_DTI_POS_W_MSG || Data->type == APRS_DTI_POS_W_TS_W_MSG) {
				memcpy(&Hmi->sta.position, &Data->position, sizeof(struct APRS_Position));
				
				if (Data->extension == APRS_DATA_EXT_CSE || Data->extension == APRS_DATA_EXT_WTH) {
					memcpy(&Hmi->sta.course, &Data->course, sizeof(struct APRS_Course));
				}

			}

			memcpy(Hmi->sta.status, Data->text, sizeof(Hmi->sta.status));
		} else if (Data->type != APRS_DTI_STATUS)
			// Show comment
			memcpy(Hmi->sta.status, Data->text, sizeof(Hmi->sta.status));

		HMI_Update_Station(Hmi);
	}

	memcpy(&Hmi->lastid, &Data->address[1], sizeof(AX25_Addr_t));

	HMI_UNLOCK;
}

static void HMI_Gps_Event_RMC(HMI_t * Hmi, esp_event_base_t event_base, int32_t event_id, GPS_Data_t * Data) {
	char str[32];
	int32_t old_ud, new_ud;
	HMI_LOCK;
	struct timeval tv;
	struct tm tm;

	// Use Date Time from system
	gettimeofday(&tv,NULL);
	gmtime_r(&tv.tv_sec,&tm);

	// Set Time
	old_ud = (int32_t)lv_obj_get_user_data(Hmi->time);
	new_ud = (tm.tm_hour*60 + tm.tm_min);
	if (old_ud != new_ud) {
		snprintf(str,sizeof(str),"%02d:%02d",tm.tm_hour,tm.tm_min);
		lv_label_set_text(Hmi->time,str);
		lv_obj_set_user_data(Hmi->time,(void*)new_ud);
	}

	// Set Date
	old_ud = (int32_t)lv_obj_get_user_data(Hmi->date);
	new_ud = (tm.tm_year*12 + tm.tm_mon)*31 +  tm.tm_mday;
	if (old_ud != new_ud) {
		snprintf(str,sizeof(str),"%02d/%02d/%02d",tm.tm_mday,tm.tm_mon,tm.tm_year%100);
		lv_label_set_text(Hmi->date,str);
		lv_obj_set_user_data(Hmi->date,(void*)new_ud);
	}

	// Set Locator
	if (GPS_IS_DATA_VALID(Data->valid, GPS_DATA_VALID_FIX_STATUS)) {
		if (GPS_IS_DATA_VALID(Data->valid, GPS_DATA_VALID_2D_FIX)) {
			char *old_loc = lv_label_get_text(Hmi->locator);
			struct APRS_Position pos;
			pos.latitude = Data->latitude;
			pos.longitude = Data->longitude;
			pos.ambiguity = APRS_AMBIGUITY_LOC_EXT_SQUARE;
			APRS_Position_To_Locator(&pos, str, sizeof(str));
			if (strcmp(old_loc,str))
				lv_label_set_text(Hmi->locator,str);
		}
	}

	HMI_UNLOCK;
}
