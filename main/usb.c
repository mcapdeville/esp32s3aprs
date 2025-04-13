/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/usb_audio_mono.h
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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/usb_phy.h"
#include "tusb.h"
#include "usb.h"
#include "usb_cdc.h"
#include "usb_audio.h"

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]     AUDIO | MIDI | HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
		_PID_MAP(MIDI, 3) | _PID_MAP(AUDIO, 4) | _PID_MAP(VENDOR, 5) )

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device = {
	.bLength            = sizeof(tusb_desc_device_t),
	.bDescriptorType    = TUSB_DESC_DEVICE,
	.bcdUSB             = 0x0200,

	// Use Interface Association Descriptor (IAD) for Audio
	// As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
	.bDeviceClass       = TUSB_CLASS_MISC,
	.bDeviceSubClass    = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol    = MISC_PROTOCOL_IAD,
	.bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

	.idVendor           = 0xCafe,
	.idProduct          = USB_PID,
	.bcdDevice          = 0x0100,

	.iManufacturer      = 0x01,
	.iProduct           = 0x02,
	.iSerialNumber      = 0x03,

	.bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void) {
	return (uint8_t const *)&desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum {
#if CFG_TUD_CDC
  ITF_CDC1_NOTIF = 0,
  ITF_CDC1_DATA,
#endif
#if CFG_TUD_AUDIO
  ITF_AUDIO_CONTROL,
  ITF_AUDIO_TRANSMITER,
  ITF_AUDIO_RECEIVER,
#endif
#if CFG_TUD_CDC >1
  ITF_CDC2_NOTIF,
  ITF_CDC2_DATA,
#endif
  ITF_NUM_TOTAL
};

enum {
	EPNUM_CONTROL = 0,
#if CFG_TUD_CDC
	EPNUM_CDC1_DATA,
#endif
#if CFG_TUD_AUDIO
	EPNUM_AUDIO_OUT,
	EPNUM_AUDIO_IN,
#endif
#if CFG_TUD_CDC >1
	EPNUM_CDC2_DATA,
#endif
#if CFG_TUD_CDC
	EPNUM_CDC1_NOTIF,
#endif
#if CFG_TUD_CDC >1
	EPNUM_CDC2_NOTIF,
#endif
};

#define CONFIG_TOTAL_LEN    	(TUD_CONFIG_DESC_LEN + ((TUD_CDC_DESC_LEN) * CFG_TUD_CDC) + ((TUD_AUDIO_DUPLEX_MONO_DESC_LEN) * CFG_TUD_AUDIO))

uint8_t const desc_configuration[] = {
	// Config number, interface count, string index, total length, attribute, power in mA
	TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL , 0, CONFIG_TOTAL_LEN, 0x00, 100),
#if CFG_TUD_CDC
	// Interface number, string index, EP Notif address, EP Notif size, EP Out & In address, EP size
	TUD_CDC_DESCRIPTOR(ITF_CDC1_NOTIF, 4, (EPNUM_CDC1_NOTIF | 0x80), 8, EPNUM_CDC1_DATA , (EPNUM_CDC1_DATA | 0x80), 64),
#endif
#if CFG_TUD_CDC >1
	// Interface number, string index, EP Notif address, EP Notif size, EP Out & In address, EP size
	TUD_CDC_DESCRIPTOR(ITF_CDC2_NOTIF, 5, (EPNUM_CDC2_NOTIF | 0x80), 8, EPNUM_CDC2_DATA , (EPNUM_CDC2_DATA | 0x80), 64),
#endif
#if CFG_TUD_AUDIO
	// string index, audio control ITF, audio out ITF, audio out EP, audio out FB EP, audio in itf, audio in EP
	TUD_AUDIO_DUPLEX_MONO_DESCRIPTOR(6, ITF_AUDIO_CONTROL, ITF_AUDIO_TRANSMITER, EPNUM_AUDIO_OUT, (EPNUM_AUDIO_OUT | 0x80), ITF_AUDIO_RECEIVER, (EPNUM_AUDIO_IN | 0x80)),
#endif
};


// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
	(void)index; // for multiple configurations
	return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const* string_desc_arr [] = {
	(const char[]) { 0x09, 0x04 },  // 0: is supported language is English (0x0409)
	"F4JMZ",                   	// 1: Manufacturer
	"ESP32S3-APRS",              	// 2: Product
	"000001",                       // 3: Serials, should use chip ID
	"Kiss",				// 4: cdc_acm kiss
	"Console",			// 5: cdc_acm console
	"Control",			// 6: Control descriptor
	"Transmiter",   	        // 7: audio Out Interface
	"Receiver",           		// 8: audio In Interface
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
	(void)langid;

	uint8_t chr_count;

	if (index == 0)	{
		memcpy(&_desc_str[1], string_desc_arr[0], 2);
		chr_count = 1;
	} else {
		// Convert ASCII string into UTF-16

		if (!(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0]))) return NULL;

		const char* str = string_desc_arr[index];

		// Cap at max char
		chr_count = (uint8_t) strlen(str);
		if (chr_count > 31) chr_count = 31;

		for (uint8_t i = 0; i < chr_count; i++) {
			_desc_str[1 + i] = str[i];
		}
	}

	// first byte is length (including header), second byte is string type
	_desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8 ) | (2*chr_count + 2));

	return _desc_str;
}

#ifndef NDEBUG
#define USB_TASK_STACK_SIZE	3072+1024	
#else
#define USB_TASK_STACK_SIZE	3072
#endif

#define USB_TASK_PRIORITY	5

#define TAG 	"USB"

struct {
	TaskHandle_t handle;
	StackType_t	stack[(USB_TASK_STACK_SIZE)/sizeof(StackType_t)];
	StaticTask_t buffer;
} USB_Task;

static usb_phy_handle_t	phy_hdl;
int USB_State;
bool usb_console;

static void USB_task(void * arg) {
	ESP_LOGI(TAG, "tinyusb task started");
	while (1) {
		tud_task_ext(portTICK_PERIOD_MS, false);
#if CFG_TUD_AUDIO && !defined(NDEBUG)
		USB_Audio_Task();	// Log audio buffer state
#endif
	}
}


int USB_Init (void) {
	// Configure USB PHY
	usb_phy_config_t phy_conf = {
		.controller = USB_PHY_CTRL_OTG,
		.otg_mode = USB_OTG_MODE_DEVICE,
		.target = USB_PHY_TARGET_INT,
	};

	USB_State = 0;

	ESP_RETURN_ON_ERROR(usb_new_phy(&phy_conf, &phy_hdl), TAG, "Install USB PHY failed");

	// init device stack on configured roothub port
	ESP_RETURN_ON_FALSE(tusb_init(), ESP_FAIL, TAG, "Init TinyUSB stack failed");

#if CFG_TUD_CDC
	USB_CDC_Init();
	USB_CDC_register_itf(0);
#if CFG_TUD_CDC >1
		USB_CDC_register_itf(1);
#endif
#endif

#if CFG_TUD_AUDIO
	USB_AUDIO_Init();
#endif

	ESP_LOGD(TAG,"Creating Task.");

	// Create a task for tinyusb device stack:
	USB_Task.handle = xTaskCreateStaticPinnedToCore(USB_task, "TinyUSB", USB_TASK_STACK_SIZE,  NULL, USB_TASK_PRIORITY, USB_Task.stack, &USB_Task.buffer, PRO_CPU_NUM);
//	xTaskCreatePinnedToCore(USB_task, "TinyUSB", USB_TASK_STACK_SIZE,  NULL, USB_TASK_PRIORITY, &USB_Task.handle, PRO_CPU_NUM);
	//xTaskCreate(USB_task, "TinyUSB", USB_TASK_STACK_SIZE,  NULL, USB_TASK_PRIORITY, &USB_Task.handle);

	ESP_RETURN_ON_FALSE(USB_Task.handle, ESP_FAIL, TAG, "create TinyUSB main task failed");

	return pdTRUE;
}

// Invoked when device is mounted
void tud_mount_cb(void) {
	USB_State = USB_STATE_ACTIVE;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
	USB_State = 0;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
	(void)remote_wakeup_en;
	USB_State = 0;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
	USB_State = USB_STATE_ACTIVE;
}


/* For some obscure reason, link order do not resolve circular dependencies */
/* between usb_audio.c and tinyusb on tud_audio_get/set_req_entity_cb symbol */
/* but dependencies are correctly resolve for usb.c file. */
/* so, this is a temporary workaround */

#if CFG_TUD_CDC > 0
#include "usb_cdc.c"
#endif

#if CFG_TUD_AUDIO >0
#include "usb_audio.c"
#endif


