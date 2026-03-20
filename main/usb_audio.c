/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/usb_audio.c
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

#include <math.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <stdatomic.h>
#include <tusb.h>
#include <SA8x8.h>
#include "usb.h"
#include "usb_audio.h"
#include "dmabuff.h"
#include "config.h"

#ifdef TAG
#undef TAG
#endif

#define TAG "USB_AUDIO"

#define USB_AUDIO_OUT_WATERMARK	((DMABUFF_MAX_BLOCKS * RADIO_FRAME_LEN * 0) / 4)	// write at start of buffer after modem encoding
#define USB_AUDIO_IN_WATERMARK  ((DMABUFF_MAX_BLOCKS * RADIO_FRAME_LEN * 2) / 4)

#define N_SAMPLE_RATES 1


extern SA8x8_t * SA8x8;
static Dmabuff_t * sample_buff;

atomic_uint usb_in_count;
atomic_uint usb_out_count;

static const uint32_t sample_rates[AUDIO_PATH_MAX][N_SAMPLE_RATES] = {{SAMPLE_RATE},{SAMPLE_RATE}};
static struct Clock_unit {
	uint32_t sample_rate;
} Clock[AUDIO_PATH_MAX];

static struct Feature_unit {
	int8_t mute[2];
	int16_t volume[2];
} Features[AUDIO_PATH_MAX];


#include <SA8x8.h>
static void USB_AUDIO_Radio_Cb(void * pArg, struct SA8x8_Msg_S * Msg);

int USB_AUDIO_Init(void) {

	int i;

	sample_buff = SA8x8_Get_Buff(SA8x8);
	if (!sample_buff) {
		ESP_LOGE(TAG,"NULL sample_buff");
		return -1;
	}

	for (i=0;i<AUDIO_PATH_MAX;i++) {
		Clock[i].sample_rate = sample_rates[i][0];
		Features[i].mute[0] = 1;
		Features[i].mute[1] = 1;
		Features[i].volume[0] = 0;
		Features[i].volume[1] = 0;
	}

	SA8x8_Register_Cb(SA8x8,(SA8x8_Cb_t)USB_AUDIO_Radio_Cb,(void*)TAG);

	return 0;
}

// Helper for clock get requests
bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request) {
	(void)rhport;

	int path = ((request->bEntityID>>4)&0x0f);
	if (path >= AUDIO_PATH_MAX)
		return false;

	switch (request->bControlSelector) {
		case AUDIO_CS_CTRL_SAM_FREQ:
			switch (request->bRequest) {
				case AUDIO_CS_REQ_CUR:
					ESP_LOGV(TAG,"Get Curent sample rate : %ld\n",Clock[path].sample_rate);
					audio_control_cur_4_t curf = { (int32_t) tu_htole32(Clock[path].sample_rate) };
					return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
				case AUDIO_CS_REQ_RANGE:
					audio_control_range_4_n_t(N_SAMPLE_RATES) rangef =
					{
						.wNumSubRanges = tu_htole16(N_SAMPLE_RATES)
					};
					for(uint8_t i = 0; i < N_SAMPLE_RATES; i++)
					{
						ESP_LOGV(TAG,"Get sample rate range %d : %ld\n",i,sample_rates[path][i]);
						rangef.subrange[i].bMin = (int32_t) sample_rates[path][i];
						rangef.subrange[i].bMax = (int32_t) sample_rates[path][i];
						rangef.subrange[i].bRes = 0;
					}
					return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
			}
			ESP_LOGW(TAG,"Unknown request\n");
			return false;
		case AUDIO_CS_CTRL_CLK_VALID:
			if (request->bRequest == AUDIO_CS_REQ_CUR) {
				ESP_LOGV(TAG,"Get clock valid\n");
				audio_control_cur_1_t cur_valid = { .bCur = 1 };
				return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
			}
			ESP_LOGW(TAG,"Unknown request\n");
			return false;	
	}
	ESP_LOGW(TAG,"Unknown selector\n");

	return false;
}

// Helper for clock set requests
bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf) {
	(void)rhport;

	int path = ((request->bEntityID>>4) & 0x0f);
	if (path >= AUDIO_PATH_MAX)
		return false;

	switch (request->bControlSelector) {
		case AUDIO_CS_CTRL_SAM_FREQ:
			Clock[path].sample_rate = (uint32_t) ((audio_control_cur_4_t const *)buf)->bCur;
			ESP_LOGV(TAG,"Sample rate set to %ld\n",Clock[path].sample_rate);
			return true;
	}
	ESP_LOGW(TAG,"Unknown selector\n");
	return false;
}

// Helper for feature unit get requests
bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request) {
	int path = ((request->bEntityID>>4) & 0x0f);
	if (path >= AUDIO_PATH_MAX)
		return false;

	switch (request->bControlSelector) {
		case AUDIO_FU_CTRL_MUTE:
			if (request->bRequest == AUDIO_CS_REQ_CUR) {
				ESP_LOGV(TAG,"Get mute state : %hhd\n",Features[path].mute[request->bChannelNumber]);
				audio_control_cur_1_t mute1 = { .bCur = Features[path].mute[request->bChannelNumber] };
				return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
			}
			ESP_LOGW(TAG,"Unknown request\n");
			return false;
		case AUDIO_FU_CTRL_VOLUME:
			switch (request->bRequest) {
				case AUDIO_CS_REQ_RANGE:
					ESP_LOGV(TAG,"Get volume range\n");
					audio_control_range_2_n_t(1) range_vol = {
						.wNumSubRanges = 1,
						.subrange[0] = { .bMin = tu_htole16(0), tu_htole16(100), tu_htole16(1) }
					};
					return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
				case AUDIO_CS_REQ_CUR:
					ESP_LOGV(TAG,"Get volume :%d\n",Features[path].volume[request->bChannelNumber]);
					audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(Features[path].volume[request->bChannelNumber]) };
					return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
			}
			ESP_LOGW(TAG,"Unknown request\n");
			return false;
	}
	ESP_LOGW(TAG,"Unknown selector\n");
	return false;
}

// Helper for feature unit set requests
bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf) {
	int path = ((request->bEntityID>>4) & 0x0f);
	if (path >= AUDIO_PATH_MAX)
		return false;

	switch (request->bControlSelector) {
		case AUDIO_FU_CTRL_MUTE:
			if (Features[path].mute[request->bChannelNumber]^((audio_control_cur_1_t const *)buf)->bCur) {
				// mute state changed
				if (!(Features[path].mute[request->bChannelNumber] = ((audio_control_cur_1_t const *)buf)->bCur)) {
					// Mute off
					/*
					if (path == AUDIO_PATH_RECEIVER && request->bChannelNumber == 0)
						SA8x8_Start_Receiver(SA8x8);
					if (path == AUDIO_PATH_TRANSMITER && request->bChannelNumber == 0)
						SA8x8_Start_Transmiter(SA8x8);
					*/
				} else {
					// Mute On
					/*
					if (path == AUDIO_PATH_RECEIVER && request->bChannelNumber == 0)
						SA8x8_Stop_Receiver(SA8x8);
					if (path == AUDIO_PATH_TRANSMITER && request->bChannelNumber == 0)
						SA8x8_Stop_Transmiter(SA8x8);
					*/
				}
			}
			ESP_LOGV(TAG,"Set mute state on path %d, channel %d : %hhd\n",
					path,
					request->bChannelNumber,
					Features[path].mute[request->bChannelNumber]);
			return true;
		case AUDIO_FU_CTRL_VOLUME:
			Features[path].volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;
			ESP_LOGV(TAG,"Set volume :%d\n",Features[path].volume[request->bChannelNumber]);
			return true;
	}
	ESP_LOGW(TAG,"Unknown selector\n");
	return false;
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
	audio_control_request_t const *request = (audio_control_request_t const *)p_request;

	ESP_LOGV(TAG,"Get request entity : entityID = 0x%02x, selector = %d\n",request->bEntityID,request->bControlSelector);
	switch (request->bEntityID & 0x0f) {
		case AUDIO_UNIT_CLOCK:
			return tud_audio_clock_get_request(rhport, request);
		case AUDIO_UNIT_FEATURE:
			return tud_audio_feature_unit_get_request(rhport, request);
	}
	ESP_LOGW(TAG,"Unknown entityID\n");
	return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
	audio_control_request_t const *request = (audio_control_request_t const *)p_request;

	ESP_LOGD(TAG,"Set request entity : entityID = 0x%02x, selector = %d\n",request->bEntityID,request->bControlSelector);
	switch (request->bEntityID & 0x0f) {
		case AUDIO_UNIT_CLOCK:
			return tud_audio_clock_set_request(rhport, request, buf);
		case AUDIO_UNIT_FEATURE:
			return tud_audio_feature_unit_set_request(rhport, request, buf);
	}
	ESP_LOGW(TAG,"Unknown entityID\n");
	return false;
}

// OUT EP callback : Asynchronous with explicit feedback
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t *feedback_param) {
  (void) func_id;
  (void) alt_itf;
  // Set feedback method to fifo counting
  feedback_param->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
  feedback_param->sample_freq = 53000 /*SAMPLE_RATE*/;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
	(void) rhport;
	(void) p_request;
	uint8_t const itf = tu_u16_low(p_request->wIndex);
	uint8_t const alt = tu_u16_low(p_request->wValue);
	(void)alt;

	ESP_LOGV(TAG,"Interface close request: itf = %d, alt = %d",itf,alt);

	switch (itf) {
		case ITF_AUDIO_TRANSMITER:
			USB_State &= ~USB_STATE_TRANSMITER_STREAMING;
			break;

		case ITF_AUDIO_RECEIVER:
			USB_State &= ~USB_STATE_RECEIVER_STREAMING;
			break;
	}

	return true;
}

// Invoked when audio class specific set request received for an interface
bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
	(void) rhport;


	// Page 91 in UAC2 specification
	uint8_t alt = TU_U16_LOW(p_request->wValue);
	uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
	uint8_t itf = TU_U16_LOW(p_request->wIndex);
	tu_fifo_t * ff;
	size_t len;

	(void) ctrlSel;

	ESP_LOGV(TAG,"Interface set request : itf %d, alt %d, ctrl  %d",itf,alt,ctrlSel);

	switch (itf) {
		case ITF_AUDIO_TRANSMITER:
			ff = tud_audio_n_get_ep_out_ff(0);
			switch (alt) {
				case 0:
					USB_State &= ~(USB_STATE_TRANSMITER_STREAMING);
					break;
				case 1:
					tu_fifo_clear(ff);

					len  = (Dmabuff_Get_Len(sample_buff,2));
					if (len >  (USB_AUDIO_OUT_WATERMARK<<1))
						Dmabuff_Advance_Ptr(sample_buff,2, len - (USB_AUDIO_OUT_WATERMARK<<1));

					USB_State |= USB_STATE_TRANSMITER_STREAMING;
					break;
			}
			break;
		case ITF_AUDIO_RECEIVER:
			ff = tud_audio_n_get_ep_in_ff(0);
			switch (alt) {
				case 0:
					USB_State &= ~USB_STATE_RECEIVER_STREAMING;
					break;
				case 1:
					tu_fifo_clear(ff);

					len  = (Dmabuff_Get_Len(sample_buff,3));
					if (len >  (USB_AUDIO_IN_WATERMARK<<1))
						Dmabuff_Advance_Ptr(sample_buff,3, len - (USB_AUDIO_IN_WATERMARK<<1));

					USB_State |= USB_STATE_RECEIVER_STREAMING;
					break;
			}
			break;
	
	}

	return true;
}

static void USB_AUDIO_Radio_Cb(void * pArg, struct SA8x8_Msg_S * Msg) {
	size_t len, len1;
	void *ptr;

	switch (Msg->type) {
		case SA8X8_SQUELCH_OPEN:
			break;
		case SA8X8_SQUELCH_CLOSED:
			break;
		case SA8X8_PTT_PUSHED:
			break;
		case SA8X8_PTT_RELEASED:
			break;
		case SA8X8_RECEIVER_DATA:
		case SA8X8_TRANSMITER_DATA:
			if (USB_State & USB_STATE_TRANSMITER_STREAMING) {
				
				len = Dmabuff_Get_Ptr(sample_buff, 2, &ptr, &len1);

				if (len > (USB_AUDIO_OUT_WATERMARK<<1)) {
					len -= (USB_AUDIO_OUT_WATERMARK<<1);

					while (len) {
						if (len1 > len)
							len1 = len;

						len1 = tud_audio_read(ptr, len1);
						if (!len1)
							break;
						atomic_fetch_add(&usb_out_count, (len1>>1));
						len -= len1;

						Dmabuff_Next_Ptr(sample_buff, 2, len1, &ptr, &len1);
					}
				}
			}

			if (USB_State & USB_STATE_RECEIVER_STREAMING) {
				len = Dmabuff_Get_Ptr(sample_buff, 3, &ptr, &len1);
				if (len > ((USB_AUDIO_IN_WATERMARK)<<1)) {

					len -= (USB_AUDIO_IN_WATERMARK)<<1;

					while (len>0) {
						if (len1 > len)
							len1 = len;

						len1 = tud_audio_write(ptr, len1);
						if (!len1)
							break;
						atomic_fetch_add(&usb_in_count, (len1>>1));
						len -= len1;

						Dmabuff_Next_Ptr(sample_buff, 3, len1, &ptr, &len1);
					}
				}
			}
			break;
		default:
	}
}
