/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/ax25_lm.c
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

#include "ax25_lm.h"
#include "ax25_phy.h"
#include "framebuff.h"

#include <string.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_rom_crc.h>

#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define TAG	"AX25_LM"

#define AX25_LM_EVENT_QUEUE_SIZE	10

enum AX25_Lm_State_E {
	AX25_LM_STATE_IDLE,
	AX25_LM_STATE_SEIZE_PENDING,
	AX25_LM_STATE_SEIZED
};

typedef enum AX25_Event_Id_E {
	// From physical to link multiplexer state machine
	AX25_PHY_SEIZE_CONFIRM = 0x0210,
	AX25_PHY_DATA_INDICATION,
	AX25_PHY_BUSY_INDICATION,
	AX25_PHY_QUIET_INDICATION,

	// From data-link layer to link multiplexer
	AX25_LM_SEIZE_REQUEST = 0x0220,
	AX25_LM_DATA_REQUEST,
	AX25_LM_RELEASE_REQUEST,
	AX25_LM_EXPEDITED_DATA_REQUEST,

	AX25_LM_MAX_EVENT_ID = 0x02FF  // Highest link multiplexer  event id
} AX25_Event_Id_t;


typedef struct AX25_Dl_S AX25_Dl_t;
typedef struct AX25_Dl_List_S AX25_Dl_List_t;

typedef struct AX25_Lm_Event_S {
	enum AX25_Event_Id_E id;
	void * arg;
} AX25_Lm_Event_t;

struct AX25_Dl_List_S {
	AX25_Dl_List_t * next;		// List of all registered DL
	AX25_Dl_List_t * next_used;	// List of used DL (awaiting or served)
	enum {
		NOWHERE = 0,
		AWAITING,
		CURRENT,
		SERVED
	} state;			// State of DL
	
	// DL interface
	void * dl_ctx;			// DL context
	AX25_Lm_Cbs_t dl;		// DL callbacks
	
	QueueHandle_t event_queue;	// DL event queue

	AX25_Addr_t *filter;
	int filter_len;
};

typedef struct AX25_Lm_Impl_S {
	AX25_Lm_t ax25_lm; 

	// PHY interface
	AX25_Phy_t * ax25_phy;
	uint32_t received;
	uint32_t good_crc;

	// Link Multiplexer
	SemaphoreHandle_t lm_lock;
	enum AX25_Lm_State_E lm_state;
	bool current_queue_proc;

	// Datalinks
	AX25_Dl_List_t * dl_list;	// list of registered data link machine
	AX25_Dl_List_t * awaiting_list;	// list of awaiting to be served dl machine (link by next_used ptr, state = AWAITING)
	AX25_Dl_List_t * served_list;	// list of allready served machine (link by next_used ptr, state = SERVED)
	AX25_Dl_List_t * current_dl;	// Current dl machine (state = CURRENT)
} AX25_Lm_Impl_t;


// LM ops
static int AX25_Lm_Impl_Register_Dl(AX25_Lm_Impl_t * Lm, void * Ctx, AX25_Lm_Cbs_t * Cbs, AX25_Addr_t *Filter);
static int AX25_Lm_Impl_Unregister_Dl(AX25_Lm_Impl_t * Lm, void * Ctx);
static int AX25_Lm_Impl_Seize_Request(AX25_Lm_Impl_t * Lm, void *Ctx);
static int AX25_Lm_Impl_Release_Request(AX25_Lm_Impl_t * Lm, void * Ctx);
static int AX25_Lm_Impl_Expedited_Data_Request(AX25_Lm_Impl_t * Lm, void * Ctx, Frame_t * Frame);
static int AX25_Lm_Impl_Data_Request(AX25_Lm_Impl_t * Lm, void * Ctx, Frame_t * Frame);

const AX25_Lm_Ops_t Ax25_Lm_Impl_Ops = {
	.register_dl = (typeof(Ax25_Lm_Impl_Ops.register_dl))AX25_Lm_Impl_Register_Dl,
	.unregister_dl = (typeof(Ax25_Lm_Impl_Ops.unregister_dl))AX25_Lm_Impl_Unregister_Dl,
	.seize_request = (typeof(Ax25_Lm_Impl_Ops.seize_request))AX25_Lm_Impl_Seize_Request,
	.release_request = (typeof(Ax25_Lm_Impl_Ops.release_request))AX25_Lm_Impl_Release_Request,
	.expedited_data_request = (typeof(Ax25_Lm_Impl_Ops.expedited_data_request))AX25_Lm_Impl_Expedited_Data_Request,
	.data_request = (typeof(Ax25_Lm_Impl_Ops.data_request))AX25_Lm_Impl_Data_Request
};

// LM Callbacks
static int AX25_Lm_Impl_Phy_Seize_Confirm_Cb(AX25_Lm_Impl_t * Lm);
static int AX25_Lm_Impl_Phy_Data_Indication_Cb(AX25_Lm_Impl_t * Lm, Frame_t * Frame);
static int AX25_Lm_Impl_Phy_Busy_Indication_Cb(AX25_Lm_Impl_t * Lm);
static int AX25_Lm_Impl_Phy_Quiet_Indication_Cb(AX25_Lm_Impl_t * Lm);

AX25_Lm_t * AX25_Lm_Impl_Init(AX25_Phy_t * Phy) {
	AX25_Lm_Impl_t * lm;

	if (!(lm = malloc(sizeof(AX25_Lm_Impl_t)))) {
		ESP_LOGE(TAG,"Error allocating AX25_Lm struct");
		return NULL;
	}
	bzero(lm,sizeof(AX25_Lm_Impl_t));
	lm->ax25_lm.ops = &Ax25_Lm_Impl_Ops;
	lm->ax25_phy = Phy;

	if (!(lm->lm_lock = xSemaphoreCreateMutex())) {
		ESP_LOGE(TAG,"Error creating transmit semaphore");
		free(lm);
		return NULL;
	}

	AX25_Phy_Cbs_t cbs = {
		.seize_confirm = (typeof(cbs.seize_confirm))AX25_Lm_Impl_Phy_Seize_Confirm_Cb,
		.data_indication = (typeof(cbs.data_indication))AX25_Lm_Impl_Phy_Data_Indication_Cb,
		.busy_indication = (typeof(cbs.busy_indication))AX25_Lm_Impl_Phy_Busy_Indication_Cb,
		.quiet_indication = (typeof(cbs.quiet_indication))AX25_Lm_Impl_Phy_Quiet_Indication_Cb
	};
	if (AX25_Phy_Register_Cbs(lm->ax25_phy, lm, &cbs))
		ESP_LOGE(TAG,"Error registerring lm callback with phy");

	return (AX25_Lm_t*)lm;
}
// Link multiplexer helper funtions

static bool AX25_Lm_Digipeat(AX25_Lm_Impl_t * Lm, Frame_t * Frame) {
	// return AX25_Phy_Expedited_Data_Request(Lm->ax25_phy, Frame);
	return false;
}

static int AX25_Lm_Finish(AX25_Lm_Impl_t * Lm) {
	AX25_Phy_Release_Request(Lm->ax25_phy);

	// Place current dl in served list
	Lm->current_dl->state = SERVED;
	Lm->current_dl->next_used = Lm->served_list;
	Lm->served_list = Lm->current_dl;
	Lm->current_dl = NULL;

	Lm->lm_state = AX25_LM_STATE_IDLE;

	return 0;
}

static bool AX25_Lm_Event_Proc(AX25_Lm_Impl_t * Lm) {
	AX25_Dl_List_t * dl_list;
	AX25_Lm_Event_t lm_event;
	int ret;

	xSemaphoreTake(Lm->lm_lock,portMAX_DELAY);

	switch (Lm->lm_state) {
		case AX25_LM_STATE_IDLE:
			if (!Lm->awaiting_list && Lm->served_list) {
				ESP_LOGD(TAG,"State : %d : Served list --> Awaiting list",Lm->lm_state);
				// Transfer DL from served to awaiting list
				dl_list = Lm->served_list;
				while (dl_list) {
					if (uxQueueMessagesWaiting(dl_list->event_queue)) {
						do
							ret = xQueueReceive(dl_list->event_queue,&lm_event,0);
						while (ret == pdPASS && lm_event.id == AX25_LM_RELEASE_REQUEST);
						if (ret != pdPASS) {
							AX25_Dl_List_t * tmp;
							// DL queue empty so remove from awaiting queue
							tmp = dl_list->next_used;
							dl_list->state = NOWHERE;
							dl_list->next_used = NULL;
							dl_list = tmp;
						} else {
							AX25_Dl_List_t * tmp;
							// Not empty so requeue event
							xQueueSendToFront(dl_list->event_queue,&lm_event,portMAX_DELAY);
							tmp = dl_list->next_used;
							// and put in awaiting queue
							dl_list->state = AWAITING;
							dl_list->next_used = Lm->awaiting_list;
							Lm->awaiting_list = dl_list;
							dl_list = tmp;
						}
					} else {
						AX25_Dl_List_t * tmp;
						// DL queue empty so remove from awaiting queue
						tmp = dl_list->next_used;
						dl_list->state = NOWHERE;
						dl_list->next_used = NULL;
						dl_list = tmp;
					}
				}
				Lm->served_list = NULL;
			}
			if (!Lm->awaiting_list) {
				ESP_LOGD(TAG,"State : %d : Awaiting list empty",Lm->lm_state);
				break;
			}
			// Get Next DL from awaiting list
			ESP_LOGD(TAG,"State : %d : Get DL from awaiting list",Lm->lm_state);
			dl_list = Lm->awaiting_list;
			Lm->awaiting_list = dl_list->next_used;
			dl_list->state = CURRENT;
			dl_list->next_used = NULL;
			Lm->current_dl = dl_list;

			if (xQueueReceive(dl_list->event_queue,&lm_event,0) != pdPASS) {
				ESP_LOGE(TAG,"Queue empty getting current dl from awaiting list");
				Lm->awaiting_list = dl_list->next_used;
				dl_list->state = NOWHERE;
				dl_list->next_used = NULL;
				break;
			}
			if (lm_event.id != AX25_LM_SEIZE_REQUEST) {
				// Requeue event
				xQueueSendToFront(dl_list->event_queue,&lm_event,portMAX_DELAY);
				Lm->current_queue_proc = false;
			} else
				Lm->current_queue_proc = true;

			AX25_Phy_Seize_Request(Lm->ax25_phy);

			Lm->lm_state = AX25_LM_STATE_SEIZE_PENDING;
			ESP_LOGD(TAG,"State : %d",Lm->lm_state);
			/* FALLTRU */
		case AX25_LM_STATE_SEIZE_PENDING:
			while (Lm->current_queue_proc && (ret = xQueueReceive(Lm->current_dl->event_queue,&lm_event,0)) == pdPASS) {
				ESP_LOGD(TAG,"State : %d Event %x",Lm->lm_state,lm_event.id);
				switch (lm_event.id) {
					case AX25_LM_RELEASE_REQUEST:
						AX25_Lm_Finish(Lm);
						Lm->current_queue_proc = false;
						Lm->lm_state = AX25_LM_STATE_IDLE;
						break;
					case AX25_LM_DATA_REQUEST:
					case AX25_LM_EXPEDITED_DATA_REQUEST:
						// Requeue event
						xQueueSendToFront(Lm->current_dl->event_queue,&lm_event,portMAX_DELAY);
						Lm->current_queue_proc = false;
						break;
					default:
					}
			}
			break;
		case AX25_LM_STATE_SEIZED:
			ret = pdPASS;
			while (Lm->current_queue_proc && (ret = xQueueReceive(Lm->current_dl->event_queue,&lm_event,0)) == pdPASS) {
				ESP_LOGD(TAG,"State : %d Event %x",Lm->lm_state,lm_event.id);
				switch (lm_event.id) {
					case AX25_LM_SEIZE_REQUEST:
						if (Lm->current_dl->dl.seize_confirm)
							Lm->current_dl->dl.seize_confirm(Lm->current_dl->dl_ctx);
						break;
					case AX25_LM_RELEASE_REQUEST:
						AX25_Lm_Finish(Lm);
						Lm->current_queue_proc = false;
						Lm->lm_state = AX25_LM_STATE_IDLE;
						break;
					case AX25_LM_DATA_REQUEST:
						AX25_Phy_Data_Request(Lm->ax25_phy, lm_event.arg);
						Framebuff_Free_Frame(lm_event.arg);
						break;
					case AX25_LM_EXPEDITED_DATA_REQUEST:
						AX25_Phy_Expedited_Data_Request(Lm->ax25_phy, lm_event.arg);
						Framebuff_Free_Frame(lm_event.arg);
						break;
					default:
					}
			}
			if (ret != pdPASS) {
				AX25_Lm_Finish(Lm);
				Lm->current_queue_proc = false;
				Lm->lm_state = AX25_LM_STATE_IDLE;
			}
			break;
	}

	xSemaphoreGive(Lm->lm_lock);

	return false;
}

// LM helper function
static AX25_Dl_List_t * AX25_Lm_Get_Dl_List(AX25_Lm_Impl_t * Lm, void *Ctx) {
	AX25_Dl_List_t * dl_list = Lm->dl_list;

	while (dl_list) {
		ESP_LOGD(TAG,"Dl list %p with conttext %p and next %p",dl_list,dl_list->dl_ctx,dl_list->next);
		if (dl_list->dl_ctx == Ctx)
			break;
		dl_list = dl_list->next;
	}

	return dl_list;
}

static int AX25_Lm_Queue_Event(AX25_Lm_Impl_t * Lm, AX25_Dl_List_t * Dl_List, AX25_Lm_Event_t *Event) {
	if (Dl_List->state == NOWHERE) {
		// Add to awaiting list
		Dl_List->state = AWAITING;
		Dl_List->next_used = Lm->awaiting_list;
		Lm->awaiting_list = Dl_List;
	}

	if (xQueueSend(Dl_List->event_queue,Event,portMAX_DELAY) != pdPASS) {
		ESP_LOGW(TAG,"Error adding event to queue");
		return -1;
	}

	return 0;
}

// Lm ops
static int AX25_Lm_Impl_Seize_Request(AX25_Lm_Impl_t * Lm, void *Ctx) {
	AX25_Dl_List_t * dl_list;
	AX25_Lm_Event_t event = {
		.id = AX25_LM_SEIZE_REQUEST,
	};

	xSemaphoreTake(Lm->lm_lock,portMAX_DELAY);

	if (!(dl_list = AX25_Lm_Get_Dl_List(Lm, Ctx))) {
		xSemaphoreGive(Lm->lm_lock);
		ESP_LOGE(TAG,"Dl not found in list for seize request");
		return -1;
	}

	if (AX25_Lm_Queue_Event(Lm, dl_list, &event)) {
		xSemaphoreGive(Lm->lm_lock);
		ESP_LOGW(TAG,"Error adding seize request event queue");
		return -1;
	}

	xSemaphoreGive(Lm->lm_lock);

	AX25_Lm_Event_Proc(Lm);

	return 0;
}

static int AX25_Lm_Impl_Release_Request(AX25_Lm_Impl_t * Lm, void * Ctx) {
	AX25_Dl_List_t * dl_list;
	AX25_Lm_Event_t event = {
		.id = AX25_LM_RELEASE_REQUEST,
	};

	xSemaphoreTake(Lm->lm_lock,portMAX_DELAY);

/*	if (Lm->state == AX25_LM_STATE_IDLE)
		xSemaphoreGive(Ax25->lm_lock);
		return 0;
*/
	if (!(dl_list = AX25_Lm_Get_Dl_List(Lm, Ctx))) {
		xSemaphoreGive(Lm->lm_lock);
		ESP_LOGE(TAG,"Dl not found in list for release request");
		return -1;
	}

	if (AX25_Lm_Queue_Event(Lm, dl_list, &event)) {
		xSemaphoreGive(Lm->lm_lock);
		ESP_LOGW(TAG,"Error adding release request event queue");
		return -1;
	}
	
	xSemaphoreGive(Lm->lm_lock);

	AX25_Lm_Event_Proc(Lm);

	return 0;	
}

static int AX25_Lm_Impl_Expedited_Data_Request(AX25_Lm_Impl_t * Lm, void * Ctx, Frame_t * Frame) {
	AX25_Lm_Event_t event = {
		.id = AX25_LM_EXPEDITED_DATA_REQUEST,
		.arg = Frame
	};
	AX25_Dl_List_t * dl_list;

	xSemaphoreTake(Lm->lm_lock,portMAX_DELAY);

	if (!(dl_list = AX25_Lm_Get_Dl_List(Lm, Ctx))) {
		xSemaphoreGive(Lm->lm_lock);
		ESP_LOGE(TAG,"Dl not found in list for expedited data request");
		return -1;
	}

	Framebuff_Inc_Frame_Usage(Frame);
	if (AX25_Lm_Queue_Event(Lm, dl_list, &event)) {
		Framebuff_Free_Frame(Frame);
		xSemaphoreGive(Lm->lm_lock);
		ESP_LOGW(TAG,"Error adding expedited data request event queue");
		return -1;
	}

	xSemaphoreGive(Lm->lm_lock);
	
	AX25_Lm_Event_Proc(Lm);

	return 0;
}

static int AX25_Lm_Impl_Data_Request(AX25_Lm_Impl_t * Lm, void * Ctx, Frame_t * Frame) {
	AX25_Dl_List_t * dl_list;
	AX25_Lm_Event_t event = {
		.id = AX25_LM_DATA_REQUEST,
		.arg = Frame
	};

	xSemaphoreTake(Lm->lm_lock,portMAX_DELAY);

	if (!(dl_list = AX25_Lm_Get_Dl_List(Lm, Ctx))) {
		xSemaphoreGive(Lm->lm_lock);
		ESP_LOGE(TAG,"Dl not found in list for data request");
		return -1;
	}

	Framebuff_Inc_Frame_Usage(Frame);
	if (AX25_Lm_Queue_Event(Lm, dl_list, &event)) {
		Framebuff_Free_Frame(Frame);
		xSemaphoreGive(Lm->lm_lock);
		ESP_LOGW(TAG,"Error adding data request event queue");
		return -1;
	}

	xSemaphoreGive(Lm->lm_lock);

	AX25_Lm_Event_Proc(Lm);

	return 0;
}

// Phy Callbacks
static int AX25_Lm_Impl_Phy_Seize_Confirm_Cb(AX25_Lm_Impl_t * Lm) {
	switch (Lm->lm_state) {
		case AX25_LM_STATE_IDLE:
			AX25_Phy_Release_Request(Lm->ax25_phy);
			break;
		case AX25_LM_STATE_SEIZE_PENDING:
			xSemaphoreTake(Lm->lm_lock,portMAX_DELAY);

			Lm->lm_state = AX25_LM_STATE_SEIZED;
			Lm->current_queue_proc = true;

			xSemaphoreGive(Lm->lm_lock);

			if (Lm->current_dl->dl.seize_confirm)
				Lm->current_dl->dl.seize_confirm(Lm->current_dl->dl_ctx);

			AX25_Lm_Event_Proc(Lm);

			break;
		case AX25_LM_STATE_SEIZED:
			// Discard
			break;
	}

	return 0;
}

static int AX25_Lm_Impl_Phy_Data_Indication_Cb(AX25_Lm_Impl_t * Lm, Frame_t * Frame) {
	AX25_Dl_List_t *next;

	if (!Frame)
		return -1;

	Lm->received++;
	if (0x0f47 == esp_rom_crc16_le(0x0,Frame->frame,Frame->frame_len)) {
		Lm->good_crc++;
		// SHOULD check for duplicate frame here
		
		ESP_LOGD(TAG,"Frame crc OK");

		// Manage digipeating
		AX25_Lm_Digipeat(Lm,Frame);

		xSemaphoreTake(Lm->lm_lock,portMAX_DELAY);
		// Lookup to which DL send frame to if any based on src and dst address
		next = Lm->dl_list;
		while (next) {
	//		if (AX25_Addr_Filter((AX25_Addr_t*)Frame->frame, next->filter)) {
				// DL found (one packet may match multiple DL)
				// TODO : May trigger a race condition if this generate LM request
				if (next->dl.data_indication)
					next->dl.data_indication(next->dl_ctx, Frame);
	//		}

			next = next->next;
		}

		xSemaphoreGive(Lm->lm_lock);
	} else
		ESP_LOGD(TAG,"Frame crc error");

	ESP_LOGD(TAG,"%lu/%lu frame received with good crc", Lm->good_crc, Lm->received);

	return 0;
}

static int AX25_Lm_Impl_Phy_Busy_Indication_Cb(AX25_Lm_Impl_t * Lm) {
	AX25_Dl_List_t * dl_list;

	xSemaphoreTake(Lm->lm_lock,portMAX_DELAY);
	dl_list = Lm->dl_list;
	while (dl_list) {
		if (Lm->dl_list->dl.busy_indication)
			Lm->dl_list->dl.busy_indication(dl_list->dl_ctx);
		dl_list = dl_list->next;
	}
	xSemaphoreGive(Lm->lm_lock);

	return 0;
}

static int AX25_Lm_Impl_Phy_Quiet_Indication_Cb(AX25_Lm_Impl_t * Lm) {
	AX25_Dl_List_t * dl_list;

	xSemaphoreTake(Lm->lm_lock,portMAX_DELAY);
	dl_list = Lm->dl_list;
	while (dl_list) {
		if (Lm->dl_list->dl.quiet_indication)
			Lm->dl_list->dl.quiet_indication(dl_list->dl_ctx);
		dl_list = dl_list->next;
	}
	xSemaphoreGive(Lm->lm_lock);

	return 0;
}

// Register lm callbacks
static int AX25_Lm_Impl_Register_Dl(AX25_Lm_Impl_t * Lm, void * Ctx, AX25_Lm_Cbs_t * Cbs, AX25_Addr_t * Filter) {
	AX25_Dl_List_t * dl_list;

	if (!Ctx || !Cbs) 
		return -1;

	xSemaphoreTake(Lm->lm_lock,portMAX_DELAY);

	if (!AX25_Lm_Get_Dl_List(Lm, Ctx)) {
		if (!(dl_list = malloc(sizeof(AX25_Dl_List_t)))) {
			xSemaphoreGive(Lm->lm_lock);
			ESP_LOGE(TAG,"Error allocating DL list element");
			return -1;
		}
		bzero(dl_list,sizeof(AX25_Dl_List_t));

		if (!(dl_list->event_queue = xQueueCreate(AX25_LM_EVENT_QUEUE_SIZE,sizeof(struct AX25_Lm_Event_S)))) {
			xSemaphoreGive(Lm->lm_lock);
			ESP_LOGE(TAG,"Error creating event queue");
			free(dl_list);
			return -1;
		}

		memcpy(&dl_list->dl, Cbs, sizeof(AX25_Lm_Cbs_t));
		dl_list->dl_ctx = Ctx;

		if (Filter && Filter->addr[0]) {
			int c = AX25_Addr_Count(Filter);
			if (c) {
				if (!(dl_list->filter = malloc(sizeof(AX25_Addr_t)*c))) {
					ESP_LOGE(TAG,"Error alocating filters");
					free(dl_list);
					return -1;
				}
				memcpy(&dl_list->filter,Filter, sizeof(AX25_Addr_t)*c);
				dl_list->filter_len = c;
			}
		}

		dl_list->next = Lm->dl_list;
		Lm->dl_list = dl_list;

		ESP_LOGD(TAG,"Registered dl_list %p with context %p and next %p",dl_list,dl_list->dl_ctx,dl_list->next);
	} else
		ESP_LOGW(TAG,"Data link already registered");

	xSemaphoreGive(Lm->lm_lock);

	return 0;
}

static int AX25_Lm_Impl_Unregister_Dl(AX25_Lm_Impl_t * Lm, void * Ctx) {
	AX25_Dl_List_t * dl_list, ** prev;

	if (!Ctx)
		return -1;
	
	xSemaphoreTake(Lm->lm_lock,portMAX_DELAY);
	prev = &Lm->dl_list;
	dl_list = *prev;
	while (dl_list && dl_list->dl_ctx != Ctx) {
		prev = &dl_list->next;
		dl_list = *prev;
	}

	if (dl_list) {
		if (dl_list->state == NOWHERE) {
			*prev = dl_list->next;
			vQueueDelete(dl_list->event_queue);
			if (dl_list->filter)
				free(dl_list->filter);
			free(dl_list);
		} else {
			xSemaphoreGive(Lm->lm_lock);
			ESP_LOGE(TAG,"Dl in use while unregistering");
			return -1;
		}
	} else {
		xSemaphoreGive(Lm->lm_lock);
		ESP_LOGE(TAG,"Dl not found while unregistering");
		return -1;
	}

	xSemaphoreGive(Lm->lm_lock);

	return 0;
}
