/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/ax25_phy_simplex.c
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

#define _AX25_PHY_PRIV_INCLUDE_
#include "ax25_phy.h"
#include "modem.h"
#include "framebuff.h"

#include <string.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_rom_crc.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define TAG	"AX25_PHY"

#ifndef NDEBUG
#define AX25_PHY_TASK_STACK_SIZE	2048+1024
#else
#define AX25_PHY_TASK_STACK_SIZE	2048
#endif

#define AX25_PHY_TASK_PRIORITY		4

#define AX25_PHY_EVENT_QUEUE_SIZE	10
#define AX25_PHY_PRIORITY_QUEUE_SIZE	5
#define AX25_PHY_NORMAL_QUEUE_SIZE	15

#define AX25_PHY_OPS_TO			30
#define AX25_PHY_CB_TO			0

// Timer default values
#define AX25_DEF_AXHANG_TIMER		50		// T100 : Repeater hang
#define AX25_DEF_PRIACK_TIMER		100		// T101 : Priority window
#define AX25_DEF_SLOT_TIMER		100		// T102 : Slot time
#define AX25_DEF_TXDELAY_TIMER		900		// T103 : Transmiter startup
#define AX25_DEF_AXDELAY_TIMER		100		// T104 : Repeater startup
#define AX25_DEF_REMOTESYNC_TIMER	200		// T105 : Remote receiver sync
#define AX25_DEF_TENMIN_TIMER		(10*60*1000)	// T106 : Ten minutes transmition limit
#define AX25_DEF_ANTIHOG_TIMER		(10*1000)	// T107 : Anti-hogging limit
#define AX25_DEF_RECEIVER_TIMER		50		// T108 : Receiver startup

#define AX25_DEF_P			(0.63f)		// p-persistance value 0<p<1.0
#define AX25_DEF_PERSISTANCE		((uint8_t)(((float)UINT8_MAX)+1.0f)*AX25_DEF_P)

enum AX25_Phy_Simplex_State_E {
	AX25_PHY_STATE_READY = 0,
	AX25_PHY_STATE_RECEIVING,
	AX25_PHY_STATE_TRANSMITER_SUPPRESSION,
	AX25_PHY_STATE_TRANSMITER_START,
	AX25_PHY_STATE_TRANSMITING,
	AX25_PHY_STATE_DIGIPEATING,
	AX25_PHY_STATE_RECEIVER_START
};

typedef enum AX25_Phy_Simplex_Event_Id_E {
	// physical state machine timers
	AX25_PHY_TIMER_T100_EXPIRY = 0x0100,
	AX25_PHY_TIMER_T101_EXPIRY,
	AX25_PHY_TIMER_T102_EXPIRY,
	AX25_PHY_TIMER_T103_EXPIRY,
	AX25_PHY_TIMER_T104_EXPIRY,
	AX25_PHY_TIMER_T105_EXPIRY,
	AX25_PHY_TIMER_T106_EXPIRY,
	AX25_PHY_TIMER_T107_EXPIRY,
	AX25_PHY_TIMER_T108_EXPIRY,

	// From radio to physical state machine
	AX25_RA_AOS_INDICATION = 0x0110,	// Acquisition of signal
	AX25_RA_LOS_INDICATION,		// Lost of signal
	AX25_RA_DATA_INDICATION,	// Frame received
	AX25_RA_FRAME_SENT,			// Frame sent

	// From link layer to pysical state machine
	AX25_PHY_EXPEDITED_DATA_REQUEST = 0x120,	// to priority queue
	AX25_PHY_SEIZE_REQUEST,		// to normal queue
	AX25_PHY_DATA_REQUEST,		// to normal queue
	AX25_PHY_RELEASE_REQUEST,	// to normal queue
	
	AX25_PHY_MAX_EVENT_ID = 0x01FF  // Highest physical event id
} AX25_Phy_Simplex_Event_Id_t;

typedef struct AX25_Phy_Simplex_Event_S {
	enum AX25_Phy_Simplex_Event_Id_E id;
	void * arg;
} AX25_Phy_Simplex_Event_t;

typedef struct AX25_Phy_Simplex_S {
	struct AX25_Phy_S ax25_phy;
	Modem_t * modem;

	TaskHandle_t phy_task;
	// Transmiter interface
	SemaphoreHandle_t transmit_done;
	Frame_t * outstanding_frame;

	// Physical layer
	enum AX25_Phy_Simplex_State_E phy_state;
	QueueHandle_t phy_event_queue;
	QueueHandle_t phy_prio_queue;
	QueueHandle_t phy_normal_queue;
	bool phy_prio_queue_processing;
	bool phy_normal_queue_processing;
	uint8_t persistance;
	bool digipeating;
	bool repeaterup;
	bool interrupted;
	TimerHandle_t t100;
	TimerHandle_t t101;
	TimerHandle_t t102;
	TimerHandle_t t103;
	TimerHandle_t t104;
	TimerHandle_t t105;
	TimerHandle_t t106;
	TimerHandle_t t107;
	TimerHandle_t t108;
} AX25_Phy_Simplex_t;

// Phy ops
static int AX25_Phy_Simplex_Seize_Request(AX25_Phy_Simplex_t * Phy);
static int AX25_Phy_Simplex_Release_Request(AX25_Phy_Simplex_t * Phy);
static int AX25_Phy_Simplex_Expedited_Data_Request(AX25_Phy_Simplex_t * Phy, Frame_t * Frame);
static int AX25_Phy_Simplex_Data_Request(AX25_Phy_Simplex_t * Phy, Frame_t * Frame);

const AX25_Phy_Ops_t Ax25_Phy_Simplex_Ops = {
	.seize_request = (typeof(Ax25_Phy_Simplex_Ops.seize_request))AX25_Phy_Simplex_Seize_Request,
	.release_request = (typeof(Ax25_Phy_Simplex_Ops.release_request))AX25_Phy_Simplex_Release_Request,
	.expedited_data_request = (typeof(Ax25_Phy_Simplex_Ops.expedited_data_request))AX25_Phy_Simplex_Expedited_Data_Request,
	.data_request = (typeof(Ax25_Phy_Simplex_Ops.data_request))AX25_Phy_Simplex_Data_Request
};

// Transmiter callback
static int AX25_Phy_Simplex_Frame_Sent_Cb(AX25_Phy_Simplex_t * Phy,Frame_t *);

// Receiver callback
static int AX25_Phy_Simplex_Frame_Received_Cb(AX25_Phy_Simplex_t * Phy, Frame_t *);
static int AX25_Phy_Simplex_Dcd_Changed_Cb(AX25_Phy_Simplex_t * Phy, bool Dcd);

const Modem_Cbs_t AX25_Phy_Simplex_Modem_Cbs = {
	.frame_sent = (typeof(AX25_Phy_Simplex_Modem_Cbs.frame_sent))AX25_Phy_Simplex_Frame_Sent_Cb,
	.frame_received = (typeof(AX25_Phy_Simplex_Modem_Cbs.frame_received))AX25_Phy_Simplex_Frame_Received_Cb,
	.dcd_changed = (typeof(AX25_Phy_Simplex_Modem_Cbs.dcd_changed))AX25_Phy_Simplex_Dcd_Changed_Cb
};

// Timers callback
static void AX25_Phy_Simplex_Timer_Event(TimerHandle_t Timer);

// Phy task
void AX25_Phy_Simplex_Task(AX25_Phy_Simplex_t * Phy);

// Create an AX25 physical layer attached to a modem
AX25_Phy_Simplex_t * AX25_Phy_Simplex_Init(Modem_t * Modem) {
	AX25_Phy_Simplex_t * phy;

	if (!Modem)
		return NULL;

	if (!(phy = malloc(sizeof(AX25_Phy_Simplex_t)))) {
		ESP_LOGE(TAG,"Error allocating AX25 struct");
		return NULL;
	}
	bzero(phy,sizeof(AX25_Phy_Simplex_t));
	phy->ax25_phy.ops = &Ax25_Phy_Simplex_Ops;
	phy->modem  = Modem;

	if (!(phy->phy_event_queue = xQueueCreate(AX25_PHY_EVENT_QUEUE_SIZE,sizeof(struct AX25_Phy_Simplex_Event_S)))) {
		ESP_LOGE(TAG,"Error creating event queue");
		free(phy);
		return NULL;
	}

	if (!(phy->phy_prio_queue = xQueueCreate(AX25_PHY_PRIORITY_QUEUE_SIZE,sizeof(struct AX25_Phy_Simplex_Event_S)))) {
		ESP_LOGE(TAG,"Error creating priority queue");
		vQueueDelete(phy->phy_event_queue);
		free(phy);
		return NULL;
	}

	if (!(phy->phy_normal_queue = xQueueCreate(AX25_PHY_NORMAL_QUEUE_SIZE,sizeof(struct AX25_Phy_Simplex_Event_S)))) {
		ESP_LOGE(TAG,"Error creating normal queue");
		vQueueDelete(phy->phy_prio_queue);
		vQueueDelete(phy->phy_event_queue);
		free(phy);
		return NULL;
	}

	if (!(phy->transmit_done = xSemaphoreCreateBinary())) {
		ESP_LOGE(TAG,"Error creating transmit done semaphore");
		vQueueDelete(phy->phy_normal_queue);
		vQueueDelete(phy->phy_prio_queue);
		vQueueDelete(phy->phy_event_queue);
		free(phy);
		return NULL;
	}

	phy->t100 = xTimerCreate("AXHANG",		pdMS_TO_TICKS(AX25_DEF_AXHANG_TIMER ), false, phy, AX25_Phy_Simplex_Timer_Event);
	phy->t101 = xTimerCreate("PRIACK",		pdMS_TO_TICKS(AX25_DEF_PRIACK_TIMER ), false, phy, AX25_Phy_Simplex_Timer_Event);
	phy->t102 = xTimerCreate("SLOTTIME",	pdMS_TO_TICKS(AX25_DEF_SLOT_TIMER   ), false, phy, AX25_Phy_Simplex_Timer_Event);
	phy->t103 = xTimerCreate("TXDELAY",	pdMS_TO_TICKS(AX25_DEF_TXDELAY_TIMER), false, phy, AX25_Phy_Simplex_Timer_Event);
	phy->t104 = xTimerCreate("AXDELAY",	pdMS_TO_TICKS(AX25_DEF_AXDELAY_TIMER), false, phy, AX25_Phy_Simplex_Timer_Event);
	phy->t105 = xTimerCreate("REMOTESYNC",	pdMS_TO_TICKS(AX25_DEF_REMOTESYNC_TIMER), false, phy, AX25_Phy_Simplex_Timer_Event);
	phy->t106 = xTimerCreate("TENMINUTE",	pdMS_TO_TICKS(AX25_DEF_TENMIN_TIMER), false, phy, AX25_Phy_Simplex_Timer_Event);
	phy->t107 = xTimerCreate("ANTIHOG",	pdMS_TO_TICKS(AX25_DEF_ANTIHOG_TIMER), false, phy, AX25_Phy_Simplex_Timer_Event);
	phy->t108 = xTimerCreate("RECEIVER",	pdMS_TO_TICKS(AX25_DEF_RECEIVER_TIMER), false, phy, AX25_Phy_Simplex_Timer_Event);

	phy->persistance = AX25_DEF_PERSISTANCE;

	if (Modem_Register_Cbs(Modem, phy, &AX25_Phy_Simplex_Modem_Cbs)) {
			ESP_LOGE(TAG,"Error registering modem callbacks");
	}
	
	xTaskCreate((void(*)(void*))AX25_Phy_Simplex_Task,TAG,AX25_PHY_TASK_STACK_SIZE,(void*)phy,AX25_PHY_TASK_PRIORITY,&phy->phy_task);

	return phy;
}

// Physical layer Helper functions

static void AX25_Phy_Simplex_Stop_Transmiter(AX25_Phy_Simplex_t * Phy) {
	Modem_Stop_Transmiter(Phy->modem);

	Phy->phy_prio_queue_processing = false;
	Phy->phy_normal_queue_processing = false;

	xTimerStart(Phy->t108,1);

	Phy->phy_state = AX25_PHY_STATE_RECEIVER_START;
}

static void AX25_Phy_Simplex_Start_Transmiter(AX25_Phy_Simplex_t * Phy) {

	Phy->phy_prio_queue_processing = false;
	Phy->phy_normal_queue_processing = false;

	xTimerStop(Phy->t101,1);
	xTimerStop(Phy->t102,1);
	xTimerStop(Phy->t103,1);
	xTimerStop(Phy->t104,1);
	xTimerStop(Phy->t105,1);
	xTimerStop(Phy->t106,1);
	xTimerStop(Phy->t107,1);

	xTimerStart(Phy->t103,1);

	AX25_Phy_Busy_Indication_Cb(&Phy->ax25_phy);

	Modem_Start_Transmiter(Phy->modem);

	Phy->phy_state = AX25_PHY_STATE_TRANSMITER_START;
}

static void AX25_Phy_Simplex_Acquisition(AX25_Phy_Simplex_t * Phy) {
	// Set repeater up
	Phy->repeaterup = true;

	// Stop all timers
	xTimerStop(Phy->t100,1);
	xTimerStop(Phy->t101,1);
	xTimerStop(Phy->t102,1);
	xTimerStop(Phy->t103,1);
	xTimerStop(Phy->t104,1);
	xTimerStop(Phy->t105,1);
	xTimerStop(Phy->t106,1);
	xTimerStop(Phy->t107,1);
	xTimerStop(Phy->t108,1);

	// Stop priority and normal queue processing
	Phy->phy_prio_queue_processing = false;
	Phy->phy_normal_queue_processing = false;

	// Send PH-BUSY Indication
	AX25_Phy_Busy_Indication_Cb(&Phy->ax25_phy);

	Phy->phy_state = AX25_PHY_STATE_RECEIVING;
}

static void AX25_Phy_Simplex_Release(AX25_Phy_Simplex_t * Phy) {
	// Stop priority and normal queue processing
	Phy->phy_prio_queue_processing = false;
	Phy->phy_normal_queue_processing = false;

	if (!uxQueueMessagesWaiting(Phy->phy_prio_queue)) {
		xTimerStop(Phy->t106,1);
		AX25_Phy_Simplex_Stop_Transmiter(Phy);
		return;
	}

	Phy->phy_prio_queue_processing = true;
	Phy->digipeating = true;

	Phy->phy_state = AX25_PHY_STATE_DIGIPEATING;
}

static void AX25_Phy_Simplex_Event_Proc(AX25_Phy_Simplex_t * Phy, AX25_Phy_Simplex_Event_t * phy_event) {

	if (phy_event->id == AX25_PHY_EXPEDITED_DATA_REQUEST) {
		if (xQueueSend(Phy->phy_prio_queue,phy_event,AX25_PHY_OPS_TO/portTICK_PERIOD_MS) != pdPASS) {
			ESP_LOGW(TAG,"Error adding frame to PH priority queue");
		}
		if (Phy->phy_state == AX25_PHY_STATE_READY || Phy->phy_state == AX25_PHY_STATE_TRANSMITER_SUPPRESSION) {
			Phy->digipeating = true;
			AX25_Phy_Simplex_Start_Transmiter(Phy);
		}
	} else if (phy_event->id >= AX25_PHY_SEIZE_REQUEST) {
		if (xQueueSend(Phy->phy_normal_queue,phy_event,AX25_PHY_OPS_TO/portTICK_PERIOD_MS) != pdPASS)
			ESP_LOGW(TAG,"Error adding request to PH normal queue");
	} if ( phy_event->id == AX25_RA_DATA_INDICATION && phy_event->arg) {
		if (Phy->phy_state == AX25_PHY_STATE_RECEIVING) {
			AX25_Phy_Data_Indication_Cb(&Phy->ax25_phy,phy_event->arg);
		}
		Framebuff_Free_Frame(phy_event->arg);
	} else {
		switch(Phy->phy_state) {
			case AX25_PHY_STATE_READY:
				switch (phy_event->id) {	
					// Timers expiry
					case AX25_PHY_TIMER_T102_EXPIRY:
						Phy->repeaterup = false;
						break;
						// From radio to physical state machine
					case AX25_RA_AOS_INDICATION:
						AX25_Phy_Simplex_Acquisition(Phy);
						break;
					default:
				}
				break;
			case AX25_PHY_STATE_RECEIVING:
				// From radio to physical state machine
				switch (phy_event->id) {
					case AX25_RA_LOS_INDICATION:
						xTimerStart(Phy->t100,1);
						xTimerStart(Phy->t101,1);
						
						AX25_Phy_Quiet_Indication_Cb(&Phy->ax25_phy);

						if (uxQueueMessagesWaiting(Phy->phy_prio_queue)) {
							Phy->digipeating = true;
							AX25_Phy_Simplex_Start_Transmiter(Phy);
							break;
						}
						Phy->phy_state = AX25_PHY_STATE_TRANSMITER_SUPPRESSION;

						break;
					default :
				}
				break;
			case AX25_PHY_STATE_TRANSMITER_SUPPRESSION:
				switch (phy_event->id) {	
					// Timers expiry
					case AX25_PHY_TIMER_T100_EXPIRY:
						Phy->repeaterup = false;
						break;
					case AX25_PHY_TIMER_T101_EXPIRY:
					case AX25_PHY_TIMER_T102_EXPIRY:
						if (uxQueueMessagesWaiting(Phy->phy_prio_queue)) {
							Phy->digipeating = true;
							AX25_Phy_Simplex_Start_Transmiter(Phy);
							break;
						}
						uint32_t r = esp_random();
						if ((uint8_t)r < Phy->persistance) {
							if (Phy->interrupted) {
								AX25_Phy_Simplex_Start_Transmiter(Phy);
								break;
							} else {
								Phy->phy_normal_queue_processing = true;
								Phy->phy_state = AX25_PHY_STATE_READY;
								break;
							}
						}

						xTimerStart(Phy->t102,1);
						break;

						// From radio to physical state machine
					case AX25_RA_AOS_INDICATION:
						AX25_Phy_Simplex_Acquisition(Phy);
						break;
					default:
				}
				break;
			case AX25_PHY_STATE_TRANSMITER_START:
				switch (phy_event->id) {	
					// Timers expiry
					case AX25_PHY_TIMER_T100_EXPIRY:
						Phy->repeaterup = false;
						break;
					case AX25_PHY_TIMER_T104_EXPIRY:
						Phy->repeaterup = true;
						xTimerStart(Phy->t105,1);
						break;
					case AX25_PHY_TIMER_T103_EXPIRY:
						if (Phy->repeaterup)
							xTimerStart(Phy->t105,1);
						else 
							xTimerStart(Phy->t104,1);
						break;
					case AX25_PHY_TIMER_T105_EXPIRY:
						xTimerStart(Phy->t106,1);
						if (Phy->digipeating) {
							Phy->phy_prio_queue_processing = true;
							Phy->phy_state = AX25_PHY_STATE_DIGIPEATING;
							break;
						}
						xTimerStart(Phy->t107,1);
						if (!Phy->interrupted) {
							AX25_Phy_Seize_Confirm_Cb(&Phy->ax25_phy);
						}
						Phy->phy_normal_queue_processing = true;
						Phy->phy_state = AX25_PHY_STATE_TRANSMITING;
						break;
					default:
				}
				break;
			case AX25_PHY_STATE_TRANSMITING:
				switch (phy_event->id) {	
					// Timers expiry
					case AX25_PHY_TIMER_T107_EXPIRY:
						Phy->interrupted = true;
						AX25_Phy_Simplex_Release(Phy);
						break;
					case AX25_PHY_TIMER_T106_EXPIRY:
						xTimerStop(Phy->t107,1);
						Phy->interrupted = true;
						AX25_Phy_Simplex_Stop_Transmiter(Phy);
						break;
					case AX25_RA_FRAME_SENT:
						if (phy_event->arg == Phy->outstanding_frame) {
							Framebuff_Free_Frame(phy_event->arg);
							Phy->outstanding_frame = NULL;
							Phy->phy_normal_queue_processing = true;
						}
					default:
				}
				break;
			case AX25_PHY_STATE_DIGIPEATING:
				switch (phy_event->id) {	
					// Timers expiry
					case AX25_PHY_TIMER_T106_EXPIRY:
						AX25_Phy_Simplex_Stop_Transmiter(Phy);
						break;
					case AX25_RA_FRAME_SENT:
						if (phy_event->arg == Phy->outstanding_frame) {
							Framebuff_Free_Frame(phy_event->arg);
							Phy->outstanding_frame = NULL;
							if (!uxQueueMessagesWaiting(Phy->phy_prio_queue)) {
								Phy->digipeating = false;
								AX25_Phy_Simplex_Stop_Transmiter(Phy);
							} else 
								Phy->phy_prio_queue_processing = true;
						}
					default:
				}
				break;
			case AX25_PHY_STATE_RECEIVER_START:
				switch (phy_event->id) {	
					// Timers expiry
					case AX25_PHY_TIMER_T108_EXPIRY:
						xTimerStart(Phy->t100,1);
						xTimerStart(Phy->t101,1);
						AX25_Phy_Quiet_Indication_Cb(&Phy->ax25_phy);
						Phy->phy_state = AX25_PHY_STATE_TRANSMITER_SUPPRESSION;
						break;
					default:
				}
				break;
			default : 
				ESP_LOGW(TAG,"Unknown physical state %d",Phy->phy_state);
				Phy->phy_state = AX25_PHY_STATE_READY;
				break;
		}
	}
}

bool AX25_Phy_Simplex_Normal_Queue_Proc(AX25_Phy_Simplex_t * Phy, AX25_Phy_Simplex_Event_t * phy_event) {
	switch (Phy->phy_state) {
		case AX25_PHY_STATE_READY:
			if (phy_event->id ==  AX25_PHY_SEIZE_REQUEST) {
				Phy->digipeating = false;
				AX25_Phy_Simplex_Start_Transmiter(Phy);
				return true;
			}
			break;

		case AX25_PHY_STATE_TRANSMITING:
			switch (phy_event->id) {
				case AX25_PHY_SEIZE_REQUEST:
					AX25_Phy_Seize_Confirm_Cb(&Phy->ax25_phy);
					break;
				case AX25_PHY_DATA_REQUEST:
					Phy->outstanding_frame = phy_event->arg;
					Modem_Send_Frame(Phy->modem, phy_event->arg);
					Phy->phy_normal_queue_processing = false;
					return true;
				case AX25_PHY_RELEASE_REQUEST:
					xTimerStop(Phy->t107,1);
					Phy->interrupted = false;
					AX25_Phy_Simplex_Release(Phy);
					return true;

				default:
			}
			break;	
		default:
			ESP_LOGE(TAG,"Normal queue must be disabled in state %d",Phy->phy_state);
			break;

	}

	return false;
}

bool AX25_Phy_Simplex_Priority_Queue_Proc(AX25_Phy_Simplex_t * Phy,AX25_Phy_Simplex_Event_t * phy_event) {
	if ( Phy->phy_state == AX25_PHY_STATE_DIGIPEATING && phy_event->id == AX25_PHY_EXPEDITED_DATA_REQUEST) {
		Phy->outstanding_frame = phy_event->arg;
		Modem_Send_Frame(Phy->modem, phy_event->arg);
		Phy->phy_prio_queue_processing = false;

	} else
		ESP_LOGE(TAG,"Prio queue enabled or invalid event 0x%x in state %d", phy_event->id, Phy->phy_state);

	return false;
}

// AX25 Phy state machine
void AX25_Phy_Simplex_Task(AX25_Phy_Simplex_t * Phy) {
	struct AX25_Phy_Simplex_Event_S phy_event;
	// Reset physycal state machine
	
	Phy->phy_prio_queue_processing = false;
	Phy->phy_normal_queue_processing = true;
	Phy->interrupted = false;
	Phy->digipeating = false;
	Phy->repeaterup = false;
	Phy->phy_state = AX25_PHY_STATE_READY;

	Modem_Start_Receiver(Phy->modem);

	do {
		// Physical layer event queue processing
		if (xQueueReceive(Phy->phy_event_queue,&phy_event,portMAX_DELAY) == pdPASS) {
			ESP_LOGD(TAG,"State %d : Event 0x%x",Phy->phy_state,phy_event.id);
			AX25_Phy_Simplex_Event_Proc(Phy,&phy_event);
		}

		// Physical layer priority queue processing
		while (Phy->phy_prio_queue_processing && xQueueReceive(Phy->phy_prio_queue,&phy_event,0) == pdPASS) {
			ESP_LOGD(TAG,"State %d : Priority queue Event 0x%x",Phy->phy_state,phy_event.id);
			if (AX25_Phy_Simplex_Priority_Queue_Proc(Phy,&phy_event))
				break;
			if (uxQueueMessagesWaiting(Phy->phy_event_queue))
				break;
		}

		// Physical layer normal queue processing
		while (Phy->phy_normal_queue_processing && xQueueReceive(Phy->phy_normal_queue,&phy_event,0) == pdPASS) {
			ESP_LOGD(TAG,"State %d : Normal queue Event 0x%x",Phy->phy_state,phy_event.id);
			if (AX25_Phy_Simplex_Normal_Queue_Proc(Phy,&phy_event))
				break;
			if (uxQueueMessagesWaiting(Phy->phy_event_queue))
				break;
		}
	} while(1);
}

// Phy ops
static int AX25_Phy_Simplex_Seize_Request(AX25_Phy_Simplex_t * Phy) {
	AX25_Phy_Simplex_Event_t phy_event = {
		.id = AX25_PHY_SEIZE_REQUEST,
	};

	if (xQueueSend(Phy->phy_event_queue, &phy_event, AX25_PHY_OPS_TO/portTICK_PERIOD_MS) != pdPASS)
		return -1;

	return 0;
}

static int AX25_Phy_Simplex_Release_Request(AX25_Phy_Simplex_t * Phy) {
	AX25_Phy_Simplex_Event_t phy_event = {
		.id = AX25_PHY_RELEASE_REQUEST,
	};

	if (xQueueSend(Phy->phy_event_queue, &phy_event, AX25_PHY_OPS_TO/portTICK_PERIOD_MS) != pdPASS)
		return -1;

	return 0;
}

static int AX25_Phy_Simplex_Expedited_Data_Request(AX25_Phy_Simplex_t * Phy, Frame_t * Frame) {
	AX25_Phy_Simplex_Event_t phy_event = {
		.id = AX25_PHY_EXPEDITED_DATA_REQUEST,
		.arg = Frame
	};

	Framebuff_Inc_Frame_Usage(Frame);
	if (xQueueSend(Phy->phy_event_queue, &phy_event, AX25_PHY_OPS_TO/portTICK_PERIOD_MS) != pdPASS) {
		Framebuff_Free_Frame(Frame);
		return -1;
	}

	return 0;
}

static int AX25_Phy_Simplex_Data_Request(AX25_Phy_Simplex_t * Phy, Frame_t * Frame) {
	AX25_Phy_Simplex_Event_t phy_event = {
		.id = AX25_PHY_DATA_REQUEST,
		.arg = Frame
	};

	Framebuff_Inc_Frame_Usage(Frame);
	if (xQueueSend(Phy->phy_event_queue, &phy_event, AX25_PHY_OPS_TO/portTICK_PERIOD_MS) != pdPASS) {
		Framebuff_Free_Frame(Frame);
		return -1;
	}

	return 0;
}

// Timer callback
static void AX25_Phy_Simplex_Timer_Event(TimerHandle_t Timer) {
	AX25_Phy_Simplex_t * phy = pvTimerGetTimerID(Timer);
	AX25_Phy_Simplex_Event_t phy_event;

	if (Timer == phy->t100)
		phy_event.id = AX25_PHY_TIMER_T100_EXPIRY;
	else if (Timer == phy->t101)
		phy_event.id = AX25_PHY_TIMER_T101_EXPIRY;
	else if (Timer == phy->t102)
		phy_event.id = AX25_PHY_TIMER_T102_EXPIRY;
	else if (Timer == phy->t103)
		phy_event.id = AX25_PHY_TIMER_T103_EXPIRY;
	else if (Timer == phy->t104)
		phy_event.id = AX25_PHY_TIMER_T104_EXPIRY;
	else if (Timer == phy->t105)
		phy_event.id = AX25_PHY_TIMER_T105_EXPIRY;
	else if (Timer == phy->t106)
		phy_event.id = AX25_PHY_TIMER_T106_EXPIRY;
	else if (Timer == phy->t107)
		phy_event.id = AX25_PHY_TIMER_T107_EXPIRY;
	else if (Timer == phy->t108)
		phy_event.id = AX25_PHY_TIMER_T108_EXPIRY;
	else 
		return;

	xQueueSend(phy->phy_event_queue, &phy_event, AX25_PHY_OPS_TO/portTICK_PERIOD_MS);
}

// Transmiter interface Callback
static int AX25_Phy_Simplex_Frame_Sent_Cb(AX25_Phy_Simplex_t * Phy, Frame_t * Frame) {
	AX25_Phy_Simplex_Event_t phy_event;

	phy_event.id = AX25_RA_FRAME_SENT;
	phy_event.arg = Frame;

	if ( xQueueSend(Phy->phy_event_queue, &phy_event, AX25_PHY_CB_TO/portTICK_PERIOD_MS) != pdPASS) {
		Framebuff_Free_Frame(Frame);
		ESP_LOGE(TAG,"Error adding sent frame to event queue");
		return -1;
	}

	return 0;
}

// Receiver interface Callback
static int AX25_Phy_Simplex_Frame_Received_Cb(AX25_Phy_Simplex_t * Phy, Frame_t * Frame) {
	AX25_Phy_Simplex_Event_t phy_event;

	if (!Frame) {
		ESP_LOGE(TAG,"Null Frame receive");
		return -1;
	}
	phy_event.id = AX25_RA_DATA_INDICATION;
	phy_event.arg = Frame;

	Framebuff_Inc_Frame_Usage(Frame);
	if ( xQueueSend(Phy->phy_event_queue, &phy_event, AX25_PHY_CB_TO/portTICK_PERIOD_MS) != pdPASS) {
		Framebuff_Free_Frame(Frame);
		ESP_LOGE(TAG,"Error adding received frame to event queue");
		return -1;
	}

	return 0;
}

static int AX25_Phy_Simplex_Dcd_Changed_Cb(AX25_Phy_Simplex_t * Phy, bool Dcd) {
	AX25_Phy_Simplex_Event_t phy_event;

	if (Dcd)
		phy_event.id = AX25_RA_AOS_INDICATION;
	else
		phy_event.id = AX25_RA_LOS_INDICATION;

	if (xQueueSend(Phy->phy_event_queue, &phy_event, AX25_PHY_CB_TO/portTICK_PERIOD_MS) != pdPASS) {
		ESP_LOGE(TAG,"Error adding %s of signal to event queue",Dcd?"acquisition":"lost");
		return -1;
	}

	return 0;
}
