/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/aprs.c
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

#include "aprs.h"

#include <stdlib.h>
#include <sys/time.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <esp_log.h>
#include <esp_rom_crc.h>
#include <esp_event.h>
#include <nvs.h>
#include <esp_spiffs.h>

#include <berkeley-db/db.h>

#include "framebuff.h"
#include "hdlc_enc.h"
#include "gps.h"
#include "hmi.h"
#include "kiss.h"
#include "sb.h"
#include "ax25.h"
#include "ax25_lm.h"
#include "aprs_parsers.h"
#include "aprs_encoder.h"
#include "aprs_log.h"

#define TAG	"APRS"

#ifndef NSDEBUG
#define APRS_TASK_STACK_SIZE	4096+1024
#else
#define APRS_TASK_STACK_SIZE	4096
#endif

#define APRS_TASK_PRIORITY	3
#define APRS_FRAME_POOL_SIZE 4
#define APRS_EVENT_SEND_TIMEOUT 100
#define APRS_STATIONS_DB_FILE	"/spiffs/stations.db"
#define APRS_QUEUE_SIZE	5

enum APRS_Event_E {
	APRS_EVENT_FRAME_RECEIVED,	// Receive frame
	APRS_EVENT_GPS,			// Receive GPS data
	APRS_SEND_STATUS,		// Request to send status (text in aprs data)
	APRS_SEND_POSITION,
};

typedef struct APRS_Event_S {
	enum APRS_Event_E type;
	time_t timestamp;
	union {
		Frame_t * frame;
		struct {
			struct APRS_Position position;
			struct APRS_Course course;
		};
		AX25_Addr_t addr;
		char text[64];
	};
} APRS_Event_t;

struct APRS_S {
	TaskHandle_t task;
	QueueHandle_t queue;
	StaticQueue_t queue_data;
	uint8_t queue_buff[sizeof(APRS_Event_t)*APRS_QUEUE_SIZE];

	Framebuff_t * out_framebuff;
	AX25_Lm_t * ax25_lm;

	nvs_handle_t nvs;

	SB_t sb;	// smart beaconing state
	bool first_beacon;
	uint32_t frame_count;

	SemaphoreHandle_t 	stations_sem;
	StaticSemaphore_t 	stations_sem_data;
	int stations_fd;
	DB * stations_db;	// station database

	SemaphoreHandle_t 	local_sem;
	StaticSemaphore_t 	local_sem_data;

	AX25_Addr_t appid;
	AX25_Addr_t digis[APRS_MAX_DIGI];
	uint8_t n_digis;

	bool callid_set;
	APRS_Station_t local;	// Local station info
};

ESP_EVENT_DEFINE_BASE(APRS_EVENT);
const char *APRS_Ssid_Symbol[16] = { "", "/a", "/U", "/f", "/b", "/Y", "/X", "/\'", "/s", "/>", "/<", "/O", "/j", "/R", "/k", "/v" };

struct {
	TaskHandle_t handle;
	StackType_t	stack[(APRS_TASK_STACK_SIZE)/sizeof(StackType_t)];
	StaticTask_t buffer;
} APRS_Tasks;

static void APRS_Gps_Event_Handler(APRS_t * Aprs,esp_event_base_t event_base, int32_t event_id, GPS_Data_t * Gps_Data);
static void APRS_Task(APRS_t * Aprs);
static int APRS_Prepare_Data(APRS_t * Aprs, APRS_Data_t * Data);
static int APRS_Send_Data(APRS_t * Aprs, APRS_Data_t * Data);

extern Kiss_t * Kiss;

static int APRS_Db_Station_Compare(const DBT * Key1, const DBT * Key2) {
	if (Key1->size == sizeof(AX25_Addr_t) && Key2->size == sizeof(AX25_Addr_t)) {
		return  AX25_Addr_Cmp(Key1->data, Key2->data);
	}
	ESP_LOGE(TAG,"Wrong key size in APRS_Db_Compare");
	return -1;
}

APRS_t * APRS_Init(AX25_Lm_t * Ax25_Lm) {
	APRS_t * aprs;
	AX25_Lm_Cbs_t ax25_cbs = {
		.seize_confirm = NULL,
		.data_indication = (typeof(ax25_cbs.data_indication))APRS_Frame_Received_Cb,
		.busy_indication = NULL,
		.quiet_indication = NULL
	};

	if (!(aprs = malloc(sizeof(APRS_t)))) {
		ESP_LOGE(TAG,"Error allocating APRS struct");
		return NULL;
	}
	bzero(aprs,sizeof(APRS_t));

	if (!(aprs->out_framebuff = Framebuff_Init(APRS_FRAME_POOL_SIZE,HDLC_MAX_FRAME_LEN)))
		ESP_LOGE(TAG,"Error allocation aprs frame pool");

	ESP_LOGD(TAG,"Init out buffer %p",aprs->out_framebuff);

	aprs->stations_sem = xSemaphoreCreateMutexStatic(&aprs->stations_sem_data);
	aprs->local_sem = xSemaphoreCreateMutexStatic(&aprs->local_sem_data);

	aprs->queue = xQueueCreateStatic(APRS_QUEUE_SIZE,sizeof(APRS_Event_t),aprs->queue_buff,&aprs->queue_data);

	aprs->ax25_lm = Ax25_Lm;
	AX25_Lm_Register_Dl(aprs->ax25_lm, aprs, &ax25_cbs, NULL); // Get All frames from phy

	return aprs;
}

int APRS_Load_Config(APRS_t *Aprs) {
	char addr[81];
	uint8_t ssid;
	int i;
	size_t len;
	nvs_open("Aprs", NVS_READWRITE, &Aprs->nvs);

	len = sizeof(addr);
	if (nvs_get_str(Aprs->nvs,"Appid",addr,&len))
#ifdef CONFIG_ESP32S3APRS_APRS_DEFAULT_APPID
		strncpy(addr,CONFIG_ESP32S3APRS_APRS_DEAFULT_APPID, sizeof(addr));
#else
		strcpy(addr,"APZ001");
#endif

	AX25_Str_To_Addr(addr,&Aprs->appid);
	AX25_Norm_Addr(&Aprs->appid);
	AX25_Addr_To_Str(&Aprs->appid,addr,sizeof(addr));
	ESP_LOGI(TAG,"Appid set to %s",addr);

	len = sizeof(addr);
	addr[0] = '\0';
	if (nvs_get_str(Aprs->nvs,"Callid",addr,&len)) {
#ifdef CONFIG_ESP32S3APRS_APRS_DEFAULT_CALLID
		strncpy(addr, CONFIG_ESP32S3APRS_APRS_DEFAULT_CALLID, sizeof(addr));
#else
		strcpy(addr, "CALLID");
#endif
	}

	if (*addr && strncmp(addr,"CALLID",6)) {
		Aprs->callid_set = true;
	} else	{
		ESP_LOGE(TAG,"You _MUST_ set your callid");
		strcpy(addr,"CALLID");
		Aprs->callid_set = false;
	}

	AX25_Str_To_Addr(addr, &Aprs->local.callid);
	AX25_Norm_Addr(&Aprs->local.callid);
	AX25_Addr_To_Str(&Aprs->local.callid,addr,sizeof(addr));
	ESP_LOGI(TAG,"CallId set to %s",addr);

	len = sizeof(addr);

	if (nvs_get_str(Aprs->nvs,"Path",addr,&len)) {
#ifdef CONFIG_ESP32S3APRS_APRS_DEFAULT_PATH
				strncpy(addr,CONFIG_ESP32S3APRS_APRS_DEFAULT_PATH,sizeof(addr));
#else
				strncpy(addr,"WIDE1-1,WIDE2-2",sizeof(addr));
#endif
	}

	char *ptr = addr, *last = addr;
	i = 0;
	ESP_LOGD(TAG,"Setting APRS path to %s", addr);
	while (ptr && *ptr) {
		ptr = strchr(last,',');
		if (ptr)
			*ptr = '\0';
		AX25_Str_To_Addr(last, &Aprs->digis[i]);
		AX25_Norm_Addr(&Aprs->digis[i]);
		ESP_LOGI(TAG,"Set digi %d to %s",i, last);
		if (ptr) {
			last = ptr+1;
			ptr = last;
		}
		i++;
	}
	Aprs->n_digis = i;

	if (i)
		Aprs->digis[i-1].ssid |= 1;  // End address marker on last digi
	else {
		Aprs->local.callid.ssid |= 1;  // End address marker on src callid
	}

	len = 3;
	if (nvs_get_str(Aprs->nvs,"Symbol",addr,&len)) {
		Aprs->local.symbol[0]='/';
		Aprs->local.symbol[1]='/';
	} else {
		Aprs->local.symbol[0]=addr[0];
		Aprs->local.symbol[1]=addr[1];
	}

	if (nvs_get_u8(Aprs->nvs,"Ambiguity",&ssid))
		Aprs->local.position.ambiguity = 2;
	else if (ssid > 5)
		Aprs->local.position.ambiguity = 5;
	else 
		Aprs->local.position.ambiguity = ssid;

	len = 64;
	if (nvs_get_str(Aprs->nvs,"DefaultStatus",Aprs->local.status, &len)) {
		strcpy(Aprs->local.status, APRS_DEFAULT_STATUS);
	}

	SB_Init(&Aprs->sb, Aprs->nvs);

	return 0;
}

int APRS_Start(APRS_t *Aprs) {
	APRS_Tasks.handle = xTaskCreateStaticPinnedToCore((void(*)(void*))APRS_Task,"APRS", APRS_TASK_STACK_SIZE, Aprs, APRS_TASK_PRIORITY, APRS_Tasks.stack, &APRS_Tasks.buffer, APP_CPU_NUM);

	esp_event_handler_register(GPS_EVENT,ESP_EVENT_ANY_ID, (esp_event_handler_t)APRS_Gps_Event_Handler,(void*)Aprs);

	return (APRS_Tasks.handle != NULL);
}

int APRS_Open_Db(APRS_t *Aprs) {
	BTREEINFO bt_info = {
		.flags = 0,
		.cachesize = 8192,
		.maxkeypage = 4,
		.minkeypage = 4,
		.psize = 512,
		.compare = APRS_Db_Station_Compare,
		.prefix = NULL,
		.lorder = 0,
	};

	if (!Aprs)
		return -1;

	// Try open btree file
	Aprs->stations_fd = open(APRS_STATIONS_DB_FILE, O_RDWR , S_IRUSR | S_IWUSR);
	if (Aprs->stations_fd < 0)
		// Create file
		Aprs->stations_fd = open(APRS_STATIONS_DB_FILE, O_RDWR | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);

	if (Aprs->stations_fd >= 0) {
		Aprs->stations_db = __bt_open((void*)Aprs->stations_fd, NULL, &bt_info, 0);
		ESP_LOGI(TAG,"%s opened", APRS_STATIONS_DB_FILE);
	} else {
		Aprs->stations_db = NULL;
		ESP_LOGE(TAG,"Can't open %s", APRS_STATIONS_DB_FILE);
		return 1;
	}

	return 0;
}

void APRS_Task(APRS_t * Aprs) {
	APRS_Event_t event;
	APRS_Data_t data;
	int i;
	size_t len;
	struct tm tm;

	while (1) {
		xQueueReceive(Aprs->queue,&event,portMAX_DELAY);
		switch (event.type) {
			case APRS_EVENT_FRAME_RECEIVED:	// Receive frame
				Aprs->frame_count++;
				ESP_LOGD(TAG,"%lu frame received",Aprs->frame_count);

				// Parse frame and fill data struct
				if ((i = APRS_Parse(event.frame,&data)) == -1) {
					ESP_LOGE(TAG,"Invalid APRS frame");
					Framebuff_Free_Frame(event.frame);
					break;
				}
				Framebuff_Free_Frame(event.frame);

				data.timestamp = event.timestamp;

				if (Aprs->stations_db) {
					xSemaphoreTake(Aprs->stations_sem,portMAX_DELAY);
					APRS_Log_Station(Aprs->stations_db, &data);
					xSemaphoreGive(Aprs->stations_sem);
				}

				// Notifie HMI of new incomming data
				esp_event_post(APRS_EVENT, i, &data, sizeof(APRS_Data_t), portMAX_DELAY);
				break;
			case APRS_EVENT_GPS:		// Receive GPS data
										// Update local info
				xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
				Aprs->local.timestamp = event.timestamp;
				event.position.ambiguity = Aprs->local.position.ambiguity;
				memcpy(&Aprs->local.position, &event.position, sizeof(struct APRS_Position));
				memcpy(&Aprs->local.course, &event.course, sizeof(struct APRS_Course));
				xSemaphoreGive(Aprs->local_sem);

				// Smart beaconing
				if (!SB_Update(&Aprs->sb,Aprs->local.timestamp,Aprs->local.course.speed,Aprs->local.course.dir,!Aprs->first_beacon))
					break;
				if (!Aprs->first_beacon) {
					len = sizeof(event.text);
					if (nvs_get_str(Aprs->nvs,"FirstBeaconText",event.text,&len)) 
						strncpy(event.text,"Start Tracking !",sizeof(event.text));
				}
				else  {
					len = sizeof(event.text);
					if (nvs_get_str(Aprs->nvs,"BeaconText",event.text,&len)) 
						strncpy(event.text,"Beaconing ...",sizeof(event.text));
				}
				Aprs->first_beacon = true;
				/* FALLTHRU */
			case APRS_SEND_POSITION:	// Send APRS Position report with timestamp, course/speed, default status text
				if (APRS_Prepare_Data(Aprs,&data)) {
					ESP_LOGE(TAG,"Error preparing data for position send request");
					break;
				}

				// Fill data to send
				data.type = APRS_DTI_POS_W_TS;
				data.extension = APRS_DATA_EXT_CSE;

				// Set timestamp of local station position
				gmtime_r(&Aprs->local.timestamp, &tm);
				data.time.month = 0;
				data.time.day = tm.tm_mday;
				data.time.hours = tm.tm_hour;
				data.time.minutes = tm.tm_min;
				data.time.seconds = 0;

				// Set position
				memcpy(&data.position, &Aprs->local.position, sizeof(struct APRS_Position));

				// Set data extension
				data.extension = APRS_DATA_EXT_CSE;
				memcpy(&data.course, &Aprs->local.course, sizeof(struct APRS_Course));

				// Copy comment from event
				if (event.text[0])
					memcpy(data.text, event.text, sizeof(data.text));

				APRS_Send_Data(Aprs,&data);
				break;
			case APRS_SEND_STATUS:	// Send APRS status report without timestamp, locator
				if (APRS_Prepare_Data(Aprs,&data)) {
					ESP_LOGE(TAG,"Error preparing data for status send request");
					break;
				}

				// Fill data to send
				data.type = APRS_DTI_STATUS;
				data.extension = APRS_DATA_EXT_NONE;

#ifdef APRS_SEND_STATUS_WITH_TIMESTAMP
				// use Time stamp of event
				gmtime_r(&event.timestamp, &tm);

				// set time for DDHHMM format
				data.time.month = 0;
				data.time.day = tm.tm_mday;
				data.time.hours = tm.tm_hour;
				data.time.minutes = tm.tm_min;
				data.time.seconds = 0;
#elif defined(APRS_SEND_STATUS_WITH_LOCATOR)
				// set position for status with locator
				data.position.longitude = Aprs->local.position.longitude;
				data.position.latitude = Aprs->local.position.latitude;
				data.position.ambiguity = APRS_AMBIGUITY_LOC_SUBSQUARE;
#endif
				if (event.text[0]) {
					// Update local station status
					xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
					// Copy status text in local station info
					strncpy(Aprs->local.status, event.text, sizeof(Aprs->local.status));
					xSemaphoreGive(Aprs->local_sem);
				}
				// Copy status text in data status text
				strncpy(data.text, Aprs->local.status, sizeof(data.text));

				APRS_Send_Data(Aprs,&data);
				break;

			default:
				ESP_LOGW(TAG,"Unknown event received : %d",event.type);
		}
	}
}

static int APRS_Prepare_Data(APRS_t * Aprs, APRS_Data_t * Data) {
	struct timeval tv;
	int i;

	if (!Aprs || !Data)
		return -1;

	bzero(Data,sizeof(APRS_Data_t));

	gettimeofday(&tv, NULL);
	Data->timestamp = tv.tv_sec;

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);

	// Fill APRS Addresses
	memcpy(&Data->address[0], &Aprs->appid, sizeof(AX25_Addr_t));
	memcpy(&Data->address[1], &Aprs->local.callid, sizeof(AX25_Addr_t));
	i=1;
	while (!(Data->address[i].ssid & 1) && (i-1)<Aprs->n_digis) {
		i++;
		memcpy(&Data->address[i], &Aprs->digis[i-2], sizeof(AX25_Addr_t));
	}

	xSemaphoreGive(Aprs->local_sem);

	Data->address[i].ssid |= 1;
	Data->from = 1;

	Data->symbol[0] = Aprs->local.symbol[0];
	Data->symbol[1] = Aprs->local.symbol[1];
	Data->comp_type = -1;

	return 0;
}

static int APRS_Send_Data(APRS_t * Aprs, APRS_Data_t * Data) {
	uint16_t crc;
	int ret;
	Frame_t * frame;
	APRS_Event_t event;

	if (!(frame = Framebuff_Get_Frame(Aprs->out_framebuff))) {
		ESP_LOGE(TAG,"Out frame buffer empty will getting frame");
		return -1;
	}

	if ((ret = APRS_Encode(Data,frame)) < 0) {
		ESP_LOGE(TAG,"Error encoding data");
		Framebuff_Free_Frame(frame);
		return -1;
	}

	// Add crc
	crc = esp_rom_crc16_le(0,frame->frame,ret);
	frame->frame[ret] = (crc)&0xff;
	frame->frame[ret+1]= (crc>>8)&0xff;
	frame->frame_len += 2;

	ESP_LOGD(TAG,"encoded data : %d, frame_len : %d", ret, frame->frame_len);

	if (Aprs->callid_set) {
		if (AX25_Lm_Data_Request(Aprs->ax25_lm, Aprs, frame)) {
			ESP_LOGE(TAG,"Error sending frame to AX25 LM");
			return -1;
		}
	} else
		ESP_LOGE(TAG,"You _MUST_ set your Callid");

	// Loop back to APRS for decoding
	event.timestamp = Data->timestamp;;
	event.type = APRS_EVENT_FRAME_RECEIVED;
	event.frame = frame;
	Framebuff_Inc_Frame_Usage(frame);
	if (xQueueSend(Aprs->queue, &event, portMAX_DELAY) != pdPASS) {
		ESP_LOGE(TAG,"Error sending frame received event");
		Framebuff_Free_Frame(frame);
	}

	// Loop back to Kiss
	Kiss_Frame_Received_Cb(Kiss,frame);

	// Free frame
	Framebuff_Free_Frame(frame);

	return 0;
}

// Get/Set methode
int APRS_Get_Symbol(APRS_t * Aprs, char Str[2]) {
	if (!Aprs)
		return -1;

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
	Str[0] = Aprs->local.symbol[0];
	Str[1] = Aprs->local.symbol[1];
	xSemaphoreGive(Aprs->local_sem);

	return 0;
}

int APRS_Set_Symbol(APRS_t * Aprs, const char Str[2]) {
	struct timeval tv;
	char sym[3];

	if (!Aprs)
		return -1;

	gettimeofday(&tv, NULL);

	if (Str[0] && Str[1]) {
		xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
		Aprs->local.timestamp = tv.tv_sec;

		Aprs->local.symbol[0] = Str[0];
		Aprs->local.symbol[1] = Str[1];
		xSemaphoreGive(Aprs->local_sem);

		sym[0] = Str[0];
		sym[1] = Str[1];
		sym[2] = '\0';
		if (nvs_set_str(Aprs->nvs,"Symbol", sym))
			ESP_LOGE(TAG,"Error writing Symbol to Nvs");
		nvs_commit(Aprs->nvs);
	}
	return 0;
}

int APRS_Get_Ambiguity(APRS_t * Aprs, uint8_t *Ambiguity) {
	if (!Aprs)
		return -1;

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
	*Ambiguity = Aprs->local.position.ambiguity;
	xSemaphoreGive(Aprs->local_sem);

	return 0;
}

int APRS_Set_Ambiguity(APRS_t * Aprs, uint8_t Ambiguity) {
	struct timeval tv;

	if (!Aprs)
		return -1;

	gettimeofday(&tv, NULL);

	if (Ambiguity < APRS_AMBIGUITY_MAX) {
		xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
		Aprs->local.timestamp = tv.tv_sec;
		Aprs->local.position.ambiguity = Ambiguity;
		xSemaphoreGive(Aprs->local_sem);

		if (nvs_set_u8(Aprs->nvs,"Ambiguity", Ambiguity))
			ESP_LOGE(TAG,"Error writing ambiguity to Nvs");
		nvs_commit(Aprs->nvs);
	}
	return 0;
}

int APRS_Get_Callid(APRS_t * Aprs, AX25_Addr_t *Addr) {
	if (!Aprs)
		return -1;

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
	memcpy(Addr, &Aprs->local.callid, sizeof(AX25_Addr_t));
	xSemaphoreGive(Aprs->local_sem);

	return 0;
}

int APRS_Set_Callid(APRS_t * Aprs, const AX25_Addr_t * Callid) {
	struct timeval tv;
	char str[16];

	if (!Aprs || !Callid)
		return -1;

	gettimeofday(&tv, NULL);

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);

	memcpy(&Aprs->local.callid, Callid, sizeof(AX25_Addr_t));
	AX25_Norm_Addr(&Aprs->local.callid);

	if (!Aprs->n_digis)
		Aprs->local.callid.ssid |= 1;  // Set end marker
									   
	Aprs->local.timestamp = tv.tv_sec;

	AX25_Addr_To_Str(&Aprs->local.callid, str, sizeof(str));

	xSemaphoreGive(Aprs->local_sem);
			
	ESP_LOGI(TAG,"Set Callid to %s", str);
	if (str[0] && strncmp(str, "CALLID", 6))
		Aprs->callid_set = true;
	else
		Aprs->callid_set = false;

	if (str[0]) {
		if (nvs_set_str(Aprs->nvs,"Callid", str))
			ESP_LOGE(TAG,"Error writing Callsign to Nvs");
		nvs_commit(Aprs->nvs);
	}

	return 0;
}

int APRS_Get_Appid(APRS_t * Aprs, AX25_Addr_t *Addr) {
	if (!Aprs)
		return -1;

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
	memcpy(Addr, &Aprs->appid, sizeof(AX25_Addr_t));
	xSemaphoreGive(Aprs->local_sem);

	return 0;
}

int APRS_Get_Digi(APRS_t * Aprs, int n, AX25_Addr_t *Addr) {
	if (!Aprs || !Addr)
		return -1;

	if (!(n<Aprs->n_digis))
			return 1;

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
	memcpy(Addr, &Aprs->digis[n], sizeof(AX25_Addr_t));
	xSemaphoreGive(Aprs->local_sem);

	return 0;
}

int APRS_Get_Digis(APRS_t * Aprs, AX25_Addr_t *Addr, int *n) {
	if (!Aprs || !n)
		return -1;

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
	if (*n>0 && Addr) {
		if (*n > APRS_MAX_DIGI)
			*n = APRS_MAX_DIGI;
		memcpy(Addr, &Aprs->digis, sizeof(AX25_Addr_t)*(*n));
	}
	*n = Aprs->n_digis;
	xSemaphoreGive(Aprs->local_sem);

	return 0;
}

int APRS_Set_Digis(APRS_t * Aprs, const AX25_Addr_t *Digi_list, int n_digis) {
	struct timeval tv;
	char str[90];

	if (!Aprs)
		return -1;

	if ((n_digis<0) || n_digis > APRS_MAX_DIGI)
			return 1;

	gettimeofday(&tv, NULL);

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);

	Aprs->local.timestamp = tv.tv_sec;

	Aprs->n_digis = n_digis;
	if (!n_digis) 
		Aprs->local.callid.ssid |= 1;
	else {
		memcpy(Aprs->digis, Digi_list, sizeof(AX25_Addr_t)*n_digis);
		n_digis--;
		AX25_Norm_Addr(&Aprs->digis[n_digis]);
		Aprs->digis[n_digis].ssid |= 1;
		while (n_digis) {
			n_digis--;
			AX25_Norm_Addr(&Aprs->digis[n_digis]);
		}
	}
	xSemaphoreGive(Aprs->local_sem);

	APRS_Get_Digis_Str(Aprs, str, sizeof(str));
	if (nvs_set_str(Aprs->nvs,"Path", str)) 
		ESP_LOGE(TAG,"Error writing Paht to  Nvs");
	nvs_commit(Aprs->nvs);

	return 0;
}
int APRS_Get_Symbol_Str(APRS_t * Aprs, char * Str, int len) {
	if (!Aprs)
		return 0;

	if (len < 3)
		return 0;

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
	Str[0] = Aprs->local.symbol[0];
	Str[1] = Aprs->local.symbol[1];
	xSemaphoreGive(Aprs->local_sem);
	Str[2] = '\0';

	return 2;
}

int APRS_Get_Callid_Str(APRS_t * Aprs, char * Str, int len) {
	int pos;

	if (!Aprs)
		return 0;

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
	pos = AX25_Addr_To_Str(&Aprs->local.callid, Str, len);
	xSemaphoreGive(Aprs->local_sem);

	return pos;
}

int APRS_Get_Appid_Str(APRS_t * Aprs, char * Str, int len) {
	int pos;

	if (!Aprs)
		return 0;

	pos = AX25_Addr_To_Str(&Aprs->appid, Str, len);

	return pos;
}

int APRS_Get_Digis_Str(APRS_t * Aprs, char * Str, int len) {
	int pos = 0;
	int i;

	if (!Aprs || !Str)
		return -1;

	if (Aprs->local.callid.ssid & 1) {	// No digis
		if (len < 1)
			return -1;
		Str[0] = '\0';
		return 0;
	}

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
	i = 0;
	do {
		if (i && ((len-pos) >2 )) {
			Str[pos++] = ',';
		}
		pos += AX25_Addr_To_Str(&Aprs->digis[i], &Str[pos], (len-pos));
		i++;
	}
	while (!(Aprs->digis[i-1].ssid & 1) && (len-pos)>9 && i < Aprs->n_digis && i < 8);
	xSemaphoreGive(Aprs->local_sem);

	Str[pos] = '\0';

	return pos;
}

int APRS_Get_Position(APRS_t * Aprs, struct APRS_Position * Position) {
	if (!Aprs || ! Position)
		return -1;

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
	memcpy(Position, &Aprs->local.position, sizeof(struct APRS_Position));
	xSemaphoreGive(Aprs->local_sem);

	return 0;
}

int APRS_Get_Local(APRS_t * Aprs, APRS_Station_t * Station) {
	if (!Aprs || ! Station)
		return -1;

	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
	memcpy(Station, &Aprs->local, sizeof(APRS_Station_t));
	xSemaphoreGive(Aprs->local_sem);

	return 0;
}

int APRS_Position_To_Locator(struct APRS_Position * Position, char * Grid,int len) {
	int pos;
	int n_pair;
	uint32_t lon, lat;

	n_pair = APRS_AMBIGUITY_LOC_FIELD +1 - Position->ambiguity;
	if (len < (n_pair<<1)) {
		return -1;
	}

	lon = Position->longitude + (180<<GPS_FIXED_POINT_DEG);
	lat = Position->latitude + (90<<GPS_FIXED_POINT_DEG);

	lon /= 20;
	lat /= 10;

	pos = 0;
	while (n_pair && (pos+3)<len) {
		if ((pos>>1)&1) {
			Grid[pos] = (lon>>GPS_FIXED_POINT_DEG) + '0';
			Grid[pos+1] = (lat>>GPS_FIXED_POINT_DEG) + '0';
			lon = (lon & ((1<<GPS_FIXED_POINT_DEG)-1))*24;
			lat = (lat & ((1<<GPS_FIXED_POINT_DEG)-1))*24;
		} else {
			Grid[pos] = (lon>>GPS_FIXED_POINT_DEG) + 'A';
			Grid[pos+1] = (lat>>GPS_FIXED_POINT_DEG) + 'A';
			lon = (lon & ((1<<GPS_FIXED_POINT_DEG)-1))*10;
			lat = (lat & ((1<<GPS_FIXED_POINT_DEG)-1))*10;
		}
		n_pair--;
		pos += 2;
	}

	if (pos < len)
		Grid[pos] = '\0';

	return pos;
}

int APRS_Locator_To_Position(char * Locator, int Len, struct APRS_Position * Pos) {
	int n_pair;
	uint32_t mult;

	if (Len&1 || Len < 2)
		return -1;
	n_pair = 0;
	mult = 1;
	while (Len) {
		if ((n_pair>>1)&1) {
			Pos->longitude *= 10;
			Pos->latitude  *= 10;
			mult *= 10;
			Pos->longitude += Locator[n_pair<<1]-'0';
			Pos->latitude += Locator[n_pair<<1]-'0';
		} else {
			if (n_pair) {
				Pos->longitude *= 24;
				Pos->latitude  *= 24;
				mult *= 24;
				Pos->longitude += Locator[(n_pair<<1)+1]-'A';
				Pos->latitude += Locator[(n_pair<<1)+1]-'A';
			} else {
				Pos->longitude = (Locator[(n_pair<<1)+1]-'A') * 20;
				Pos->latitude = (Locator[(n_pair<<1)+1]-'A') * 10;
			}
		}
		n_pair++;
		Len -=2;
	}
	
	Pos->longitude = ((((uint64_t)Pos->longitude)<<GPS_FIXED_POINT_DEG)/mult) - (180<<GPS_FIXED_POINT_DEG);
	Pos->latitude = ((((uint64_t)Pos->latitude)<<GPS_FIXED_POINT_DEG)/mult) - (90<<GPS_FIXED_POINT_DEG);
	
	if (n_pair > (APRS_AMBIGUITY_LOC_FIELD-APRS_AMBIGUITY_LOC_EXT_SQUARE)) {
			ESP_LOGW(TAG,"Locator precision > extended square");
			Pos->ambiguity = APRS_AMBIGUITY_LOC_EXT_SQUARE;
	} else {
		Pos->ambiguity = APRS_AMBIGUITY_LOC_FIELD +1 - n_pair;
	}

	return n_pair<<1;
}

int APRS_Send_Status(APRS_t * Aprs, const char * Status) {
	APRS_Event_t event;
	struct timeval tv;

	gettimeofday(&tv, NULL);
	event.timestamp = tv.tv_sec;

	event.type = APRS_SEND_STATUS;
	event.text[0] = '\0';
	if (Status && Status[0])
		strncpy(event.text, Status, sizeof(event.text));
	if (xQueueSend(Aprs->queue,&event,portMAX_DELAY) != pdPASS) {
		ESP_LOGE(TAG,"Error requesting sending status");
		return -1;
	}

	return 0;
}

int APRS_Get_Status(APRS_t * Aprs, char * Status, int len) {
	xSemaphoreTake(Aprs->local_sem, portMAX_DELAY);
	strncpy(Status, Aprs->local.status, len);
	xSemaphoreGive(Aprs->local_sem);
	return 0;
}

int APRS_Send_Position(APRS_t * Aprs, const char * Comment) {
	APRS_Event_t event;

	event.type = APRS_SEND_POSITION;
	event.text[0] = '\0';
	if (Comment && Comment[0])
		strncpy(event.text, Comment, sizeof(event.text));
	if (xQueueSend(Aprs->queue, &event, portMAX_DELAY) != pdPASS) {
		ESP_LOGE(TAG,"Error requesting sending position");
		return -1;
	}

	return 0;
}

int APRS_Get_Station(APRS_t * Aprs, AX25_Addr_t *Id, APRS_Station_t * Station) {
	DBT key, data;
	int ret;

	if (!Aprs || !Id || !Station)
		return -1;

	if (Aprs->stations_db) {
		key.size = sizeof(AX25_Addr_t);
		key.data = Id;
	
		xSemaphoreTake(Aprs->stations_sem,portMAX_DELAY);
		if ((ret = Aprs->stations_db->get(Aprs->stations_db, &key, &data, 0))<0) {
			ESP_LOGE(TAG,"Error getting station db");
			return -1;
		}
		if (!ret) {
			if (data.size > sizeof(APRS_Station_t))
				data.size = sizeof(APRS_Station_t);
			memcpy(Station, data.data, data.size);
		}
		xSemaphoreGive(Aprs->stations_sem);
	} else 
		ret = -1;


	return ret;
}

int APRS_Stations_Seq(APRS_t * Aprs, AX25_Addr_t *Addr, int flags, APRS_Station_t *Station) {
	int ret;
	DBT key, data;
	AX25_Addr_t addr = {.addr = {0,0,0,0,0,0,0}};

	if (Aprs->stations_db) {
		if (!Addr)
			key.data = &addr;
		else
			key.data = Addr;
		key.size = sizeof(AX25_Addr_t);

		xSemaphoreTake(Aprs->stations_sem, portMAX_DELAY);
		ret = Aprs->stations_db->seq(Aprs->stations_db, &key, &data, flags);
		if (!ret && Station)
			memcpy(Station, data.data, sizeof(APRS_Station_t));
		xSemaphoreGive(Aprs->stations_sem);
	} else
		ret = -1;

	return ret;
}

// AX25 LM callbacks
int APRS_Frame_Received_Cb(APRS_t * Aprs, Frame_t * Frame) {
	APRS_Event_t event;
	struct timeval tv;

	gettimeofday(&tv, NULL);
	event.timestamp = tv.tv_sec;

	event.type = APRS_EVENT_FRAME_RECEIVED;
	event.frame = Frame;
	Framebuff_Inc_Frame_Usage(Frame);
	if (xQueueSend(Aprs->queue, &event, portMAX_DELAY) != pdPASS) {
		ESP_LOGE(TAG,"Error sending frame received event");
		Framebuff_Free_Frame(Frame);
	}

	return 0;
}

// GPS Event loop handler
static void APRS_Gps_Event_Handler(APRS_t * Aprs,esp_event_base_t event_base, int32_t event_id, GPS_Data_t * Gps_Data) {
	APRS_Event_t event;
	struct tm tm;

	if (event_base != GPS_EVENT)
		return;

	if (GPS_IS_DATA_VALID(Gps_Data->valid, GPS_DATA_VALID_FIX_STATUS) && Gps_Data->fix_status) {

		event.type = APRS_EVENT_GPS;

		// Get timestamp from gps time
		bzero(&tm, sizeof(struct tm));
		tm.tm_sec = Gps_Data->time.seconds;
		tm.tm_min = Gps_Data->time.minutes;
		tm.tm_hour = Gps_Data->time.hours;
		tm.tm_mday = Gps_Data->date.day;
		tm.tm_mon = Gps_Data->date.month;
		tm.tm_year = Gps_Data->date.year;
		
		event.timestamp = mktime(&tm);
		
		if (GPS_IS_DATA_VALID(Gps_Data->valid, GPS_DATA_VALID_2D_FIX)) {
			event.position.latitude = Gps_Data->latitude;
			event.position.longitude = Gps_Data->longitude;
			if (Gps_Data->fix_status == GPS_FIX_STATUS_ESTIMATED)
				event.position.ambiguity = APRS_AMBIGUITY_TENTH_MIN;
			else
				event.position.ambiguity = APRS_AMBIGUITY_NONE;
		}

		if (GPS_IS_DATA_VALID(Gps_Data->valid, GPS_DATA_VALID_3D_FIX)) {
			// Altitude in feet
			event.position.altitude = (1+(Gps_Data->altitude)/((uint32_t)((12<<(GPS_FIXED_POINT-1))*0.0254f+0.5f)))>>1 ;
		}

		if (GPS_IS_DATA_VALID(Gps_Data->valid, GPS_DATA_VALID_HEADING)) {
			event.course.speed = (Gps_Data->speed_kn + (1<<(GPS_FIXED_POINT-1))) >>GPS_FIXED_POINT;
			event.course.dir = (Gps_Data->heading +(1<<(GPS_FIXED_POINT-1))) >> GPS_FIXED_POINT;
		}

		if (xQueueSend(Aprs->queue,&event,portMAX_DELAY) != pdPASS) {
			ESP_LOGE(TAG,"Error sending gps data to aprs");
		}
	}
}

