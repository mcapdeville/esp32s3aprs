/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/aprs.h
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

#ifndef _APRS_H_
#define _APRS_H_

#include <stdint.h>
#include <time.h>
#include <esp_event.h>
#include "ax25.h"
#include "ax25_lm.h"

#define APRS_MAX_DIGI	8
#define APRS_MAX_FRAME_LEN	(2+((APRS_MAX_DIGI)*sizeof(AX25_Addr_t)) + 2 + 64 + 2)	// SRC+DST+DIGIS[8]+CTRL+PID+INFO[64]+CRC[2]

//#define APRS_SEND_STATUS_WITH_TIMESTAMP
#define APRS_SEND_STATUS_WITH_LOCATOR

#include "gps.h"

#define APRS_DEFAULT_STATUS	"ESP32s3APRS by F4JMZ : https://github.com/mcapdeville/esp32s3aprs"

typedef struct APRS_S APRS_t;

enum APRS_DTI_E {	// DATA TYPE IDENTIFIER
	APRS_DTI_NONE = 0,	// Not an aprs data type
	APRS_DTI_CUR_MICE_R0 = 0x1C,
	APRS_DTI_OLD_MICE_R0 = 0x1d,
	APRS_DTI_POS = '!',
	APRS_DTI_RAW_GPS = '$',
	APRS_DTI_OLD_MICE_TMD700 = '\'',
	APRS_DTI_ITEM = ')',
	APRS_DTI_TEST = ',',
	APRS_DTI_POS_W_TS = '/',
	APRS_DTI_MESSAGE = ':',
	APRS_DTI_OBJECT = ';',
	APRS_DTI_STATION_CAP = '<',
	APRS_DTI_POS_W_MSG = '=',
	APRS_DTI_STATUS = '>',
	APRS_DTI_QUERY = '?',
	APRS_DTI_POS_W_TS_W_MSG = '@',
	APRS_DTI_TELEMETRIE = 'T',
	APRS_DTI_MH_LOCATOR = '[',
	APRS_DTI_WEATHER = '_',
	APRS_DTI_CUR_MICE = '`',
	APRS_DTI_USER_DEF = '{',
	APRS_DTI_THIRD_PARTY = '}',
};

enum APRS_DATA_EXT_E {	// DATA extension type
	APRS_DATA_EXT_NONE = 0,
	APRS_DATA_EXT_CSE,	// Course/Speed
	APRS_DATA_EXT_CSE_NRQ,	// Course/Speed/bearing/NRQ
	APRS_DATA_EXT_WTH,	// Wind Direction/Speed (weather report)
	APRS_DATA_EXT_PHG,	// Power/Height/Gain
	APRS_DATA_EXT_RNG,	// Radio range
	APRS_DATA_EXT_DFS,	// Direction finding
	APRS_DATA_EXT_OBJ,	// Area object
	APRS_DATA_EXT_BEAM,	// Beam heading and ERP
	APRS_DATA_EXT_WEATHER,	// Weather report
	APRS_DATA_EXT_Storm,	// Storm report
};


enum APRS_Status_Type_E {
	APRS_Status_Type_Msg,
	APRS_Status_Type_DHM_Msg,
	APRS_Status_Type_Grid_Msg,
};

enum APRS_Ambiguity_E {
	APRS_AMBIGUITY_NONE = 0,
	APRS_AMBIGUITY_TENTH_MIN,
	APRS_AMBIGUITY_MIN,
	APRS_AMBIGUITY_TEN_MIN,
	APRS_AMBIGUITY_DEG,
	APRS_AMBIGUITY_TEN_DEG,
	APRS_AMBIGUITY_LOC_EXT_SQUARE = 0x10,
	APRS_AMBIGUITY_LOC_SUBSQUARE,
	APRS_AMBIGUITY_LOC_SQUARE,
	APRS_AMBIGUITY_LOC_FIELD,
	APRS_AMBIGUITY_MAX,
};

struct APRS_Position {
	enum APRS_Ambiguity_E ambiguity;
	union {
		struct {
			int32_t latitude;	// deg<<22
			int32_t longitude;	// deg<<22
		};
	};
	int16_t altitude;	// feet
};

struct APRS_Course {
	uint16_t dir;	// deg
	uint16_t speed;	// knot
};

struct APRS_NRQ {
	uint16_t bearing; // bearing in deg
	uint8_t number; // number of hit
	uint16_t range;	// in miles
	uint8_t quality; // quality 0-9
};

struct APRS_PHG {
	uint16_t power; // watt
	uint16_t height;// height above mean ground level in feet
	uint8_t gain;	// antenna gain in dB
	uint16_t dir;	// deg (0 = omni, 360 = N)
};

struct APRS_DFS {
	uint8_t strength; // in S-points
	uint16_t height;// height above mean ground level in feet
	uint8_t gain;	// antenna gain in dB
	uint16_t dir;	// deg (0 = omni, 360 = N)
};

struct APRS_Beam {
	uint16_t heading; // deg
	uint16_t power;	// watt
};

struct APRS_Wind {
	uint16_t dir;	// deg
	uint16_t speed;	// knot
};
struct APRS_Time {
	uint8_t month;
	uint8_t day;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	bool local;
};

struct APRS_Object {
	uint8_t obj_type;
	uint8_t y_off;
	uint8_t obj_color;
	uint8_t x_off;
	union {
		uint16_t corridor_width;	// Coridor half width (\l1yy/Cxx or \l6yy/Cyy)
		char signpost_ovlerlay[4];	// 3 chars signpost (\m) overlay
	};
};

struct APRS_Weather {
	int16_t wind_dir;	// °
	int16_t wind_speed;	// knot
	int16_t gust;		// knot
	int16_t temp;		// °C
	int16_t rain_last_1h;	// 1/100 inch
	int16_t rain_last_24h;	// 1/100 inch
	int16_t rain_since_midnight; // 1/100 inch
	int8_t humidity;	// %
	int16_t pressure;	// 1/10 hPa
	int16_t luminosity;	// W/(m^2)
	int16_t snow_fall;	// inch
	int16_t rain_counter;
};

struct APRS_Storm {
	int32_t dir;		// direction
	int32_t speed;		// speed
	char st[2];		// storm type identifier
	int32_t sustained;	// sustained wind speed
	int32_t gust;		// peak wind speed
	int32_t pressure;	// central pressure
	int32_t r_huricane;	// radius of hjuricaine winds
	int32_t r_storm;	// radius of storm wind
	int32_t r_gale;		// radius of whole gale wind
};

typedef struct APRS_Data_S {
	time_t timestamp;			// Time of arrival
	AX25_Addr_t address[2 + APRS_MAX_DIGI];	// AX25 address : dst, src, digipeatiers
	uint8_t from;				// received from address[from] (src or digipeater)
	enum APRS_DTI_E	type;			// APRS data type identifier
	enum APRS_DATA_EXT_E extension;		// APRS data extension type if any
	char symbol[2];				// 2 char symbol identifier
	bool messaging;				// Station have messaging capability
	uint8_t comp_type;			// T byte of compressed report
	union {
		struct {
			struct APRS_Time time;
			struct APRS_Position position;
			// APRS Data extension
			union {
				struct {
					struct APRS_Course course;
					struct APRS_NRQ nrq;
				};
				struct APRS_PHG phg;
				int16_t range;
				struct APRS_DFS dfs;
				struct APRS_Object object;
				struct APRS_Beam beam;
				struct APRS_Weather weather;
				struct APRS_Storm storm;
			};
			char text[64];
		};
		// Raw gps data
		char nmea[NMEA_MAX_LENGTH];
	};
} APRS_Data_t;

typedef struct APRS_Station_S {
	time_t timestamp;
	AX25_Addr_t callid;
	char symbol[2];
	struct APRS_Position position;
	struct APRS_Course course;
	struct APRS_PHG phg;
	uint16_t range;	// in miles
	struct APRS_Beam beam;
	union {
		struct APRS_DFS dfs;
		struct APRS_NRQ nrq;
	};
	char status[64];
} APRS_Station_t;

ESP_EVENT_DECLARE_BASE(APRS_EVENT);
#define APRS_EVENT_RECEIVE	0

extern const char *APRS_Ssid_Symbol[16];

APRS_t * APRS_Init(AX25_Lm_t * Ax25_Lm);
int APRS_Load_Config(APRS_t *Aprs);
int APRS_Open_Db(APRS_t *Aprs);
int APRS_Start(APRS_t *Aprs);

int APRS_Get_Callid(APRS_t * Aprs, AX25_Addr_t * Callid);
int APRS_Set_Callid(APRS_t * Aprs, const AX25_Addr_t * Callid);
int APRS_Get_Digi(APRS_t * Aprs, int n, AX25_Addr_t * Callid);
int APRS_Get_Digis(APRS_t * Aprs, AX25_Addr_t *Addr, int *n);
int APRS_Set_Digis(APRS_t * Aprs, const AX25_Addr_t *Digi_list, int n_digis);
int APRS_Get_Symbol(APRS_t * Aprs, char Str[2]);
int APRS_Set_Symbol(APRS_t * Aprs, const char Str[2]);
int APRS_Get_Ambiguity(APRS_t * Aprs, uint8_t *Ambiguity);
int APRS_Set_Ambiguity(APRS_t * Aprs, uint8_t Ambiguity);

int APRS_Get_Callid_Str(APRS_t * Aprs, char * Str, int len);
int APRS_Get_Appid_Str(APRS_t * Aprs, char * Str, int len);
int APRS_Get_Digis_Str(APRS_t * Aprs, char * Str, int len);
int APRS_Get_Symbol_Str(APRS_t * Aprs, char *, int len);

int APRS_Get_Status(APRS_t * Aprs, char * Status, int len);
int APRS_Send_Status(APRS_t * Aprs, const char * Status);
int APRS_Get_Position(APRS_t * Aprs, struct APRS_Position * Position);
int APRS_Send_Position(APRS_t * Aprs, const char * Comment);

int APRS_Get_Local(APRS_t * Aprs, APRS_Station_t * Station);
int APRS_Get_Station(APRS_t * Aprs, AX25_Addr_t *Id, APRS_Station_t * Station);
int APRS_Stations_Seq(APRS_t *Aprs, AX25_Addr_t *Addr, int flags, APRS_Station_t *Station);

// Maidenhead locator helper
int APRS_Position_To_Locator(struct APRS_Position * Pos, char * Grid,int len);
int APRS_Locator_To_Position(char * Locator, int Len, struct APRS_Position * Position);

// Raw Frame input
int APRS_Frame_Received_Cb(APRS_t * Aprs, Frame_t * Frame);

#endif
