/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/gps.h
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

#ifndef _GPS_H_
#define _GPS_H_

#include <esp_event.h>

typedef struct GPS_S GPS_t;

#define NMEA_MAX_LENGTH	82

#define GPS_FIXED_POINT_DEG	22	// 1 bit sign, 9 bits integer, 22 bits fractional part
#define GPS_FIXED_POINT		11	// 1 bit sign, 20 bits integer, 11 bits fractional part

#define GPS_DATA_VALID	0x01
#define GPS_DATA_VALID_TIME 0x02
#define GPS_DATA_VALID_DATE 0x04
#define GPS_DATA_VALID_LATITUDE 0x08
#define GPS_DATA_VALID_LONGITUDE 0x10
#define GPS_DATA_VALID_ALTITUDE 0x20
#define GPS_DATA_VALID_SPEED_KN 0x40
#define GPS_DATA_VALID_SPEED_KMH 0x80
#define GPS_DATA_VALID_HEADING 0x100
#define GPS_DATA_VALID_MAGVAR 0x200
#define GPS_DATA_VALID_FIX_STATUS 0x400
#define GPS_DATA_VALID_SAT_IN_USE 0x800
#define GPS_DATA_VALID_HDOP 0x1000
#define GPS_DATA_VALID_GEOID 0x2000
#define GPS_DATA_VALID_DGPS 0x4000
#define GPS_DATA_VALID_MSG 0x8000

#define GPS_DATA_VALID_2D_FIX	(GPS_DATA_VALID_LATITUDE | GPS_DATA_VALID_LONGITUDE)
#define GPS_DATA_VALID_3D_FIX	(GPS_DATA_VALID_2D_FIX | GPS_DATA_VALID_ALTITUDE)

#define GPS_IS_DATA_VALID(data,mask) (!(((data)&(mask))^(mask)))

#define GPS_FIX_STATUS_NOFIX	0
#define GPS_FIX_STATUS_AUTONOMOUS 1
#define GPS_FIX_STATUS_DIFFERENTIAL 2
#define GPS_FIX_STATUS_ESTIMATED 6

enum GPS_Parser_Id_E {
	GPS_PARSER_RMC,
	GPS_PARSER_GGA,
	GPS_PARSER_TXT,
	GPS_PARSER_PMTK001,
	GPS_PARSER_PMTK010,
	GPS_PARSER_PMTK011,
	GPS_PARSER_MAX
};

typedef struct GPS_Date_S {
	uint8_t year;
	uint8_t month;
	uint8_t day;
} GPS_Date_t;

typedef struct GPS_Time_S {
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint16_t milliseconds;
} GPS_Time_t;

typedef struct GPS_Data_S {
	enum GPS_Parser_Id_E id;
	char type[8];
	uint32_t valid;
	uint8_t fix_status;
	GPS_Date_t date;
	GPS_Time_t time;
	int32_t longitude;
 	int32_t latitude;
	int32_t altitude;
	int32_t speed_kn;
	int32_t heading;
	int32_t magvar;
	int8_t sat_in_use;
	int32_t hdop;
	int32_t geoid;
	uint16_t dgps_age;
	uint16_t dgps_id;
	int8_t num_of_msg;
	int8_t msg_number;
	int8_t msg_severity;
	char msg_txt[64];
	int16_t	ack;
	int8_t ack_flag;
	int8_t sys_msg;
} GPS_Data_t;

ESP_EVENT_DECLARE_BASE(GPS_EVENT);

GPS_t * GPS_Init(int uart_num, int Baud, int tx_pin,int rx_pin,int reset_pin);
void GPS_DeInit(GPS_t * Gps);
void GPS_Reset(GPS_t * Gps);
int GPS_Send(GPS_t * Gps,const char * Command);
int GPS_QueueCommand(GPS_t * Gps,const char * Command);

#endif
