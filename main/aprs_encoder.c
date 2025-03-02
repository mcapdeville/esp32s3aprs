/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/aprs_encoder.c
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

#include <esp_log.h>

#include "aprs.h"
#include "aprs_encoder.h"
#include "framebuff.h"

#define TAG "APRS_ENCODER"

static int APRS_Encode_Ts(APRS_Data_t * Data, uint8_t * ptr, int len);
static int APRS_Encode_Pos(APRS_Data_t * Data, uint8_t * ptr, int len);
static int APRS_Encode_Locator(APRS_Data_t * Data, uint8_t * ptr, int len);
static int APRS_Encode_Altitude(APRS_Data_t * Data, uint8_t * ptr, int len);
static int APRS_Encode_Comment(APRS_Data_t * Data, uint8_t * ptr, int len);
static int APRS_Encode_Beam(APRS_Data_t * Data, uint8_t * ptr, int len);

static int APRS_Encode_Extension(APRS_Data_t * Data, uint8_t * ptr, int len);

int APRS_Encode(APRS_Data_t * Data, Frame_t * Frame) {
	uint8_t *ptr;
	int ret, pos, i;

	if (!Data || !Frame)
		return -1;

	if (Frame->frame_size < APRS_MAX_FRAME_LEN) {
		ESP_LOGE(TAG,"Frame too short for encoding report");
		return -1;
	}

	ptr = Frame->frame;
	pos = 0;

	// encode addresses
	memcpy(ptr,&Data->address[0], 2*sizeof(AX25_Addr_t));
	ptr += 2*sizeof(AX25_Addr_t);
	pos += 2*sizeof(AX25_Addr_t);

	i=1;
	while (!(Data->address[i].ssid & 1)) {
		i++;
		memcpy(ptr, &Data->address[i], sizeof(AX25_Addr_t));
		ptr += sizeof(AX25_Addr_t);
		pos += sizeof(AX25_Addr_t);
	}

	ESP_LOGD(TAG,"Found %d digis",i-1);

	// Set control byte
	*(ptr++) = 0x03;	// UI frame
	pos++;

	// Set PID
	*(ptr++) = 0xf0;
	pos++;

	// Set DIT
	*(ptr++) = Data->type;
	pos++;

	// Set Datas
	switch (Data->type) {
		case APRS_DTI_POS_W_TS:
		case APRS_DTI_POS_W_TS_W_MSG:
			ret = APRS_Encode_Ts(Data, ptr, APRS_MAX_FRAME_LEN-2 - pos);
			if (ret < 0) {
				ESP_LOGE(TAG,"Error encoding timestamp");
				return -1;
			}
			pos += ret;
			ptr += ret;
			/* FALLTHRU */
		case APRS_DTI_POS:
		case APRS_DTI_POS_W_MSG:
			if (!Data->position.latitude && !Data->position.longitude) { // no position information
				ESP_LOGE(TAG,"Can't encode position without latitude/longitude datas.");
				return -1;
			}

			ret = APRS_Encode_Pos(Data, ptr, APRS_MAX_FRAME_LEN-2 - pos);
			if (ret < 0) {
				ESP_LOGE(TAG,"Error encoding position");
				return -1;
			}
			pos += ret;
			ptr += ret;

			if (Data->extension) { // Add data extension
				ret = APRS_Encode_Extension(Data, ptr, APRS_MAX_FRAME_LEN-2 - pos);
				if (ret < 0) {
					ESP_LOGE(TAG,"Error encoding data extension");
					return -1;
				}
				pos += ret;
				ptr += ret;
			}

			if (Data->position.altitude) { // Add altitude
				ret = APRS_Encode_Altitude(Data, ptr, APRS_MAX_FRAME_LEN-2 - pos);
				if (ret < 0) {
					ESP_LOGE(TAG,"Error encoding altitude");
					return -1;
				}
				pos += ret;
				ptr += ret;
			}

			ret = APRS_Encode_Comment(Data, ptr, APRS_MAX_FRAME_LEN-2 - pos);
			if (ret < 0) {
				ESP_LOGE(TAG,"Error encoding comment text");
				return -1;
			}
			pos += ret;
			ptr += ret;

			Frame->frame_len = pos;
			return pos;
		case APRS_DTI_STATUS:
			if (Data->time.day) { // With timestamp
				ret = APRS_Encode_Ts(Data, ptr, APRS_MAX_FRAME_LEN-2 - pos);
				if (ret < 0) {
					ESP_LOGE(TAG,"Error encoding timestamp");
					return -1;	
				}
				pos += ret;
				ptr += ret;
			} else if ((Data->position.latitude || Data->position.longitude) &&	Data->symbol[0] ) { // With locator
				ret = APRS_Encode_Locator(Data, ptr, APRS_MAX_FRAME_LEN-2 - pos);
				if (ret < 0) {
					ESP_LOGE(TAG,"Error encoding locator");
					return -1;	
				}
				pos += ret;
				ptr += ret;

				*(ptr++) = Data->symbol[0];
				*(ptr++) = Data->symbol[1];
				pos += 2;

				if (Data->text[0] || Data->extension == APRS_DATA_EXT_BEAM) { // Add space if status text or beam report
					*(ptr++) = ' ';
					pos++;
				}
			}

			ESP_LOGD(TAG,"%d bytes for comment at pos %d.",APRS_MAX_FRAME_LEN-2-pos, pos);
			ret = APRS_Encode_Comment(Data, ptr, APRS_MAX_FRAME_LEN-2 - pos);
			if (ret < 0) {
				ESP_LOGE(TAG,"Error encoding status text");
				return -1;	
			}
			pos += ret;
			ptr += ret;

			if (Data->extension == APRS_DATA_EXT_BEAM) { // Add beam power/heading
				ret = APRS_Encode_Beam(Data, ptr, APRS_MAX_FRAME_LEN-2 - pos);
				if (ret < 0) {
					ESP_LOGE(TAG,"Error encoding beam power/heading.");
					return -1;	
				}
				pos += ret;
				ptr += ret;
			}

			Frame->frame_len = pos;
			return pos;

		default:
			ESP_LOGE(TAG,"Unimplemented encoder for DTI \"%c\".",Data->type);
			return -1;
	}
}

static int APRS_Encode_Ts(APRS_Data_t * Data, uint8_t * ptr, int len) {
	int pos =0;

	if (Data->type == APRS_DTI_WEATHER) {
		if (len < 8) {
			ESP_LOGE(TAG,"Timestamp need 8 chars for weather report");
			return -1;
		}
		*(ptr++) = (Data->time.month/10) + '0';
		*(ptr++) = (Data->time.month%10) + '0';
		pos = 2;
	} else if (len<7) {
		ESP_LOGE(TAG,"Timestamp need 7 chars");
		return -1;
	}

	if (Data->type == APRS_DTI_STATUS && Data->time.local) {
		ESP_LOGE(TAG,"Time in status report MUST NOT be local time");
		return -1;
	}

	if (Data->type == APRS_DTI_STATUS && Data->time.day == 0) {
		ESP_LOGE(TAG,"Time in status report MUST be DDHHMMz format");
		return -1;
	}

	if (Data->time.day) {
		*(ptr++) = (Data->time.day/10) + '0';
		*(ptr++) = (Data->time.day%10) + '0';
		pos+=2;
	}

	*(ptr++) = (Data->time.hours/10) + '0';
	*(ptr++) = (Data->time.hours%10) + '0';
	*(ptr++) = (Data->time.minutes/10) + '0';
	*(ptr++) = (Data->time.minutes%10) + '0';
	pos += 4;

	if (Data->time.day)
		*(ptr++) = Data->time.local?'/':'z';
	else {
		*(ptr++) = (Data->time.seconds/10) + '0';
		*(ptr++) = (Data->time.seconds%10) + '0';
		*(ptr++) = 'h';
		pos+=2;
	}
	pos += 1;

	return pos;
}

int APRS_Encode_Pos(APRS_Data_t * Data, uint8_t * ptr, int len) {
	uint32_t deg,min,cent;
	uint8_t dir;

	if (len < 19)
		return -1;

	if (Data->position.ambiguity > APRS_AMBIGUITY_TEN_DEG) {
		ESP_LOGE(TAG,"Ambiguity must be in degree and can't be more than 10°.");
		return -1;
	}

	if (Data->position.latitude<0) {
		deg = -Data->position.latitude;
		dir = 'S';
	} else {
		deg = Data->position.latitude;
		dir = 'N';
	}

	min = ((deg&((1<<GPS_FIXED_POINT_DEG)-1))*60);
	deg = deg>>GPS_FIXED_POINT_DEG;
	cent = ((min&((1<<GPS_FIXED_POINT_DEG)-1))*100 + (1<<(GPS_FIXED_POINT-1)))>>GPS_FIXED_POINT_DEG;
	min >>= GPS_FIXED_POINT_DEG;

	ESP_LOGD(TAG,"latitude = %ld, deg = %lu, min = %lu , cent = %lu",Data->position.latitude,deg,min,cent);

	*(ptr++) = (deg/10) + '0';
	if (Data->position.ambiguity < APRS_AMBIGUITY_TEN_DEG) {
		*(ptr++) = (deg%10) + '0';
		if (Data->position.ambiguity < APRS_AMBIGUITY_DEG) {
			*(ptr++) = (min/10) + '0';
			if (Data->position.ambiguity < APRS_AMBIGUITY_TEN_MIN) {
				*(ptr++) = (min%10) + '0';
				if (Data->position.ambiguity < APRS_AMBIGUITY_MIN) {
					*(ptr++) = '.';
					*(ptr++) = (cent/10) + '0';
					if (Data->position.ambiguity < APRS_AMBIGUITY_TENTH_MIN)
						*(ptr++) = (cent%10) + '0';
					else 
						*(ptr++) = ' ';
				} else {
					*(ptr++) = '.';
					*(ptr++) = ' ';
						*(ptr++) = ' ';
				}
			} else {
				*(ptr++) = ' ';
					*(ptr++) = '.';
					*(ptr++) = ' ';
						*(ptr++) = ' ';
			}
		} else {
			*(ptr++) = ' ';
				*(ptr++) = ' ';
					*(ptr++) = '.';
					*(ptr++) = ' ';
						*(ptr++) = ' ';
		}
	} else {
		*(ptr++) = ' ';
			*(ptr++) = ' ';
				*(ptr++) = ' ';
					*(ptr++) = '.';
					*(ptr++) = ' ';
						*(ptr++) = ' ';
	}

	*(ptr++) = dir;
	*(ptr++) = Data->symbol[0];

	if (Data->position.longitude<0) {
		deg = -Data->position.longitude;
		dir = 'W';
	} else {
		deg = Data->position.longitude;
		dir = 'E';
	}

	min = ((deg&((1<<GPS_FIXED_POINT_DEG)-1))*60);
	deg = deg>>GPS_FIXED_POINT_DEG;
	cent = ((min&((1<<GPS_FIXED_POINT_DEG)-1))*100 + (1<<(GPS_FIXED_POINT-1)))>>GPS_FIXED_POINT_DEG;
	min >>= GPS_FIXED_POINT_DEG;

	*(ptr++) = (deg/100) + '0';
	deg %= 100;
	*(ptr++) = (deg/10) + '0';
	if (Data->position.ambiguity < APRS_AMBIGUITY_TEN_DEG) {
		*(ptr++) = (deg%10) + '0';
		if (Data->position.ambiguity < APRS_AMBIGUITY_DEG) {
			*(ptr++) = (min/10) + '0';
			if (Data->position.ambiguity < APRS_AMBIGUITY_TEN_MIN) {
				*(ptr++) = (min%10) + '0';
				if (Data->position.ambiguity < APRS_AMBIGUITY_MIN) {
					*(ptr++) = '.';
					*(ptr++) = (cent/10) + '0';
					if (Data->position.ambiguity < APRS_AMBIGUITY_TENTH_MIN)
						*(ptr++) = (cent%10) + '0';
					else 
						*(ptr++) = ' ';
				} else {
					*(ptr++) = '.';
					*(ptr++) = ' ';
						*(ptr++) = ' ';
				}
			} else {
				*(ptr++) = ' ';
					*(ptr++) = '.';
					*(ptr++) = ' ';
						*(ptr++) = ' ';
			}
		} else {
			*(ptr++) = ' ';
				*(ptr++) = ' ';
					*(ptr++) = '.';
					*(ptr++) = ' ';
						*(ptr++) = ' ';
		}
	} else {
		*(ptr++) = ' ';
			*(ptr++) = ' ';
				*(ptr++) = ' ';
					*(ptr++) = '.';
					*(ptr++) = ' ';
						*(ptr++) = ' ';
	}

	*(ptr++) = dir;
	*(ptr++) = Data->symbol[1];

	return 19;
}

int APRS_Encode_Locator(APRS_Data_t * Data, uint8_t * ptr, int len) {
	int pos;

	if (Data->position.ambiguity < APRS_AMBIGUITY_LOC_SUBSQUARE)
		Data->position.ambiguity = APRS_AMBIGUITY_LOC_SUBSQUARE;
	else if (Data->position.ambiguity > APRS_AMBIGUITY_LOC_SQUARE)
		Data->position.ambiguity = APRS_AMBIGUITY_LOC_SQUARE;

	pos = APRS_Position_To_Locator(&Data->position, (char*)ptr, len);

	ptr[pos] = '\0';
	ESP_LOGD(TAG,"latitude %d.%03d longitude %d.%03d\n\t\tlocator %s",
			(int)(Data->position.latitude>>GPS_FIXED_POINT_DEG),
			(int)((Data->position.latitude&((1<<GPS_FIXED_POINT_DEG)-1))*1000)>>GPS_FIXED_POINT_DEG,
			(int)(Data->position.longitude>>GPS_FIXED_POINT_DEG),
			(int)((Data->position.longitude&((1<<GPS_FIXED_POINT_DEG)-1))*1000)>>GPS_FIXED_POINT_DEG,
			ptr
		);

	return pos;
}

int APRS_Encode_Altitude(APRS_Data_t * Data, uint8_t * ptr, int len) {
	int32_t val;

	if (len < 9)
		return -1;

	val = (Data->position.altitude + (1<<(GPS_FIXED_POINT-1))) >> GPS_FIXED_POINT;

	*(ptr++) = '/';
	*(ptr++) = 'A';
	*(ptr++) = '=';
	*(ptr++) = val/100000 + '0';
	val %= 100000;
	*(ptr++) = val/10000 + '0';
	val %= 10000;
	*(ptr++) = val/1000 + '0';
	val %= 1000;
	*(ptr++) = val/100 + '0';
	val %= 100;
	*(ptr++) = val/10 + '0';
	*(ptr++) = val%10 + '0';

	return 9;
}

int APRS_Encode_Comment(APRS_Data_t * Data, uint8_t * ptr, int len) {
	int i;

	for (i=0; i<sizeof(Data->text) && i<len; i++)
		if (!Data->text[i])
			break;

	memcpy(ptr, Data->text, i);

	return i;
}

static int APRS_Encode_Beam(APRS_Data_t * Data, uint8_t * ptr, int len) {
	uint8_t code;

	if (len < 3)
		return -1;

	*(ptr++) = '^';
	code = (Data->beam.heading+5)/10;
	if (code > 35)
		code = 0;

	if (code < 10)
		*(ptr++) = code + '0';
	else 
		*(ptr++) = code + 'A';

	code = (uint8_t)(sqrt((float)Data->beam.power/10.0) + 0.5f);
	*(ptr++) = code + '0';

	return 3;
}

static int APRS_Encode_Extension(APRS_Data_t * Data, uint8_t * ptr, int len) {
	int pos = 0;
	uint16_t val;

	if (len < 7)
		return -1;

	switch (Data->extension) {
		case APRS_DATA_EXT_CSE:		// Course/Speed
		case APRS_DATA_EXT_CSE_NRQ:	// Course/Speed/Bearing/NRQ
			val = Data->course.dir % 360;
			*(ptr++) = val/100 + '0';
			val %= 100;
			*(ptr++) = val/10 + '0';
			*(ptr++) = val%10 + '0';
			*(ptr++) = '/';
			if (Data->course.speed > 999)
				val = 999;
			else
				val = Data->course.speed;
			*(ptr++) = val/100 + '0';
			val %= 100;
			*(ptr++) = val/10 + '0';
			*(ptr++) = val%10 + '0';
			pos += 7;
			if (Data->extension == APRS_DATA_EXT_CSE_NRQ && (len-pos)>=8) {
				*(ptr++) = '/';
				val = Data->nrq.bearing % 360;
				*(ptr++) = val/100 + '0';
				val %= 100;
				*(ptr++) = val/10 + '0';
				*(ptr++) = val%10 + '0';
				*(ptr++) = '/';
				*(ptr++) = Data->nrq.number %10;
				*(ptr++) = Data->nrq.range % 10;
				*(ptr++) = Data->nrq.quality % 10;
				pos += 8;
			}
			break;
		case APRS_DATA_EXT_WTH:		// Wind Direction/Speed (weather report)
			val = Data->weather.wind_dir % 360;
			*(ptr++) = val/100 + '0';
			val %= 100;
			*(ptr++) = val/10 + '0';
			*(ptr++) = val%10 + '0';
			*(ptr++) = '/';
			if (Data->weather.wind_speed > 999)
				val = 999;
			else
				val = Data->weather.wind_speed % 1000;
			*(ptr++) = val/100 + '0';
			val %= 100;
			*(ptr++) = val/10 + '0';
			*(ptr++) = val%10 + '0';
			pos += 7;
			break;
		case APRS_DATA_EXT_PHG:		// Power/Height/Gain
			*(ptr++) = 'P';
			*(ptr++) = 'H';
			*(ptr++) = 'G';
			*(ptr++) = (uint8_t)(sqrt((float)Data->phg.power) + 0.5f) + '0';
			*(ptr++) = (uint8_t)(log2((float)Data->phg.height/10.0f) + 0.5f) + '0';
			*(ptr++) = Data->phg.gain + '0';
			*(ptr++) = (Data->phg.dir + 22)/45 + '0';
			pos += 7;
			break;
		case APRS_DATA_EXT_RNG:		// Radio range
			*(ptr++) = 'R';
			*(ptr++) = 'N';
			*(ptr++) = 'G';
			val = Data->range % 10000;
			*(ptr++) = val/1000 + '0';
			val %= 100;
			*(ptr++) = val/100 + '0';
			val %= 100;
			*(ptr++) = val/10 + '0';
			*(ptr++) = val%10 + '0';
			pos += 7;
			break;
		case APRS_DATA_EXT_DFS:		// Direction finding
			*(ptr++) = 'D';
			*(ptr++) = 'F';
			*(ptr++) = 'S';
			*(ptr++) = Data->dfs.strength + '0';
			*(ptr++) = (uint8_t)(log2((float)Data->dfs.height/10.0f) + 0.5f) + '0';
			*(ptr++) = Data->dfs.gain + '0';
			*(ptr++) = (Data->dfs.dir + 22)/45 + '0';
			pos += 7;
			break;
		default:
			ESP_LOGE(TAG,"Unimplemented encoder for data extexion %d",Data->extension);
			return -1;
	}

	return pos;
}
