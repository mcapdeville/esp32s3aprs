/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/gps_parsers.c
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

#include <string.h>
#include <esp_log.h>
#include "gps.h"
#include "gps_parsers.h"

#define TAG "GPS_PARSER"


typedef int (*GPS_Parser_t)(GPS_Data_t *Data,char * ptr, uint8_t idx);

static int GPS_Parser_RMC(GPS_Data_t * Data,char * ptr, uint8_t idx);
static int GPS_Parser_GGA(GPS_Data_t * Data,char * ptr, uint8_t idx);
static int GPS_Parser_TXT(GPS_Data_t * Data,char * ptr, uint8_t idx);
static int GPS_Parser_PMTK001(GPS_Data_t * Data,char * ptr, uint8_t idx);
static int GPS_Parser_PMTK010(GPS_Data_t * Data,char * ptr, uint8_t idx);
static int GPS_Parser_PMTK011(GPS_Data_t * Data,char * ptr, uint8_t idx);

/*
static int GPS_Parser_GLL(GPS_Data_t * Data,char * ptr, uint8_t idx);
static int GPS_Parser_VTG(GPS_Data_t * Data,char * ptr, uint8_t idx);
static int GPS_Parser_ZDA(GPS_Data_t * Data,char * ptr, uint8_t idx);
static int GPS_Parser_GSV(GPS_Data_t * Data,char * ptr, uint8_t idx);
static int GPS_Parser_GSA(GPS_Data_t * Data,char * ptr, uint8_t idx);
*/

static const struct GPS_Parsers_S {
	enum GPS_Parser_Id_E id;
	GPS_Parser_t GPS_Parser;
} GPS_Parsers[] = {
	{GPS_PARSER_RMC,GPS_Parser_RMC},
	{GPS_PARSER_GGA,GPS_Parser_GGA},
	{GPS_PARSER_TXT,GPS_Parser_TXT},
	{GPS_PARSER_PMTK001,GPS_Parser_PMTK001},
	{GPS_PARSER_PMTK010,GPS_Parser_PMTK010},
	{GPS_PARSER_PMTK011,GPS_Parser_PMTK011},
	{GPS_PARSER_MAX,NULL}
};

static const char HexTab[16] = "0123456789ABCDEF";

int GPS_Parse(char * Buffer, size_t Len, GPS_Data_t * Data) {
	int i, field;
	enum GPS_Parser_Id_E id;

	char *n, *ptr, store;

	// clear datas
	bzero(Data,sizeof(GPS_Data_t));

	ptr = Buffer;

	// Look for checksum symbol '*'
	for (i=0, n=ptr; i<Len  && *n!='*';i++,n++);

	if (*n == '*' && i<(Len-2)) {
		n = ptr;
		uint8_t cs;

		if (*ptr == '$' || *ptr == '!') // Accept missing start char if checksum is correct
			n++;
		else
			ESP_LOGW(TAG,"Missing start char");
		ptr = n;
		cs = 0;
		while (*n != '*') {
			if (*n == '$' || *n =='!') {	// Restart after break
				n++;
				ptr = n;
				cs = 0;
			}

			cs ^= *n;
			n++;
		}
		if (*(n+1) != HexTab[((cs>>4)&0x0f)] || *(n+2) != HexTab[cs&0x0f]) {
			ESP_LOGE(TAG,"Invalid checksum");
			return -1;
		}
	} else {
		if (*ptr != '$' && *ptr != '!') {
			ESP_LOGE(TAG,"Missing start char and no checksum");
			return -1;
		} else if (*n == '*') {
			ESP_LOGV(TAG,"%s",Buffer);
			ESP_LOGE(TAG,"start char and checksum indication but malformed checksum");
			return -1;
		}
		ptr++;
	}
	
	store = *n;
	*n = '\0';
	ESP_LOGD(TAG,"Parse sentence : %s",Buffer);

	n=strchr(ptr,',');
	if (n)
		*n = '\0';
	id = 0;
	field = 0;
	while (GPS_Parsers[id].GPS_Parser && GPS_Parsers[id].GPS_Parser(Data,ptr,field))
		id++;
	ESP_LOGD(TAG,"Parser ID : %s",ptr);
	if (n)
	       *n = ',';

	Data->id = GPS_Parsers[id].id;

	if (GPS_Parsers[id].GPS_Parser) {
		while (n) {
			field++;
			ptr = n+1;
			n=strchr(ptr,',');
			if (n)
				*n = '\0';
			ESP_LOGD(TAG,"Field : %d : %s",field,ptr);
			GPS_Parsers[id].GPS_Parser(Data,ptr,field);
			if (n)
				*n = ',';
		}
	} else {
		ESP_LOGW(TAG,"Unknown NMEA sentence : %s",ptr);
	}

	Buffer[i] = store;

	return id;
}

static int GPS_Parser_Get_Time(GPS_Time_t * Time, char * ptr) {
	if (strlen(ptr)>=6) {
		Time->hours = (ptr[0]-'0')*10 + (ptr[1]-'0');
		ptr+=2;
		Time->minutes = (ptr[0]-'0')*10 + (ptr[1]-'0');
		ptr+=2;
		Time->seconds = (ptr[0]-'0')*10 + (ptr[1]-'0');
		ptr+=2;
		Time->milliseconds = 0;
		if (*ptr=='.') {
			int dec=3;
			while (*ptr && dec) {
				dec--;
				Time->milliseconds = (Time->milliseconds*10) + ((*ptr++)-'0');
			}
			if (!dec) {
				if (*ptr>='5')
					Time->milliseconds++;
			}
			else {
				while (dec) {
					Time->milliseconds*=10;
					dec--;
				}
			}
		}
		return 0;
	}
	return -1;
}

static int GPS_Parser_Get_Date(GPS_Date_t * Date, char * ptr) {
	if (strlen(ptr) == 6) {
		Date->day = (ptr[0]-'0')*10 + (ptr[1]-'0');
		ptr+=2;
		Date->month = (ptr[0]-'0')*10 + (ptr[1]-'0');
		ptr+=2;
		Date->year = (ptr[0]-'0')*10 + (ptr[1]-'0');
		if (Date->year == 80 && Date->month == 1 && (Date->day == 5 || Date->day == 6))
			return -1;
		return 0;
	}
	return -1;
}

int GPS_Parser_Get_Coord(int32_t * Coord,char * ptr) {
	char * dot = strchr(ptr,'.');
	int dlen;
	int32_t mult;
	int deg;
	int pos = 0;

	if (!dot)
		return -1;

	dlen = (dot-ptr)-2;
	if (dlen<2 || dlen >3)
		return -1;

	deg = 0;
	while (dlen) { 
		if (*ptr >= '0' && *ptr <= '9') {
			deg = deg*10 + *ptr-'0';
		}
		else
			deg = deg*10;
		pos++;
		ptr++;
		dlen--;
	}

	*Coord=0;
	while (*ptr!='.') {
		if (*ptr >= '0' && *ptr <= '9') {
			*Coord = *Coord*10 + (*ptr-'0');
		}
		else
			*Coord = *Coord*10;
		pos++;
		ptr++;
	}
	*Coord += deg*60;
	mult=60;

	ptr++;
	pos++;
	while (*ptr >= '0' && *ptr <= '9') {
		mult*=10;
		*Coord = *Coord*10 + (*ptr-'0');
		pos++;
		ptr++;
	}

	*Coord = (int32_t)(((((int64_t)*Coord)<<(GPS_FIXED_POINT_DEG+1))/mult+1)>>1);

	return pos;
}

static int GPS_Parser_Get_Frac(int32_t * Val, char * ptr) {
	int mult = 1;

	if (strlen(ptr) >=1) {
		*Val = 0;
		while (*ptr && *ptr!= '.') {
			*Val = *Val*10 + (*ptr-'0');
			ptr++;
		}
		if (*ptr == '.') {
			ptr++;
			while (*ptr) {
				mult *= 10;
				*Val = *Val*10 + (*ptr-'0');
				ptr++;
			}
		}
		*Val = (int32_t)(((((int64_t)*Val)<<(GPS_FIXED_POINT+1))/mult+1)>>1);
		return 0;
	}
	return -1;
}

int GPS_Parser_RMC(GPS_Data_t * Data,char * ptr, uint8_t idx) {
	if (!Data || !ptr || idx>12 || *ptr == '\0')
		return -1;

	switch (idx) {
		case 0:	// Sentence id
			if (strlen(ptr) == 5 && !strcmp(ptr+2,"RMC")) {
				strcpy(Data->type,ptr);
				return 0;
			}
			return -1;
		case 1: // Time
			Data->valid &= ~GPS_DATA_VALID_TIME;
			if (!GPS_Parser_Get_Time(&Data->time,ptr)) {
				Data->valid |= GPS_DATA_VALID_TIME;
				return 0;
			}
			return -1;
		case 2: // Valid
			Data->valid &= ~GPS_DATA_VALID;
			if (strlen(ptr) == 1) {
				Data->valid |= (*ptr=='A')?GPS_DATA_VALID:0;
				return 0;
			}
			return -1;
		case 3: // Latitude
			Data->valid &= ~GPS_DATA_VALID_LATITUDE;
			if (GPS_Parser_Get_Coord(&Data->latitude,ptr) != -1) {
				Data->valid |= GPS_DATA_VALID_LATITUDE;
				return 0;
			}
			return -1;
		case 4: // Latitude cardinal
			if (Data->valid & GPS_DATA_VALID_LATITUDE && strlen(ptr) == 1) {
				if (*ptr == 'S') {
					Data->latitude = -Data->latitude;
					return 0;
				}
				else if (*ptr == 'N') {
					return 0;
				}
			}
			Data->valid &= ~GPS_DATA_VALID_LATITUDE;
			return -1;
		case 5: // Longitude
			Data->valid &= ~GPS_DATA_VALID_LONGITUDE;
			if (GPS_Parser_Get_Coord(&Data->longitude,ptr) != -1) {
				Data->valid |= GPS_DATA_VALID_LONGITUDE;
				return 0;
			}
			return -1;
		case 6: // Longitude cardinal
			if (Data->valid & GPS_DATA_VALID_LONGITUDE && strlen(ptr) == 1) {
				if (*ptr == 'W') {
					Data->longitude = -Data->longitude;
					return 0;
				}
				else if (*ptr == 'E') {
					return 0;
				}
			}
			Data->valid &= ~GPS_DATA_VALID_LONGITUDE;
			return -1;
		case 7: // Speed in knots
			Data->valid &= ~GPS_DATA_VALID_SPEED_KN;
			if (!GPS_Parser_Get_Frac(&Data->speed_kn,ptr)) {
				Data->valid |= GPS_DATA_VALID_SPEED_KN;
				return 0;
			}
			return -1;
		case 8: // heading in degree
			Data->valid &= ~GPS_DATA_VALID_HEADING;
			if (!GPS_Parser_Get_Frac(&Data->heading,ptr)) {
				Data->valid |= GPS_DATA_VALID_HEADING;
				return 0;
			}
			return -1;
		case 9: // Date
			Data->valid &= ~GPS_DATA_VALID_DATE;
			if (!GPS_Parser_Get_Date(&Data->date,ptr)) {
				Data->valid |= GPS_DATA_VALID_DATE;
				return 0;
			}
			return -1;
		case 10: // Magnetic variation in degree
			Data->valid &= ~GPS_DATA_VALID_MAGVAR;
			if (!GPS_Parser_Get_Frac(&Data->magvar,ptr)) {
				Data->valid |= GPS_DATA_VALID_MAGVAR;
				return 0;
			}
			return -1;
		case 11: // Magnetic variation E/W
			if (Data->valid & GPS_DATA_VALID_MAGVAR && strlen(ptr) == 1) {
				if (*ptr == 'W') {
					Data->magvar = -Data->magvar;
					return 0;
				}
				else if (*ptr == 'E') {
					return 0;
				}
			}
			Data->valid &= ~GPS_DATA_VALID_MAGVAR;
			return -1;
		case 12: // Fix status
			Data->valid &= ~GPS_DATA_VALID_FIX_STATUS;
			if (strlen(ptr) == 1) {
				if (*ptr == 'N')
					Data->fix_status = GPS_FIX_STATUS_NOFIX;
				else if (*ptr == 'A')
					Data->fix_status = GPS_FIX_STATUS_AUTONOMOUS;
				else if (*ptr == 'D')
					Data->fix_status = GPS_FIX_STATUS_DIFFERENTIAL;
				else
					return -1;
				Data->valid |= GPS_DATA_VALID_FIX_STATUS;
				return 0;
			}
			return -1;
	}

	return -1;
}

int GPS_Parser_GGA(GPS_Data_t * Data,char * ptr, uint8_t idx) {
	char * end;
	if (!Data || !ptr || idx>14 || *ptr == '\0')
		return -1;

	switch (idx) {
		case 0:	// Sentence id
			if (strlen(ptr) == 5 && !strcmp(ptr+2,"GGA")) {
				strcpy(Data->type,ptr);
				return 0;
			}
			return -1;
		case 1: // Time
			Data->valid &= ~GPS_DATA_VALID_TIME;
			if (!GPS_Parser_Get_Time(&Data->time,ptr)) {
				Data->valid |= GPS_DATA_VALID_TIME;
				return 0;
			}
			return -1;
		case 2: // Latitude
			Data->valid &= ~GPS_DATA_VALID_LATITUDE;
			if (GPS_Parser_Get_Coord(&Data->latitude,ptr) != -1) {
				Data->valid |= GPS_DATA_VALID_LATITUDE;
				return 0;
			}
			return -1;
		case 3: // Latitude cardinal
			if (Data->valid & GPS_DATA_VALID_LATITUDE && strlen(ptr) == 1) {
				if (*ptr == 'S') {
					Data->latitude = -Data->latitude;
					return 0;
				}
				else if (*ptr == 'N') {
					return 0;
				}
			}
			Data->valid &= ~GPS_DATA_VALID_LATITUDE;
			return -1;
		case 4: // Longitude
			Data->valid &= ~GPS_DATA_VALID_LONGITUDE;
			if (GPS_Parser_Get_Coord(&Data->longitude,ptr) != -1) {
				Data->valid |= GPS_DATA_VALID_LONGITUDE;
				return 0;
			}
			return -1;
		case 5: // Longitude cardinal
			if (Data->valid & GPS_DATA_VALID_LONGITUDE && strlen(ptr) == 1) {
				if (*ptr == 'W') {
					Data->longitude = -Data->longitude;
					return 0;
				}
				else if (*ptr == 'E') {
					return 0;
				}
			}
			Data->valid &= ~GPS_DATA_VALID_LONGITUDE;
			return -1;
		case 6: // Fix Status
			Data->valid &= ~GPS_DATA_VALID_FIX_STATUS;
			if (strlen(ptr) == 1 && *ptr >='0' && *ptr <='9') {
				Data->valid |= GPS_DATA_VALID_FIX_STATUS;
				Data->fix_status = atoi(ptr);
				return 0;
			}
			return -1;
		case 7: // Number of satellites in use
			Data->valid &= ~GPS_DATA_VALID_SAT_IN_USE;
			if (strlen(ptr) >= 1 && *ptr >='0' && *ptr <='9') {
				Data->sat_in_use = strtol(ptr,&end,10);
				if (*end == '\0') {
					Data->valid |= GPS_DATA_VALID_SAT_IN_USE;
					return 0;
				}
			}
			return -1;
		case 8: // Horizontal dilution of precision
			Data->valid &= ~GPS_DATA_VALID_HDOP;
			if (!GPS_Parser_Get_Frac(&Data->hdop,ptr)) {
				Data->valid |= GPS_DATA_VALID_HDOP;
				return 0;
			}
			return -1;
		case 9: // Altitude 
			Data->valid &= ~GPS_DATA_VALID_ALTITUDE;
			if (!GPS_Parser_Get_Frac(&Data->altitude,ptr)) {
				Data->valid |= GPS_DATA_VALID_ALTITUDE;
				return 0;
			}
			return -1;
		case 10: // Altitude unit (meter)
			if (*ptr != 'M') {
				Data->valid &= ~GPS_DATA_VALID_ALTITUDE;
				return -1;
			}
			return 0;
		case 11: // Geiod : Height of mean see level above WGS84
			Data->valid &= ~GPS_DATA_VALID_GEOID;
			if (!GPS_Parser_Get_Frac(&Data->geoid,ptr)) {
				Data->valid |= GPS_DATA_VALID_GEOID;
				return 0;
			}
			return -1;
		case 12: // geoid unit (meter)
			if (*ptr != 'M') {
				Data->valid &= ~GPS_DATA_VALID_GEOID;
				return -1;
			}
			return 0;
		case 13: // DGPS Age in seconds
			Data->valid &= ~GPS_DATA_VALID_DGPS;
			if (*ptr >= '0' && *ptr <='9') {
				Data->dgps_age = strtol(ptr,&end,10);
				if (*end == '\0') {
					Data->valid |= GPS_DATA_VALID_DGPS;
					return 0;
				}
			}
			return -1;
		case 14: // DGPS station ID
			if (*ptr >= '0' && *ptr <='9') {
				Data->dgps_id = strtol(ptr,&end,10);
				if (*end == '\0') {
					Data->valid &= ~GPS_DATA_VALID_DGPS;
					return -1;
				}
				return 0;
			}
			return -1;
	}

	return -1;
}


int GPS_Parser_TXT(GPS_Data_t * Data,char * ptr, uint8_t idx) {
	char * end;

	if (!Data || !ptr || idx>4 || *ptr == '\0')
		return -1;

	switch (idx) {
		case 0:	// Sentence id
			if (strlen(ptr) == 5 && !strcmp(ptr+2,"TXT")) {
				strcpy(Data->type,ptr);
				return 0;
			}
			return -1;
		case 1: // Total number of message
			if (strlen(ptr) >= 1 && *ptr >='0' && *ptr <='9') {
				Data->num_of_msg = strtol(ptr,&end,10);
				if (*end == '\0') {
					Data->valid |= GPS_DATA_VALID_MSG;
					return 0;
				}
			}
			Data->valid &= ~GPS_DATA_VALID_MSG;
			return -1;
		case 2: // message number
			if (strlen(ptr) >= 1 && *ptr >='0' && *ptr <='9') {
				Data->msg_number = strtol(ptr,&end,10);
				if (*end == '\0') {
					return 0;
				}
			}
			Data->valid &= ~GPS_DATA_VALID_MSG;
			return -1;
		case 3: // message severity
			if (strlen(ptr) >= 1 && *ptr >='0' && *ptr <='9') {
				Data->msg_severity = strtol(ptr,&end,10);
				if (*end == '\0') {
					return 0;
				}
			Data->valid &= ~GPS_DATA_VALID_MSG;
			}
			return -1;
		case 4: // message text
			if (strlen(ptr) >= 1) {
				if (Data->msg_number <= Data->num_of_msg) {
					strncpy(Data->msg_txt,ptr,sizeof(Data->msg_txt)-1);
					Data->valid |= GPS_DATA_VALID_MSG;
					switch (Data->msg_severity) {
						case 0:
							ESP_LOGE(TAG,"GPTXT : %d/%d : Error : %s",Data->msg_number,Data->num_of_msg,Data->msg_txt);
							break;
						case 1:  
							ESP_LOGW(TAG,"GPTXT : %d/%d : Warning : %s",Data->msg_number,Data->num_of_msg,Data->msg_txt);
							break;
						case 2:  
							ESP_LOGD(TAG,"GPTXT : %d/%d : Notice : %s",Data->msg_number,Data->num_of_msg,Data->msg_txt);
							break;
						case 7:  
							ESP_LOGD(TAG,"GPTXT : %d/%d : User :  %s",Data->msg_number,Data->num_of_msg,Data->msg_txt);
							break;
						default:  
							ESP_LOGD(TAG,"GPTXT : %d/%d : (%d) :  %s",Data->msg_severity,Data->msg_number,
									Data->num_of_msg,Data->msg_txt);
							break;
					}
					return 0;
				}
			}
			Data->valid &= ~GPS_DATA_VALID_MSG;
			return -1;
	}

	return -1;
}

int GPS_Parser_PMTK001(GPS_Data_t * Data,char * ptr, uint8_t idx) {

	if (!Data || !ptr || idx>2 || *ptr == '\0')
		return -1;

	switch (idx) {
		case 0:	// Sentence id
			if (strlen(ptr) == 7 && !strcmp(ptr,"PMTK001")) {
				strcpy(Data->type,ptr);
				return 0;
			}
			return -1;
		case 1: // Command acked
			if (strlen(ptr) >= 1 && *ptr >='0' && *ptr <='9') {
				Data->ack = atoi(ptr);
				return 0;
			}
			return -1;
		case 2: // Ack flag
			if (strlen(ptr) >= 1 && *ptr >='0' && *ptr <='3') {
				Data->ack_flag = atoi(ptr);
				switch (Data->ack_flag) {
					case 0:
						ESP_LOGE(TAG,"Invalid PMTK%03d packet.",Data->ack);
						break;
					case 1:
						ESP_LOGE(TAG,"Unsuported PMTK%03d command.",Data->ack);
						break;
					case 2:
						ESP_LOGE(TAG,"Failed PMTK%03d command.",Data->ack);
						break;
					case 3:
						ESP_LOGD(TAG,"Succeded PMTK%03d command.",Data->ack);
						break;
					default :
						ESP_LOGE(TAG,"Invalid flag %d for PMTK%03d command.",Data->ack_flag,Data->ack);
						break;
				}
				return 0;
			}
			return -1;
	}

	return -1;
}

int GPS_Parser_PMTK010(GPS_Data_t * Data,char * ptr, uint8_t idx) {
	if (!Data || !ptr || idx>1 || *ptr == '\0')
		return -1;

	switch (idx) {
		case 0:	// Sentence id
			if (strlen(ptr) == 7 && !strcmp(ptr,"PMTK010")) {
				strcpy(Data->type,ptr);
				return 0;
			}
			return -1;
		case 1: // system message
			if (strlen(ptr) >= 1 && *ptr >='0' && *ptr <='9') {
				Data->sys_msg = atoi(ptr);
				switch (Data->sys_msg) {
					case 0:
						ESP_LOGW(TAG,"Unknown GPS receiver state");
						break;
					case 1:
						ESP_LOGD(TAG,"GPS receiver is starting");
						break;
					case 2:
						ESP_LOGD(TAG,"GPS receiver request EPO");
						break;
					case 3:
						ESP_LOGD(TAG,"GPS receiver successfully transitioned to normal mode");
						break;
					default:
						ESP_LOGW(TAG,"Unknown GPS receiver system message %d",Data->sys_msg);
				}
				return 0;
			}
			return -1;
	}

	return -1;
}

int GPS_Parser_PMTK011(GPS_Data_t * Data,char * ptr, uint8_t idx) {
	if (!Data || !ptr || idx>1 || *ptr == '\0')
		return -1;

	switch (idx) {
		case 0:	// Sentence id
			if (strlen(ptr) == 7 && !strcmp(ptr,"PMTK011")) {
				strcpy(Data->type,ptr);
				return 0;
			}
			return -1;
		case 1: // system  message
			if (strlen(ptr) >= 1) {
				strncpy(Data->msg_txt,ptr,sizeof(Data->msg_txt)-1);
				Data->num_of_msg = 1;
				Data->msg_number = 1;
				Data->valid |= GPS_DATA_VALID_MSG;
				ESP_LOGD(TAG,"System message : %s",Data->msg_txt);
				return 0;
			}
			return -1;
	}

	return -1;
}

int GPS_Parser_GSA(GPS_Data_t * Data,char * ptr, uint8_t idx) {
	return -1;
}

int GPS_Parser_GLL(GPS_Data_t * Data,char * ptr, uint8_t idx) {
	return -1;
}

int GPS_Parser_GSV(GPS_Data_t * Data,char * ptr, uint8_t idx) {
	return -1;
}

int GPS_Parser_VTG(GPS_Data_t * Data,char * ptr, uint8_t idx) {
	return -1;
}
