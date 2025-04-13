/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/aprs_parsers.c
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

#include "aprs_parsers.h"
#include "aprs.h"
#include "gps.h"
#include "gps_parsers.h"

#include "framebuff.h"
#include "ax25.h"
#include "gps_parsers.h"

#include <esp_log.h>
#include <math.h>
#include <ctype.h>

#define TAG "APRS_PARSERS"

static int APRS_Parser_CUR_MICE_R0(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_OLD_MICE_R0(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_POS(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_RAW_GPS(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_OLD_MICE_TMD700(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_ITEM(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_TEST(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_MESSAGE(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_OBJECT(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_STATION_CAP(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_STATUS(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_QUERY(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_TELEMETRIE(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_MH_LOCATOR(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_WEATHER(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_CUR_MICE(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_USER_DEF(int pos, Frame_t * Frame, APRS_Data_t * Data);
static int APRS_Parser_THIRD_PARTY(int pos, Frame_t * Frame, APRS_Data_t * Data);

const struct APRS_Parsers_S {
	enum APRS_DTI_E	type;
	int	(*parser)(int pos, Frame_t * Frame, APRS_Data_t * Date);
} APRS_Parsers[] = {
	{APRS_DTI_CUR_MICE_R0, APRS_Parser_CUR_MICE_R0},
	{APRS_DTI_OLD_MICE_R0, APRS_Parser_OLD_MICE_R0},
	{APRS_DTI_POS, APRS_Parser_POS},
	{APRS_DTI_RAW_GPS, APRS_Parser_RAW_GPS},
	{APRS_DTI_OLD_MICE_TMD700, APRS_Parser_OLD_MICE_TMD700},
	{APRS_DTI_ITEM, APRS_Parser_ITEM},
	{APRS_DTI_TEST, APRS_Parser_TEST},
	{APRS_DTI_POS_W_TS, APRS_Parser_POS},
	{APRS_DTI_MESSAGE, APRS_Parser_MESSAGE},
	{APRS_DTI_OBJECT, APRS_Parser_OBJECT},
	{APRS_DTI_STATION_CAP, APRS_Parser_STATION_CAP},
	{APRS_DTI_POS_W_MSG, APRS_Parser_POS},
	{APRS_DTI_STATUS, APRS_Parser_STATUS},
	{APRS_DTI_QUERY, APRS_Parser_QUERY},
	{APRS_DTI_POS_W_TS_W_MSG, APRS_Parser_POS},
	{APRS_DTI_TELEMETRIE, APRS_Parser_TELEMETRIE},
	{APRS_DTI_MH_LOCATOR, APRS_Parser_MH_LOCATOR},
	{APRS_DTI_WEATHER, APRS_Parser_WEATHER},
	{APRS_DTI_CUR_MICE, APRS_Parser_CUR_MICE},
	{APRS_DTI_USER_DEF, APRS_Parser_USER_DEF},
	{APRS_DTI_THIRD_PARTY, APRS_Parser_THIRD_PARTY},
	{0,NULL}		// end marker
};

int APRS_Parse(Frame_t * Frame, APRS_Data_t * Data) {
	int i, pos, ret;
	char addr[10];
	char store = Frame->frame[Frame->frame_len-2];


	if (!Frame || Frame->frame_len < (HDLC_MIN_FRAME_LEN+2) || !Data)
		return -1;

	bzero(Data,sizeof(APRS_Data_t));

	pos = 0;
	i=0;
	Data->from = 1;
	do {
		memcpy(&Data->address[i],&Frame->frame[pos],sizeof(AX25_Addr_t));
		AX25_Addr_To_Str(&Data->address[i],addr,sizeof(addr));
		if (i > 1 && Data->address[i].ssid & 0x80)
			Data->from = i;
		ESP_LOGD(TAG,"Address[%d] = %s%c",i,addr, Data->address[i].ssid & 0x80?'*':'\0');
		i++;
		pos+=sizeof(AX25_Addr_t);
	} while (!(Data->address[i-1].ssid & 1) && i < (2+APRS_MAX_DIGI));

	if (i == (2+APRS_MAX_DIGI)) {
		ESP_LOGE(TAG,"Too many digipeater");
		return -1;
	}

	if (Frame->frame_len < (pos+5)) { // pos + ctrl + pid + DTI + crc16
		ESP_LOGE(TAG,"Frame too short");
		return -1;
	}

	if (Frame->frame[pos] != 0x03)	{// Control field 
		ESP_LOGE(TAG,"Not an UI frame (ctrl = 0x%02x)",Frame->frame[pos]);
		return -1; // frame is not an AX25 UI frame
	}
	pos++;

	if (Frame->frame[pos] != 0xf0) { // Pid field
		ESP_LOGE(TAG,"Bad PID (PID = 0x%02x)",Frame->frame[pos]);
		return -1; // Invalid PID
	}
	pos++;

	Frame->frame[Frame->frame_len-2] = '\0';
	ESP_LOGD(TAG,"Frame : %s",&Frame->frame[pos]);

	i=0;
	ret = 0;
	while (APRS_Parsers[i].type) {
		if (Frame->frame[pos] == APRS_Parsers[i].type && APRS_Parsers[i].parser) {
			Data->type = APRS_Parsers[i].type;
			ESP_LOGD(TAG,"DTI = %c",Data->type);
			pos++;
			ret = APRS_Parsers[i].parser(pos,Frame,Data);
			break;

		}
		i++;
	}

	if (!APRS_Parsers[i].type) {
		// Exeption for '!' on X1J TNC digipeaters
		i=1;
		while (i<40) {
			if (Frame->frame[pos+i] == APRS_DTI_POS) {
				Data->type = APRS_DTI_POS;
				ESP_LOGD(TAG,"Found DTI = %c at pos %d ", Frame->frame[pos+i],i);
				ret = APRS_Parser_POS(pos+i,Frame,Data);
				if (ret != -1) {
					// Get comment before '!'
					memcpy(Data->text,Frame->frame+pos,i);
					pos += ret + i;

					ret = Frame->frame_len-(pos+1);
					if (ret > sizeof(Data->text)-1)
						ret = sizeof(Data->text)-1;
					if (ret) {
						Data->text[i] = ' ';
						i++;
						memcpy(Data->text+i,&Frame->frame[pos],ret);
					}
					Data->text[i+ret] = '\0';
					return Data->type;
				}
				
			}
			i++;
		}

		if (i==40) {
			ESP_LOGE(TAG,"Unknown DTI = %c",Frame->frame[pos]);
			return -1;
		}
	}

	Frame->frame[Frame->frame_len-2] = store;

	if (ret == -1) {
		ESP_LOGE(TAG,"Parser error");
		return -1;
	}

	pos += ret;

	// Get Comment if any
	i = Frame->frame_len-(pos+2);
	if (i > sizeof(Data->text)-1)
		i = sizeof(Data->text)-1;
	if (i) {
		memcpy(Data->text,&Frame->frame[pos],i);
		ESP_LOGD(TAG,"Comment : %s",Data->text);
	}

	Data->text[i] = '\0';


	return Data->type;
}

static int APRS_Get_Time(uint8_t * Ptr, APRS_Data_t * Data) {
	int n;
	// Count number of digit
	for (n=0;n<8 && isdigit(Ptr[n]);n++);
	if (n==0 && (Ptr[6] == 'z' || Ptr[6] == '/' || Ptr [6] == 'h') && (strncmp((char*)Ptr,"......",6) || strncmp((char*)Ptr,"      ",6))) {
		bzero(&Data->time,sizeof(Data->time));
		return 7;
	}

	if (n==6 && (Ptr[6] == 'z' || Ptr[6] == '/')) {
		Data->time.month = 0;
		Data->time.day = (Ptr[0]-'0')*10 + (Ptr[1]-'0');
		Data->time.hours = (Ptr[2]-'0')*10 + (Ptr[3]-'0');
		Data->time.minutes = (Ptr[4]-'0')*10 + (Ptr[5]-'0');
		Data->time.seconds = 0;
		Data->time.local = Ptr[6] == '/';
		ESP_LOGD(TAG,"DHM : %02d%02d%02d%c",
				Data->time.day,
				Data->time.hours,
				Data->time.minutes,
				Data->time.local?'/':'z');
		return 7;
	} else if (n == 6 && Ptr[6] == 'h') {
		Data->time.month = 0;
		Data->time.day = 0;
		Data->time.hours = (Ptr[0]-'0')*10 + (Ptr[1]-'0');
		Data->time.minutes = (Ptr[2]-'0')*10 + (Ptr[3]-'0');
		Data->time.seconds = (Ptr[4]-'0')*10 + (Ptr[5]-'0');
		Data->time.local = false;
		ESP_LOGD(TAG,"HMS : %02d%02d%02dh",
				Data->time.hours,
				Data->time.minutes,
				Data->time.seconds);
		return 7;
	} else if (n==8) {
		Data->time.month = (Ptr[0]-'0')*10 + (Ptr[1]-'0');
		Data->time.day = (Ptr[2]-'0')*10 + (Ptr[3]-'0');
		Data->time.hours = (Ptr[4]-'0')*10 + (Ptr[5]-'0');
		Data->time.minutes = (Ptr[6]-'6')*10 + (Ptr[7]-'7');
		Data->time.seconds = 255;
		Data->time.local = false;
		ESP_LOGD(TAG,"MDHM : %02d%02d%02d%02d",
				Data->time.month,
				Data->time.day,
				Data->time.hours,
				Data->time.minutes);
		return 8;
	} else 
		bzero(&Data->time, sizeof(struct APRS_Time));

	return -1;
}

static int APRS_Get_Locator(uint8_t * Ptr, APRS_Data_t * Data) {
	int pos,ret;
	char c;

	for (pos=0; pos<6 ; pos++) {
		c = Ptr[pos];
		if (pos & 2) {
			if (!(c >= '0' && c <= '9'))
				break;
		} else {
		       c = toupper(Ptr[pos]);
		       if (pos < 2) {
			       if (!(c >= 'A' && c <= 'R'))
				       break;
		       } else {
			       if (!(c >= 'A' && c <= 'X'))
				       break;
		       }
		}
	}

	if (pos < 4 || pos & 1) {
		ESP_LOGE(TAG, "Invalid locator");
		return -1;
	}

	if (APRS_Locator_To_Position((char*)Ptr, pos, &Data->position) != pos) {
		ESP_LOGE(TAG,"Error converting locator to position");
		return -1;
	}

	c = Ptr[pos];
	if (c != ']') {
		// locator in status frame
		// Get symbol
		Data->symbol[0] = c;
		c = Ptr[pos+1];
		Data->symbol[1] = c;
		ret = pos+2;
	} else
		ret = pos;

	return ret;
}

static int APRS_Parser_Get_Position(uint8_t * Ptr, APRS_Data_t * Data) {
	int pos = 0;
	int ret;

	if ((ret = GPS_Parser_Get_Coord(&Data->position.latitude,(char*)Ptr)) == -1 || ret < 1 || ret > 7)
			return -1;

	if (ret == 1)
		Data->position.latitude *=10;	// ambiguity of 10°

	Data->position.ambiguity = 0;
	pos += ret;
	Ptr += ret;
	while (*Ptr == ' ' || *Ptr == '.') {
		Ptr++;
		pos++;
		if (*Ptr == ' ')
			Data->position.ambiguity++;
	}

	if (*Ptr == 'S')
		Data->position.latitude = - Data->position.latitude;
	else if (*Ptr != 'N' && Data->position.latitude)	// Somme tracker put '0' when no valid position
		return -1;
	Ptr++;
	pos++;

	Data->symbol[0] = *Ptr;
	Ptr++;
	pos++;

	if ((ret = GPS_Parser_Get_Coord(&Data->position.longitude,(char*)Ptr)) == -1 || ret < 1 || ret > 8)
		return -1;

	if (ret == 1)
		Data->position.latitude *=100;	// ambiguity of 100°
	else if (ret == 2)
		Data->position.latitude *=10;	// ambiguity of 10°

	pos += ret;
	Ptr += ret;
	while (*Ptr == ' ' || *Ptr == '.') {
		Ptr++;
		pos++;
	}

	if (*Ptr == 'W')
		Data->position.longitude = - Data->position.longitude;
	else if (*Ptr != 'E' && Data->position.longitude)	// Somme tracker put '0' when no valid position
		return -1;
	Ptr++;
	pos++;

	Data->symbol[1] = *Ptr;
	Ptr++;
	pos++;

	ESP_LOGD(TAG,"Found uncompressed position %f %f with symbol %c%c",
			(float)Data->position.longitude/((float)(1<<GPS_FIXED_POINT_DEG)),
			(float)Data->position.latitude/((float)(1<<GPS_FIXED_POINT_DEG)),
			Data->symbol[0],
			Data->symbol[1]);
			
	return pos;
}

uint32_t APRS_To91(uint32_t val, int i) {
	uint32_t b91 = 0x21212121;
	uint32_t m;
	while (i) {
		m = val%91;
		b91 = (b91<<8) | (m + '!');
		val = val/91;
		i--;
	}

	return b91;
}

uint32_t APRS_From91(uint32_t b91, int i) {
	uint32_t val = 0;

	while(i) {
		val *= 91;
		val += ((b91&0xff)-'!');
		b91 >>= 8;
		i--;
	}

	return val;
}


static int APRS_Get_Compressed_Data(uint8_t *Ptr, APRS_Data_t *Data) {
	int pos = 0;

	if (*Ptr >= 'a' && *Ptr <= 'j')
		Data->symbol[0] = *Ptr-'a'+'0';
	else if ((*Ptr >= 'A' && *Ptr <= 'Z') || *Ptr == '/' || *Ptr == '\\')
		Data->symbol[0] = *Ptr;
	else
		return -1;
	pos++;
	Ptr++;

	Data->position.latitude = (90<<GPS_FIXED_POINT_DEG) - ((uint64_t)APRS_From91(*(uint32_t*)Ptr,4)<<GPS_FIXED_POINT_DEG) / 380926;
	Ptr+=4;
	pos+=4;

	Data->position.longitude = (180<<GPS_FIXED_POINT_DEG) - ((uint64_t)APRS_From91(*(uint32_t*)Ptr,4)<<GPS_FIXED_POINT_DEG) / 190463;
	Ptr+=4;
	pos+=4;

	Data->symbol[1] = *Ptr;
	Ptr++;
	pos++;

	ESP_LOGD(TAG,"Found compressed position %f %f with symbol %c%c",
			(float)Data->position.longitude/((float)(1<<GPS_FIXED_POINT_DEG)),
			(float)Data->position.latitude/((float)(1<<GPS_FIXED_POINT_DEG)),
			Data->symbol[0],
			Data->symbol[1]);

	if (*Ptr != ' ') {
		if ((*(Ptr+3) & 0X18) == 0x10) {
			// Altitude in feet
			Data->position.altitude = (int32_t)(pow(1.002,APRS_From91(*(uint16_t*)Ptr,2)) + 0.5f);
			Ptr+=2;
			pos+=2;
			ESP_LOGD(TAG,"Altitude %dft",Data->position.altitude);
		} else if (*Ptr >= '!' && *Ptr <= 'z') {
			// course(°)/speed(Kn)
			Data->course.dir = ((*Ptr-'!') * 4);
			Ptr++;
			pos++;
			Data->course.speed =  (uint32_t)((pow(1.08f,*Ptr-'!') - 1) + 0.5f);
			Ptr++;
			pos++;
			Data->extension |= APRS_DATA_EXT_CSE;
			ESP_LOGD(TAG,"course heading %d°, speed %d Knot", Data->course.dir, Data->course.speed);
		} else if (*Ptr == '{') {
			Ptr++;
			pos++;
			// radio range in miles
			Data->range = ((pow(1.08f,*Ptr-'!') * 2.0f) + 0.5f);
			Ptr++;
			pos++;
			Data->extension |= APRS_DATA_EXT_RNG;
			ESP_LOGD(TAG,"radio range %d", Data->range);
		}
	} else {
		Ptr += 2;
		pos += 2;
	}

	Data->comp_type = *Ptr;

	pos++;

	return pos;
}

// Data extension for uncompressed position report
static int APRS_Get_Pos_Extended_Data(uint8_t *Ptr, APRS_Data_t *Data) {
	int pos = 0;
	int n;

	Data->extension = APRS_DATA_EXT_NONE;

	if (*(Ptr+3) == '/') {
		if (Data->symbol[1] == '_')
			Data->extension = APRS_DATA_EXT_WTH;	// Weather report (Wind speed/dir)
		else if (Data->symbol[0] == '/' && Data->symbol[1] == '\\')
			Data->extension = APRS_DATA_EXT_CSE_NRQ; // (course/speed/bearing/NRQ)
		else 
			Data->extension = APRS_DATA_EXT_CSE;

		// Count number of digit
		for (n=0;n<3 && isdigit(Ptr[n]);n++);
		if (n==3)
			for (n=4;n<7 && isdigit(Ptr[n]);n++);
		if (n == 7) {
			// Valid course/speed ( or wind dir/speed )
			if (Data->extension == APRS_DATA_EXT_WTH) {
				Data->weather.wind_dir = (*(Ptr++) -'0') * 100;
				Data->weather.wind_dir = (*(Ptr++) -'0') * 10;
				Data->weather.wind_dir = (*(Ptr++) -'0') * 10;
				Ptr++;
				Data->weather.wind_speed = (*(Ptr++) -'0') *100;
				Data->weather.wind_speed = (*(Ptr++) -'0') *10;
				Data->weather.wind_speed = (*(Ptr++) -'0') *1;
			} else {
				Data->course.dir = (*(Ptr++) -'0') * 100;
				Data->course.dir = (*(Ptr++) -'0') * 10;
				Data->course.dir = (*(Ptr++) -'0') * 10;
				Ptr++;
				Data->course.speed = (*(Ptr++) -'0') *100;
				Data->course.speed = (*(Ptr++) -'0') *10;
				Data->course.speed = (*(Ptr++) -'0') *1;
			}
			pos+=7;
		} else if ( (!strncmp((char*)Ptr,"   ",3) && !strncmp((char*)Ptr+4,"   ",3)) ||
			    (!strncmp((char*)Ptr,"...",3) && !strncmp((char*)Ptr+4,"...",3)) ) {
			// Valid NULL course/speed ( or wind dir/speed)
			if (Data->extension == APRS_DATA_EXT_WTH) {
				Data->weather.wind_dir = 0;
				Data->weather.wind_speed = 0;
			} else {
				Data->course.dir = 0;
				Data->course.speed = 0;
			}
			pos+=7;
			Ptr+=7;
		}

		if (Data->extension && *Ptr == '/' && *(Ptr+4) == '/') {
			if (Data->extension == APRS_DATA_EXT_CSE || Data->extension == APRS_DATA_EXT_CSE_NRQ) {
				// Count number of digit
				for (n=1;n<4 && isdigit(Ptr[n]);n++);
				if (n==4)
					for (n=5;n<8 && isdigit(Ptr[n]);n++);
				if (n==8) {
					Data->extension = APRS_DATA_EXT_CSE_NRQ;
					Data->nrq.bearing = atoi((char*)(Ptr+1)) << GPS_FIXED_POINT;
					Data->nrq.number = *(Ptr+5) - '0';
					Data->nrq.range = (pow((float)(*(Ptr+6) - '0'),2) + 0.5f); // in meter
					Data->nrq.quality = *(Ptr+7) - '0';
					pos+=8;
					Ptr+=8;
				} else
					Data->extension = APRS_DATA_EXT_CSE;
			}
		}
		
	}

	if (!Data->extension && !strncmp((char*)Ptr,"PHG",3)) {
	       Data->extension = APRS_DATA_EXT_PHG;
		ESP_LOGD(TAG,"PHG Found : not implemented");
		Ptr+=7;
		pos+=7;
	}
       
       if (!Data->extension && !strncmp((char*)Ptr,"RNG",3)) {
	       Data->extension = APRS_DATA_EXT_RNG;
		ESP_LOGD(TAG,"RNG Found : not implemented");
		Ptr+=7;
		pos+=7;
	}
       
       if (!Data->extension && !strncmp((char*)Ptr,"DFS",3)) {
	       Data->extension = APRS_DATA_EXT_DFS;
		ESP_LOGD(TAG,"DFS Found : not implemented");
		Ptr+=7;
		pos+=7;
	}

	return pos;
}

static int APRS_Parser_CUR_MICE_R0(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_OLD_MICE_R0(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_POS(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	int ret,i;
	uint8_t *ptr;

	i=0;
	ptr = &Frame->frame[pos];

	if (Data->type == APRS_DTI_POS_W_TS || Data->type == APRS_DTI_POS_W_TS_W_MSG) {
		Data->messaging = true;

		ret = APRS_Get_Time(ptr,Data);
		if (ret < 0) {
			ESP_LOGE(TAG,"Error getting timestamp from position report");
			return -1;
		}
		i += ret;
		pos+= ret;
	} else
		Data->messaging = false;

	if (((pos+21)<Frame->frame_len) && (ret = APRS_Parser_Get_Position(&Frame->frame[pos],Data)) != -1) {
		// Uncompressed data format
		pos+= ret;
		i+=ret;
		// Get Extended data
		if ((pos+9 < Frame->frame_len) && ((ret = APRS_Get_Pos_Extended_Data(&Frame->frame[pos],Data)) != -1)) {
			pos += ret;
			i += ret;
		}

		// TODO : get weather extra data if Symbol[1] == '_'
	} else if (((pos+16)<Frame->frame_len) && (ret = APRS_Get_Compressed_Data(&Frame->frame[pos],Data)) != -1) {
		// Compressed data format
		pos+=ret;
		i+=ret;
	} else
		return -1;

	return i;
}

static int APRS_Parser_RAW_GPS(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_OLD_MICE_TMD700(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_ITEM(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_TEST(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_MESSAGE(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_OBJECT(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_STATION_CAP(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_STATUS(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	int ret = 0;
	int n;
	uint8_t * ptr;

	ptr = &Frame->frame[pos];

	n = 0;
	if ((pos+9)<Frame->frame_len) 
		for (n=0;n<7 && isdigit(ptr[n]);n++);
	if (n == 6 && ((ret =APRS_Get_Time(&Frame->frame[pos],Data)) == 7))
			pos += ret;	// Status with timestamp
	else if ( ((Frame->frame_len  == pos+8) && (ret = APRS_Get_Locator(Frame->frame+pos,Data))==6)
		|| ((Frame->frame_len  == pos+10) && (ret = APRS_Get_Locator(Frame->frame+pos,Data))==8) )
		pos += ret; // status with locator but no comment
	else if (  ((Frame->frame_len  > pos+9) && (*(Frame->frame+pos+6) == ' ') && (ret = APRS_Get_Locator(Frame->frame+pos,Data))==6)
		|| ((Frame->frame_len  > pos+11) && (*(Frame->frame+pos+8) == ' ') && (ret = APRS_Get_Locator(Frame->frame+pos,Data))==8) ) {
		pos += ret+1; // status with locator and comment
		ret++;
	}
	
	// Meteor scatter beam heading and ERP
	if (Frame->frame[Frame->frame_len-5] == '^') {
		char c =Frame->frame[Frame->frame_len-4];
		if (isdigit(c))
			Data->beam.heading = ((c - '0')*10);
		else if (isupper(c))
			Data->beam.heading = ((c - 'A')*10);
		else {
			ESP_LOGE(TAG,"Meteor scatter Letter code must be upper case");
			return ret;
		}

		c = Frame->frame[Frame->frame_len-3];
		Data->beam.power = (((int32_t)(c-'0')*(int32_t)(c-'0'))*10);
		Data->extension = APRS_DATA_EXT_BEAM;

		ESP_LOGD(TAG,"Beam heading %d , ERP %d",Data->beam.heading,Data->beam.power);
	}

	return ret;
}

static int APRS_Parser_QUERY(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_TELEMETRIE(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_MH_LOCATOR(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_WEATHER(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_CUR_MICE(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_USER_DEF(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

static int APRS_Parser_THIRD_PARTY(int pos, Frame_t * Frame, APRS_Data_t * Data) {
	return 0;
}

