/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/ax25.c
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

#include "ax25.h"
#include <esp_log.h>
#include <ctype.h>

#define TAG "AX25"

int AX25_Make_Addr(const char * Callid, uint8_t Ssid, AX25_Addr_t * Addr) {
	int i;
	char * ptr = (char*)&Addr->callid;

	if (!Addr)
		return -1;

	bzero(Addr,sizeof(AX25_Addr_t));

	if (Callid && *Callid) {
		i = 6;
		while (*Callid && *Callid != '*' && i) {
			*(ptr++)=(*(Callid++))<<1;
			i--;
		}

		while (i) {
			if (*Callid == '*')
				*(ptr++) = '\0';
			else
				*(ptr++) = ' ' << 1;
			i--;
		}

	}
	if (Ssid != 0xff)
		Addr->ssid = (Ssid&0x0f)<<1 | 0x60; // Reserved bit set to '1'
	else Addr->ssid = 0;	// SSID undef

	return 0;
}

int AX25_Str_To_Addr(const char *Str, AX25_Addr_t * Addr) {
	uint8_t ssid;
	char * ptr, *id;
	char buf[10];

	strncpy(buf, Str, sizeof(buf));
	ESP_LOGD(TAG,"Make addr from %s",buf);

	ptr = strchr(buf,'-');
	if (ptr) {
		*ptr = '\0';
		ptr++;
		if (*ptr != '*' && (*ptr != '\0'))
			ssid = atoi(ptr);
		else
			ssid = 0xff;
	} else
		ssid = 0;

	id = buf;
	while (*id == ' ') id++;
	
	ptr = strchr(id, ' ');
	if (ptr) {
		*ptr = '\0';
	}

	if (ptr) {
		*ptr = '\0';
	}

	return AX25_Make_Addr(id, ssid, Addr);
}

void AX25_Norm_Addr(AX25_Addr_t * Addr) {
	int i = 0;

	while (i < 6 && (Addr->addr[i]) && isalnum(Addr->addr[i]>>1)) {
		Addr->addr[i] = toupper(Addr->addr[i]>>1)<<1;
		i++;
	}

	while (i < 6) {
		Addr->addr[i] = ' '<<1;
		i++;
	}

	Addr->ssid = (Addr->ssid & 0x1e) | 0x60;
}

int AX25_Addr_To_Str(const AX25_Addr_t *Addr, char *Str, int Len) {
	int i;
	char * ptr = (char*)&Addr->callid;
	int pos = 0;
	char c;

	for (i=0;i<6 && pos < (Len-1);i++,ptr++) {
		c = (*ptr&0xfe)>>1;
		if (c == ' ')
			break;
		if (c == '\0') {
			*(Str++) = '*';
			pos++;
			break;
		}
		else 
			*(Str++) = c;
		pos++;
	}
	for (;i<6;i++,ptr++);

	if (Addr->ssid) {
		if (((Addr->ssid & 0x1e)>>1) <10) {
			if ((Addr->ssid & 0x1e) && pos<(Len-3)) {
				*(Str++) = '-';
				*(Str++) = ((Addr->ssid&0x1e)>>1) + '0';
				pos+=2;
			}
		} else if (pos<(Len-4)) {
			*(Str++) = '-';
			*(Str++) = '1';
			*(Str++) = ((Addr->ssid&0x1e)>>1)-10 + '0';
			pos+=3;
		}
	} else if (pos<(Len-3)) {
		*(Str++) = '-';
		*(Str++) = '*';
		pos+=2;
	}

	*Str = '\0';

	return pos;
}

int AX25_Addr_Cmp(const AX25_Addr_t *Addr1, const AX25_Addr_t *Addr2) {
	char * ptr1 = (char*)&Addr1->callid;
	char * ptr2 = (char*)&Addr2->callid;
	int i;

	if (!Addr1 || !Addr2)
		return -1;

	i = 0;
	while ( i < 6 ) {
		if (!(*ptr1) || !(*ptr2)) {
			break;
		}
		i++;
		if ((*ptr1&0xfe) != (*ptr2&0xfe)) {
			if ((*ptr1&0xfe) - (*ptr2&0xfe) < 0)
				return -i;
			return i;
		}

		ptr1++;
		ptr2++;
	}

	if (!Addr1->ssid || !Addr2->ssid)
		return 0;

	if ((Addr1->ssid&0x1e) != (Addr2->ssid&0x1e)) {
		if ((Addr1->ssid&0x1e) - (Addr2->ssid&0x1e) < 0)
			return -7;
		return 7;
	}

	return 0;
}

int AX25_Addr_Count(AX25_Addr_t *Addr) {
	int count = 0;

	if (!Addr)
		return 0;

	while (count < (AX25_MAX_ADDR) && !(Addr->ssid&1)) {
		Addr++;
		count++;
	}

	return count+1;
}

int AX25_Addr_Filter(AX25_Addr_t *Addr, AX25_Addr_t *Filter) {
	int count = 0;
	int ret;

	if (!Addr || !Filter)
		return -1;

	while (count < (AX25_MAX_ADDR) && !(ret = AX25_Addr_Cmp(Addr,Filter)) && !(Addr->ssid&1) && !(Filter->ssid&1)) {
		Addr++;
		count++;
		Filter++;
	}

	return ret;
}

uint8_t AX25_Get_Addr(int pos, const Frame_t * Frame, AX25_Addr_t *Addr) {
	memcpy(Addr, &Frame->frame[pos],sizeof(AX25_Addr_t));
	return ((uint8_t*)Addr)[6];
}
