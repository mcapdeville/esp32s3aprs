/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/aprs_log.c
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


#include <berkeley-db/db.h>
#include "aprs.h"
#include "aprs_log.h"
#include <errno.h>


#include <esp_log.h>

#define TAG	"APRS_LOG"

int APRS_Show_Station(APRS_Station_t * Station) {
	char str[16];
	struct tm tm;
	uint32_t lat, lon;
	char ns,ew;

	if (!Station ||!Station->callid.callid[0])
		return -1;

	AX25_Addr_To_Str(&Station->callid, str, sizeof(str));
	ESP_LOGI(TAG, "Callid : %s symbol : %c%c", str, Station->symbol[0], Station->symbol[1]);
	if (Station->timestamp) {
		gmtime_r(&Station->timestamp, &tm);
		ESP_LOGI(TAG,"timestamp %d/%d/%d %d:%d:%d",
				tm.tm_mday, tm.tm_mon, tm.tm_year,
				tm.tm_hour, tm.tm_min, tm.tm_sec);
	}

	if (Station->status[0])
		ESP_LOGI(TAG, "Status : %s", Station->status);

	if (Station->position.longitude || Station->position.latitude) {
		if (Station->position.latitude<0) {
			lat = -Station->position.latitude;
			ns = 'S';
		} else {
			lat = Station->position.latitude;
			ns = 'N';
		}

		if (Station->position.longitude<0) {
			lon = -Station->position.longitude;
			ew = 'W';
		} else {
			lon = Station->position.longitude;
			ew = 'E';
		}

		ESP_LOGI(TAG,"Position : %u.%03u %c %u.%03u %c",
				(unsigned)(lat>>GPS_FIXED_POINT_DEG),
				(unsigned)(((lat&((1L<<GPS_FIXED_POINT_DEG)-1))*1000)>>GPS_FIXED_POINT_DEG),
				ns,
				(unsigned)(lon>>GPS_FIXED_POINT_DEG),
				(unsigned)(((lon&((1L<<GPS_FIXED_POINT_DEG)-1))*1000)>>GPS_FIXED_POINT_DEG),
				ew);

		if (Station->position.altitude) {
			ESP_LOGI(TAG,"Altitude : %dft", Station->position.altitude);
		}

		if (Station->course.speed || Station->course.dir) {
			if (Station->symbol[1] == '_')
				ESP_LOGI(TAG, "Wind : %dkn / %d°", Station->course.speed, Station->course.dir);
			else
				ESP_LOGI(TAG, "Course : %dkn / %d°", Station->course.speed, Station->course.dir);
		}
	}

	return 0;
}

int APRS_Log_Station(DB * Station_db, APRS_Data_t * Data) {
	struct tm tm;
	DBT key, data;
	char str[16];
	bool mod = false;

	int ret;
	APRS_Station_t station;

	if (!Station_db || !Data)
		return -1;

	// Get Last data
	bzero(&station, sizeof(station));
	
	memcpy(&station.callid,&Data->address[1],sizeof(AX25_Addr_t));
	AX25_Norm_Addr(&station.callid);

	key.data = &station.callid;
	key.size = sizeof(AX25_Addr_t);
	ret = Station_db->get(Station_db, &key, &data, 0);
	if (ret <0) {
		ESP_LOGE(TAG,"Error geting station from db)");
		return -1;
	}

	if (ret == 1) {
		AX25_Addr_To_Str(&station.callid,str,sizeof(str));
		ESP_LOGI(TAG,"New station : %s",str);

		mod = true;
		ret = 0;
	} else {
		memcpy(&station, data.data, data.size<sizeof(station)?data.size:sizeof(station));
		AX25_Norm_Addr(&station.callid);
		AX25_Addr_To_Str(&station.callid,str,sizeof(str));
		ESP_LOGI(TAG,"station : %s",str);
	}

	if (station.timestamp != Data->timestamp) {
		station.timestamp = Data->timestamp;
		mod = true;
	}

	if (Data->symbol[0] && Data->symbol[1]) {
		if (station.symbol[0] != Data->symbol[0]) {
			station.symbol[0] = Data->symbol[0];
			mod = true;
		}

		if (station.symbol[1] != Data->symbol[1]) {
			station.symbol[1] = Data->symbol[1];
			mod = true;
		}
	}

	gmtime_r(&Data->timestamp, &tm);
	if (Data->time.month) {
		tm.tm_mon = Data->time.month;
		mod = true;
	}

	if (Data->time.day) {
		tm.tm_mday = Data->time.day;
			mod = true;
	}

	if (Data->time.hours) {
		tm.tm_hour = Data->time.hours;
		mod = true;
	}

	if (Data->time.minutes) {
		tm.tm_min = Data->time.minutes;
		mod = true;
	}
	if (Data->time.seconds) {
		tm.tm_sec = Data->time.seconds;
		mod = true;
	}

	time_t ts = mktime(&tm);
	if (ts>0 && ts != station.timestamp) {
		station.timestamp = ts;
		mod = true;
	}

	switch(Data->type) {
		case APRS_DTI_POS_W_TS_W_MSG:
		case APRS_DTI_POS_W_TS:
		case APRS_DTI_POS_W_MSG:
		case APRS_DTI_POS:
			if (memcmp(&station.position, &Data->position, sizeof(struct APRS_Position))) {
				memcpy(&station.position, &Data->position, sizeof(struct APRS_Position));
				mod = true;
			}
			switch (Data->extension) {
				case APRS_DATA_EXT_CSE_NRQ:
					if (memcmp(&station.nrq, &Data->nrq, sizeof(struct APRS_NRQ))) {
						memcpy(&station.nrq, &Data->nrq, sizeof(struct APRS_NRQ));
						mod = true;
					}
					/* FALLTHRU */
				case APRS_DATA_EXT_CSE:
					if (memcmp(&station.course, &Data->course, sizeof(struct APRS_Course))) {
						memcpy(&station.course, &Data->course, sizeof(struct APRS_Course));
						mod = true;
					}
					break;
				case APRS_DATA_EXT_PHG:
					if (memcmp(&station.phg, &Data->phg, sizeof(struct APRS_PHG))) {
						memcpy(&station.phg, &Data->phg, sizeof(struct APRS_PHG));
						mod = true;
					}
					break;
				case APRS_DATA_EXT_DFS:
					if (memcmp(&station.dfs, &Data->dfs, sizeof(struct APRS_DFS))) {
						memcpy(&station.dfs, &Data->dfs, sizeof(struct APRS_DFS));
						mod = true;
					}
					break;
				case APRS_DATA_EXT_RNG:
					if (station.range != Data->range) {
						station.range = Data->range;
						mod = true;
					}
					break;
				case APRS_DATA_EXT_WTH:
					if (memcmp(&station.course, &Data->weather, sizeof(struct APRS_Course))) {
						memcpy(&station.course, &Data->weather, sizeof(struct APRS_Course));
						mod = true;
					}
					break;
				default:
					bzero(&station.course, sizeof(struct APRS_Course));
					station.range = 0;
					bzero(&station.dfs, sizeof(struct APRS_DFS));
					bzero(&station.phg, sizeof(struct APRS_PHG));
					bzero(&station.nrq, sizeof(struct APRS_NRQ));
			}

			break;
		case APRS_DTI_STATUS:
			if (Data->position.ambiguity >= APRS_AMBIGUITY_LOC_EXT_SQUARE) {
				if (memcmp(&station.position, &Data->position, sizeof(struct APRS_Position))) {
					memcpy(&station.position, &Data->position, sizeof(struct APRS_Position));
					mod = true;
				}
			}
			if (Data->extension == APRS_DATA_EXT_BEAM)
				if (memcmp(&station.beam, &Data->beam, sizeof(struct APRS_Beam))) {
					memcpy(&station.beam, &Data->beam, sizeof(struct APRS_Beam));
					mod = true;
				}
	
			if (Data->text[0] && memcmp(station.status, Data->text, sizeof(station.status))) { 
				strncpy(station.status, Data->text, sizeof(station.status)); 
				mod = true;
			}
		
			break;
		default:
	}

	if (mod) {
		memcpy(&station.callid,&Data->address[1],sizeof(AX25_Addr_t));
		AX25_Norm_Addr(&station.callid);

		data.size = sizeof(APRS_Station_t);
		data.data = &station;
		key.data = &station.callid;
		key.size = sizeof(AX25_Addr_t);

		ret = Station_db->put(Station_db, &key, &data, 0);
		if (ret == -1)
			ESP_LOGE(TAG,"Error putting station in DB (%d)",errno);
		if (ret == 1)
			ESP_LOGE(TAG,"Can't overwrite station in DB (%d)",errno);

		ret = Station_db->sync(Station_db, 0);
		if (ret)
			ESP_LOGE(TAG,"Error syncing db file (%d)",errno);
	}

	return ret;
}
