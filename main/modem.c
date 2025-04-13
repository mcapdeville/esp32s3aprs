/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/modem.c
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

#define _MODEM_PRIV_INCLUDE_
#include "modem.h"
#include <stdlib.h>
#include <string.h>
#include <esp_log.h>

#define TAG "MODEM"

struct Modem_Cbs_List_S {
	struct Modem_Cbs_List_S * next;
	Modem_Cbs_t cbs;
	void * cbs_ctx;
};

int Modem_Register_Cbs(Modem_t * Modem, void *Ctx, const Modem_Cbs_t * Cbs) {
	struct Modem_Cbs_List_S * cbs_list;
	if (Ctx && Cbs) {
		if (!(cbs_list = malloc(sizeof(struct Modem_Cbs_List_S)))) {
			ESP_LOGE(TAG,"Error allocating cbs_list");
			return -1;
		}
		bzero(cbs_list,sizeof(struct Modem_Cbs_List_S));

		memcpy(&cbs_list->cbs,Cbs,sizeof(Modem_Cbs_t));
		cbs_list->cbs_ctx = Ctx;

		cbs_list->next = Modem->cbs_list;
		Modem->cbs_list = cbs_list;
		return 0;
	}

	return -1;
}

int Modem_Unregister_Cbs(Modem_t * Modem, void *Ctx) {
	struct Modem_Cbs_List_S * cbs_list, **prev;
	if (Ctx) {
		prev = &Modem->cbs_list;
		cbs_list = *prev;
		while (cbs_list) {
			if (cbs_list->cbs_ctx == Ctx) {
				*prev = cbs_list->next;
				free(cbs_list);
			} else
				prev = &cbs_list->next;
			cbs_list = *prev;
		}
		return 0;
	}
	return -1;
}

void Modem_Receiver_Started_Cb(Modem_t * Modem) {
	struct Modem_Cbs_List_S * cbs_list = Modem->cbs_list;

	while (cbs_list) {
		if (cbs_list->cbs.receiver_started)
			cbs_list->cbs.receiver_started(cbs_list->cbs_ctx);
		cbs_list = cbs_list->next;
	}
}

void Modem_Receiver_Stopped_Cb(Modem_t * Modem) {
	struct Modem_Cbs_List_S * cbs_list = Modem->cbs_list;

	while (cbs_list) {
		if (cbs_list->cbs.receiver_stopped)
			cbs_list->cbs.receiver_stopped(cbs_list->cbs_ctx);
		cbs_list = cbs_list->next;
	}
}

void Modem_Dcd_Changed_Cb(Modem_t * Modem, bool Dcd) {
	struct Modem_Cbs_List_S * cbs_list = Modem->cbs_list;

	while (cbs_list) {
		if (cbs_list->cbs.dcd_changed)
			cbs_list->cbs.dcd_changed(cbs_list->cbs_ctx, Dcd);
		cbs_list = cbs_list->next;
	}
}

void Modem_Frame_Received_Cb(Modem_t * Modem, Frame_t * Frame) {
	struct Modem_Cbs_List_S * cbs_list = Modem->cbs_list;

	while (cbs_list) {
		if (cbs_list->cbs.frame_received)
			cbs_list->cbs.frame_received(cbs_list->cbs_ctx, Frame);
		cbs_list = cbs_list->next;
	}
}

void Modem_Transmiter_Started_Cb(Modem_t * Modem) {
	struct Modem_Cbs_List_S * cbs_list = Modem->cbs_list;

	while (cbs_list) {
		if (cbs_list->cbs.transmiter_started)
			cbs_list->cbs.transmiter_started(cbs_list->cbs_ctx);
		cbs_list = cbs_list->next;
	}
}

void Modem_Transmiter_Stopped_Cb(Modem_t * Modem) {
	struct Modem_Cbs_List_S * cbs_list = Modem->cbs_list;

	while (cbs_list) {
		if (cbs_list->cbs.transmiter_stopped)
			cbs_list->cbs.transmiter_stopped(cbs_list->cbs_ctx);
		cbs_list = cbs_list->next;
	}
}

void Modem_Frame_Sent_Cb(Modem_t * Modem, Frame_t * Frame) {
	struct Modem_Cbs_List_S * cbs_list = Modem->cbs_list;

	while (cbs_list) {
		if (cbs_list->cbs.frame_sent)
			cbs_list->cbs.frame_sent(cbs_list->cbs_ctx, Frame);
		cbs_list = cbs_list->next;
	}
}
