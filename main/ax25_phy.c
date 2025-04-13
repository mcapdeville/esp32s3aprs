/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/ax25_phy.c
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
#include <stdlib.h>
#include <string.h>
#include <esp_log.h>

#define TAG "AX25_PHY"

struct AX25_Phy_Cbs_List_S {
	struct AX25_Phy_Cbs_List_S * next;
	AX25_Phy_Cbs_t cbs;
	void * cbs_ctx;
};

int AX25_Phy_Register_Cbs(AX25_Phy_t * Phy, void *Ctx, const AX25_Phy_Cbs_t * Cbs) {
	struct AX25_Phy_Cbs_List_S * cbs_list;
	if (Ctx && Cbs) {
		if (!(cbs_list = malloc(sizeof(struct AX25_Phy_Cbs_List_S)))) {
			ESP_LOGE(TAG,"Error allocating cbs_list");
			return -1;
		}
		bzero(cbs_list,sizeof(struct AX25_Phy_Cbs_List_S));

		memcpy(&cbs_list->cbs, Cbs, sizeof(AX25_Phy_Cbs_t));
		cbs_list->cbs_ctx = Ctx;

		cbs_list->next = Phy->cbs_list;
		Phy->cbs_list = cbs_list;
		return 0;
	}

	return -1;
}

int AX25_Phy_Unregister_Cbs(AX25_Phy_t * Phy, void *Ctx) {
	struct AX25_Phy_Cbs_List_S * cbs_list, **prev;
	if (Ctx) {
		prev = &Phy->cbs_list;
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

void AX25_Phy_Seize_Confirm_Cb(AX25_Phy_t * Phy) {
	struct AX25_Phy_Cbs_List_S * cbs_list = Phy->cbs_list;

	while (cbs_list) {
		if (cbs_list->cbs.seize_confirm)
			cbs_list->cbs.seize_confirm(cbs_list->cbs_ctx);
		cbs_list = cbs_list->next;
	}
}

void AX25_Phy_Data_Indication_Cb(AX25_Phy_t * Phy, Frame_t * Frame) {
	struct AX25_Phy_Cbs_List_S * cbs_list = Phy->cbs_list;

	while (cbs_list) {
		if (cbs_list->cbs.data_indication)
			cbs_list->cbs.data_indication(cbs_list->cbs_ctx, Frame);
		cbs_list = cbs_list->next;
	}
}

void AX25_Phy_Busy_Indication_Cb(AX25_Phy_t * Phy) {
	struct AX25_Phy_Cbs_List_S * cbs_list = Phy->cbs_list;

	while (cbs_list) {
		if (cbs_list->cbs.busy_indication)
			cbs_list->cbs.busy_indication(cbs_list->cbs_ctx);
		cbs_list = cbs_list->next;
	}
}

void AX25_Phy_Quiet_Indication_Cb(AX25_Phy_t * Phy) {
	struct AX25_Phy_Cbs_List_S * cbs_list = Phy->cbs_list;

	while (cbs_list) {
		if (cbs_list->cbs.quiet_indication)
			cbs_list->cbs.quiet_indication(cbs_list->cbs_ctx);
		cbs_list = cbs_list->next;
	}
}
