/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/ax25_phy.h
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

#ifndef _AX25_PHY_H_
#define _AX25_PHY_H_

#include "modem.h"

typedef struct Framebuff_Frame_S Frame_t;
typedef struct AX25_Phy_S AX25_Phy_t;
typedef struct AX25_Phy_Ops_S AX25_Phy_Ops_t;
typedef struct AX25_Phy_Cbs_S AX25_Phy_Cbs_t;

struct AX25_Phy_Ops_S {
	// Phy requests (From Lm)
	int (*seize_request)(AX25_Phy_t * Phy);
	int (*release_request)(AX25_Phy_t * Phy);
	int (*expedited_data_request)(AX25_Phy_t * Phy, Frame_t * Frame);
	int (*data_request)(AX25_Phy_t * Phy, Frame_t * Frame);
};

struct AX25_Phy_Cbs_S {
	// Phy callbacks (To Lm)
	int (*seize_confirm)(void * Ctx);
	int (*data_indication)(void * Ctx, Frame_t * Frame);
	int (*busy_indication)(void * Ctx);
	int (*quiet_indication)(void * Ctx);
};

// Interface
struct AX25_Phy_S {
	// Ops
	const struct AX25_Phy_Ops_S *ops;
	// Callbacks list
	struct AX25_Phy_Cbs_List_S *cbs_list;
};

static inline int AX25_Phy_Seize_Request(AX25_Phy_t * Phy) {
	return Phy->ops->seize_request(Phy);
}

static inline int AX25_Phy_Release_Request(AX25_Phy_t * Phy) {
	return Phy->ops->release_request(Phy);
}

static inline int AX25_Phy_Data_Request(AX25_Phy_t * Phy, Frame_t * Frame) {
	return Phy->ops->data_request(Phy, Frame);
}

static inline int AX25_Phy_Expedited_Data_Request(AX25_Phy_t * Phy, Frame_t * Frame) {
	return Phy->ops->expedited_data_request(Phy, Frame);
}

// Register callback
int AX25_Phy_Register_Cbs(AX25_Phy_t * Phy, void *Ctx, const AX25_Phy_Cbs_t * Cbs);
int AX25_Phy_Unregister_Cbs(AX25_Phy_t * Phy, void *Ctx);

// Callbacks (From Phy to LM)
#ifdef _AX25_PHY_PRIV_INCLUDE_
void AX25_Phy_Seize_Confirm_Cb(AX25_Phy_t * Phy);
void AX25_Phy_Data_Indication_Cb(AX25_Phy_t * Phy, Frame_t * Frame);
void AX25_Phy_Busy_Indication_Cb(AX25_Phy_t * Phy);
void AX25_Phy_Quiet_Indication_Cb(AX25_Phy_t * Phy);
#endif

#endif
