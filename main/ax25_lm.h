/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/ax25_lm.h
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

#ifndef _AX25_LM_H_
#define _AX25_LM_H_

#include "ax25_phy.h"
#include "ax25.h"

typedef struct Framebuff_Frame_S Frame_t;
typedef struct AX25_Lm_S AX25_Lm_t;
typedef struct AX25_Lm_Ops_S AX25_Lm_Ops_t;
typedef struct AX25_Lm_Cbs_S AX25_Lm_Cbs_t;

struct AX25_Lm_Ops_S {
	// Register / Unregister data link
	int (*register_dl)(AX25_Lm_t * Lm, void * Ctx, AX25_Lm_Cbs_t * Cbs, AX25_Addr_t *Filter);
	int (*unregister_dl)(AX25_Lm_t * Lm, void * Ctx);

	// Lm requests (from Dl)
	int (*seize_request)(AX25_Lm_t * Lm, void * Ctx);
	int (*release_request)(AX25_Lm_t * Lm, void * Ctx);
	int (*expedited_data_request)(AX25_Lm_t * Lm, void * Ctx, Frame_t * Frame);
	int (*data_request)(AX25_Lm_t * Lm, void * Ctx, Frame_t * Frame);
};

struct AX25_Lm_Cbs_S {
	// Lm callbacks (To Dl)
	int (*seize_confirm)(AX25_Lm_t * Ctx);
	int (*data_indication)(AX25_Lm_t * Ctx, Frame_t * Frame);
	int (*busy_indication)(AX25_Lm_t * Ctx);
	int (*quiet_indication)(AX25_Lm_t * Ctx);
};

extern const struct AX25_Lm_Ops_S Ax25_Lm_Ops;
struct AX25_Lm_S {
	const struct AX25_Lm_Ops_S *ops;
};


// Ops
static inline int AX25_Lm_Register_Dl(AX25_Lm_t * Lm, void * Ctx, AX25_Lm_Cbs_t * Cbs, AX25_Addr_t *Filter) {
	return Lm->ops->register_dl(Lm, Ctx, Cbs, Filter);
}

static inline int AX25_Lm_Unregister_Dl(AX25_Lm_t * Lm, void * Ctx) {
	return Lm->ops->unregister_dl(Lm, Ctx);
}

static inline int AX25_Lm_Seize_Request(AX25_Lm_t *Lm, void * Ctx) {
	return Lm->ops->seize_request(Lm, Ctx);
}

static inline int AX25_Lm_Release_Request(AX25_Lm_t * Lm, void * Ctx) {
	return Lm->ops->release_request(Lm, Ctx);
}

static inline int AX25_Lm_Data_Request(AX25_Lm_t * Lm, void * Ctx, Frame_t *Frame) {
	return Lm->ops->data_request(Lm, Ctx,Frame);
}

static inline int AX25_Lm_Expedited_Data_Request(AX25_Lm_t * Lm, void * Ctx, Frame_t *Frame) {
	return Lm->ops->expedited_data_request(Lm, Ctx, Frame);

}

// Create an AX25 link multiplexer layer attached to an AX25 physical layer
AX25_Lm_t * AX25_Lm_Impl_Init(AX25_Phy_t * Phy);

static inline AX25_Lm_t * AX25_Lm_Init(AX25_Phy_t * Phy) {
	return AX25_Lm_Impl_Init(Phy);
}


#endif
