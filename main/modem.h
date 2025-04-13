/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/modem.h
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

#ifndef _MODEM_H_
#define _MODEM_H_

#include <stdbool.h>

typedef struct Framebuff_Frame_S Frame_t;
typedef struct Modem_S Modem_t;
typedef struct Modem_Ops_S Modem_Ops_t;
typedef struct Modem_Cbs_S Modem_Cbs_t;
typedef struct Modem_Cbs_List_S Modem_Cbs_List_t;

struct Modem_Ops_S {
	// Receiver Ops
	int (*start_receiver)(Modem_t * Modem);
	int (*stop_receiver)(Modem_t * Modem);
	
	// Transmiter ops
	int (*start_transmiter)(Modem_t * Modem);
	int (*stop_transmiter)(Modem_t * Modem);
	int (*send_frame)(Modem_t * Modem, Frame_t * Frame);
};

struct Modem_Cbs_S {
	// Receiver callbacks
	int (*receiver_started)(void * Ctx);	// Called when squelch open
	int (*receiver_stopped)(void * Ctx);	// Called when squelch closed
	int (*dcd_changed)(void * Ctx, bool Dcd); // Called when carrier detected
	int (*frame_received)(void * Ctx, Frame_t *); // Called when frame received

	// Transmiter callbacks
	int (*transmiter_started)(void * Ctx);	// Called when ptt pushed
	int (*transmiter_stopped)(void * Ctx);	// Called when ptt released
	int (*frame_sent)(void * Ctx, Frame_t *); // Called when frame sent
};

// Modem interface
struct Modem_S {
	// Ops
	const struct Modem_Ops_S *ops;
	// Callbacks list
	struct Modem_Cbs_List_S *cbs_list;
};

// Modem ops
static inline int Modem_Start_Receiver(Modem_t * modem) {
	return modem->ops->start_receiver(modem);
}

static inline int Modem_Stop_Receiver(Modem_t * Modem) {
	return Modem->ops->stop_receiver(Modem);
}

static inline int Modem_Start_Transmiter(Modem_t * Modem) {
	return Modem->ops->start_transmiter(Modem);
}

static inline int Modem_Stop_Transmiter(Modem_t * Modem) {
	return Modem->ops->stop_transmiter(Modem);
}

static inline int Modem_Send_Frame(Modem_t * Modem, Frame_t * Frame) {
	return Modem->ops->send_frame(Modem, Frame);
}

int Modem_Register_Cbs(Modem_t * Modem, void * Ctx, const Modem_Cbs_t * Cbs);
int Modem_Unregister_cbs(Modem_t, void * Ctx);

#ifdef _MODEM_PRIV_INCLUDE_
// Modem Callbacks
void Modem_Receiver_Started_Cb(Modem_t * Modem);
void Modem_Receiver_Stopped_Cb(Modem_t * Modem);
void Modem_Dcd_Changed_Cb(Modem_t * Modem, bool Dcd);
void Modem_Frame_Received_Cb(Modem_t * Modem, Frame_t * Frame);
void Modem_Transmiter_Started_Cb(Modem_t * Modem);
void Modem_Transmiter_Stopped_Cb(Modem_t * Modem);
void Modem_Frame_Sent_Cb(Modem_t * Modem, Frame_t * Frame);
#endif

#endif
