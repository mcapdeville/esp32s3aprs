/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/nrzi.h
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

#ifndef _NRZI_H_
#define _NRZI_H_

static __inline__ void NRZI_Encode(bool *state,uint8_t *bitstream, size_t len) { // len in bits
	size_t pos=0;
	while (pos < len) {
		*state ^= !(*bitstream&1);
		*bitstream >>=1;
		*bitstream |= *state?0x80:00;
		pos++;
		if (!(pos&7))
			bitstream++;
	}

	if (pos&7)
		*bitstream >>= 8-(pos&7);
}

static __inline__ void NRZI_Decode(bool *state,uint8_t *bitstream, size_t len) { // len in bits
	size_t pos=0;
	while (pos < len) {
		bool bit;
		bit = !(*state ^ (*bitstream&1));
		*state = *bitstream&1;
		*bitstream >>=1;
		*bitstream |= bit?0x80:0;
		pos++;
		if (!(pos&7))
			bitstream++;
	}

	if (pos&7)
		*bitstream >>= 8-(pos&7);
}

#endif
