/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/afsk_mod.c
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

#include "afsk_mod.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <stdbool.h>

#define AFSK_MOD_AMP	0.707f

struct AFSK_Mod_S {
	int16_t * sine;
	size_t sine_len;
	uint8_t mark_stride;
	uint8_t space_stride;
	size_t phase;
	int32_t bit_clock;
	int32_t clock_step;
	uint8_t out;
	uint8_t bit_pos;
	bool current_bit;
};

#define TAG	"AFSK_MOD"

AFSK_Mod_t* AFSK_Mod_Init(AFSK_Config_t const *Config) {
	AFSK_Mod_t * mod;
	int16_t gcd,tmp1,tmp2;

	if (!(mod = heap_caps_malloc(sizeof(struct AFSK_Mod_S),MALLOC_CAP_INTERNAL))) {
		ESP_LOGE(TAG,"Error allocating AFSK_Mod struct");
		return NULL;
	}

	
	// Calc GCD of mark and space freq	
	gcd = Config->mark_freq;
	tmp1 = Config->space_freq % gcd;
	while (tmp1 != 0) {
		tmp2 = gcd;
		gcd = tmp1;
		tmp1 = tmp2 % gcd;
	}

	mod->mark_stride = Config->mark_freq/gcd;
	mod->space_stride = Config->space_freq/gcd;
	mod->sine_len = ((((int32_t)Config->sample_rate<<1)/gcd) + 1)>>1;

	ESP_LOGI(TAG,"Sine len : %d, mark stride : %d, space stride : %d",mod->sine_len, mod->mark_stride,mod->space_stride);

	if (!(mod->sine = malloc(sizeof(int16_t)*mod->sine_len))) {
		ESP_LOGE(TAG,"Error allocating sine table");
		free(mod);
		return NULL;
	}

	for (tmp1=0;tmp1 < mod->sine_len ; tmp1++) {
		mod->sine[tmp1] = (int16_t)roundf((AFSK_MOD_AMP*INT16_MAX)*sinf((tmp1*2.0f*M_PI)/mod->sine_len));
	}

	// Clock generation
	mod->clock_step = ((int64_t)INT32_MAX*Config->baud_rate<<1)/Config->sample_rate;

	AFSK_Mod_Reset(mod);

	return mod;
}

// return the number of samples generated
// update Bit_len
__attribute__((hot))
uint16_t AFSK_Mod_Output(AFSK_Mod_t * Mod,uint8_t ** Bitstream, uint16_t *Bitstream_len,int16_t * Buff,uint16_t Buff_len) {
	int32_t last_clock;
	uint16_t bit_count = 0;
	uint16_t sample_count = 0;

	if (!Mod)
		return 0;

	while (Buff_len) {
		last_clock = Mod->bit_clock;
		Mod->bit_clock += Mod->clock_step;
		if (last_clock >0 && Mod->bit_clock<0) {
			if (!Mod->bit_pos) {
				if (*Bitstream_len) {
					// Take a full byte if possible
					Mod->out = **Bitstream;
					(*Bitstream)++;
					if (*Bitstream_len >=8)
						Mod->bit_pos = 8;
					else
						Mod->bit_pos = *Bitstream_len;
					*Bitstream_len -= Mod->bit_pos;
				} else
					return sample_count;

			}
			Mod->bit_pos--;
			Mod->current_bit = Mod->out & 1;
			Mod->out>>=1;
			bit_count++;
		}

		*(Buff++) = Mod->sine[Mod->phase];
		Buff_len--;
		sample_count++;

		Mod->phase += Mod->current_bit ? Mod->mark_stride : Mod->space_stride;
		if (Mod->phase >= Mod->sine_len)
			Mod->phase -= Mod->sine_len;
	}

	return sample_count;
}

void AFSK_Mod_Reset(AFSK_Mod_t * Mod) {
	if (!Mod)
		return;

	Mod->phase = 0;
	Mod->bit_clock = INT32_MAX;
	Mod->bit_pos = 0;
}
