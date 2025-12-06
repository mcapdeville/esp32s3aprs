/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/afsk_demod.c
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

#include <stddef.h>
#include <malloc.h>
#include <string.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_dsp.h>
#include <math.h>
#include "afsk_demod.h"

#define TAG "AFSK_Demod"

#define AFSK_DEMOD_EXTRA_BW	(0.5f)	// extra band width in regard of baud_rate
#define BPF_FIR_LEN		(2.0f)
#define LPF_CUTOFF		(1.0f)	// lpf cutoff frequency in regard to baud_rate
#define LPF_FIR_LEN		(4.0f)
#define LPF_QUALITY		(0.7f)	// lpf quality factor
#define AGC_ATTACK_TAU	(0.01f) // AGC Attack coef
#define AGC_DECAY_TAU	(0.5f)   // AGC decay coef
#define HYSTERESIS		(0.001f) // Hysteresis
#define MIN_GAIN		(1.0f/10000.0f)	// Minimum gain
#define MAX_GAIN		(1.0f/2500.0f)	// Maximum gain
#define TRANSITION_GOOD		1	// transition time window in inverse power of 2 : 1/(2^x) bit len
#define PLL_SEARCH_SHIFT	2	// pll search inertia
#define PLL_LOCKED_SHIFT	3	// pll locked inertia
#define DCD_THRESHOLD_ON	30	// In number of dcd_flags TRUE
#define DCD_THRESHOLD_OFF	2	// In number of dcd_flags FALSE

#define NO_BPF 0
#define NO_LPF 0

#define BPF_FIR 1
#define COS_W 1
#define LPF_FIR 1
#define AGC	1
#define GAIN_LIM 0

#if BPF_FIR || LPF_FIR
#include "fir.h"
#endif

/* sample rate must be multiple of mark_freq*4 and space_freq*4
 * so 52800Hz for 1200Hz and 2200Hz
 */

struct AFSK_Demod_S {
	uint16_t baud_rate;
	// Input buffer
	float * input_buff;	// input samples buffer
	float * input_end;	// end of input samples buffer
	float * input_pos;	// input pointer in input samples buffer
	float * input_save;	// to debug input filter
	uint8_t input_skip;	// skip next 0..3 input sample

	// Input band pass filter
#if !NO_BPF
#if BPF_FIR
	fir_t bpf_fir;
#else
	float bpf_coefs[5];	// input bandpass filter coefs	
	float bpf_state[2];	// input bandpass filter state
#endif
#endif

	// goertzel filter
	uint16_t decim_len;	// Number of sample after decimation
	uint16_t goertzel_len;	// len in input sample to aplly goertzel filter
#if COS_W
	float mark_cos_w;	// 2*cos(w)
	float space_cos_w;	// 2*cos(w)
#else
	uint16_t mark_stride;	// number of sample for pi/2 phase at mark tone freq
	uint16_t space_stride;	// number of sample for pi/2 phase at space tone freq
#endif

	float * mark_buff;	// mark power buffer (decimated by 4)
	float * space_buff;	// space power buffer (decimated by 4)

	// Low pass filter
#if !NO_LPF
#if LPF_FIR
	fir_t mark_lpf_fir;
	fir_t space_lpf_fir;
#else
	float lpf_coefs[5];	// Lowpass filter coefs	
	float lpf_mark_state[2];// Lowpass mark filter state
	float lpf_space_state[2];// Lowpass space filter state
#endif
#endif

							 // AGC
	float agc_attack;	// Attack coefs
	float agc_decay;	// Decay coefs
	float mark_peak,mark_valley,mark_gain;	// Peak/valley level for mark tone
	float space_peak,space_valley,space_gain;	// Peak/valley level for space tone
												// detection state
	bool symbol_state;		// Current symbol state
							// Clock Recovery
	int32_t pll_step;		// Pll step
	int32_t pll_count;		// pll count
							// data carrier detect
	bool good_tr,bad_tr;		// Last transition status
	uint32_t good_flags;		// Good transition history
	uint32_t bad_flags;		// Bad transition history
	uint32_t dcd_flags;		// dcd (Good-bad>2) flags
	bool dcd;			// Dcd state
};

AFSK_Demod_t * AFSK_Demod_Init(AFSK_Config_t const * Config) {
	AFSK_Demod_t * demod;
	uint16_t input_len;	// input buffer len
	float tau,ts;

	// Allocate demod struct
	if (!(demod = heap_caps_malloc(sizeof(struct AFSK_Demod_S),MALLOC_CAP_INTERNAL))) {
		ESP_LOGE(TAG,"Error allocating AFSK_Demod struct");
		return NULL;
	}
	memset(demod,0,sizeof(AFSK_Demod_t));

	demod->baud_rate = Config->baud_rate;

	// Tone detection on 1 bit len
	demod->goertzel_len = (((int32_t)Config->sample_rate<<1)/Config->baud_rate+1)>>1;

	// input buffer of 2 time goertzel len
	input_len = demod->goertzel_len<<1;

	if (!(demod->input_buff = heap_caps_malloc(input_len*sizeof(float),MALLOC_CAP_INTERNAL))) {
		ESP_LOGE(TAG,"Error allocating input_buffer");
		heap_caps_free(demod);
		return NULL;
	}
	demod->input_pos = demod->input_buff;
	demod->input_end = demod->input_buff+input_len;
	demod->input_skip = 0;

	if (!(demod->mark_buff = heap_caps_malloc((input_len>>2)*sizeof(float),MALLOC_CAP_INTERNAL))) {
		ESP_LOGE(TAG,"Error allocating mark_buffer");
		heap_caps_free(demod->input_buff);
		heap_caps_free(demod);
		return NULL;
	}

	if (!(demod->space_buff = heap_caps_malloc((input_len>>2)*sizeof(float),MALLOC_CAP_INTERNAL))) {
		ESP_LOGE(TAG,"Error allocating mark_buff");
		heap_caps_free(demod->mark_buff);
		heap_caps_free(demod->input_buff);
		heap_caps_free(demod);
		return NULL;
	}

#if !NO_BPF
	// input bandpass filter
	uint16_t center_freq = ((Config->mark_freq+Config->space_freq+1)>>1);
	uint16_t bandwidth;

	if (Config->space_freq < Config->mark_freq)
		bandwidth = Config->mark_freq - Config->space_freq;
	else
		bandwidth = Config->space_freq - Config->mark_freq;

	bandwidth = bandwidth + AFSK_DEMOD_EXTRA_BW*Config->baud_rate;

#if BPF_FIR
	demod->bpf_fir.N = (((int)round(((float)(Config->sample_rate/Config->baud_rate)*(BPF_FIR_LEN))))+3) & ~3;
	if (!(demod->bpf_fir.coeffs = memalign(16, (demod->bpf_fir.N+4) * sizeof(float)))) {
		ESP_LOGE(TAG,"bpf_fir : Error allocating BPF_FIR coefficients");
		heap_caps_free(demod->space_buff);
		heap_caps_free(demod->mark_buff);
		heap_caps_free(demod->input_buff);
		heap_caps_free(demod);
		return NULL;
	}

	int ret;
	if ((ret = dsps_fir_init_f32(&demod->bpf_fir, demod->bpf_fir.coeffs, NULL, demod->bpf_fir.N)) != ESP_OK)
		ESP_LOGE(TAG,"bpf_fir : Error in fir_init(%d)",ret);
	else
		ESP_LOGD(TAG,"bpf_fir : N = %d",demod->bpf_fir.N);

	ESP_LOGD(TAG, "bpf_fir : center-freq = %d, bandwidth = %d", center_freq, bandwidth);

	fir_coeffs_init(&demod->bpf_fir);

	fir_gen_bpf(&demod->bpf_fir,
			(float)(center_freq - ((bandwidth+1)>>1))/(float)Config->sample_rate,
			(float)(center_freq + ((bandwidth+1)>>1))/(float)Config->sample_rate
			);

	//fir_norm(&demod->bpf_fir);

#else
	float Q = (float)center_freq/((float)bandwidth);
	dsps_biquad_gen_bpf0db_f32(demod->bpf_coefs,(float)center_freq/(float)Config->sample_rate,Q);
#endif
#endif

	// Goertzel filter
#if COS_W
	demod->mark_cos_w = 2.0f * cos(2.0f * M_PI * (float)Config->mark_freq/(float)Config->sample_rate);
	demod->space_cos_w = 2.0f * cos(2.0f * M_PI * (float)Config->space_freq/(float)Config->sample_rate);
#else
	demod->mark_stride = ((Config->sample_rate/Config->mark_freq) + 2)>>2;  // 4 sample per period
	demod->space_stride = ((Config->sample_rate/Config->space_freq) + 2)>>2; // 4 sample per period
#endif

	// LPF filters
#if !NO_LPF
#if LPF_FIR
	demod->mark_lpf_fir.N = (((int)round(((float)(Config->sample_rate/Config->baud_rate)*(LPF_FIR_LEN))))+3) & ~3;
	if (!(demod->mark_lpf_fir.coeffs = memalign(16, (demod->mark_lpf_fir.N+4) * sizeof(float)))) {
		ESP_LOGE(TAG,"lpf_fir : Error allocating BPF_FIR coefficients");
		heap_caps_free(demod->space_buff);
		heap_caps_free(demod->mark_buff);
		heap_caps_free(demod->input_buff);
		heap_caps_free(demod);
		return NULL;
	}

	fir_coeffs_init(&demod->mark_lpf_fir);
	fir_gen_sinc(&demod->mark_lpf_fir, (float)Config->baud_rate*LPF_CUTOFF/(((float)Config->sample_rate)/4.0f) );
	//fir_norm(&demod->mark_lpf_fir);

	if ((ret = dsps_fir_init_f32(&demod->mark_lpf_fir, demod->mark_lpf_fir.coeffs, NULL, demod->mark_lpf_fir.N)) != ESP_OK)
		ESP_LOGE(TAG,"lpf_fir : Error in fir_init(%d)",ret);
	else
		ESP_LOGD(TAG,"lpf_fir : N = %d",demod->mark_lpf_fir.N);

	if ((ret = dsps_fir_init_f32(&demod->space_lpf_fir, demod->mark_lpf_fir.coeffs, NULL, demod->mark_lpf_fir.N)) != ESP_OK)
		ESP_LOGE(TAG,"lpf_fir : Error in fir_init(%d)",ret);
	else
		ESP_LOGD(TAG,"lpf_fir : N = %d",demod->space_lpf_fir.N);


#else
	dsps_biquad_gen_lpf_f32(demod->lpf_coefs, (float)Config->baud_rate*LPF_CUTOFF/(((float)Config->sample_rate)/4), LPF_QUALITY);
#endif
#endif

	// AGC Constant
	ts = 1.0f/(Config->sample_rate);
	tau = AGC_ATTACK_TAU; // tau in number of bits
	demod->agc_attack = ts/(ts + tau);
	ESP_LOGD(TAG,"Attack : ts = %f tau = %f coef = %f",ts,tau,demod->agc_attack);
	tau = AGC_DECAY_TAU;
	demod->agc_decay = ts/(ts + tau);
	ESP_LOGD(TAG,"Decay : ts = %f tau = %f coef = %f",ts,tau,demod->agc_decay);


	// Clock recovery
	demod->pll_step = round(((float)(1LL<<32)*(float)(Config->baud_rate<<2))/(float)Config->sample_rate);

	// Start in a clean state
	AFSK_Demod_Reset(demod);

	return demod;
}

/* Inputs:
 * - a2: fsrc
 * - a3: decim_len
 * - a4: goertzel_len
 * - a5: tone_stride:
 * - a6: fdst
 */
extern void goertzel_decimate(float * fsrc, int32_t decim_len, int32_t goertzel_len, int32_t tone_stride, float * fdst, float gain);

__attribute__((hot))
	uint16_t AFSK_Demod_Input(AFSK_Demod_t * Demod,int16_t *Samples,uint16_t Len,uint8_t * Out_buff, int16_t Buff_size, uint16_t *Out_len) {
		int16_t i,j;
		int16_t in_len;
		float *fdst, *fsrc, *ffilter;
		float Q0,Q1,Q2;
		bool prev_state;
		int32_t prev_count;

		if (!Demod || !Samples || !Out_buff || !Out_len)
			return 0;

		in_len = Demod->input_end - Demod->input_pos;
		Demod->input_save = Demod->input_pos;

		if (Len > in_len)
			Len = in_len;

		*Out_len = 0;
		*Out_buff = 0;	

#if !NO_BPF
		float *in_ptr = Demod->input_pos;
#endif
		// Convert to float
		for (i=Len;i;i--,Demod->input_pos++,Samples++)
			*Demod->input_pos = (float)*Samples;

		// Apply bandpass filter inplace
#if !NO_BPF
#if BPF_FIR
		dsps_fir_f32(&Demod->bpf_fir, in_ptr, in_ptr, Len); 
#else
		dsps_biquad_f32(in_ptr,in_ptr,Len,Demod->bpf_coefs,Demod->bpf_state);
#endif
#endif

		// Decimate by 4 and apply goertzel filter
		in_len = (Demod->input_pos - Demod->input_buff) - Demod->input_skip;
		if (in_len < Demod->goertzel_len) { // not enough data
			Demod->decim_len = 0;
			return Len;
		}

		Demod->decim_len = 1 + ((in_len-Demod->goertzel_len)>>2) ;

		// mark tone
#if 0
		goertzel_decimate(Demod->input_buff + Demod->input_skip, Demod->decim_len, Demod->goertzel_len, Demod->mark_stride, Demod->mark_buff, 1.0f/((float)Demod->space_stride)); 
		goertzel_decimate(Demod->input_buff + Demod->input_skip, Demod->decim_len, Demod->goertzel_len, Demod->space_stride, Demod->space_buff, 1.0f/((float)Demod->mark_stride)); 
		fsrc = Demod->input_buff + Demod->input_skip + (Demod->decim_len<<2);
#else
		fsrc = Demod->input_buff + Demod->input_skip;
		fdst = Demod->mark_buff;
		for (j = Demod->decim_len;j;j--,fdst++,fsrc+=4) {        // Decimation loop
			Q1 = 0;
			Q2 = 0;
#if COS_W
			for (i=Demod->goertzel_len,ffilter=fsrc;i>0;i--,ffilter++) {
				Q0 = *ffilter + Demod->mark_cos_w*Q1 - Q2;
#else
			for (i=Demod->goertzel_len,ffilter=fsrc;i>0;i-=Demod->mark_stride,ffilter+=Demod->mark_stride) {
				Q0 = *ffilter-Q2;       // Q0 = *fsrc + 2*cos(w)*Q1 - Q2; but cos(w) = 0
#endif
				Q2 = Q1;
				Q1 = Q0;
			}
#if COS_W
			*fdst = sqrt(Q2*Q2 + Q1*Q1 - Demod->mark_cos_w*Q2*Q1); // power = sqrt(Q2*Q2 + Q1*Q1 - 2*cos(w)*Q2*Q1)
#else
			*fdst = sqrt(Q2*Q2 + Q1*Q1) / Demod->space_stride; // power = sqrt(Q2*Q2 + Q1*Q1 - 2*cos(w)*Q2*Q1); again, cos(w) = 0
#endif
		}

		// space tone
		fsrc = Demod->input_buff + Demod->input_skip;
		fdst = Demod->space_buff;
		for (j = Demod->decim_len;j;j--,fdst++,fsrc+=4) {        // Decimation loop
			Q1 = 0;
			Q2 = 0;
#if COS_W
			for (i=Demod->goertzel_len,ffilter=fsrc;i>0;i--,ffilter++) {
				Q0 = *ffilter + Demod->space_cos_w*Q1 - Q2;
#else
			for (i=Demod->goertzel_len,ffilter=fsrc;i>0;i-=Demod->space_stride,ffilter+=Demod->space_stride) {
				Q0 = *ffilter-Q2;       // Q0 = *fsrc + 2*cos(w)*Q1 - Q2; but cos(w) = 0
#endif

				Q2 = Q1;
				Q1 = Q0;
			}
#if COS_W
			*fdst = sqrt(Q2*Q2 + Q1*Q1 - Demod->space_cos_w*Q2*Q1); // power = sqrt(Q2*Q2 + Q1*Q1 - 2*cos(w)*Q2*Q1)
#else
			*fdst = sqrt(Q2*Q2 + Q1*Q1) / Demod->mark_stride; // power = sqrt(Q2*Q2 + Q1*Q1 - 2*cos(w)*Q2*Q1); again, cos(w) = 0
#endif
		}
#endif
		// move unused input data to the beginning of the input buffer
		if (fsrc<Demod->input_pos) {  // there is unused data
			memcpy(Demod->input_buff,fsrc,(Demod->input_pos-fsrc)<<2);
			Demod->input_pos = Demod->input_buff + (Demod->input_pos-fsrc);
			Demod->input_skip = 0;
		} else { // no remaining data
			Demod->input_pos = Demod->input_buff;
			Demod->input_skip = fsrc - Demod->input_pos;	// skip 0 to 3 next input data
		}

		// Low pass filters
#if !NO_LPF
#if LPF_FIR
		dsps_fir_f32(&Demod->mark_lpf_fir, Demod->mark_buff, Demod->mark_buff, Demod->decim_len);
		dsps_fir_f32(&Demod->space_lpf_fir, Demod->space_buff, Demod->space_buff, Demod->decim_len);
#else
		dsps_biquad_f32(Demod->mark_buff,Demod->mark_buff,Demod->decim_len,Demod->lpf_coefs,Demod->lpf_mark_state);
		dsps_biquad_f32(Demod->space_buff,Demod->space_buff,Demod->decim_len,Demod->lpf_coefs,Demod->lpf_space_state);
#endif
#endif

		fsrc = Demod->mark_buff;
		fdst = Demod->space_buff;

		for (i=Demod->decim_len;i;i--,fsrc++,fdst++) {
#if AGC
			// mark AGC
			if (*fsrc > Demod->mark_peak) {
				Demod->mark_peak += (*fsrc - Demod->mark_peak) * Demod->agc_attack;
			}
			else 
				Demod->mark_peak += (*fsrc - Demod->mark_peak) * Demod->agc_decay;

			if (*fsrc < Demod->mark_valley) {
				Demod->mark_valley += (*fsrc - Demod->mark_valley) * Demod->agc_attack;
			}
			else 
				Demod->mark_valley += (*fsrc - Demod->mark_valley) * Demod->agc_decay;

			if (Demod->mark_peak > Demod->mark_valley)
				Demod->mark_gain = 1.0f/(Demod->mark_peak - Demod->mark_valley);
			else
				Demod->mark_gain = 0.0f;

			if (Demod->mark_gain >0.0f) {
#if GAIN_LIM
				if (Demod->mark_gain > MAX_GAIN)
					Demod->mark_gain = MAX_GAIN;
				else if (Demod->mark_gain < MIN_GAIN)
					Demod->mark_gain = MIN_GAIN;
#endif

				*fsrc = (*fsrc - 0.5f*(Demod->mark_peak + Demod->mark_valley))*Demod->mark_gain;
			}
			else
				*fsrc = 0;

			// space AGC
			if (*fdst > Demod->space_peak) {
				Demod->space_peak += (*fdst - Demod->space_peak) * Demod->agc_attack;
			}
			else 
				Demod->space_peak += (*fdst - Demod->space_peak) * Demod->agc_decay;

			if (*fdst < Demod->space_valley) {
				Demod->space_valley += (*fdst - Demod->space_valley) * Demod->agc_attack;
			}
			else 
				Demod->space_valley += (*fdst - Demod->space_valley) * Demod->agc_decay;

			if (Demod->space_peak > Demod->space_valley)
				Demod->space_gain = 1.0f/(Demod->space_peak - Demod->space_valley);
			else
				Demod->space_gain = 0.0f;

			if (Demod->space_gain > 0.0f) {
#if GAIN_LIM
				if (Demod->space_gain > MAX_GAIN)
					Demod->space_gain = MAX_GAIN;
				else if (Demod->space_gain < MIN_GAIN)
					Demod->space_gain = MIN_GAIN;
#endif

				*fdst = (*fdst - 0.5f * (Demod->space_peak + Demod->space_valley))*Demod->space_gain;
			}
			else
				*fdst = 0;
#endif
			// Take decision
			prev_state = Demod->symbol_state;
			if ((*fsrc - *fdst) > HYSTERESIS )
				Demod->symbol_state = true;
			else if ((*fdst - *fsrc) > HYSTERESIS)
				Demod->symbol_state = false;

			// Clock recovery
			prev_count = Demod->pll_count;
			Demod->pll_count += Demod->pll_step;

			if (Demod->pll_count <0 && prev_count>0) { // PLL count overflow
				Demod->good_flags <<= 1;
				Demod->good_flags |= Demod->good_tr;
				Demod->good_tr = 0;

				Demod->bad_flags <<= 1;
				Demod->bad_flags |= Demod->bad_tr;
				Demod->bad_tr = 0;

				Demod->dcd_flags <<= 1;
				Demod->dcd_flags |= (((signed)__builtin_popcount(Demod->good_flags)-(signed)__builtin_popcount(Demod->bad_flags)) >= 3 );

				int score = __builtin_popcount(Demod->dcd_flags);

				if (!Demod->dcd && score > DCD_THRESHOLD_ON) {
					Demod->dcd = true;
				}
				else if (Demod->dcd && score < DCD_THRESHOLD_OFF) {
					Demod->dcd = false;
				}

				// Sample time

				if (Out_buff && Out_len) {
					(*Out_buff)>>=1;
					if (Demod->dcd) 
						(*Out_buff) |= Demod->symbol_state ? 0x80:0;
					else
						(*Out_buff) |= 0X80;	// Indicate carrier lost

					(*Out_len)++;
					if (!((*Out_len)&7)) {
						if ((*Out_len)>>3 == Buff_size)
							return Len;
						Out_buff++;
					}
				}
			} 

			if (Demod->symbol_state != prev_state) {
				// Transition event
				if (Demod->pll_count < (INT32_MAX>>TRANSITION_GOOD) && Demod->pll_count > (INT32_MIN>>TRANSITION_GOOD)) {
					// Transition windows good
					Demod->good_tr = true;
				}
				else {
					// Transition windows bad
					Demod->bad_tr = true;
				}

				if (Demod->dcd)
					Demod->pll_count -= Demod->pll_count>>PLL_LOCKED_SHIFT;
				else
					Demod->pll_count -= Demod->pll_count>>PLL_SEARCH_SHIFT;
			}
		}

		if ((*Out_len)&7)
			(*Out_buff)>>=(8-((*Out_len)&7));

		return Len;
	}

void AFSK_Demod_Reset(AFSK_Demod_t * Demod) {

	if (!Demod)
		return;

	Demod->input_skip = 0;
	Demod->input_pos = Demod->input_buff;
#if !NO_BPF
#if BPF_FIR
	for (int j=0;j<Demod->bpf_fir.N+4;j++) 
		Demod->bpf_fir.delay[j] = 0;
#else
	Demod->bpf_state[0] = 0;
	Demod->bpf_state[1] = 0;
#endif
#endif
	Demod->decim_len = 0;
#if !NO_LPF
#if LPF_FIR
	for (int j=0;j<Demod->mark_lpf_fir.N+4;j++) {
		Demod->mark_lpf_fir.delay[j] = 0;
		Demod->space_lpf_fir.delay[j] = 0;
	}
#else
	Demod->lpf_mark_state[0] = 0;
	Demod->lpf_mark_state[1] = 0;
	Demod->lpf_space_state[0] = 0;
	Demod->lpf_space_state[1] = 0;
#endif
#endif
	Demod->mark_peak = 0;
	Demod->mark_valley = 0;
	Demod->space_peak = 0;
	Demod->space_valley = 0;
	Demod->symbol_state = false;
	Demod->pll_count = 0;
	Demod->good_flags = 0;
	Demod->bad_flags = 0;
	Demod->dcd_flags = 0;
	Demod->dcd = false;
}

void AFSK_Demod_Get_Buffs(AFSK_Demod_t * Demod,float ** Input,uint16_t *Input_len,float ** Mark, float ** Space,uint16_t * Decim_len) {
	if (!Demod) 
		return ;

	if (Input)
		*Input = Demod->input_save;
	if (Input_len)
		*Input_len = Demod->input_pos - Demod->input_buff;
	if (Mark)
		*Mark = Demod->mark_buff;
	if(Space)
		*Space = Demod->space_buff;
	if (Decim_len)
		*Decim_len = Demod->decim_len;

	return;
}

bool AFSK_Demod_Get_DCD(AFSK_Demod_t * Demod) {
	if (!Demod)
		return false;

	return Demod->dcd;
}

void AFSK_Demod_Get_Gain(AFSK_Demod_t * Demod, float * mark_gain, float * space_gain) {
	*mark_gain = Demod->mark_gain;
	*space_gain = Demod->space_gain;
}
