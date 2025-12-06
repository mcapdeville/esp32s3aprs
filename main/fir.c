#include "fir.h"
#include <malloc.h>
#include <string.h>
#include <math.h>

#define TAG "FIR"

#if 0
#define LOGE(FMT, ...) fprintf(stderr,"E : " TAG " : " FMT "\n" __VA_ARGS__)
#define LOGD(FMT, ...) fprintf(stderr,"D : " TAG " : " FMT "\n",__VA_ARGS__)
#define MALLOC(SIZE) malloc(SIZE)
#define MEMALIGN(A,SIZE) memalign(A,SIZE)
#define FREE(PTR) free(PTR)
#else
#include <esp_log.h>
#define LOGE(FMT, ...) ESP_LOGE(TAG, FMT, __VA_ARGS__)
#define LOGD(FMT, ...)  ESP_LOGD(TAG, FMT, __VA_ARGS__)
#define MALLOC(SIZE) malloc(SIZE)
#define MEMALIGN(A,SIZE) memalign(A,SIZE)
#define FREE(PTR) free(PTR)
#endif

int fir(fir_t *fir, const float *input, float *output, int len) {
	float acc;
	int coeff_pos;
	int n,i;

    for (i = 0 ; i < len ; i++) {
		acc = 0;
		coeff_pos = 0;
        fir->delay[fir->pos] = input[i];
        fir->pos++;
        if (fir->pos >= fir->N) {
            fir->pos = 0;
        }
        for (n = fir->pos; n < fir->N ; n++) {
            acc += fir->coeffs[coeff_pos++] * fir->delay[n];
        }
        for (n = 0; n < fir->pos ; n++) {
            acc += fir->coeffs[coeff_pos++] * fir->delay[n];
        }
        output[i] = acc;
    }
    return 0;
}

int fir_coeffs_init(fir_t * fir) {
	int j;
	if (!fir->N & 1) {
		fir->coeffs[0] = 0;
		j = 1;
	} else {
		j = 0;
	}

	for (; j<fir->N ; j++) {
		fir->coeffs[j] = 1.0f;
	}
	
	return 0;
}

int fir_norm(fir_t * fir) {
	int j;
	float acc=0;

	for (j=0 ; j<fir->N ; j++) {
		acc += fir->coeffs[j];
	}

	for (j=0 ; j<fir->N ; j++) {
		fir->coeffs[j] /= acc;;
		ESP_LOGD(TAG, "coeffs[%d] = %e", j, fir->coeffs[j]);
	}

	return 0;
}

int fir_gen_bpf (fir_t * fir, float f1, float f2) {
	int j;
	float center;
	float w1 = 2.0f * M_PI * f1;
	float w2= 2.0f * M_PI * f2;

	ESP_LOGD(TAG, "bpf_fir : w1 = %f, w2 = %f", w1, w2);

	if (!fir->N & 1) {
		fir->coeffs[0] = 0;
		j = 1;
		center = (float)fir->N * 0.5f;
	} else {
		j = 0;
		center = (float)(fir->N-1) * 0.5f;
	}
	for (; j < fir->N; j++) {
		if (j - center == 0) {
			fir->coeffs[j] *= 2.0f * (f2 - f1);
		}
		else {
			fir->coeffs[j] *= (sin(w2 * (float)(j-center)) - sin(w1 * (float)(j-center))) / (M_PI*(float)(j-center));
		}
	}

	return 0;
}

int fir_gen_lpf (fir_t * fir, float fc) {
	int j;
	float center;
	float w = 2.0f * M_PI * fc;

	if (!fir->N & 1) {
		fir->coeffs[0] = 0;
		j = 1;
		center = (float)fir->N * 0.5f;
	} else {
		j = 0;
		center = (float)(fir->N-1) * 0.5f;
	}

	for (; j<fir->N; j++) {
		if (j - center == 0) {
			fir->coeffs[j] *= 2.0f * fc;
		}
		else {
			fir->coeffs[j] *= sin(w * (float)(j-center)) / (M_PI*(float)(j-center));
		}
	}

	return 0;
}

int fir_gen_sinc(fir_t * fir, float fc) {
	int j;
	float center;
	float w = 2.0f * M_PI * fc;

	if (!fir->N & 1) {
		fir->coeffs[0] = 0;
		j = 1;
		center = (float)fir->N * 0.5f;
	} else {
		j = 0;
		center = (float)(fir->N-1) * 0.5f;
	}

	for (; j<fir->N; j++) {
		if (j - center == 0) {
			fir->coeffs[j] *= 1;
		}
		else {
			fir->coeffs[j] *= sin(w * (float)(j-center)) / (w*(float)(j-center));
		}
	}

	return 0;
}
