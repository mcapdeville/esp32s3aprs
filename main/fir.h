#ifndef _FIR_H_
#define _FIR_H_

#include <stddef.h>
#include <stdbool.h>

#include <dsps_fir.h>

typedef fir_f32_t fir_t;

int fir(fir_t *fir, const float *input, float *output, int len);

int fir_coeffs_init(fir_t * fir);
int fir_norm(fir_t * fir);
int fir_gen_bpf (fir_t *fir, float f1, float f2);
int fir_gen_lpf (fir_t * fir, float fc);
int fir_gen_sinc(fir_t * fir, float fc);

#endif
