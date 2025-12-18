/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/goertzel_decimate.s
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


/* Inputs:
 * - a2: fsrc
 * - a3: decim_len
 * - a4: goertzel_len
 * - a5: tone_stride
 * - a6: fdst
 * - a7: gain
 * 
 * Outputs:
 * - decimated (by 4) tone amplitude
 */

.global goertzel_decimate
.section .text

goertzel_decimate:
    entry a1, 32                /* Create stack frame */

    /* Save registers for output buffer and loop variables */
    mov a9, a6                  /* a9 = fdst (output buffer pointer) */
    mov a10, a3                  /* a10 = decim_len (outer loop counter) */

    /* Convert stride (samples) to reciprocal float for normalization */
    wfr f2, a7                  /* Convert stride (a5) to floating-point register */

    /* Multiply stride by sizeof(float) (4 bytes) to get memory offset */
    slli a5, a5, 2              /* a5 = a5 * 4 (stride in memory offset) */

loop_decimation:
    mov a11, a4                 /* a11 = goertzel_len (inner loop counter) */
    sub.s f3, f3, f3              /* f3 = Q1 (Initialize Q1 = 0) */
    sub.s f4, f4, f4              /* f4 = Q2 (Initialize Q2 = 0) */
    mov a8, a2                  /* a8 = Current input pointer for Goertzel loop */

    /* Process one decimated sample using Goertzel filter */
loop_goertzel:
    lsi f0, a8, 0               /* Load *fsrc into f0 */

    sub.s f1, f0, f4          /* Q0 = *fsrc - Q2 */
    mov.s f4, f3              /* Q2 = Q1 */
    mov.s f3, f1               /* Q1 = Q0 */

    add a8, a8, a5              /* Increment *fsrc += stride (Goertzel step size in sample) */
    addi a11, a11, -1           /* Decrement inner loop counter */
    bnez a11, loop_goertzel     /* Continue inner loop if a11 > 0 */

    /* Calculate power = sqrt(Q1*Q1 + Q2*Q2) */
    mul.s f3, f3, f3        /* Q1 * Q1 */
    mul.s f4, f4, f4        /* Q2 * Q2 */
    add.s f5, f3, f4        /* Q1^2 + Q2^2 */
    sqrt0.s f5, f5          /* sqrt_fast(Q1^2 + Q2^2) */
							/* TODO : one or tow Newton-Raphson iteration
							 * for less noise
							 */

    /* Normalize power by stride using reciprocal */
    mul.s f5, f5, f2        /* f5 = power * gain */

    /* Store result in output buffer */
    ssi f5, a9, 0              /* Store normalized power value into fdst */

    addi a9, a9, 4               /* Increment output buffer pointer */
    addi a2, a2, 16              /* Increment input pointer by decimation value (4 * sizeof(float)) */
    addi a10, a10, -1             /* Decrement outer loop counter */
    bnez a10, loop_decimation    /* Continue outer loop if a10 > 0 */

    retw                        /* Return */
