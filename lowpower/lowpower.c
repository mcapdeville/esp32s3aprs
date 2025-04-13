/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * lowpower.c
 *
 * Copyright (C) 2025  Marc CAPDEVILLE F4JMZ
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

/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* ULP riscv example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <inttypes.h>
#include "esp_sleep.h"
#include "ulp_riscv.h"
#include "ulp_adc.h"
#include "ulp_main.h"
#include "ulp_lowpower.h"
#include "ulp/example_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


extern const uint8_t ulp_lowpower_bin_start[] asm("_binary_ulp_lowpower_bin_start");
extern const uint8_t ulp_lowpower_bin_end[]   asm("_binary_ulp_lowpower_bin_end");

static void init_lowpower_program(void);

int lowpower_start(void)
{

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    /* not a wakeup from ULP, load the firmware */
    if ((cause != ESP_SLEEP_WAKEUP_ULP) && (cause != ESP_SLEEP_WAKEUP_TIMER)) {
        printf("Not a ULP-RISC-V wakeup (cause = %d), initializing it! \n", cause);
        init_lowpower_program();
	return 0;
    }

    /* ULP Risc-V read and detected a temperature above the limit */
    if (cause == ESP_SLEEP_WAKEUP_ULP) {
        printf("ULP-RISC-V woke up the main CPU\n");
        printf("Threshold: high = %"PRIu32"\n", ulp_adc_threshold);
        printf("Value = %"PRIu32" was above threshold\n", ulp_wakeup_result);
	return 1;
    }

    /* Go back to sleep, only the ULP Risc-V will run */
    printf("Entering in deep sleep\n\n");

    /* RTC peripheral power domain needs to be kept on to keep SAR ADC related configs during sleep */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup());

    esp_deep_sleep_start();
}

static void init_lowpower_program(void)
{
    ulp_adc_cfg_t cfg = {
        .adc_n    = ADC_UNIT_2,
        .channel  = ADC_CHANNEL_0,
        .width    = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
        .ulp_mode = ADC_ULP_MODE_RISCV,
    };

    ESP_ERROR_CHECK(ulp_adc_init(&cfg));

    esp_err_t err = ulp_riscv_load_binary(ulp_lowpower_bin_start, (ulp_lowpower_bin_end - ulp_lowpower_bin_start));
    ESP_ERROR_CHECK(err);

    /* The first argument is the period index, which is not used by the ULP-RISC-V timer
     * The second argument is the period in microseconds, which gives a wakeup time period of: 60s
     */
    ulp_set_wakeup_period(0, 60000000);

    /* Start the program */
    err = ulp_riscv_run();
    ESP_ERROR_CHECK(err);
}
