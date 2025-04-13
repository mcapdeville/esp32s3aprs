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

#include <stdio.h>
#include <inttypes.h>
#include "esp_sleep.h"
#include "ulp_riscv.h"
#include "ulp_adc.h"
#include "ulp_main.h"
#include "ulp_lowpower.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


extern const uint8_t ulp_lowpower_bin_start[] asm("_binary_ulp_lowpower_bin_start");
extern const uint8_t ulp_lowpower_bin_end[]   asm("_binary_ulp_lowpower_bin_end");

static void init_lowpower_program(void);

int lowpower_start(void)
{

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if ((cause != ESP_SLEEP_WAKEUP_ULP) && (cause != ESP_SLEEP_WAKEUP_TIMER)) {
		// POR start
        init_lowpower_program();
		return 0;
    }

    if (cause == ESP_SLEEP_WAKEUP_ULP) {
		// ULP wakup
		return 1;
    }

	// Other wakup
	// Configure RTC power domain
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_sleep_enable_ulp_wakeup();
    esp_deep_sleep_start();
}

static void init_lowpower_program(void) {
	// Setup  ulp_adc
    ulp_adc_cfg_t cfg = {
        .adc_n    = ADC_UNIT_2,
        .channel  = ADC_CHANNEL_0,
        .width    = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
        .ulp_mode = ADC_ULP_MODE_RISCV,
    };

    ulp_adc_init(&cfg);

	// Load ulp binary
    ulp_riscv_load_binary(ulp_lowpower_bin_start, (ulp_lowpower_bin_end - ulp_lowpower_bin_start));

    /* The first argument is the period index, which is not used by the ULP-RISC-V timer
     * The second argument is the period in microseconds, which gives a wakeup time period of: 60s
     */
    ulp_set_wakeup_period(0, 60000000);

    /* Start the program */
    ulp_riscv_run();
}
