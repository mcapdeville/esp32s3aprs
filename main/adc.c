/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/adc.c
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

#include <freertos/FreeRTOS.h>

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <esp_pm.h>

static adc_cali_handle_t cali[10];
static adc_unit_t unit;
static adc_oneshot_unit_handle_t handle;
static esp_pm_lock_handle_t adc_pm_lock;


int ADC_Init(adc_unit_t Num) {
	if (handle)
		return -1;

	esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "adc2", &adc_pm_lock);

	unit = Num;
	adc_oneshot_unit_init_cfg_t init_config = {
		.unit_id = Num,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};

	return adc_oneshot_new_unit(&init_config, &handle);
}

int ADC_Config_Channel(int Pin,int Atten) {
	adc_channel_t Channel;
	adc_unit_t lUnit;


	if (!handle)
		return -1;

	adc_oneshot_io_to_channel(Pin,&lUnit,&Channel);

	if (lUnit != unit)
		return -1;

	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_BITWIDTH_DEFAULT,
		.atten = Atten,
	};

	if (adc_oneshot_config_channel(handle, Channel, &config))
		return -1;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED

	if (cali[Channel]) {
		adc_cali_delete_scheme_curve_fitting(cali[Channel]);
		cali[Channel] = NULL;
	}

	adc_cali_curve_fitting_config_t cali_config = {
		.unit_id = unit,
		.chan = Channel,
		.atten = Atten,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};

	adc_cali_create_scheme_curve_fitting(&cali_config, &cali[Channel]);
	
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED

	if (cali[Channel]) {
		adc_cali_delete_scheme_line_fitting(cali[Channel]);
		cali[Channel] = NULL;
	}

	adc_cali_line_fitting_config_t cali_config = {
		.unit_id = unit,
		.chan = Channel,
		.atten = atten,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	
	adc_cali_create_scheme_line_fitting(&cali_config, &cali[Channel]);

#endif

return 0;

}

int ADC_Deinit() {

	int i;

	if (!handle)
		return -1;

	adc_oneshot_del_unit(handle);
	handle = 0;

	for (i = 0; i<(sizeof(cali)/sizeof(cali[0])); i++)
		if (cali[i]) 
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
			adc_cali_delete_scheme_curve_fitting(cali[i]);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
			adc_cali_delete_scheme_line_fitting(cali[i]);
#endif
	esp_pm_lock_delete(adc_pm_lock);

			return 0;
}

int ADC_Read(int Pin, int *raw, int *volt) {

	adc_channel_t Channel;
	adc_unit_t lUnit;
	int res;

	if (!handle)
		return -1;

	adc_oneshot_io_to_channel(Pin,&lUnit,&Channel);

	if (lUnit != unit)
		return -1;

	esp_pm_lock_acquire(adc_pm_lock);

	adc_oneshot_read(handle,Channel,&res);
	
	esp_pm_lock_release(adc_pm_lock);

	if (raw)
		*raw = res;

	if (cali[Channel] && volt)
		adc_cali_raw_to_voltage(cali[Channel],res,volt);

	return 0;
}
