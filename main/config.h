/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/config.h
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

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "afsk_config.h"
#include <SA8x8.h>
#include <ssd1680.h>
#include <esp_pm.h>
#include <driver/spi_master.h>
#include <esp_spiffs.h>
#include <driver/uart.h>

/*********** Adc/Dac config *********************/
#define SAMPLE_RATE	((unsigned long)CONFIG_ESP32S3APRS_RADIO_SAMPLE_RATE)

// USB frame len : unit for sample_buff watermark
#define FRAME_LEN ((SAMPLE_RATE + 500)/1000)

/*********** EPD config ************************/
#define EPD_SPI_HOST	(SPI2_HOST)

/*********** brownout detect ******************/
#define BROWNOUT_DET_LVL	((CONFIG_BROWNOUT_DET_LVL) + 1)

extern const AFSK_Config_t AFSK_Config;
extern const esp_pm_config_t pm_config;
extern const SA8x8_config_t SA8x8_config;
extern const SSD1680_Config_t epd_config;
extern const spi_bus_config_t spi_config;
extern const esp_vfs_spiffs_conf_t Spiffs_Config;
extern const uart_config_t console_config;

#endif
