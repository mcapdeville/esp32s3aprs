/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * ssd1680.h
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

#ifndef _SSD1680_H_
#define _SSD1680_H_

#include <driver/spi_master.h>
#include <freertos/semphr.h>
#include "ssd1680_def.h"
#include <lvgl.h>

typedef struct SSD1680_S SSD1680_t;
typedef struct SSD1680_Config_S SSD1680_Config_t;

struct SSD1680_Config_S {
	uint16_t xres;
	uint16_t yres;
	uint16_t dpi;
	struct SSD1680_Driver_Output_Ctrl_S driver_output_control;
	struct SSD1680_Border_Waveform_S border_waveform;
	struct SSD1680_Lut_End_Option_S lut_end;
	struct SSD1680_Boost_SS_Ctrl_S  soft_start;
	struct SSD1680_Temp_Sensor_S  temp_sensor;
	struct SSD1680_Display_Update_Sequence_S load_sequence;
	struct SSD1680_Display_Update_Sequence_S update_sequence;
};

enum SSD1680_Orientation_E {
	SSD1680_ORIENTATION_0 = 0,
	SSD1680_ORIENTATION_90,
	SSD1680_ORIENTATION_180, 
	SSD1680_ORIENTATION_270,
};

SSD1680_t * SSD1680_Init(spi_host_device_t Host, int Power_Pin, int Cs_Pin, int Dc_Pin, int Busy_Pin, const SSD1680_Config_t * Config,unsigned char *logo);
int SSD1680_Reset(SSD1680_t * Epd);
int SSD1680_Setup(SSD1680_t *Epd);
int SSD1680_Write_Image(SSD1680_t * Epd);
int SSD1680_Update(SSD1680_t * Epd);
int SSD1680_Deep_Sleep(SSD1680_t * Epd);
void SSD1680_PowerUp(SSD1680_t * Epd);
void SSD1680_PowerDown(SSD1680_t * Epd);
int SSD1680_Redraw(SSD1680_t * Epd);
int SSD1680_Power_Redraw(SSD1680_t *Epd,bool Full);
int SSD1680_Background_Redraw(SSD1680_t *Epd,bool full);
void SSD1680_Fill_Random(SSD1680_t * Epd);
void SSD1680_Fill(SSD1680_t * Epd,uint8_t c);
void SSD1680_Fill_Bitmap(SSD1680_t * Epd,uint8_t * plane_1,uint8_t * plane_2);
void SSD1680_Set_Orientation(SSD1680_t * Epd,enum SSD1680_Orientation_E Orient);
void SSD1680_Set_Border(SSD1680_t * Epd,uint8_t Lut);
bool SSD1680_Is_Dirty(SSD1680_t * Epd);
void SSD1680_Flush_cb(SSD1680_t * Epd, lv_area_t * Area, uint8_t *Colormap);
void SSD1680_Wait_cb(SSD1680_t * Epd);
void SSD1680_Drv_Update_cb(SSD1680_t * Epd);


#endif
