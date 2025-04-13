# SPDX-License-Identifier: GPL-3.0-or-later
#
# ESP32s3APRS by F4JMZ
#
# main/CMakeLists.txt
#
# Copyright (c) 2025 Marc CAPDEVILLE (F4JMZ)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

idf_component_register(
	SRCS
		"config.c"
		"main.c"
		"usb.c"
		"modem.c"
		"modem_afsk1200.c"
		"adc.c"
		"hmi.c"
		"gps.c"
		"gps_parsers.c"
		"afsk_demod.c"
		"afsk_mod.c"
		"hdlc_dec.c"
		"hdlc_enc.c"
		"kiss.c"
		"framebuff.c"
		"aprs.c"
		"aprs_parsers.c"
		"aprs_encoder.c"
		"aprs_log.c"
		"sb.c"
		"ax25.c"
		"ax25_phy.c"
		"ax25_phy_simplex.c"
		"ax25_lm.c"
		"lv_theme/lv_theme_mono_epd.c"
	INCLUDE_DIRS
		"."
	REQUIRES
		"driver"
	    "ssd1680"
		"usb"
		"tinyusb"
		"vfs"
		"dmabuff"
		"SA8x8"
		"esp-dsp"
		"esp_adc"
		"lvgl"
		"esp_event"
		"esp_hw_support"
		"nvs_flash"
		"esp_pm"
		"spiffs"
		#"berkeley-db"
		"micropython"
		"esp_psram"
		"ulp"
)
