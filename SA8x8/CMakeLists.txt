# SPDX-License-Identifier: GPL-3.0-or-later
#
# ESP32s3APRS by F4JMZ
#
# SA8x8/CMakeLists.txt
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

idf_component_register(SRCS "uart.c" "receiver.c" "transmiter.c" "gpio.c" "SA8x8.c"
                    INCLUDE_DIRS "include"
		    PRIV_INCLUDE_DIRS "include_priv"
		    REQUIRES	driver esp_adc dmabuff nvs_flash
		    )

