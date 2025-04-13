# SPDX-License-Identifier: GPL-3.0-or-later
#
# ESP32s3APRS by F4JMZ
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

# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html

cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS dmabuff SA8x8 tinyusb ssd1680 lvgl micropython)
set(MICROPY_USER_C_MODULES "../modules/esp32s3aprs/esp32s3aprs.cmake")
set(MICROPY_PY_BTREE ON)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(IDF_COMPONENT_MANAGER 1)
idf_build_set_property(MINIMAL_BUILD 1)

project(esp32s3aprs)

#nvs_create_partition_image("nvs" "nvs.csv" FLASH_IN_PROJECT)
#nvs_create_partition_image("nvs" "nvs.csv")
#spiffs_create_partition_image("spiffs" "spiffs" FLASH_IN_PROJECT)
#spiffs_create_partition_image("spiffs" "spiffs")
