# SPDX-License-Identifier: GPL-3.0-or-later
#
# ESP32s3APRS by F4JMZ
#
# modules/esp32s3aprs/esp32s3aprs.cmake
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

add_library(usermod_esp32s3aprs INTERFACE)

target_sources(usermod_esp32s3aprs INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/esp32s3aprs.c
	${CMAKE_CURRENT_LIST_DIR}/mp_aprs.c
	${CMAKE_CURRENT_LIST_DIR}/mp_aprs_position.c
	${CMAKE_CURRENT_LIST_DIR}/mp_aprs_station.c
	${CMAKE_CURRENT_LIST_DIR}/mp_radio.c
	${CMAKE_CURRENT_LIST_DIR}/mp_ax25_addr.c
	${CMAKE_CURRENT_LIST_DIR}/mp_templ.c
	${CMAKE_CURRENT_LIST_DIR}/mp_aprs_stations_db.c
)

target_include_directories(usermod_esp32s3aprs INTERFACE
	${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_esp32s3aprs)
