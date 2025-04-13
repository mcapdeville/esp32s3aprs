# SPDX-License-Identifier: MIT
#
# The MIT License (MIT)
#
# Copyright (c) 2025 CAPDEVILLE Marc (F4JMZ)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

idf_build_get_property(target IDF_TARGET)

# Set location of base MicroPython directory.
set(MICROPY_DIR ${COMPONENT_DIR}/micropython)
set(MICROPY_PORT_DIR ${MICROPY_DIR}/ports/esp32)
set(MICROPY_COMPONENT_DIR ${COMPONENT_DIR})
set(BERKELEY_DB_CONFIG_FILE "${MICROPY_COMPONENT_DIR}/berkeley_db_config_port.h")

if(CONFIG_MICROPY_USER_BOARD_DIR)
	set(MICROPY_BOARD_DIR ${CONFIG_MICROPY_USER_BOARD_DIR})	
endif()

# Set the board directory and check that it exists.
if(NOT MICROPY_BOARD_DIR)
	# Set the board if it's not already set.
	if(NOT MICROPY_BOARD)
		set(MICROPY_BOARD ESP32_GENERIC_S3)
	endif()

    set(MICROPY_BOARD_DIR ${MICROPY_PORT_DIR}/boards/${MICROPY_BOARD})
endif()

if(NOT EXISTS ${MICROPY_BOARD_DIR}/mpconfigboard.cmake)
	message(FATAL_ERROR "Invalid MICROPY_BOARD specified: ${MICROPY_BOARD_DIR}")
endif()

list(APPEND MICROPY_CPP_INC_EXTRA ${MICROPY_BOARD_DIR})
list(APPEND MICROPY_C_INC_EXTRA ${MICROPY_BOARD_DIR})

include(${MICROPY_BOARD_DIR}/mpconfigboard.cmake)

if (CONFIG_MICROPY_FROZEN_MANIFEST)
	get_filename_component(MICROPY_USER_FROZEN_MANIFEST "${CONFIG_MICROPY_FROZEN_MANIFEST}"
                                ABSOLUTE BASE_DIR "${project_dir}")
endif()

# Set the frozen manifest file. Note if MICROPY_FROZEN_MANIFEST is set from the cmake
# command line, then it will override the default and any manifest set by the board.
if (MICROPY_USER_FROZEN_MANIFEST)
    set(MICROPY_FROZEN_MANIFEST ${MICROPY_USER_FROZEN_MANIFEST})
elseif (NOT MICROPY_FROZEN_MANIFEST)
	set(MICROPY_FROZEN_MANIFEST ${MICROPY_BOARD_DIR}/manifest.py)
endif()

if (MICROPY_USER_C_MODULES)
	set(USER_C_MODULES ${MICROPY_USER_C_MODULES})
endif()

# Include core source components.
include(${MICROPY_DIR}/py/py.cmake)

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    # Enable extmod components that will be configured by extmod.cmake.
    # A board may also have enabled additional components.
	# set(MICROPY_PY_BTREE ON)
    include(${MICROPY_DIR}/py/usermod.cmake)
    include(${MICROPY_DIR}/extmod/extmod.cmake)
endif()

set(MICROPY_SOURCE_SHARED
	${MICROPY_DIR}/shared/readline/readline.c
	${MICROPY_DIR}/shared/runtime/pyexec.c
	${MICROPY_DIR}/shared/runtime/stdout_helpers.c
    	${MICROPY_DIR}/shared/runtime/sys_stdio_mphal.c
    	${MICROPY_DIR}/shared/runtime/interrupt_char.c
    	${MICROPY_DIR}/shared/timeutils/timeutils.c
)

if(IDF_TARGET STREQUAL "esp32c3")
	list(APPEND MICROPY_SOURCE_SHARED ${MICROPY_DIR}/shared/runtime/gchelper_generic.c)
endif()

set(MICROPY_SOURCE_LIB
)

set(MICROPY_SOURCE_DRIVERS
)

set(MICROPY_SOURCE_PORT
	${MICROPY_PORT_DIR}/gccollect.c
   	${MICROPY_PORT_DIR}/mpthreadport.c
)

if (CONFIG_MICROPY_VFS STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/esp32_partition.c)
endif()

if (CONFIG_MICROPY_VFS_POSIX STREQUAL "y")
	list(APPEND MICROPY_SOURCE_LIB ${MICROPY_DIR}/lib/oofatfs/ff.c)
	list(APPEND MICROPY_SOURCE_LIB ${MICROPY_DIR}/lib/oofatfs/ffunicode.c)
endif()

if (CONFIG_MICROPY_VFS_FAT STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/fatfs_port.c)
	list(APPEND MICROPY_SOURCE_LIB ${MICROPY_DIR}/lib/oofatfs/ff.c)
	list(APPEND MICROPY_SOURCE_LIB ${MICROPY_DIR}/lib/oofatfs/ffunicode.c)
endif()

if (CONFIG_MICROPY_VFS_LFS1 STREQUAL "y")
	list(APPEND MICROPY_SOURCE_LIB ${MICROPY_DIR}/lib/littlefs/lfs1.c)
	list(APPEND MICROPY_SOURCE_LIB ${MICROPY_DIR}/lib/littlefs/lfs1_util.c)
endif()

if (CONFIG_MICROPY_VFS_LFS2 STREQUAL "y")
	list(APPEND MICROPY_SOURCE_LIB ${MICROPY_DIR}/lib/littlefs/lfs2.c)
	list(APPEND MICROPY_SOURCE_LIB ${MICROPY_DIR}/lib/littlefs/lfs2_util.c)
endif()

if (CONFIG_MICROPY_MACHINE STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/modmachine.c)
endif()

if (CONFIG_MICROPY_MACHINE_RTC STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/machine_rtc.c)
endif()

if (CONFIG_MICROPY_MACHINE_PIN STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/machine_pin.c)
endif()

if (CONFIG_MICROPY_MACHINE_PULSE STREQUAL "y")
	list(APPEND MICROPY_SOURCE_DRIVER ${MICROPY_DIR}/drivers/dht/dht.c)
endif()

if (CONFIG_MICROPY_MACHINE_TIMER STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/machine_timer.c)
endif()

if (CONFIG_MICROPY_MACHINE_BITSTREAM STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/machine_bitstream.c)
endif()

if (CONFIG_MICROPY_MACHINE_I2C STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/machine_i2c.c)
endif()

if (CONFIG_MICROPY_MACHINE_SPI STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/machine_hw_spi.c)
endif()

if (CONFIG_MICROPY_MACHINE_SOFTSPI STREQUAL "y")
	list(APPEND MICROPY_SOURCE_DRIVERS ${MICROPY_DIR}/drivers/bus/softspi.c)
endif()

if (CONFIG_MICROPY_MACHINE_I2S STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/machine_i2s.c)
endif()

if (CONFIG_MICROPY_ESP STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/modesp.c)
	list(APPEND MICROPY_SOURCE_DRIVERS ${MICROPY_DIR}/drivers/dht/dht.c)
endif()

if (CONFIG_MICROPY_ESP32 STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/modesp32.c)
endif()

if (CONFIG_MICROPY_ESP32_NVS STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/esp32_nvs.c)
endif()

if (CONFIG_MICROPY_ESP32_PARTITION STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/esp32_partition.c)
endif()

if (CONFIG_MICROPY_ESP32_RMT STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/esp32_rmt.c)
endif()

if (CONFIG_MICROPY_ESP32_ULP STREQUAL "y")
	list(APPEND MICROPY_SOURCE_PORT ${MICROPY_PORT_DIR}/esp32_ulp.c)
endif()

set(MICROPY_SOURCE_QSTR
    ${MICROPY_SOURCE_PY}
    ${MICROPY_SOURCE_SHARED}
    ${MICROPY_SOURCE_EXTMOD}
    ${MICROPY_SOURCE_LIB}
    ${MICROPY_SOURCE_PORT}
    ${MICROPY_SOURCE_BOARD}
    ${MICROPY_SOURCE_USERMOD}
)

set(IDF_COMPONENTS
    app_update
    bootloader_support
	#bt
    esp_timer
	driver
    esp_driver_gpio
    esp_driver_uart
	esp_driver_i2s
	esp_driver_i2c
	esp_driver_spi
	esp_adc
    esp_common
    esp_eth
    esp_event
    esp_partition
    esp_app_format
    esp_psram
    esp_rom
    esp_ringbuf
    esp_wifi
    freertos
    heap
    log
    lwip
    mbedtls
	#mdns
    newlib
    nvs_flash
    sdmmc
    soc
    spi_flash
	#tcpip_adapter
    ulp
    vfs
    xtensa
	esp_bootloader_format
)

if(IDF_VERSION_MINOR GREATER_EQUAL 1 OR IDF_VERSION_MAJOR GREATER_EQUAL 5)
    list(APPEND IDF_COMPONENTS esp_netif)
endif()

if(IDF_VERSION_MINOR GREATER_EQUAL 2 OR IDF_VERSION_MAJOR GREATER_EQUAL 5)
    list(APPEND IDF_COMPONENTS esp_system)
    list(APPEND IDF_COMPONENTS esp_timer)
endif()

if(IDF_VERSION_MINOR GREATER_EQUAL 3 OR IDF_VERSION_MAJOR GREATER_EQUAL 5)
    list(APPEND IDF_COMPONENTS esp_hw_support)
    list(APPEND IDF_COMPONENTS esp_pm)
    list(APPEND IDF_COMPONENTS hal)
endif()

if(IDF_TARGET STREQUAL "esp32")
    list(APPEND IDF_COMPONENTS esp32)
elseif(IDF_TARGET STREQUAL "esp32c3")
    list(APPEND IDF_COMPONENTS esp32c3)
    list(APPEND IDF_COMPONENTS riscv)
elseif(IDF_TARGET STREQUAL "esp32s2")
    list(APPEND IDF_COMPONENTS esp32s2)
    list(APPEND IDF_COMPONENTS tinyusb)
elseif(IDF_TARGET STREQUAL "esp32s3")
	#    list(APPEND IDF_COMPONENTS esp32s3)
	#    list(APPEND IDF_COMPONENTS tinyusb)
endif()

if(NOT MICROPY_SOURCE_BOARD)
	set(MICROPY_SOURCE_BOARD "")
endif()

# Register the main IDF component.
idf_component_register(
    SRCS
        ${MICROPY_SOURCE_PY}
        ${MICROPY_SOURCE_EXTMOD}
        ${MICROPY_SOURCE_SHARED}
        ${MICROPY_SOURCE_LIB}
        ${MICROPY_SOURCE_DRIVERS}
	    ${MICROPY_SOURCE_PORT}
	    ${MICROPY_SOURCE_BOARD}
		"micropython.c"
    INCLUDE_DIRS
		${MICROPY_COMPONENT_DIR}
    	${MICROPY_DIR}
        ${MICROPY_INC_CORE}
        ${MICROPY_INC_USERMOD}
	    ${MICROPY_PORT_DIR}
		${MICROPY_BOARD_DIR}
        ${CMAKE_BINARY_DIR}
		#    ${MICROPY_LIB_BERKELEY_DIR}/include
    REQUIRES
        ${IDF_COMPONENTS}
)

# Set the MicroPython target as the current (main) IDF component target.
set(MICROPY_TARGET ${COMPONENT_TARGET})

# Define mpy-cross flags, for use with frozen code.
set(MICROPY_CROSS_FLAGS -march=xtensawin)

# Set compile options for this port.
target_compile_definitions(${MICROPY_TARGET} PUBLIC
    ${MICROPY_DEF_CORE}
    FFCONF_H=\"${MICROPY_OOFATFS_DIR}/ffconf.h\"
	MICROPY_BOARD_DIR=${MICROPY_BOARD_DIR}
)


# Disable some warnings to keep the build outpEut clean.
target_compile_options(${MICROPY_TARGET} PUBLIC
    -Wno-clobbered
    -Wno-deprecated-declarations
    -Wno-missing-field-initializers
	-Wno-dangling-pointer
	-I${MICROPY_COMPONENT_DIR}
)

# Add additional extmod and usermod components.
target_link_libraries(${MICROPY_TARGET} micropy_extmod_btree)
target_link_libraries(${MICROPY_TARGET} usermod)


# Collect all of the include directories and compile definitions for the IDF components.
foreach(comp ${IDF_COMPONENTS})
    micropy_gather_target_properties(__idf_${comp})
endforeach()

if(IDF_VERSION_MINOR GREATER_EQUAL 2 OR IDF_VERSION_MAJOR GREATER_EQUAL 5)
    # These paths cannot currently be found by the IDF_COMPONENTS search loop above,
    # so add them explicitly.
    list(APPEND MICROPY_CPP_INC_EXTRA ${IDF_PATH}/components/soc/soc/${IDF_TARGET}/include)
    list(APPEND MICROPY_CPP_INC_EXTRA ${IDF_PATH}/components/soc/soc/include)
    if(IDF_VERSION_MINOR GREATER_EQUAL 3)
        list(APPEND MICROPY_CPP_INC_EXTRA ${IDF_PATH}/components/tinyusb/additions/include)
        list(APPEND MICROPY_CPP_INC_EXTRA ${IDF_PATH}/components/tinyusb/tinyusb/src)
    endif()
endif()

# Include the main MicroPython cmake rules.
include(${MICROPY_DIR}/py/mkrules.cmake)
