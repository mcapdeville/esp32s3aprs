# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS dmabuff SA8x8 tinyusb ssd1680 lvgl micropython)
set(MICROPY_USER_C_MODULES "../modules/esp32s3aprs/esp32s3aprs.cmake")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(IDF_COMPONENT_MANAGER 1)
idf_build_set_property(MINIMAL_BUILD 1)

project(esp32s3aprs)

#nvs_create_partition_image("nvs" "nvs.csv" FLASH_IN_PROJECT)
nvs_create_partition_image("nvs" "nvs.csv")
#spiffs_create_partition_image("spiffs" "spiffs" FLASH_IN_PROJECT)
spiffs_create_partition_image("spiffs" "spiffs")
