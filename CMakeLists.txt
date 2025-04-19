# Copyright (c)  2024 LVGL <felipe@lvgl.io>
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(SHIELD x_nucleo_iks01a3 adafruit_2_8_tft_touch_v2)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(SquareLine_Project)

set(LVGL_DIR ${ZEPHYR_LVGL_MODULE_DIR})

# Ajout des dossiers d'include
target_include_directories(app PRIVATE src/rtc src/ui src/chrono src/BLE)

# Ajout des fichiers sources
FILE(GLOB app_sources 
    src/*.c 
    src/ui/*.c 
    src/ui/images/*.c 
    src/ui/fonts/*.c 
    src/ui/screens/*.c 
    src/ui/components/*.c 
    src/sensor/*.c 
    src/rtc/*.c
    src/chrono/*.c
    src/BLE/*.c

)
target_sources(app PRIVATE ${app_sources})
