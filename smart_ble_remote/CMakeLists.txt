# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20)
# this need for custom shields
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

list(APPEND ZEPHYR_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# it should be located after find_package

project(smart_ble_remote)

set(app_sources src/main.c)
target_sources(app PRIVATE ${app_sources})