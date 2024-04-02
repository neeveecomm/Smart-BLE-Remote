# ST Microelectronics IIS2DH 3-axis accelerometer driver
#
# Copyright (c) 2020 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(fxls8974cf.c)

zephyr_library_sources(fxls8974cf.h)

zephyr_library_sources(fxls8974cf_i2c.c)

zephyr_library_include_directories(../stmemsc)
