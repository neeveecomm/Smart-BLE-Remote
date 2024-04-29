# SPDX-License-Identifier: Apache-2.0


zephyr_library()

zephyr_library_sources_ifdef(CONFIG_KSCAN_SX1508	kscan_sx1508.c)
