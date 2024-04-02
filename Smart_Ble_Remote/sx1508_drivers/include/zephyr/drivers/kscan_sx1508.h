/*
 * Copyright (c) 2019 Henrik Brix Andersen <henrik@brixandersen.dk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_INCLUDE_DRIVERS_KEYPAD_SX1508_H_
#define ZEPHYR_INCLUDE_DRIVERS_KEYPAD_SX1508_H_

#include <zephyr/drivers/kscan.h>

/**
 * Register a SX1508 keyscan device to be notified of relevant
 * keyscan events by the keyscan interrupt thread in the SX1508
 * parent driver.
 *
 * @param parent SX1508 parent device.
 * @param callback Keyscan callback function.
 * @return 0 if successful, negative errno code on failure.
 */
static int sx1508_register_keyscan_callback(const struct device *parent,
				                            kscan_callback_t callback);

#endif /* ZEPHYR_INCLUDE_DRIVERS_KEYPAD_SX1508 */
