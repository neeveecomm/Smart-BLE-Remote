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
 * keyscan events by the keyscan interrupt thread in the HT16K33
 * parent driver.
 *
 * @param parent SX1508 parent device.
 * @param child SX1508 child device.
 * @param callback Keyscan callback function.
 * @return 0 if successful, negative errno code on failure.
 */
#define REG_DIR 0x07
#define REG_DIR_DATA 0xF0
#define REG_OPENDRAIN 0x05
#define REG_OPENDRAIN_DATA 0x0F
#define REG_PULLUP 0x03
#define REG_PULLUP_DATA 0xFF
#define REG_PULLDOWN 0x04
#define REG_PULLDOWN_DATA 0x00
#define REG_DEBOUNCE_ENABLE 0x13
#define REG_DEBOUNCE_ENABLE_DATA 0xF0
#define REG_DEBOUNCE_CONFIG 0x12
#define REG_DEBOUNCE_CONFIG_DATA 0x05
#define REG_KEY_CONFIG 0x14
#define REG_KEY_CONFIG_DATA 0x7D
#define REG_CLOCK 0x0F
#define REG_CLOCK_DATA 0x40
#define REG_DATA 0x08
#define REG_DATA_WRITE 0x03
#define REG_INTERRUPT_MASK 0x09
#define REG_INTERRUPT_MASK_ENABLE 0x0F
#define REG_SENSE 0x0A
#define REG_SENSE_FALLING_EDGE_TRIG 0x00
#define REG_KEY_DATA 0x15
#define REG_INTERRUPT_SOURCE 0x0C
#define REG_INTERRUPT_CLEAR_FLAG 0xFF
#define REG_EVENT_STATUS 0x0D
#define REG_EVENT_CLEAR_FLAG 0xFF

int sx1508_register_keyscan_callback(const struct device *parent,
				      const struct device *child,
				      kscan_callback_t callback);

#endif /* ZEPHYR_INCLUDE_DRIVERS_KEYPAD_SX1508_H_ */
