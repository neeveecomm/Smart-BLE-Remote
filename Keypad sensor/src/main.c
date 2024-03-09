/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/kscan_sx1508.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define I2C0_NODE DT_NODELABEL(sx1509b)


static void keyscan_callback(const struct device *dev, uint32_t row,
			     uint32_t column, bool pressed)
{
	printk("button pressed");
} 

int main(void)
{
       const struct device *const dev = DEVICE_DT_GET(KEY_NODE);
    
    if (!device_is_ready(key)) {
		LOG_ERR("Keyscan device not ready");
		return 0;
	}

    err = kscan_config(key, keyscan_callback);
	if (err) {
		LOG_ERR("Failed to add keyscan callback (err %d)", err);
	}

    return 0;
}