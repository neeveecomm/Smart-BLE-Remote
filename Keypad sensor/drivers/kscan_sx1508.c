/*
 * Copyright (c) 2019 - 2021 Henrik Brix Andersen <henrik@brixandersen.dk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT semtech_sx1508_keyscan

/**
 * @file
 * @brief Keyscan driver for the SX1508 I2C KEYPAD driver
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/kscan_sx1508.h>
#include <zephyr/logging/log.h>

#define CONFIG_KSCAN_INIT_PRIORITY 90                         //

LOG_MODULE_REGISTER(kscan_sx1508, CONFIG_KSCAN_LOG_LEVEL);


struct kscan_sx1508_cfg {
	const struct device *parent;
};

static int kscan_sx1508_config(const struct device *dev,
				kscan_callback_t callback)
    {
        const struct kscan_sx1508_cfg *config = dev->config;

        return sx1508_register_keyscan_callback(config->parent, dev, callback);

    }

static int kscan_sx1508_init (const struct device *dev)
{
    const struct kscan_sx1508_cfg *config =dev->config;

    if (!device_is_ready(config->parent)) {
		LOG_ERR("sx1508 parent device not ready");
		return -EINVAL;
	}
    return 0;

}

static const struct kscan_driver_api kscan_ht16k33_api = {
	.config = kscan_ht16k33_config,
};


#define KSCAN_SX1508_DEVICE(id)                            \
  static const struct kscan_sx1508_cfg kscan_sx1508_##id##_cfg ={        \
        .parent = DEVICE_DT_GET(DT_INST_BUS(id)),
  };                                           \
                                                \
  DEVICE_DT_INST_DEFINE(id,&kscan_sx1508_init,                        \
                                    NULL,NULL,                        \
                                    &kscan_sx1508_##id##_cfg,POST_KERNEL,            \
                                    CONFIG_KSCAN_INIT_PRIORITY,                      \
                                    &kscan_driver_api);                       \

DT_INST_FOREACH_STATUS_OKAY(KSCAN_HT16K33_DEVICE)
