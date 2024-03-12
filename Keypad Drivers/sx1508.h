/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_SX1509B_H_
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_SX1509B_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>



/* callback function */
typedef int (*button_user_cb_t)(struct device* dev,int key_rx);

__subsystem struct callback_driver_api {
	button_user_cb_t usr_cb;
};

/**
 * @brief Add an application callback.
 * @param port Pointer to the device structure for the driver instance.
 * @param callback A valid Application's callback structure pointer.
 * @return 0 if successful, negative errno code on failure.
 *
 * @note Callbacks may be added to the device from within a callback
 * handler invocation, but whether they are invoked for the current
 * GPIO event is not specified.
 *
 * Note: enables to add as many callback as needed on the same port.
 */
static inline int sx1509_add_callback(const struct device *port,
				    button_user_cb_t *callback)
{
	const struct callback_driver_api *api =
		(const struct callback_driver_api *)port->api;

	if (api->usr_cb == NULL) {
		return -ENOTSUP;
	}

	return api->usr_cb(port, callback);
}


#endif /* ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_SX1509B_H_ */