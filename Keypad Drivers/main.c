/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/kscan_sx1508.h>

#define LOG_LEVEL LOG_LEVEL_DBG         

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define I2C0_NODE DT_NODELABEL(sx1508)

static const struct i2c_dt_spec i2c_node = I2C_DT_SPEC_GET(I2C0_NODE);

static struct device *dev = DEVICE_DT_GET(DT_NODELABEL(sx1508));

#define key_press_detect DT_NODELABEL(sx1508)

static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(key_press_detect, nint_gpios);



void keyscan_callback(const struct device *dev, uint32_t row,
					  uint32_t column, bool pressed)
{
	printk("button pressed\n");

	printk("row %d, col %d, pressed %d\n",row ,column, pressed);
}

int main()
{
	int err;

	if (!device_is_ready(i2c_node.bus))
	{
		printk("I2C bus %s is not ready! \n\r", i2c_node.bus->name);
	}

	if (!device_is_ready(irq.port))
	{
		printf("pin is not ready\n");
	}

     err = kscan_config(dev, keyscan_callback);
	 if (err)
	{
		printk("Failed to add keyscan callback err %d\n", err);
	}

}
