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
// #include <zephyr/drivers/kscan_sx1508.h>

#define LOG_LEVEL LOG_LEVEL_DBG

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

// acclerometer
static const struct i2c_dt_spec i2c_node = I2C_DT_SPEC_GET(DT_NODELABEL(fxls8974cf));

static struct device *dev = DEVICE_DT_GET(DT_NODELABEL(fxls8974cf));

// keypad
static const struct i2c_dt_spec i2c_node_2 = I2C_DT_SPEC_GET(DT_NODELABEL(sx1508));

static struct device *dev_2 = DEVICE_DT_GET(DT_NODELABEL(sx1508));

#define key_press_detect DT_NODELABEL(sx1508)

static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(key_press_detect, nint_gpios);

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

void keyscan_callback(const struct device *dev, uint32_t row,
					  uint32_t column, bool pressed)
{
	printk("row %d, col %d, pressed %d\n", row, column, pressed);
}

static void fetch_and_display(const struct device *sensor)
{
	struct sensor_value accel[3];
	int rc = sensor_sample_fetch(sensor);

	if (rc == 0)
	{
		rc = sensor_channel_get(sensor,
								SENSOR_CHAN_ACCEL_XYZ,
								accel);
	}
	if (rc < 0)
	{
		printk("ERROR: Update failed: %d\n", rc);
	}
	else
	{
		printk("X = %f, Y = %f, Z = %f\n",
			   sensor_value_to_double(&accel[0]),
			   sensor_value_to_double(&accel[1]),
			   sensor_value_to_double(&accel[2]));
	}

	printk("\n");
}

int main()
{
	int err;
	int button;

	if (!device_is_ready(irq.port))
	{
		printf("pin is not ready\n");
	}

	err = kscan_config(dev_2, keyscan_callback);
	if (err)
	{
		printk("Failed to add keyscan callback err %d\n", err);
	}


	while (true) {
		fetch_and_display(dev);

		k_sleep(K_MSEC(2000));

	}
}
