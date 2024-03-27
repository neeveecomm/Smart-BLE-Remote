/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#define LOG_LEVEL LOG_LEVEL_DBG

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define I2C0_NODE DT_NODELABEL(fxls8974)

static const struct i2c_dt_spec i2c_node = I2C_DT_SPEC_GET(I2C0_NODE);

static struct device *dev = DEVICE_DT_GET(DT_NODELABEL(fxls8974));

static void fetch_and_display(const struct device *sensor)
{
	static unsigned int count;
	struct sensor_value accel[3];  
	// const char *overrun = "";
	int rc = sensor_sample_fetch(sensor);

	++count;

	if (rc == 0) {
		rc = sensor_channel_get(sensor,
					SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc < 0) {
		printk("ERROR: Update failed: %d\n", rc);
	} else {
		printk("X = %f, Y = %f, Z = %f\n",
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]));
	}

		printk("\n");
	
}


int main(void)
{
	//  const struct device *sensor;
    
     printk("enter the program\n");

	// int err;

	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev->name);
		
	}


	if (!device_is_ready(i2c_node.bus))
	{
		printk("I2C bus %s is not ready! \n\r", i2c_node.bus->name);
	}



	while (true) {
		fetch_and_display(dev);

		
		// sample_fetch(dev,SENSOR_CHAN_ACCEL_XYZ);

		k_sleep(K_MSEC(2000));
	}

}
