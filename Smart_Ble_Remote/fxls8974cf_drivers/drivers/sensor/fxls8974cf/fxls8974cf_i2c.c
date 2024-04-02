/* NXP Microelectronics FXLS8974CF 3-axis accelerometer driver
 *
 * Copyright (c) 2020 NXP Microelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT nxp_fxls8974cf

#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "fxls8974cf.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int fxls8974cf_i2c_read_data(const struct device *dev, uint8_t reg_addr,
				 uint8_t *value, uint8_t len)
{
	const struct fxls8974cf_device_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->i2c, reg_addr, value,
				 len);
}

static int fxls8974cf_i2c_write_data(const struct device *dev, uint8_t reg_addr,
				  uint8_t *value, uint8_t len)
{
	const struct fxls8974cf_device_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->i2c, reg_addr, value,
				  len);
}

static int fxls8974cf_i2c_read_reg(const struct device *dev, uint8_t reg_addr,
				uint8_t *value)
{
	const struct fxls8974cf_device_config *cfg = dev->config;

	return i2c_reg_read_byte_dt(&cfg->i2c, reg_addr, value);
}

static int fxls8974cf_i2c_write_reg(const struct device *dev, uint8_t reg_addr,
				uint8_t value)
{
	const struct fxls8974cf_device_config *cfg = dev->config;

	return i2c_reg_write_byte_dt(&cfg->i2c, reg_addr, value);
}

static int fxls8974cf_i2c_update_reg(const struct device *dev, uint8_t reg_addr,
				  uint8_t mask, uint8_t value)
{
	const struct fxls8974cf_device_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->i2c, reg_addr, mask, value);
}

static const struct fxls8974cf_transfer_function fxls8974cf_i2c_transfer_fn = {
	.read_data = fxls8974cf_i2c_read_data,
	.write_data = fxls8974cf_i2c_write_data,
	.read_reg  = fxls8974cf_i2c_read_reg,
	.write_reg  = fxls8974cf_i2c_write_reg,
	.update_reg = fxls8974cf_i2c_update_reg,
};

int fxls8974cf_i2c_init(const struct device *dev)
{
	struct fxls8974cf_data *data = dev->data;
	const struct fxls8974cf_device_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		printk("Bus device is not ready");
		return -ENODEV;
	}

	data->hw_tf = &fxls8974cf_i2c_transfer_fn;

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
