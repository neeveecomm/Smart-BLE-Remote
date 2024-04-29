/* NXP Microelectronics FXLS8974CF 3-axis accelerometer driver
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT nxp_fxls8974cf


#include <zephyr/init.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/drivers/i2c.h>

#include "fxls8974cf.h"

#define ACCEL_SCALE(sensitivity) (sensitivity)

LOG_MODULE_REGISTER(FXLS8974CF, CONFIG_SENSOR_LOG_LEVEL);

static uint32_t fxls8974cf_reg_val_to_scale[] = {
	ACCEL_SCALE(980),
	ACCEL_SCALE(1950),
	ACCEL_SCALE(3910),
	ACCEL_SCALE(7810),
};

static uint8_t sensor_comm_write(const struct i2c_dt_spec *dev_i2c, uint16_t offset, uint16_t size,
				 uint8_t *pWritebuffer)
{
	uint8_t buf[size + 1];

	buf[0] = (uint8_t)(offset);
	memcpy(buf + 1, pWritebuffer, size);

	return i2c_write_dt(dev_i2c, buf, sizeof(buf));
}

static uint8_t sensor_comm_read(const struct i2c_dt_spec *dev_i2c, uint16_t offset, uint16_t size,
				uint8_t *pReadbuffer)
{
	return i2c_write_read_dt(dev_i2c, &offset, 1, pReadbuffer, size);
}

static inline void fxls8974cf_convert(struct sensor_value *val, int16_t raw_val, uint32_t scale)
{

	int32_t converted_val;

	converted_val = raw_val * scale;
	val->val1 = converted_val / 1000000;
	val->val2 = converted_val % 1000000;

}

static int fxls8974cf_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct fxls8974cf_data *data = dev->data;

	data->selected_range = 0;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		fxls8974cf_convert(val, data->bufx[data->selected_range], data->scale);
		data->selected_range++;
		break;
	case SENSOR_CHAN_ACCEL_Y:
		fxls8974cf_convert(val, data->bufy[data->selected_range], data->scale);
		data->selected_range++;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		fxls8974cf_convert(val, data->bufz[data->selected_range], data->scale);
		data->selected_range++;
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		fxls8974cf_convert(val++, data->bufx[data->selected_range], data->scale);
		fxls8974cf_convert(val++, data->bufy[data->selected_range], data->scale);
		fxls8974cf_convert(val, data->bufz[data->selected_range], data->scale);
		data->selected_range++;
		break;
	default:
		LOG_DBG("Channel not supported");
		return -ENOTSUP;
		break;
	}
	return 0;
}


uint8_t set_mode(struct fxls8974cf_device_config *fxls8974cf, fxls8974cf_mode_t mode,
		 uint8_t *ctrl1)
{

	uint8_t ctrlReg1;
	uint8_t status = SENSOR_SUCCESS;
	// Get the current control register value;

	status = sensor_comm_read(&fxls8974cf->i2c, FXLS8974CF_SENS_CONFIG1, 1, &ctrlReg1);

	if (status != SENSOR_SUCCESS) {

		return status;
	}

	ctrlReg1 = (ctrlReg1 & (~FXLS8974CF_SENS_CONFIG1_ACTIVE_MASK)) | mode;

	// Write it back control data with desired mode.
	status = sensor_comm_write(&fxls8974cf->i2c, FXLS8974CF_SENS_CONFIG1, 1, &ctrlReg1);
	// return the current control register value.
	if (NULL != ctrl1) {
		*ctrl1 = ctrlReg1;
	}

	return status;
}

static int fxls8974cf_read_accel_raw(struct fxls8974cf_device_config *fxls8974cf, uint8_t nRead,
				     fxls8974cf_accel_raw_t *pAccelData)
{

	if (NULL == fxls8974cf) {
		return SENSOR_INVALIDPARAM_ERR;
	}
	uint8_t status;
	// Read the sensor data.
	status = set_mode(fxls8974cf, ACTIVE, NULL);

	status = sensor_comm_read(&fxls8974cf->i2c, FXLS8974CF_OUT_X_LSB, nRead,
				  (uint8_t *)pAccelData);
	// Return the status.
	return status;
}

static int fxls8974cf_configure_accel(struct fxls8974cf_device_config *pDriver,
				      fxls8974cf_odr_t odr, fxls8974cf_accel_config_t *pConfig)
{

	if (NULL == pDriver) {
		return SENSOR_INVALIDPARAM_ERR;
	}
	uint8_t status = SENSOR_SUCCESS;
	FXLS8974CF_SENS_CONFIG1_t ctrlReg1 = {0};
	FXLS8974CF_SENS_CONFIG2_t ctrlReg2 = {0};
	FXLS8974CF_SENS_CONFIG3_t odrReg = {0};
	// Read the CRTL1 and preserve the existing configuration bits of the control registers
	// other than standby bit
	status = set_mode(pDriver, STANDBY, &ctrlReg1.w);

	if (status != SENSOR_SUCCESS) {
		return status;
	}
	// Configure the ODR
	odrReg.w = odr;

	// update the odr
	status = sensor_comm_write(&pDriver->i2c, FXLS8974CF_SENS_CONFIG3, 1, &odrReg.w);

	if (NULL != pConfig) {

		// Configure the resolution and Filter enable or disable.
		ctrlReg1.b.fsr = pConfig->control.range;
		// updating the fast mode for 8-bit data read
		ctrlReg2.b.f_read = pConfig->control.fastmode;
		status = sensor_comm_write(&pDriver->i2c, FXLS8974CF_SENS_CONFIG2, 1, &ctrlReg2.w);
		// Configure the fifo
		status = sensor_comm_write(&pDriver->i2c, FXLS8974CF_BUF_CONFIG1, 1,
					   &pConfig->fifosetup.config1.w);
		status = sensor_comm_write(&pDriver->i2c, FXLS8974CF_BUF_CONFIG2, 1,
					   &pConfig->fifosetup.config2.w);
	}
	// Update the ctr1 with active bit set.
	ctrlReg1.b.active = ACTIVE;
	status = sensor_comm_write(&pDriver->i2c, FXLS8974CF_SENS_CONFIG1, 1, &ctrlReg1.w);

	return status;
}


static int fxls8974cf_sample_fetch(const struct device *dev, enum sensor_channel chan)
{

	fxls8974cf_accel_raw_t accel_data;

	uint8_t samples_count = 0;

	struct fxls8974cf_data *data = dev->data;

	struct fxls8974cf_device_config *cfg = dev->config;

	/* fetch raw data sample */

	fxls8974cf_read_accel_raw(cfg, ACCEL_SAMPLE_SIZE, &accel_data);

	/*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
	data->bufx[samples_count] =
		((int16_t)accel_data.accel[1] << 8) | (int16_t)accel_data.accel[0];
	data->bufy[samples_count] =
		((int16_t)accel_data.accel[3] << 8) | (int16_t)accel_data.accel[2];
	data->bufz[samples_count] =
		((int16_t)accel_data.accel[5] << 8) | (int16_t)accel_data.accel[4];

	return 0;
}

static const struct sensor_driver_api fxls8974cf_driver_api = {
	.sample_fetch = fxls8974cf_sample_fetch,
	.channel_get = fxls8974cf_channel_get,
};

static int fxls8974cf_init(const struct device *dev)
{

	struct fxls8974cf_data *drv_data = dev->data;
	const struct fxls8974cf_device_config *cfg = dev->config;
	int status;
	uint8_t id;

	drv_data->dev = dev;

	if (!device_is_ready(cfg->i2c.bus)) {
		printk("I2C bus %s is not ready! \n\r", cfg->i2c.bus->name);
	}


	if (fxls8974cf_i2c_init(dev))
	{
        printk("fxls8974cf_i2c_init  %s is not ready\n", dev->name);
	}

    status = drv_data->hw_tf->read_reg(dev,FXLS8974CF_WHO_AM_I,&id);
		if (status < 0) {
		LOG_ERR("Failed to read chip id %d wai %d.\n",status,id);
		return status;
	}

	/*! Initialize sensor self-test metadata. */

	fxls8974cf_configure_accel(cfg, FXLS8974CF_WAKE_ODR_100HZ, NULL);

	/* set full scale range and store it for later conversion */
	drv_data->scale = fxls8974cf_reg_val_to_scale[FXLS8974CF_FS_IDX];

	status = drv_data->hw_tf->write_reg(dev, FXLS8974CF_SENS_CONFIG1, FXLS8974CF_FS_MASK | FXLS8974CF_HR_BIT );

	return 0;
}

#define FXLS8974CF_I2C(inst) (.i2c = I2C_DT_SPEC_INST_GET(inst),)

#define FXLS8974CF_DEFINE(inst)                                                                    	\
	static struct fxls8974cf_data fxls8974cf_data_##inst;                                      		\
                                                                                                   	\
	static const struct fxls8974cf_device_config fxls8974cf_device_config_##inst = {           		\
		COND_CODE_1(DT_INST_ON_BUS(inst, i2c), FXLS8974CF_I2C(inst), ()) IF_ENABLED(       			\
			CONFIG_FXLS8974CF_TRIGGER,                                                 				\
			(.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, {0}), ))};         				\
                                                                                                   	\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, fxls8974cf_init, NULL, &fxls8974cf_data_##inst,         		\
				     &fxls8974cf_device_config_##inst, POST_KERNEL,                					\
				     CONFIG_SENSOR_INIT_PRIORITY, &fxls8974cf_driver_api);

DT_INST_FOREACH_STATUS_OKAY(FXLS8974CF_DEFINE)
