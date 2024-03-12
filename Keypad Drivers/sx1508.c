/*
 * Copyright (c) 2018 Aapo Vienamo
 * Copyright (c) 2018 Peter Bigot Consulting, LLC
 * Copyright (c) 2019-2020 Nordic Semiconductor ASA
 * Copyright (c) 2020 ZedBlox Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT semtech_sx1509b

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_sx1509b.h>
#include <zephyr/dt-bindings/gpio/semtech-sx1509b.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sx1509b, CONFIG_GPIO_LOG_LEVEL);

#include <zephyr/drivers/gpio/gpio_utils.h>

#define RESET_DELAY_MS 2.5

/** Runtime driver data */
struct sx1509b_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;

	struct k_sem lock;

#ifdef CONFIG_GPIO_SX1509B_INTERRUPT

	struct gpio_callback gpio_cb;

	button_user_cb_t usr_cb;

	struct k_work work;

	const struct device *dev;

#endif /* CONFIG_GPIO_SX1509B_INTERRUPT */
};

/** Configuration data */
struct sx1509b_config {

	struct i2c_dt_spec bus;

#ifdef CONFIG_GPIO_SX1509B_INTERRUPT

	struct gpio_dt_spec nint_gpio;

#endif /* CONFIG_GPIO_SX1509B_INTERRUPT */
};

/* General configuration register addresses */
enum {
	/* TODO: Add rest of the regs */
	SX1509B_REG_CLOCK = 0x0F,

	SX1509B_REG_RESET = 0x7D,
};

/* Magic values for softreset */
enum {

	SX1509B_REG_RESET_MAGIC0 = 0x12,

	SX1509B_REG_RESET_MAGIC1 = 0x34,
};

/* Register bits for SX1509B_REG_CLOCK */
enum {
	SX1509B_REG_CLOCK_FOSC_OFF = 0 << 5,

	SX1509B_REG_CLOCK_FOSC_EXT = 1 << 5,

	SX1509B_REG_CLOCK_FOSC_INT_2MHZ = 2 << 5,
};

/* Register bits for SX1509B_REG_MISC */
enum {
	SX1509B_REG_MISC_LOG_A = 1 << 3,
	SX1509B_REG_MISC_LOG_B = 1 << 7,
	/* ClkX = fOSC */
	SX1509B_REG_MISC_FREQ = 1 << 4,
};

/* Pin configuration register addresses */
enum {
	SX1509B_REG_INPUT_DISABLE = 0x00,
	SX1509B_REG_PULL_UP = 0x03,
	SX1509B_REG_PULLDOWN_DATA = 0X00,
	SX1509B_REG_PULLUP_DATA = 0xFF,
	SX1509B_REG_PULL_DOWN = 0x04,
	SX1509B_REG_OPEN_DRAIN = 0x05,
	SX1509B_REG_DIR = 0x07,
	SX1509B_REG_DIR_DATA = 0xF0,
	SX1509B_REG_OPENDRAIN = 0x05,
	SX1509B_REG_OPENDRAIN_DATA = 0x0F,
	SX1509B_REG_KEY_CONFIG = 0X14,
	SX1509B_REG_KEY_CONFIG_DATA = 0X7D,
	SX1509B_REG_CLOCK_DATA = 0x40,
	SX1509B_REG_DATA = 0x08,
	SX1509B_REG_DATA_WRITE = 0x03,
	SX1509B_REG_INTERRUPT_MASK = 0x09,
	SX1509B_REG_INTERRUPT_MASK_ENABLE = 0x0F,
	SX1509B_REG_INTERRUPT_SENSE = 0x0A,
	SX1509B_REG_SENSE = 0x0A,
	SX1509B_REG_SENSE_FALLING_EDGE_TRIG = 0X00,
	SX1509B_REG_KEY_DATA = 0x15,
	SX1509B_REG_INTERRUPT_SENSE_B = 0x14,
	SX1509B_REG_INTERRUPT_SENSE_A = 0x16,
	SX1509B_REG_INTERRUPT_SOURCE = 0x0C,
	SX1509B_REG_INTERRUPT_CLEAR_FLAG = 0xFF,
	SX1509B_REG_MISC = 0x10,
	SX1509B_REG_LED_DRV_ENABLE = 0x11,
	SX1509B_REG_DEBOUNCE_CONFIG = 0x12,
	SX1509B_REG_DEBOUNCE_CONFIG_DATA = 0X05,
	SX1509B_REG_DEBOUNCE_ENABLE = 0x13,
	SX1509B_REG_DEBOUNCE_ENABLE_DATA = 0XFF,
};

/* Edge sensitivity types */
enum {
	SX1509B_EDGE_NONE = 0x00,
	SX1509B_EDGE_RISING = 0x01,
	SX1509B_EDGE_FALLING = 0x02,
	SX1509B_EDGE_BOTH = 0x03,
};

/**
 * @brief Write a big-endian byte to an internal address of an I2C slave.
 *
 * @param bus Pointer to the I2C bus spec.
 * @param reg_addr Address of the internal register being written.
 * @param value Value to be written to internal register.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */

#ifdef CONFIG_GPIO_SX1509B_INTERRUPT
static int sx1509b_handle_interrupt(const struct device *dev)
{

	const struct sx1509b_config *cfg = dev->config;
	struct sx1509b_drv_data *drv_data = dev->data;
	int ret = 0;
	uint8_t int_source;
	uint8_t key_data = SX1509B_REG_KEY_DATA;
	uint8_t key_data_rx;
	

	k_sem_take(&drv_data->lock, K_FOREVER);
	

	ret = i2c_write_read_dt(&cfg->bus, &key_data, 1, &key_data_rx, 1);

	if (ret != 0) {
		LOG_ERR("error keydata\n");
		goto out;
	}

	uint8_t buf[2];
	buf[0] = SX1509B_REG_INTERRUPT_SOURCE;
	buf[1] = SX1509B_REG_INTERRUPT_CLEAR_FLAG;

	ret = i2c_write_dt(&cfg->bus, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("i2c device not ready interrupt clear\n");
	}

out:
	k_sem_give(&drv_data->lock);
	if (ret == 0) {

		drv_data->usr_cb(dev,key_data_rx);

	}

	return ret;
}

static void sx1509b_work_handler(struct k_work *work)
{

	struct sx1509b_drv_data *drv_data = CONTAINER_OF(work, struct sx1509b_drv_data, work);

	sx1509b_handle_interrupt(drv_data->dev);
	
}

static void sx1509_int_cb(const struct device *dev, struct gpio_callback *gpio_cb, uint32_t pins)
{

	struct sx1509b_drv_data *drv_data = CONTAINER_OF(gpio_cb, struct sx1509b_drv_data, gpio_cb);

	ARG_UNUSED(pins);


	k_work_submit(&drv_data->work);
}
#endif

/**
 * @brief Initialization function of SX1509B
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int sx1509b_init(const struct device *dev)
{
	const struct sx1509b_config *cfg = dev->config;
	struct sx1509b_drv_data *drv_data = dev->data;
	int rc;
	uint8_t ret;

	if (!device_is_ready(cfg->bus.bus)) {

		LOG_ERR("I2C bus not ready");

		rc = -ENODEV;

		goto out;
	}

#ifdef CONFIG_GPIO_SX1509B_INTERRUPT

	drv_data->dev = dev;

	if (!gpio_is_ready_dt(&cfg->nint_gpio)) {

		LOG_ERR("Interrupt not ready");

		rc = -ENODEV;
		goto out;
	}

	k_work_init(&drv_data->work, sx1509b_work_handler);

	gpio_pin_configure_dt(&cfg->nint_gpio, GPIO_INPUT);

	gpio_pin_interrupt_configure_dt(&cfg->nint_gpio, GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&drv_data->gpio_cb, sx1509_int_cb, BIT(cfg->nint_gpio.pin));

	gpio_add_callback(cfg->nint_gpio.port, &drv_data->gpio_cb);

#endif

	rc = i2c_reg_write_byte_dt(&cfg->bus, SX1509B_REG_RESET, SX1509B_REG_RESET_MAGIC0); // not
	if (rc != 0) {
		LOG_ERR("%s: reset m0 failed: %d\n", dev->name, rc);
		goto out;
	}

	rc = i2c_reg_write_byte_dt(&cfg->bus, SX1509B_REG_RESET, SX1509B_REG_RESET_MAGIC1); //
	if (rc != 0) {
		LOG_ERR("%s: reset m1 failed: %d\n", dev->name, rc);
		goto out;
	}

	k_sleep(K_MSEC(RESET_DELAY_MS));

	uint8_t buf[2];

	/*Configures direction for each IO. output => 0 , Input => 1 */
	buf[0] = SX1509B_REG_DIR;
	buf[1] = SX1509B_REG_DIR_DATA;
	ret = i2c_write_dt(&cfg->bus, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("the i2c device not ready dir_data \n");
	}

	/*Enables open drain operation for  IO0 to I03*/
	buf[0] = SX1509B_REG_OPENDRAIN;
	buf[1] = SX1509B_REG_OPENDRAIN_DATA;

	ret = i2c_write_dt(&cfg->bus, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("the i2c device not ready\n");
	}

	/*Enables the pull-down for each  IO*/
	buf[0] = SX1509B_REG_PULL_UP;
	buf[1] = SX1509B_REG_PULLUP_DATA;
	ret = i2c_write_dt(&cfg->bus, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("the i2c device not ready\n");
	}

	/*Enables debouncing for each [input-configured] IO*/
	buf[0] = SX1509B_REG_DEBOUNCE_ENABLE;
	buf[1] = SX1509B_REG_DEBOUNCE_ENABLE_DATA;

	ret = i2c_write_dt(&cfg->bus, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("the i2c device not ready\n");
	}

	/*Configure debouncing Time*/
	buf[0] = SX1509B_REG_DEBOUNCE_CONFIG;
	buf[1] = SX1509B_REG_DEBOUNCE_CONFIG_DATA;
	ret = i2c_write_dt(&cfg->bus, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("the i2c device not ready\n");
	};

	/*Configure number of keypad ROWs and column*/
	buf[0] = SX1509B_REG_KEY_CONFIG;
	buf[1] = SX1509B_REG_KEY_CONFIG_DATA;
	ret = i2c_write_dt(&cfg->bus, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("the i2c device not ready\n");
	}

	/*Configure Oscillator frequency (fOSC) source*/
	buf[0] = SX1509B_REG_CLOCK;
	buf[1] = SX1509B_REG_CLOCK_DATA;
	ret = i2c_write_dt(&cfg->bus, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("the i2c device not ready\n");
	}

	/*Configures which [input-configured] IO will trigger an interrupt on NINT pin*/
	buf[0] = SX1509B_REG_INTERRUPT_MASK;
	buf[1] = SX1509B_REG_INTERRUPT_MASK_ENABLE;
	ret = i2c_write_dt(&cfg->bus, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("the i2c device not ready\n");
	}

	/*Configures EDGE SENSE*/
	buf[0] = SX1509B_REG_SENSE;
	buf[1] = SX1509B_REG_SENSE_FALLING_EDGE_TRIG;
	ret = i2c_write_dt(&cfg->bus, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("the i2c device not ready\n");
	}

out:
	if (rc != 0) {

		LOG_ERR("%s init failed: %d", dev->name, rc);
	} else {
		LOG_INF("%s init ok %d", dev->name, rc);
	}
	k_sem_give(&drv_data->lock);
	return rc;
}

static int gpio_sx1509b_manage_callback(const struct device *dev, button_user_cb_t *callback)
{
	struct sx1509b_drv_data *drv_data = dev->data;

	drv_data->usr_cb = callback;
	
}

static const struct callback_driver_api api_table = {

	.usr_cb = gpio_sx1509b_manage_callback,

};

#define GPIO_SX1509B_DEFINE(inst)                                                               \
	static const struct sx1509b_config sx1509b_cfg##inst = {                                   	\
                                                                                               	\
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 		\
		IF_ENABLED(CONFIG_GPIO_SX1509B_INTERRUPT,                                          		\
			   (GPIO_DT_SPEC_INST_GET(inst, nint_gpios)))};                            			\
                                                                                               	\
	static struct sx1509b_drv_data sx1509b_drvdata##inst = {                                    \
		.lock = Z_SEM_INITIALIZER(sx1509b_drvdata##inst.lock, 1, 1),                       		\
	};                                                                                          \
                                                                                                \
	DEVICE_DT_INST_DEFINE(inst, sx1509b_init, NULL, &sx1509b_drvdata##inst,                    	\
			      &sx1509b_cfg##inst, POST_KERNEL, CONFIG_GPIO_SX1509B_INIT_PRIORITY,  			\
			      &api_table);

DT_INST_FOREACH_STATUS_OKAY(GPIO_SX1509B_DEFINE)
