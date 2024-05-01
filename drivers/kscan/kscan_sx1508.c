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
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#define REG_DIR                     0x07
#define REG_DIR_DATA                0xF0
#define REG_OPENDRAIN               0x05
#define REG_OPENDRAIN_DATA          0x0F
#define REG_PULLUP                  0x03
#define REG_PULLUP_DATA             0xFF
#define REG_PULLDOWN                0x04
#define REG_PULLDOWN_DATA           0x00
#define REG_DEBOUNCE_ENABLE         0x13
#define REG_DEBOUNCE_ENABLE_DATA    0xF0
#define REG_DEBOUNCE_CONFIG         0x12
#define REG_DEBOUNCE_CONFIG_DATA    0x05
#define REG_KEY_CONFIG              0x14
#define REG_KEY_CONFIG_DATA         0x7D
#define REG_CLOCK                   0x0F
#define REG_CLOCK_DATA              0x40
#define REG_DATA                    0x08
#define REG_DATA_WRITE              0x03
#define REG_INTERRUPT_MASK          0x09
#define REG_INTERRUPT_MASK_ENABLE   0x0F
#define REG_SENSE_HIGH              0x0A
#define REG_SENSE_LOW				0X0B
#define REG_SENSE_FALLING_EDGE_TRIG 0x00
#define REG_SENSE_FALLING_EDGE_TRIG_1 0x00
#define REG_KEY_DATA                0x15
#define REG_INTERRUPT_SOURCE        0x0C
#define REG_INTERRUPT_CLEAR_FLAG    0xFF
#define REG_EVENT_STATUS            0x0D
#define REG_EVENT_CLEAR_FLAG        0xFF
#define REG_RESET                   0x7D
#define REG_RESET_MAGIC0            0x12
#define REG_RESET_MAGIC1            0X34
#define SX1508_KEYSCAN_ROWS          4
#define SX1508_KEYSCAN_COLS          4
#define SX1508_KEYSCAN_DATA_SIZE     6

#define KEYSCAN_DEBOUNCE_MSEC       25

#define SX1508_DISP_DATA_SIZE        6 

#define SX1508_KEYSCAN_IRQ_THREAD_STACK_SIZE 400
#define SX1508_KEYSCAN_IRQ_THREAD_PRIO       2

LOG_MODULE_REGISTER(kscan_sx1508, CONFIG_KSCAN_LOG_LEVEL);

struct kscan_sx1508_cfg {
	struct i2c_dt_spec i2c;
	bool irq_enabled;
#ifdef CONFIG_KSCAN_SX1508
	struct gpio_dt_spec irq;
#endif /* CONFIG_SX1508_KEYSCAN */
};

struct kscan_sx1508_data {
	const struct device *dev;
	/* Shadow buffer for the display data RAM */
	uint8_t buffer[SX1508_DISP_DATA_SIZE];
#ifdef CONFIG_KSCAN_SX1508
	struct k_mutex lock;
	const struct device *parent;
	kscan_callback_t kscan_cb;
	struct gpio_callback irq_cb;
	struct k_thread irq_thread;
	struct k_sem irq_sem;
	struct k_timer timer;
	uint16_t key_state[SX1508_KEYSCAN_ROWS];

	K_KERNEL_STACK_MEMBER(irq_thread_stack,CONFIG_SX1508_KEYSCAN_IRQ_THREAD_STACK_SIZE);  //
#endif /* CONFIG_SX1508_KEYSCAN */
};

void find_zero_indexes(uint8_t val, int *row, int *column)
{
	// Initialize row and column indexes
	*row = -1;
	*column = -1;

	// Iterate through each bit of the byte from left to right
	for (int i = 7; i >= 0; i--) {
		// Check if the bit at position i is zero
		if ((val & (1 << i)) == 0) {
			// If row is not set, set it to the current index
			if (*row == -1) {
				*row = i;
			} else {
				// If column is not set, set it to the current index
				if (*column == -1) {
					*column = i;
					return; // Found both row and column indexes, return
				}
			}
		}
	}
}
	
#ifdef CONFIG_KSCAN_SX1508
static bool sx1508_process_keyscan_data(const struct device *dev)
{

	const struct kscan_sx1508_cfg *config = dev->config;
	struct kscan_sx1508_data *data = dev->data;
	uint8_t key;
	bool pressed = false;
	uint8_t buf[2];
	uint16_t state;
    uint16_t changed;
	int row;
	int col;
	int err;

	changed=gpio_pin_get_dt(&config->irq);

      
	uint8_t key_data = REG_KEY_DATA;
	err = i2c_write_read_dt(&config->i2c, &key_data, 1, &key, sizeof(key));
	if (err) {
		LOG_WRN("Failed to to read SX1508 key data (err %d)", err);
		/* Reprocess */
		return true;
	}

	buf[0] = REG_INTERRUPT_SOURCE;
	buf[1] = REG_INTERRUPT_CLEAR_FLAG;
          
	err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
	if (err != 0) {
		LOG_ERR("the i2c device not ready \n");
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	{
		find_zero_indexes(key, &row, &col);
		if ((row) && (col)) {
			pressed = false;
		}

		if (data->kscan_cb == NULL) {
			goto out;
		}

		data->kscan_cb(data->parent, row, col,changed);

     pressed =false;
    
	out:
		k_mutex_unlock(&data->lock);

		return changed;
	}


}


static void sx1508_irq_thread(struct kscan_sx1508_data *data)
{
	const struct kscan_sx1508_cfg *config;
	uint8_t buf[2];
	int err;
	bool pressed;

	while (true) {
		k_sem_take(&data->irq_sem, K_FOREVER);
		do {
			k_sem_reset(&data->irq_sem);
			pressed = sx1508_process_keyscan_data(data->dev);
			k_msleep(KEYSCAN_DEBOUNCE_MSEC);
		} while (pressed);
	}
}

struct kscan_sx1508_data *data_cb = NULL;
	const struct kscan_sx1508_cfg *config_cb = NULL;
static void sx1508_irq_callback(const struct device *gpiob, struct gpio_callback *cb, uint32_t pins)
{
	struct kscan_sx1508_data *data;

	ARG_UNUSED(gpiob);
	ARG_UNUSED(pins);

	data = CONTAINER_OF(cb, struct kscan_sx1508_data, irq_cb);

	k_sem_give(&data->irq_sem);
}

static void sx1508_timer_callback(struct k_timer *timer)
{

	struct kscan_sx1508_data *data;

	data = CONTAINER_OF(timer, struct kscan_sx1508_data, timer);
	k_sem_give(&data->irq_sem);
}

static int kscan_sx1508_config(const struct device *parent, kscan_callback_t callback)
{

	struct kscan_sx1508_data *data = parent->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->kscan_cb = callback;
	k_mutex_unlock(&data->lock);

	return 0;
}
#endif /* CONFIG_SX1508_KEYSCAN */

static int kscan_sx1508_init(const struct device *dev)
{

	const struct kscan_sx1508_cfg *config = dev->config;
	struct kscan_sx1508_data *data = dev->data;
	config_cb=config;
	data_cb=data;
	int err;
	int button;

	data->dev = dev;

	if (!device_is_ready(config->i2c.bus)) {
		printk("sx1508 parent device not ready\n");
		return -EINVAL;
	}


#ifdef CONFIG_KSCAN_SX1508
	k_mutex_init(&data->lock);
	k_sem_init(&data->irq_sem, 0, 1);

	/* Configure interrupt */

	if (config->irq_enabled) {
		uint8_t keys[SX1508_DISP_DATA_SIZE];

		if (!gpio_is_ready_dt(&config->irq)) {
			LOG_ERR("IRQ device not ready\n");
			return -EINVAL;
		}

		/* Flush key data before enabling interrupt */
		err = i2c_burst_read_dt(&config->i2c, REG_DATA, keys, sizeof(keys));
		if (err) {
			LOG_ERR("Failed to to read sx1508 key data");
			return -EIO;
		}

		err = i2c_reg_write_byte_dt(&config->i2c, REG_RESET,
					    REG_RESET_MAGIC0); 
		if (err != 0) {
			printk("%s: reset m0 failed: %d\n", dev->name, err);
		}

		err = i2c_reg_write_byte_dt(&config->i2c, REG_RESET, REG_RESET_MAGIC1); //
		if (err != 0) {
			printk("%s: reset m1 failed: %d\n", dev->name, err);
		}

		uint8_t buf[2];

		/*Configures direction for each IO. output => 0 , Input => 1 */
		buf[0] = REG_DIR;
		buf[1] = REG_DIR_DATA;
		err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
		if (err != 0) {
			LOG_ERR(" i2c device not ready \n");
		}

		/*Enables open drain operation for  IO0 to I03*/
		buf[0] = REG_OPENDRAIN;
		buf[1] = REG_OPENDRAIN_DATA;

		err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
		if (err != 0) {
			LOG_ERR("i2c device not ready\n");
		}

		/*Enables the pull-down for each  IO*/
		buf[0] = REG_PULLUP;
		buf[1] = REG_PULLUP_DATA;
		err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
		if (err != 0) {
			LOG_ERR(" i2c device not ready\n");
		}

		/*Enables debouncing for each [input-configured] IO*/
		buf[0] = REG_DEBOUNCE_ENABLE;
		buf[1] = REG_DEBOUNCE_ENABLE_DATA;

		err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
		if (err != 0) {
			LOG_ERR(" i2c device not ready\n");
		}

		/*Configure debouncing Time*/
		buf[0] = REG_DEBOUNCE_CONFIG;
		buf[1] = REG_DEBOUNCE_CONFIG_DATA;
		err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
		if (err != 0) {
			LOG_ERR(" i2c device not ready \n");
		}

		/*Configure number of keypad ROWs and column*/
		buf[0] = REG_KEY_CONFIG;
		buf[1] = REG_KEY_CONFIG_DATA;
		err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
		if (err != 0) {
			LOG_ERR(" i2c device not ready \n");
		}

		/*Configure Oscillator frequency (fOSC) source*/
		buf[0] = REG_CLOCK;
		buf[1] = REG_CLOCK_DATA;
		err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
		if (err != 0) {
			LOG_ERR(" i2c device not ready \n");
		}

		/*Configures which [input-configured] IO will trigger an interrupt on NINT
		 * pin*/
		buf[0] = REG_INTERRUPT_MASK;
		buf[1] = REG_INTERRUPT_MASK_ENABLE;
		err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
		if (err != 0) {
			LOG_ERR(" i2c device not ready \n");
		}

		/*Configures EDGE SENSE*/
		buf[0] = REG_SENSE_HIGH;
		buf[1] = REG_SENSE_FALLING_EDGE_TRIG;
		err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
		if (err != 0) {
			LOG_ERR(" i2c device not ready \n");
		}

		gpio_pin_configure_dt(&config->irq, GPIO_INPUT | GPIO_ACTIVE_LOW );

		gpio_init_callback(&data->irq_cb, sx1508_irq_callback, BIT(config->irq.pin));

		gpio_add_callback(config->irq.port, &data->irq_cb);

		gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);

		// gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_FALLING);


	}

	k_thread_create(&data->irq_thread, data->irq_thread_stack,
			CONFIG_SX1508_KEYSCAN_IRQ_THREAD_STACK_SIZE, (k_thread_entry_t)sx1508_irq_thread,
			data, NULL, NULL, K_PRIO_COOP(CONFIG_SX1508_KEYSCAN_IRQ_THREAD_PRIO), 0,
			K_NO_WAIT);

#endif

	return 0;
}

// static const struct gpio_driver_api api_table

static const struct kscan_driver_api kscan_sx1508_api = {
	.config = kscan_sx1508_config,
};

#define KSCAN_SX1508_DEVICE(id)                                                                    \
	static const struct kscan_sx1508_cfg kscan_sx1508_##id##_cfg = {                           \
		.i2c = I2C_DT_SPEC_INST_GET(id),                                                   \
		.irq_enabled = true,                                                               \
		.irq = GPIO_DT_SPEC_INST_GET(id, nint_gpios),                                      \
	};                                                                                         \
	\ 
  static struct kscan_sx1508_data kscan_sx1508_##id##_data;                                        \
	\                
         DEVICE_DT_INST_DEFINE(id, &kscan_sx1508_init, NULL, &kscan_sx1508_##id##_data,            \
			       &kscan_sx1508_##id##_cfg, POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY, &kscan_sx1508_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_SX1508_DEVICE)
