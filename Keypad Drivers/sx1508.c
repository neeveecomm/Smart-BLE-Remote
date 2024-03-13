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
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/kscan_sx1508.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
 
#define REG_DIR                      0x07
#define REG_DIR_DATA                 0xF0
#define REG_OPENDRAIN                0x05
#define REG_OPENDRAIN_DATA           0x0F
#define REG_PULLUP                   0x03
#define REG_PULLUP_DATA              0xFF
#define REG_PULLDOWN                 0x04
#define REG_PULLDOWN_DATA            0x00
#define REG_DEBOUNCE_ENABLE          0x13
#define REG_DEBOUNCE_ENABLE_DATA     0xF0
#define REG_DEBOUNCE_CONFIG          0x12
#define REG_DEBOUNCE_CONFIG_DATA     0x05
#define REG_KEY_CONFIG               0x14
#define REG_KEY_CONFIG_DATA          0x7D
#define REG_CLOCK                    0x0F
#define REG_CLOCK_DATA               0x40
#define REG_DATA                     0x08
#define REG_DATA_WRITE               0x03
#define REG_INTERRUPT_MASK           0x09
#define REG_INTERRUPT_MASK_ENABLE    0x0F
#define REG_SENSE                    0x0A
#define REG_SENSE_FALLING_EDGE_TRIG  0x00
#define REG_KEY_DATA                 0x15
#define REG_INTERRUPT_SOURCE         0x0C
#define REG_INTERRUPT_CLEAR_FLAG     0xFF
#define REG_EVENT_STATUS             0x0D
#define REG_EVENT_CLEAR_FLAG         0xFF
#define SX1508_KEYSCAN_ROWS           4
#define SX1508_KEYSCAN_COLS           4
#define SX1508_KEYSCAN_DATA_SIZE      6

#define KEYSCAN_DEBOUNCE_MSEC        50

#define SX1508_DISP_DATA_SIZE        6             

#define SX1508_KEYSCAN_IRQ_THREAD_STACK_SIZE 400    //
#define SX1508_KEYSCAN_IRQ_THREAD_PRIO 2

LOG_MODULE_REGISTER(kscan_sx1508, CONFIG_KSCAN_LOG_LEVEL);

struct kscan_sx1508_cfg {
	struct i2c_dt_spec i2c;
	bool irq_enabled;
#ifdef  CONFIG_KSCAN_SX1508
	struct gpio_dt_spec irq;
#endif /* CONFIG_SX1508_KEYSCAN */
};

struct kscan_sx1508_data {
	const struct device *dev;
	 /* Shadow buffer for the display data RAM */
	uint8_t buffer[SX1508_DISP_DATA_SIZE];       //
#ifdef CONFIG_KSCAN_SX1508
	struct k_mutex lock;
	const struct device *parent;
	kscan_callback_t kscan_cb;
	struct gpio_callback irq_cb;
	struct k_thread irq_thread;
	struct k_sem irq_sem;
	struct k_timer timer;
	uint16_t key_state[SX1508_KEYSCAN_ROWS];          //

	K_KERNEL_STACK_MEMBER(irq_thread_stack,
			     SX1508_KEYSCAN_IRQ_THREAD_STACK_SIZE);
#endif /* CONFIG_SX1508_KEYSCAN */
};

#ifdef CONFIG_KSCAN_SX1508
static bool sx1508_process_keyscan_data(const struct device *dev)
{
          
;
	const struct kscan_sx1508_cfg *config = dev->config;
	struct kscan_sx1508_data *data = dev->data;
	uint8_t keys[SX1508_DISP_DATA_SIZE];
	bool pressed = false;
	uint16_t state;
	uint16_t changed;
	int row;
	int col;
	int err;


	err = i2c_burst_read_dt(&config->i2c, REG_DATA, keys, sizeof(keys));
	if (err) {
		LOG_WRN("Failed to to read SX1508 key data (err %d)", err);
		/* Reprocess */
		return true;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
         {
 
	for (row = 0; row < SX1508_KEYSCAN_ROWS; row++) {
		state = sys_get_le16(&keys[row * 2]);
		changed = data->key_state[row] ^ state;
		data->key_state[row] = state;

		if (state) {
			pressed = true;
		}

		if (data->kscan_cb == NULL) {
			continue;
		}

		for (col = 0; col < SX1508_KEYSCAN_COLS; col++) {
			if (changed & BIT(col)) {
				data->kscan_cb(data->parent, row, col,
					state & BIT(col));
			}
		}
	}


	k_mutex_unlock(&data->lock);

	return pressed;

      }

}


static void sx1508_irq_thread(struct kscan_sx1508_data *data)
{
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
static void sx1508_irq_callback(const struct device *gpiob,
				 struct gpio_callback *cb, uint32_t pins)
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

static int kscan_sx1508_config(const struct device *parent,
				      kscan_callback_t callback)
{
 

    struct kscan_sx1508_data *data = parent->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->kscan_cb = callback;
	k_mutex_unlock(&data->lock);

	return 0;
} 
#endif   /* CONFIG_SX1508_KEYSCAN */


static int kscan_sx1508_init (const struct device *dev)
{
     
        const struct kscan_sx1508_cfg *config = dev->config;
	struct kscan_sx1508_data *data = dev->data;
	int err;


         
    data->dev = dev;

    if (!device_is_ready(config->i2c.bus)) {
		printk("sx1508 parent device not ready\n");
		return -EINVAL;
	}

      else 
	{
		printk("I2C bus %s is ready! \n\r",config->i2c.bus->name);
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
        
        gpio_pin_configure_dt(&config->irq, GPIO_INPUT);

        
        gpio_init_callback(&data->irq_cb,sx1508_irq_callback,
				   BIT(config->irq.pin));

        gpio_add_callback(config->irq.port, &data->irq_cb);

	

        /* Flush key data before enabling interrupt */
	err = i2c_burst_read_dt(&config->i2c, REG_DATA , keys,
					sizeof(keys));
	if (err) {
		LOG_ERR("Failed to to read HT16K33 key data");
		return -EIO;
	}

        gpio_pin_interrupt_configure_dt(&config->irq,
			              GPIO_INT_EDGE_TO_INACTIVE);  

         }                            


        k_thread_create(&data->irq_thread, data->irq_thread_stack,
			SX1508_KEYSCAN_IRQ_THREAD_STACK_SIZE,
			(k_thread_entry_t)sx1508_irq_thread, data, NULL, NULL,
			K_PRIO_COOP(SX1508_KEYSCAN_IRQ_THREAD_PRIO),
			0, K_NO_WAIT);


    #endif  

    return 0;

}

static const struct kscan_driver_api kscan_sx1508_api = {
	.config = kscan_sx1508_config,
};


#define KSCAN_SX1508_DEVICE(id)                            		        \
  static const struct kscan_sx1508_cfg kscan_sx1508_##id##_cfg ={            	\
        .i2c          = I2C_DT_SPEC_INST_GET(id),                            	\
	.irq_enabled  = true,					                \
        .irq          = GPIO_DT_SPEC_INST_GET(id, irq_gpios),                   \
};                                           			                \
                                                                                \ 
  static struct kscan_sx1508_data kscan_sx1508_##id##_data;		        \                
                                                		                \
  DEVICE_DT_INST_DEFINE(id, &kscan_sx1508_init,                        	        \
                                NULL,&kscan_sx1508_##id##_data,                 \
                                &kscan_sx1508_##id##_cfg,POST_KERNEL,	        \
                                CONFIG_KSCAN_INIT_PRIORITY,                     \
                                &kscan_sx1508_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_SX1508_DEVICE)
