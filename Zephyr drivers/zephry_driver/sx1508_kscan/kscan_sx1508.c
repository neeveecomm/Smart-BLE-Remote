/*
 * Copyright (c) 2019 - 2021 Henrik Brix Andersen <henrik@brixandersen.dk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sx1508_keyscan

/**
 * @file
 * @brief Keyscan driver for the SX1508 I2C BUTTON driver
 */
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sx1508_kscan/sx1508_kscan.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(kscan_sx1508, CONFIG_KSCAN_LOG_LEVEL);

/*sx1508 options */

#define SX1508_KEY_DATA           0x08
#define SX1508_KEYSCAN_ROWS       4
#define SX1508_KEYSCAN_COLS       4
#define SX1508_KEYSCAN_DATA_SIZE  6



BUILD_ASSERT(CONFIG_KSCAN_INIT_PRIORITY > CONFIG_BUTTON_INIT_PRIORITY,
	"sx1508 keyscan driver must be initialized after SX1508 KEYPAD driver");

struct sx1508_cfg {
	struct i2c_dt_spec i2c;
	bool irq_enabled;
	// const struct device *parent;
#ifdef CONFIG_SX1508_KEYSCAN
    struct gpio_dt_spec irq;
	#endif /* CONFIG_SX1508_KEYSCAN */
};

struct ht16k33_data {
	const struct device *dev;

#ifdef CONFIG_SX1508_KEYSCAN
    struct k_mutex lock;         //
	const struct device *child; //
	kscan_callback_t kscan_cb;
	struct gpio_callback irq_cb;
	struct k_thread irq_thread;
	struct k_sem irq_sem;
	struct ktimer timer;
	
	K_KERNEL_STACK_MEMBER(irq_thread_stack,
	                        CONFIG_HT16K33_KEYSCAN_IRQ_THREAD_STACK_SIZE);
	#endif /*CONFIG_SX1508_KEYSCAN*/
	};
#ifdef CONFIG_SX1508_KEYSCAN
static bool sx1508_process_keyscan_data(const struct device *dev)
{
	const struct sx1508_cfg *config =dev->config;
	struct sx1508_data *data =dev->data;
	uint8_t keys[SX1508_KEYSCAN_DATA_SIZE];//
	bool pressed =false;
	uint16_t state;
	uint16_t changed;
	int row;
	int col;
	int err;

	err = i2c_burst_read_dt(&config->i2c,SX1508_KEY_DATA, keys,sizeof(keys));
	if(err){
		LOG_WRN("Failed to read sx1508 key data (err %d)",err);
		return true;
	}
	k_mutex_lock(&data->lock,K_FOREVER);

	for(row=0; row < sx1508_KEYSCAN_ROWS;row++){
		state = sys_get_le16(&keys[row * 2]);      //
		changed = data->key_state[row] ^ state;
		data->key_state[row] =state;
		if(state)
		{
			pressed =true;
		}
		if(data->kscan_cb==NULL){
			continue;
		}
	
	for (col =0; col< SX1508_KEYSCAN_ROWS; row++)
	{
		if(changed & BIT(col)){
			data->kscan_cb(data->child,row,col,
			        state & BIT(col));
		}
	}
} 
    k_mutex_unlock(&data -> lock);

	return pressed;
}
static void sx1508_irq_thread(struct sx1508_data *data)
{
	bool pressed;
	while(true)
	{
		k_sem_take(&data->irq_sem,K_FOREVER);
        do{
		pressed = sx1508_process_keyscan_data(data->dev);
		k_msleep(CONFIG_SX1508_KEYSCAN_DEBOUNCE_MSEC);
		} while(pressed);
	}
}

static void sx1508_irq_callback(const struct device *gpiob,
                        struct gpio_callback *cb,uint32_t pins)
	{
		struct sx1508_data *data;

		ARG_UNUSED(gpiob);
		ARG_UNUSED(pins);
		data = CONTAINER_OF(cb, struct k_timer *timer);
		k_sem_give (&data->irq_sem);
	}

static void sx1508_timer_callback(struct k_timer,*timer)
{
	struct sx1508_data *data;
	 
	 data = CONTAINER_OF(timer,struct sx1508_data,timer);
	 k_sem_give(&data->irq_sem);

}


static int sx1508_keyscan_callback (const struct device *parent,
                const struct device *child,
				kscan_callback_t callback)
{
	struct ht16k33_data *data = parent->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->child = child;
	data->kscan_cb = callback;
	k_mutex_unlock(&data->lock);

	return 0;
}
#endif /* CONFIG_sx1508_KEYSCAN */

static int kscan_sx1508_init(const struct device *dev)
{
	const struct sx1508_cfg *config = dev->config;
	struct sx1508_data *data = dev->data;
    int err;
	data->dev =dev;

	if (!device_is_ready(config->parent)) {
		LOG_ERR("sx1508 parent device not ready");
		return -EINVAL;
	}
	if(!device_is_ready(config->i2c.bus)){
		LOG ERR ("I2C bus device not ready");
		return -EINVAL;
	}

#ifdef CONFIG_SX1508_KEYSCAN
    k_mutex_init(&data->lock);
	k_sem_init(&data->irq_sem,0,1);

	/*configure interrupt*/
	if (config->irq_enabled){
		uint8_t keys[SX1508_KEYSCAN_DATA_SIZE]; 
		
		if(!device_is_ready(config->irq.port)){
			LOG_ERR("IRQ device not ready");
			return -EINVAL;
		}
		err = gpio_pin_configure_dt(&config->irq,GPIO_INPUT);
		if(err)
		{
			LOG_ERR("Failed to configure IRQ pin (err %d)",err);
			return -EINVAL;
		}
		gpio_init_callback(&data->irq_cb,&sx1508_irq_callback,
		            BIT(config->irq.pin));
		err =gpio_add_callback(config->irq.port,&data->irq_cb);
		if(err){
			LOG_ERR("Failed to add irq callback (err %d)",err);
			return -EINVAL;
		}	
	/* Flush key data before enabling interrupt */
		err = i2c_burst_read_dt(&config->i2c, SX1508_KEY_DATA, keys,
					sizeof(keys));
		
	}

#define KSCAN__DEVICE(id)					\
	static const struct kscan_sx1508_cfg kscan_sx1508_##id##_cfg = { \
		.parent = DEVICE_DT_GET(DT_INST_BUS(id)),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(id, &kscan_sx1508_init,			\
			      NULL, NULL,				\
			      &kscan_sx1508_##id##_cfg, POST_KERNEL,	\
			      CONFIG_KSCAN_INIT_PRIORITY,		\
			      &kscan_sx1508_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_SX1508_DEVICE)
