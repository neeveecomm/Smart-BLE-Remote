#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#define SLAVE_ADDRESS 0x18
#define REG_ADDRESS 0x12

#define I2C_NODE DT_NODELABEL(mysensor)
struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

void tt_i2c_write(uint8_t slaveAddr, uint8_t regAddr)
{
	uint8_t buf;

	buf = regAddr;

	i2c_write_dt(&dev_i2c, &buf, sizeof(buf));
	return;
}

uint8_t tt_i2c_read(uint8_t slaveAddr)
{
	uint8_t data;
	i2c_read_dt(&dev_i2c, &data, sizeof(data));
	return data;
}

void MEASURE_DATA(void)
{
	uint32_t data;

	tt_i2c_write(SLAVE_ADDRESS, REG_ADDRESS);
	data = tt_i2c_read(SLAVE_ADDRESS);
	printk("read  data : %02d \n", data);

	k_sleep(K_MSEC(1000));
}

void main()
{

	while (1)
	{
		if (!device_is_ready(dev_i2c.bus))
		{
			printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
			return;
		}
		// MEASURE_RELATIVE_HUMIDITY_NO_HOLD_MASTER_MODE ();

		MEASURE_DATA();
	}
}