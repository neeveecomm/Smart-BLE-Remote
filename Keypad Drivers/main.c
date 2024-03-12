#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio/gpio_sx1509b.h>


#define I2C0_NODE DT_NODELABEL(sx1509b)

static const struct i2c_dt_spec i2c_node = I2C_DT_SPEC_GET(I2C0_NODE);

static struct device *sx = DEVICE_DT_GET(DT_NODELABEL(sx1509b));                   


int gpio_sx1509b_callback(struct device *dev,int key)
{  
         printk("keypad data%d\n",key);

}
int main()
{

        if (!device_is_ready(sx))
        {
                printk("dev bus %s is not ready! \n\r", sx->name);
        }

        if (!device_is_ready(i2c_node.bus))
        {
                printk("I2C bus %s is not ready! \n\r", i2c_node.bus->name);
        }
          

        sx1509_add_callback(sx,gpio_sx1509b_callback);

        return 0;
}
