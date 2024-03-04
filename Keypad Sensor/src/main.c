#include <zephyr/kernel.h>
// #include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio/gpio_sx1509b.h>

#define REG_KEY_DATA 0x15

#define I2C0_NODE DT_NODELABEL(sx1509b)
bool rec_val = true;
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
        rec_val = true;
        printk("here\n");
}

static struct gpio_callback sx_int_cb;
// static struct gpio_dt_spec sx_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(button_0), gpios, 0);
#define key_press_detect DT_NODELABEL(sx1509b)
static const struct gpio_dt_spec sx_gpio = GPIO_DT_SPEC_GET(key_press_detect,nint_gpios);
// static const struct gpio_dt_spec sx_intr = GPIO_DT_SPEC_GET(I2C0_NODE,gpios);

static const struct i2c_dt_spec i2c_node = I2C_DT_SPEC_GET(I2C0_NODE);
static struct device *sx = DEVICE_DT_GET(DT_NODELABEL(sx1509b));                   
// static  struct i2c_dt_spec i2c_node = I2C_DT_SPEC_GET(I2C0_NODE);
void keypad_press_decode(uint8_t switch_press_data)
{
        printk("some sw Pressed\n");
        // printk("inetrrupt flag clear\n");
        switch (switch_press_data)
        {
        case 0xEE:
                printk("SW2 Pressed\n");
                break;
        }
}

void i2c_keypad_write(uint8_t *data)
{
        uint8_t buf[2];
        buf[0] = data[0];
        buf[1] = data[1];
        int ret = i2c_write_dt(&i2c_node, buf, sizeof(buf));
        if (ret != 0)
        {
                printk("the i2c device not ready\n");
        }
}
#define REG_INTERRUPT_SOURCE 0x0C
#define REG_INTERRUPT_CLEAR_FLAG 0xFF
/* This function used to clear interrupt flag*/
void keypad_interrupt_clear()
{
        // printk("interrupt flag clear\n");
        uint8_t buf[2];
        buf[0] = REG_INTERRUPT_SOURCE;
        buf[1] = REG_INTERRUPT_CLEAR_FLAG;
        i2c_keypad_write(buf);
}

int main()
{

        if (!device_is_ready(sx))
        {
                printk("dev bus %s is not ready! \n\r", sx->name);
        }
        else
        {
                printk("dev bus %s is ready!\n\r", sx->name);
        }

        if (!device_is_ready(i2c_node.bus))
        {
                printk("I2C bus %s is not ready! \n\r", i2c_node.bus->name);
        }
        else
        {
                printk("I2C bus %s is ready!\n\r", i2c_node.bus->name);
        }

        // gpio_init_callback(&sx_int_cb, button_pressed, sx_gpio.pin);

        gpio_pin_configure_dt(&sx_gpio, GPIO_INPUT| GPIO_INT_EDGE | GPIO_ACTIVE_HIGH);
        
        gpio_pin_interrupt_configure_dt(&sx_gpio, GPIO_INT_EDGE_TO_ACTIVE);

        gpio_init_callback(&sx_int_cb, button_pressed, BIT(sx_gpio.pin));

        gpio_add_callback(sx_gpio.port, &sx_int_cb);



        while (1)
        {
                /*if key pressed this rec_val flag set in callback function*/
                if (rec_val == true)
                {
                        uint8_t i2c_transmit_buf = REG_KEY_DATA;
                        uint8_t i2c_receive_buf;
                        rec_val = false;
                        i2c_write_read_dt(&i2c_node, &i2c_transmit_buf, 1, &i2c_receive_buf, 1);
                        // printk("sREG_DATA %d and %x\n",i2c_receive_buf,i2c_receive_buf);
                        keypad_press_decode(i2c_receive_buf);
                        printk("i2c_receive_buf =%d \n", i2c_receive_buf);
                        keypad_interrupt_clear();
                }

                k_sleep(K_MSEC(50));
        }  

        return 0;
}
