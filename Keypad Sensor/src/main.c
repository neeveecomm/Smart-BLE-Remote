#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>


/*----------------------------------Macro--------------------------------*/
#define REG_DIR 0x07
#define REG_DIR_DATA 0xF0
#define REG_OPENDRAIN 0x05
#define REG_OPENDRAIN_DATA 0x0F
#define REG_PULLUP 0x03
#define REG_PULLUP_DATA 0xFF
#define REG_PULLDOWN 0x04
#define REG_PULLDOWN_DATA 0x00
#define REG_DEBOUNCE_ENABLE 0x13
#define REG_DEBOUNCE_ENABLE_DATA 0xF0
#define REG_DEBOUNCE_CONFIG 0x12
#define REG_DEBOUNCE_CONFIG_DATA 0x05
#define REG_KEY_CONFIG 0x14
#define REG_KEY_CONFIG_DATA 0x7D
#define REG_CLOCK 0x0F
#define REG_CLOCK_DATA 0x40
#define REG_DATA 0x08
#define REG_DATA_WRITE 0x03
#define REG_INTERRUPT_MASK 0x09
#define REG_INTERRUPT_MASK_ENABLE 0x0F
#define REG_SENSE 0x0A
#define REG_SENSE_FALLING_EDGE_TRIG 0x00
#define REG_KEY_DATA 0x15
#define REG_INTERRUPT_SOURCE 0x0C
#define REG_INTERRUPT_CLEAR_FLAG 0xFF
#define REG_EVENT_STATUS 0x0D
#define REG_EVENT_CLEAR_FLAG 0xFF
/*-------------------------------------------------------------------*/

/*--------------------Function prototype-----------------------------*/
void keypad_press_decode(uint8_t);
void keypad_configuration();
void sensor_write_byte(uint8_t);
int sensor_read();
void keypad_press_decode(uint8_t);
void keypad_interrupt_clear();

/*-------------------------------------------------------------------*/
/*---------------------Global variables------------------------------*/
uint8_t ret;
uint8_t rec_val;
/*-------------------------------------------------------------------*/



#define I2C_node DT_NODELABEL(i2c_sens)

static const struct i2c_dt_spec i2c_node = I2C_DT_SPEC_GET(I2C_node);

#define key_press_detect DT_NODELABEL(sensor_trig)

static const struct gpio_dt_spec key_interrupt = GPIO_DT_SPEC_GET(key_press_detect, gpios);



void sensor_write_byte(uint8_t data)
{
        uint8_t buf[1];
        buf[0] = data;
        ret = i2c_write_dt(&i2c_node, buf, sizeof(buf));
        if (ret != 0)
        {
                printf("the i2c device not ready\n");
        }
}

void i2c_keypad_write(uint8_t *data)
{
        uint8_t buf[2];
        buf[0] = data[0];
        buf[1] = data[1];
        ret = i2c_write_dt(&i2c_node, buf, sizeof(buf));
        if (ret != 0)
        {
                printf("the i2c device not ready\n");
        }
}

int sensor_read()
{
        uint8_t data;
        ret = i2c_read_dt(&i2c_node, &data, sizeof(data));
        if (ret != 0)
        {
                printf("i2c read not happed\n");
        }
        return data;
}

void keypad_press_decode(uint8_t switch_press_data)
{
        // printk("inetrrupt flag clear\n");
        switch (switch_press_data)
        {
        case 0xEE:
                printk("SW2 Pressed\n");
                break;
        case 0xED:
                printk("SW3 Pressed\n");
                break;
        case 0xEB:
                printk("SW4 Pressed\n");
                break;
        case 0xE7:
                printk("SW5 Pressed\n");
                break;

        case 0xDE:
                printk("SW6 Pressed\n");
                break;
        case 0xDD:
                printk("SW7 Pressed\n");
                break;
        case 0xDB:
                printk("SW8 Pressed\n");
                break;
        case 0xD7:
                printk("SW9 Pressed\n");
                break;

        case 0xBE:
                printk("SW10 Pressed\n");
                break;
        case 0xBD:
                printk("SW11 Pressed\n");
                break;
        case 0xBB:
                printk("SW12 Pressed\n");
                break;
        case 0xB7:
                printk("SW13 Pressed\n");
                break;

        case 0x7E:
                printk("SW14 Pressed\n");
                break;
        case 0x7D:
                printk("SW15 Pressed\n");
                break;
        case 0x7B:
                printk("SW16 Pressed\n");
                break;
        case 0x77:
                printk("SW17 Pressed\n");
                break;
        }
}

/* This function used to clear interrupt flag*/
void keypad_interrupt_clear()
{
        // printk("inetrrupt flag clear\n");
        uint8_t buf[2];
        buf[0] = REG_INTERRUPT_SOURCE;
        buf[1] = REG_INTERRUPT_CLEAR_FLAG;
        i2c_keypad_write(buf);
}

void keypad_configuration()
{
        uint8_t buf[2];
/*Configures direction for each IO. output => 0 , Input => 1 */
        buf[0] = REG_DIR;
        buf[1] = REG_DIR_DATA;
        i2c_keypad_write(buf);

/*Enables open drain operation for  IO0 to I03*/
        buf[0] = REG_OPENDRAIN;
        buf[1] = REG_OPENDRAIN_DATA;
        i2c_keypad_write(buf);

/*Enables the pull-down for each  IO*/
        buf[0] = REG_PULLUP;
        buf[1] = REG_PULLUP_DATA;
        i2c_keypad_write(buf);

/*Enables debouncing for each [input-configured] IO*/
        buf[0] = REG_DEBOUNCE_ENABLE;
        buf[1] = REG_DEBOUNCE_ENABLE_DATA;
        i2c_keypad_write(buf);

/*Configure debouncing Time*/
        buf[0] = REG_DEBOUNCE_CONFIG;
        buf[1] = REG_DEBOUNCE_CONFIG_DATA;
        i2c_keypad_write(buf);

/*Configure number of keypad ROWs and column*/
        buf[0] = REG_KEY_CONFIG;
        buf[1] = REG_KEY_CONFIG_DATA;
        i2c_keypad_write(buf);

/*Configure Oscillator frequency (fOSC) source*/
        buf[0] = REG_CLOCK;
        buf[1] = REG_CLOCK_DATA;
        i2c_keypad_write(buf);

/*Configures which [input-configured] IO will trigger an interrupt on NINT pin*/
        buf[0] = REG_INTERRUPT_MASK;
        buf[1] = REG_INTERRUPT_MASK_ENABLE;
        i2c_keypad_write(buf);

/*Configures EDGE SENSE*/
        buf[0] = REG_SENSE;
        buf[1] = REG_SENSE_FALLING_EDGE_TRIG;
        i2c_keypad_write(buf);

}

/*  Define the callback function */
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{

       
        rec_val = true;
}

/* Define a variable of type static struct gpio_callback */
static struct gpio_callback button_cb_data;

int main(void)
{
        uint8_t val;
printk("program start\n");

        if (!device_is_ready(i2c_node.bus))
        {
                printf("node is not ready\n");
        }

        if (!device_is_ready(key_interrupt.port))
        {
                printf("pin is not ready\n");
        }
        /*configure keypad interrupt pin as input*/
        ret = gpio_pin_configure_dt(&key_interrupt, GPIO_INPUT);        
        if (ret < 0)
        {
                return;
        }

        /* Configure the interrupt on the button's pin */
        ret = gpio_pin_interrupt_configure_dt(&key_interrupt, GPIO_INT_EDGE_TO_INACTIVE); // GPIO_INT_EDGE_TO_INACTIVE

        /* Initialize the static struct gpio_callback variable   */
        gpio_init_callback(&button_cb_data, button_pressed, BIT(key_interrupt.pin));

        /* Add the callback function by calling gpio_add_callback()   */

        gpio_add_callback(key_interrupt.port, &button_cb_data);

        keypad_configuration();

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
                        keypad_interrupt_clear();
                }

                k_sleep(K_MSEC(50));
        }
}
