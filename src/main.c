/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <assert.h>
#include <zephyr/spinlock.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/bluetooth/services/bas.h>
#include <bluetooth/services/hids.h>
#include <zephyr/bluetooth/services/dis.h>
#include <dk_buttons_and_leds.h>


// keypad
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/logging/log.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BASE_USB_HID_SPEC_VERSION 0x0101

#define OUTPUT_REPORT_MAX_LEN 1
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK 0x02
#define KEYPAD_INPUT_REP_KEYS_REF_ID 1
#define OUTPUT_REP_KEYS_REF_ID 0
#define MODIFIER_KEY_POS 0
#define SHIFT_KEY_CODE 0x02
#define SCAN_CODE_POS 2
#define KEYS_MAX_LEN (CONSUMER_INPUT_REPORT_KEYS_MAX_LEN - \
					  SCAN_CODE_POS)

#define ADV_LED_BLINK_INTERVAL 1000

#define ADV_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define LED_CAPS_LOCK DK_LED3
#define NFC_LED DK_LED4
#define KEY_TEXT_MASK DK_BTN1_MSK
#define KEY_SHIFT_MASK DK_BTN2_MSK
#define KEY_ADV_MASK DK_BTN4_MSK

/* Key used to accept or reject passkey value */
#define KEY_PAIRING_ACCEPT DK_BTN1_MSK
#define KEY_PAIRING_REJECT DK_BTN2_MSK

/* HIDs queue elements. */
#define HIDS_QUEUE_SIZE 10

#define CONSUMER_INPUT_REPORT_KEYS_MAX_LEN 1
 
#define CONSUMER_INPUT_REP_KEYS_REF_ID 2

#define ADVANCE_CONSUMER_INPUT_REP_KEYS_REF_ID 3


/* ********************* */
/* Buttons configuration */

/* Note: The configuration below is the same as BOOT mode configuration
 * This simplifies the code as the BOOT mode is the same as REPORT mode.
 * Changing this configuration would require separate implementation of
 * BOOT mode report generation.
 */
#define KEY_CTRL_CODE_MIN 224 /* Control key codes - required 8 of them */
#define KEY_CTRL_CODE_MAX 231 /* Control key codes - required 8 of them */
#define KEY_CODE_MIN 0		  /* Normal key codes */
#define KEY_CODE_MAX 101	  /* Normal key codes */
#define KEY_PRESS_MAX 6		  /* Maximum number of non-control keys \
							   * pressed simultaneously             \
							   */

/* Number of bytes in key report
 *
 * 1B - control keys
 * 1B - reserved
 * rest - non-control keys
 */
#define INPUT_REPORT_KEYS_MAX_LEN (1 + 1 + KEY_PRESS_MAX)

/* keypad */ 
#define LOG_LEVEL LOG_LEVEL_DBG
#define key_press_detect DT_NODELABEL(sx1508)

// Key codes
#define ARROW_UP     0x52
#define ARROW_DOWN   0x51
#define ARROW_LEFT   0X50
#define ARROW_RIGHT  0X4F
#define ARROW_ENTER  0X28
#define VOLUME_UP 	 0x01
#define VOLUME_DOWN  0x02
#define MUTE		 0x04
#define HOME 		 0x08
#define MENU 		 0x10
#define PREV_TRACK   0x20
#define NEXT_TRACK   0X40 
#define BACK		 0X80
#define POWER		 0X01
#define PLAY_PAUSE	 0X02
#define VOICE_CMD	 0X04
#define RELEASED_KEY 0X00

/* keypad */
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);
static const struct i2c_dt_spec i2c_node_2 = I2C_DT_SPEC_GET(DT_NODELABEL(sx1508));
static struct device *dev_2 = DEVICE_DT_GET(DT_NODELABEL(sx1508));
static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(key_press_detect, nint_gpios);

/* Current report map construction requires exactly 8 buttons */
BUILD_ASSERT((KEY_CTRL_CODE_MAX - KEY_CTRL_CODE_MIN) + 1 == 8);

/* OUT report internal indexes.
 *
 * This is a position in internal report table and is not related to
 * report ID.
 */
enum
{
	OUTPUT_REP_KEYS_IDX = 0
};

/* INPUT report internal indexes.
 *
 * This is a position in internal report table and is not related to
 * report ID.
 */
enum
{
	INPUT_REP_KEYS_IDX = 0
};

enum
{
	CONSUMER_INPUT_REP_KEYS_IDX =1,
	ADV_CONSUMER_INPUT_REP_KEYS_IDX =2

};

/* HIDS instance. */
BT_HIDS_DEF(hids_obj,
			OUTPUT_REPORT_MAX_LEN,
			CONSUMER_INPUT_REPORT_KEYS_MAX_LEN);

static volatile bool is_adv;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
				  (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
				  (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
				  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct conn_mode
{
	struct bt_conn *conn;
	bool in_boot_mode;
} conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];

/* Current report status
 */
static struct keyboard_state
{
	uint8_t ctrl_keys_state; /* Current keys state */
	uint8_t keys_state[KEY_PRESS_MAX];
} hid_keyboard_state;

static struct k_work pairing_work;
struct pairing_data_mitm
{
	struct bt_conn *conn;
	unsigned int passkey;
};

K_MSGQ_DEFINE(mitm_queue,
			  sizeof(struct pairing_data_mitm),
			  CONFIG_BT_HIDS_MAX_CLIENT_COUNT,
			  4);

static void advertising_start(void)
{
	int err;
	struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
		BT_LE_ADV_OPT_CONNECTABLE |
			BT_LE_ADV_OPT_ONE_TIME,
		BT_GAP_ADV_FAST_INT_MIN_2,
		BT_GAP_ADV_FAST_INT_MAX_2,
		NULL);

	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd,
						  ARRAY_SIZE(sd));
	if (err)
	{
		if (err == -EALREADY)
		{
			printk("Advertising continued\n");
		}
		else
		{
			printk("Advertising failed to start (err %d)\n", err);
		}

		return;
	}

	is_adv = true;
	printk("Advertising successfully started\n");
}

static void pairing_process(struct k_work *work)
{
	int err;
	struct pairing_data_mitm pairing_data;

	char addr[BT_ADDR_LE_STR_LEN];

	err = k_msgq_peek(&mitm_queue, &pairing_data);
	if (err)
	{
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(pairing_data.conn),
					  addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, pairing_data.passkey);
	printk("Press Button 1 to confirm, Button 2 to reject.\n");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err)
	{
		printk("Failed to connect to %s (%u)\n", addr, err);
		return;
	}

	printk("Connected %s\n", addr);
	dk_set_led_on(CON_STATUS_LED);

	err = bt_hids_connected(&hids_obj, conn);

	if (err)
	{
		printk("Failed to notify HID service about connection\n");
		return;
	}

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++)
	{
		if (!conn_mode[i].conn)
		{
			conn_mode[i].conn = conn;
			conn_mode[i].in_boot_mode = false;
			break;
		}
	}

#if CONFIG_NFC_OOB_PAIRING == 0
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++)
	{
		if (!conn_mode[i].conn)
		{
			advertising_start();
			return;
		}
	}
#endif
	is_adv = false;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err;
	bool is_any_dev_connected = false;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected from %s (reason %u)\n", addr, reason);

	err = bt_hids_disconnected(&hids_obj, conn);

	if (err)
	{
		printk("Failed to notify HID service about disconnection\n");
	}

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++)
	{
		if (conn_mode[i].conn == conn)
		{
			conn_mode[i].conn = NULL;
		}
		else
		{
			if (conn_mode[i].conn)
			{
				is_any_dev_connected = true;
			}
		}
	}

	if (!is_any_dev_connected)
	{
		dk_set_led_off(CON_STATUS_LED);
	}

	advertising_start() ;
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
							 enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err)
	{
		printk("Security changed: %s level %u\n", addr, level);
	}
	else
	{
		printk("Security failed: %s level %u err %d\n", addr, level,
			   err);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

static void caps_lock_handler(const struct bt_hids_rep *rep)
{
	uint8_t report_val = ((*rep->data) & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) ? 1 : 0;
	dk_set_led(LED_CAPS_LOCK, report_val);
}

static void hids_outp_rep_handler(struct bt_hids_rep *rep,
								  struct bt_conn *conn,
								  bool write)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (!write)
	{
		printk("Output report read\n");
		return;
	};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Output report has been received %s\n", addr);
	caps_lock_handler(rep);
}

static void hids_boot_kb_outp_rep_handler(struct bt_hids_rep *rep,
										  struct bt_conn *conn,
										  bool write)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (!write)
	{
		printk("Output report read\n");
		return;
	};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Boot Keyboard Output report has been received %s\n", addr);
	caps_lock_handler(rep);
}

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt,
								struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	size_t i;

	for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++)
	{
		if (conn_mode[i].conn == conn)
		{
			break;
		}
	}

	if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT)
	{
		printk("Cannot find connection handle when processing PM");
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	switch (evt)
	{
	case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
		printk("Boot mode entered %s\n", addr);
		conn_mode[i].in_boot_mode = true;
		break;

	case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
		printk("Report mode entered %s\n", addr);
		conn_mode[i].in_boot_mode = false;
		break;

	default:
		break;
	}
}

static void hid_init(void)
{
	int err;
	struct bt_hids_init_param hids_init_obj = {0};
	struct bt_hids_inp_rep *hids_inp_rep;
	struct bt_hids_outp_feat_rep *hids_outp_rep;

	static const uint8_t report_map[] = {
      0x05, 0x01,                 // Usage Page (Generic Desktop)
        0x09, 0x06,                 // Usage (Keyboard)
        0xA1, 0x01,                 // Collection (Application)
	    0x85, 0x01,                 //     Report Id (1)
        0x05, 0x07,                 //     Usage Page (Key Codes)
        0x19, 0xe0,                 //     Usage Minimum (224)
        0x29, 0xe7,                 //     Usage Maximum (231)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x25, 0x01,                 //     Logical Maximum (1)
        0x75, 0x01,                 //     Report Size (1)
        0x95, 0x08,                 //     Report Count (8)
        0x81, 0x02,                 //     Input (Data, Variable, Absolute)

        0x95, 0x01,                 //     Report Count (1)
        0x75, 0x08,                 //     Report Size (8)
        0x81, 0x01,                 //     Input (Constant) reserved byte(1)

        0x95, 0x05,                 //     Report Count (5)
        0x75, 0x01,                 //     Report Size (1)
        0x05, 0x08,                 //     Usage Page (Page# for LEDs)
        0x19, 0x01,                 //     Usage Minimum (1)
        0x29, 0x05,                 //     Usage Maximum (5)
        0x91, 0x02,                 //     Output (Data, Variable, Absolute), Led report
        0x95, 0x01,                 //     Report Count (1)
        0x75, 0x03,                 //     Report Size (3)
        0x91, 0x01,                 //     Output (Data, Variable, Absolute), Led report padding

        0x95, 0x06,                 //     Report Count (6)
        0x75, 0x08,                 //     Report Size (8)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x25, 0x65,                 //     Logical Maximum (101)
        0x05, 0x07,                 //     Usage Page (Key codes)
        0x19, 0x00,                 //     Usage Minimum (0)
        0x29, 0x65,                 //     Usage Maximum (101)
        0x81, 0x00,                 //     Input (Data, Array) Key array(6 bytes)

        0x09, 0x05,                 //     Usage (Vendor Defined)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x26, 0xFF, 0x00,           //     Logical Maximum (255)
        0x75, 0x08,                 //     Report Count (2)
        0x95, 0x02,                 //     Report Size (8 bit)
        0xB1, 0x02,                 //     Feature (Data, Variable, Absolute)
        0xC0,                       // End Collection (Application)
		
        // Report ID 2: Advanced buttons
        0x05, 0x0C,                     // Usage Page (Consumer)
        0x09, 0x01,                     // Usage (Consumer Control)
        0xA1, 0x01,                     // Collection (Application)
        0x85, 0x02,                     //     Report Id (2)
        0x15, 0x00,                     //     Logical minimum (0)
        0x25, 0x01,                     //     Logical maximum (1)
        0x75, 0x01,                     //     Report Size (1)
        0x95, 0x01,                     //     Report Count (1)

        0x09, 0xE9,                     //     Usage (Play/Pause)
        0x81, 0x02,                     //     Input (Data,Value,Relative,Bit Field)
        0x09, 0xEA,                     //     Usage (AL Consumer Control Configuration)
        0x81, 0x02,                     //     Input (Data,Value,Relative,Bit Field)
        0x09, 0xE2,                     //     Usage (Scan Next Track)
        0x81, 0x02,                     //     Input (Data,Value,Relative,Bit Field)
        0x0A, 0x23, 0x02,               //     Usage (Scan Previous Track)
        0x81, 0x02,                     //     Input (Data,Value,Relative,Bit Field)

        0x09, 0x40,                     //     Usage (Volume Down)
        0x81, 0x02,                     //     Input (Data,Value,Relative,Bit Field)
        0x09, 0xB6,                     //     Usage (Volume Up)
        0x81, 0x02,                     //     Input (Data,Value,Relative,Bit Field)
        0x09, 0xB5,                     //     Usage (AC Forward)
        0x81, 0x02,                     //     Input (Data,Value,Relative,Bit Field)
        0x0A, 0x24, 0x02,               //     Usage (AC Back)
        0x81, 0x02,                     //     Input (Data,Value,Relative,Bit Field)
        0xC0,                           // End Collection

        // Report ID 3: Advanced buttons
        0x05, 0x0C,                     // Usage Page (Consumer)
        0x09, 0x01,                     // Usage (Consumer Control)
        0xA1, 0x01,                     // Collection (Application)
        0x85, 0x03,                     //     Report Id (3)
        0x15, 0x00,                     //     Logical minimum (0)
        0x25, 0x01,                     //     Logical maximum (1)
        0x75, 0x01,                     //     Report Size (1)
        0x95, 0x01,                     //     Report Count (1)

        0x09, 0x30,                     //     Usage (Play/Pause)
        0x81, 0x02,                     //     Input (Data,Value,Relative,Bit Field)
        0x09, 0xcd,                     //     Usage (AL Consumer Control Configuration)
        0x81, 0x02,                     //     Input (Data,Value,Relative,Bit Field)
        0x09, 0xCF,                     //     Usage (Scan Next Track)
        0x81, 0x02,                     //     Input (Data,Value,Relative,Bit Field)
        0xC0                            // End Collection	

	};

	hids_init_obj.rep_map.data = report_map;
	hids_init_obj.rep_map.size = sizeof(report_map);

	hids_init_obj.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
	hids_init_obj.info.b_country_code = 0x00;
	hids_init_obj.info.flags = (BT_HIDS_REMOTE_WAKE |
								BT_HIDS_NORMALLY_CONNECTABLE);

	hids_inp_rep =
		&hids_init_obj.inp_rep_group_init.reports[INPUT_REP_KEYS_IDX];
	hids_inp_rep->size = INPUT_REPORT_KEYS_MAX_LEN;
	hids_inp_rep->id = KEYPAD_INPUT_REP_KEYS_REF_ID;
	hids_init_obj.inp_rep_group_init.cnt++;

	hids_outp_rep =
		&hids_init_obj.outp_rep_group_init.reports[OUTPUT_REP_KEYS_IDX];
	hids_outp_rep->size = OUTPUT_REPORT_MAX_LEN;
	hids_outp_rep->id = OUTPUT_REP_KEYS_REF_ID;
	hids_outp_rep->handler = hids_outp_rep_handler;
	hids_init_obj.outp_rep_group_init.cnt++;

	hids_inp_rep =
		&hids_init_obj.inp_rep_group_init.reports[CONSUMER_INPUT_REP_KEYS_IDX];
	hids_inp_rep->size = CONSUMER_INPUT_REPORT_KEYS_MAX_LEN;
	hids_inp_rep->id = CONSUMER_INPUT_REP_KEYS_REF_ID;
	hids_init_obj.inp_rep_group_init.cnt++;

	hids_inp_rep =
		&hids_init_obj.inp_rep_group_init.reports[ADV_CONSUMER_INPUT_REP_KEYS_IDX];
	hids_inp_rep->size = CONSUMER_INPUT_REPORT_KEYS_MAX_LEN;
	hids_inp_rep->id = ADVANCE_CONSUMER_INPUT_REP_KEYS_REF_ID;
	hids_init_obj.inp_rep_group_init.cnt++;

	hids_init_obj.is_kb = true;
	hids_init_obj.boot_kb_outp_rep_handler = hids_boot_kb_outp_rep_handler;
	hids_init_obj.pm_evt_handler = hids_pm_evt_handler;
	
	

	err = bt_hids_init(&hids_obj, &hids_init_obj);
	__ASSERT(err == 0, "HIDS initialization failed\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	int err;

	struct pairing_data_mitm pairing_data;

	pairing_data.conn = bt_conn_ref(conn);
	pairing_data.passkey = passkey;

	err = k_msgq_put(&mitm_queue, &pairing_data, K_NO_WAIT);
	if (err)
	{
		printk("Pairing queue is full. Purge previous data.\n");
	}

	/* In the case of multiple pairing requests, trigger
	 * pairing confirmation which needed user interaction only
	 * once to avoid display information about all devices at
	 * the same time. Passkey confirmation for next devices will
	 * be proccess from queue after handling the earlier ones.
	 */
	if (k_msgq_num_used_get(&mitm_queue) == 1)
	{
		k_work_submit(&pairing_work);
	}
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	struct pairing_data_mitm pairing_data;

	if (k_msgq_peek(&mitm_queue, &pairing_data) != 0)
	{
		return;
	}

	if (pairing_data.conn == conn)
	{
		bt_conn_unref(pairing_data.conn);
		k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT);
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d\n", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed};

/** @brief Function process keyboard state and sends it
 *
 *  @param pstate     The state to be sent
 *  @param boot_mode  Information if boot mode protocol is selected.
 *  @param conn       Connection handler
 *
 *  @return 0 on success or negative error code.
 */
static int keypad_key_report_con_send(const struct keyboard_state *state,
							   bool boot_mode,
							   struct bt_conn *conn)
{
	int err = 0;
	uint8_t *key_data;
	const uint8_t *key_state;
	size_t n;
    uint8_t data[INPUT_REPORT_KEYS_MAX_LEN];

	data[0] = state->ctrl_keys_state;
	data[1] = 0;
	key_data = &data[2];
	key_state = state->keys_state;

	for (n = 0; n < KEY_PRESS_MAX; ++n)
	{
		*key_data++ = *key_state++;
	}

	if (boot_mode)
	{
		err = bt_hids_boot_kb_inp_rep_send(&hids_obj, conn, data,
										   sizeof(data), NULL);
	}
	else
	{
		err = bt_hids_inp_rep_send(&hids_obj, conn,
								   INPUT_REP_KEYS_IDX, data,
								   sizeof(data), NULL);
	}
	return err;
}

//consumer page
static int consumer_key_report_con_send(const struct keyboard_state *state,
							   bool boot_mode,
							   struct bt_conn *conn, uint8_t rep_index)
{
	int err = 0;
	uint8_t data[CONSUMER_INPUT_REPORT_KEYS_MAX_LEN];

	data[0] = state->ctrl_keys_state;

	if (boot_mode)
	{
		err = bt_hids_boot_kb_inp_rep_send(&hids_obj, conn, data,
										   sizeof(data), NULL);
	}
	else
	{
		err = bt_hids_inp_rep_send(&hids_obj, conn,
								   rep_index, &data,
								   sizeof(data), NULL);
	}
	return err;
}

/** @brief Function process and send keyboard state to all active connections
 *
 * Function process global keyboard state and send it to all connected
 * clients.
 *
 * @return 0 on success or negative error code.
 */
static int key_report_send(uint8_t rep_index)
{
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++)
	{
		if (conn_mode[i].conn)
		{
			int err;

			if(!rep_index) {

			err = keypad_key_report_con_send(&hid_keyboard_state,
									  conn_mode[i].in_boot_mode,
									  conn_mode[i].conn);
			}else {
			
			err = consumer_key_report_con_send(&hid_keyboard_state,
									  conn_mode[i].in_boot_mode,
									  conn_mode[i].conn,rep_index);

			}
			if (err)
			{
				printk("Key report send error: %d\n", err);
				return err;
			}
		}
	}
	return 0;
}

/** @brief Change key code to ctrl code mask
 *
 *  Function changes the key code to the mask in the control code
 *  field inside the raport.
 *  Returns 0 if key code is not a control key.
 *
 *  @param key Key code
 *
 *  @return Mask of the control key or 0.
 */
static uint8_t button_ctrl_code(uint8_t key)
{
	if (KEY_CTRL_CODE_MIN <= key && key <= KEY_CTRL_CODE_MAX)
	{
		return (uint8_t)(1U << (key - KEY_CTRL_CODE_MIN));
	}
	return 0;
}

static int hid_kbd_state_key_set(uint8_t key)
{
	uint8_t ctrl_mask = button_ctrl_code(key);

	if (ctrl_mask)
	{
		hid_keyboard_state.ctrl_keys_state |= ctrl_mask;
		return 0;
	}
	for (size_t i = 0; i < KEY_PRESS_MAX; ++i)
	{
		if (hid_keyboard_state.keys_state[i] == 0)
		{
			hid_keyboard_state.keys_state[i] = key;
			return 0;
		}
	}
	
	/* All slots busy */
	return -EBUSY;
}
static int hid_kbd_con_state_key_set(uint8_t key)
{
	uint8_t ctrl_mask = button_ctrl_code(key);

	if (ctrl_mask)
	{
		hid_keyboard_state.ctrl_keys_state |= ctrl_mask;
		return 0;
	}
		hid_keyboard_state.ctrl_keys_state = key;
		return 0;
	/* All slots busy */
	return -EBUSY;
}

static int hid_kbd_state_key_clear(uint8_t key)
{
	uint8_t ctrl_mask = button_ctrl_code(key);

	if (ctrl_mask)
	{
		hid_keyboard_state.ctrl_keys_state &= ~ctrl_mask;
		return 0;
	}
	for (size_t i = 0; i < KEY_PRESS_MAX; ++i)
	{
		if (hid_keyboard_state.keys_state[i] == key)
		{
			hid_keyboard_state.keys_state[i] = 0;
			return 0;
		}
	}
	/* Key not found */
	return -EINVAL;
}



/** @brief Press a button and send report
 *
 *  @note Functions to manipulate hid state are not reentrant
 *  @param keys
 *  @param cnt
 *
 *  @return 0 on success or negative error code.
 */
static int hid_buttons_press(const uint8_t *keys, size_t cnt, uint8_t rep_index)
{
	if (rep_index == 0){
	   while (cnt--)
	{
		int err;

		err = hid_kbd_state_key_set(*keys++);
		if (err)
		{
			printk("Cannot set selected key.\n");
			return err;
		}
	}
	}
	else{
	  while (cnt--)
	{
		int err;

		err = hid_kbd_con_state_key_set(*keys++);
		if (err)
		{
			printk("Cannot clear selected key.\n");
			return err;
		}
	}
	}
    return key_report_send(rep_index);
}

/** @brief Release the button and send report
 *
 *  @note Functions to manipulate hid state are not reentrant
 *  @param keys
 *  @param cnt
 *
 *  @return 0 on success or negative error code.
 */
static int hid_buttons_release(const uint8_t *keys, size_t cnt, uint8_t rep_index)
{
	while (cnt--)
	{
		int err;

		err = hid_kbd_state_key_clear(*keys++);
		if (err)
		{
			printk("Cannot clear selected key.\n");
			return err;
		}
	}

	return key_report_send(rep_index);
}

static void keypad_button_press(const uint8_t *keys, bool down, uint8_t rep_index)
{
	if (down)
	{
		hid_buttons_press(keys , 1, rep_index);
	}
	else
	{
		hid_buttons_release(keys, 1, rep_index);
	}
}
static void consumer_button_press(const uint8_t *keys, bool down, uint8_t rep_index) 
{
	if (down)
	{
		hid_buttons_press(keys, 1, rep_index);
	}
	else
	{
		hid_buttons_press(RELEASED_KEY, 1, rep_index);
	}
}

static void num_comp_reply(bool accept)
{
	struct pairing_data_mitm pairing_data;
	struct bt_conn *conn;

	if (k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT) != 0)
	{
		return;
	}

	conn = pairing_data.conn;

	if (accept)
	{
		bt_conn_auth_passkey_confirm(conn);
		printk("Numeric Match, conn %p\n", conn);
	}
	else
	{
		bt_conn_auth_cancel(conn);
		printk("Numeric Reject, conn %p\n", conn);
	}

	bt_conn_unref(pairing_data.conn);

	if (k_msgq_num_used_get(&mitm_queue))
	{
		k_work_submit(&pairing_work);
	}
}

static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level)
	{
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}



void keyscan_callback(const struct device *dev, uint32_t row,
					  uint32_t column, bool pressed)
{
	printk("row %d column %d pressed %d\n",row,column,pressed);
	static bool pairing_button_pressed;
	uint8_t keycode = 0;
    const uint8_t * key_code = &keycode;
	/** Report index defined in the HIDS Report Map. */
	uint8_t report_idx = 0xff;
	
	if (k_msgq_num_used_get(&mitm_queue))
	{
		if ((row == 4) && (column == 1) & KEY_PAIRING_ACCEPT)
		{
		  	pairing_button_pressed = true;
			num_comp_reply(true);
			return;
		}
		if ((row == 4) && (column == 2) & KEY_PAIRING_REJECT)
		{
			pairing_button_pressed = true;
			num_comp_reply(false);
			return;
		}
	}

	if(((row == 4) && (column == 0)) || ((row == 4) && (column == 1))  || ((row == 4) && (column == 2))
	   || ((row == 5) && (column == 2))  || ((row == 5) && (column == 0))) 
	{

	report_idx = INPUT_REP_KEYS_IDX;

    if ((row == 4) && (column == 0))//sw2
	{
		keycode = ARROW_UP;
	}

	else if ((row == 5) && (column == 0))//sw6
	{
		keycode = ARROW_RIGHT;
	}

	else if ((row == 5) && (column == 2))//sw8
	{
		keycode = ARROW_DOWN;
	}

	else if ((row == 4) && (column == 1))//sw3
	{
		keycode = ARROW_LEFT;
	}
	
	else if ((row == 4) && (column == 2))//sw4
	{
		keycode = ARROW_ENTER;
	}

	else 
	{
        //nothing
	}

	keypad_button_press(key_code, pressed, report_idx);

	}

	else{

	if(((row == 4) && (column == 3)) || ((row == 5) && (column == 1)) || ((row == 6) && (column == 0)) || ((row == 6) && (column == 1)) || ((row == 6) && (column == 3)) || ((row == 7) && (column == 0)) || ((row == 7) && (column == 1))
		|| ((row == 7) && (column == 2) )) 
	{

	report_idx = CONSUMER_INPUT_REP_KEYS_IDX;
	
	if ((row == 4) && (column == 3))//sw5
	{
		keycode = MUTE;   
	}
	else if ((row == 5) && (column == 1))//sw7
	{
		keycode = VOLUME_UP;   
	}
	else if ((row == 6) && (column == 0))//sw10
	{
		keycode = MENU;   
	}
	else if ((row == 6) && (column == 1))//sw11
	{
		keycode = HOME;   
	}
	else if ((row == 6) && (column == 3))//sw13
	{
		keycode = NEXT_TRACK;   
	}
	else if ((row == 7) && (column == 0))//sw14
	{
		keycode = VOLUME_DOWN;   
	}
	else if ((row == 7) && (column == 1))//sw15
	{
		keycode = PREV_TRACK;   
	}
	else if ((row == 7) && (column == 2))//sw16
	{
		keycode = BACK;   
	}

   }

	else if(((row == 5) && (column == 3)) || ((row == 6) && (column == 2)) || ((row == 7) && (column == 3)))
	{
	
	report_idx = ADV_CONSUMER_INPUT_REP_KEYS_IDX;

    if ((row == 5) && (column == 3))//sw9
	{
		keycode = POWER;   
	}
	else if ((row == 6) && (column == 2))//sw12
	{
		keycode = PLAY_PAUSE;   
	}
	else if ((row == 7) && (column == 3))//sw17
	{	
		keycode = VOICE_CMD;   
	}

	}

	else {
		//nothing 
	}

	consumer_button_press(key_code, pressed, report_idx);

}

}

int main(void)
{
	int err;
	int blink_status = 0;

	if (!device_is_ready(i2c_node_2.bus))
	{
		printk("I2C bus %s is not ready! \n\r", i2c_node_2.bus->name);
	}

	if (!device_is_ready(irq.port))
	{
		printf("pin is not ready\n");
	}

	err = kscan_config(dev_2, keyscan_callback);
	if (err)
	{
		printk("Failed to add keyscan callback err %d\n", err);
	}

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err)
	{
		printk("Failed to register authorization callbacks.\n");
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err)
	{
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

	hid_init();

	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized successfully\n");

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	advertising_start();

	k_work_init(&pairing_work, pairing_process);

	for (;;)
	{
		if (is_adv)
		{
			dk_set_led(ADV_STATUS_LED, (++blink_status) % 2);
		}
		else
		{
			dk_set_led_off(ADV_STATUS_LED);
		}

		k_sleep(K_MSEC(ADV_LED_BLINK_INTERVAL));

		/* Battery level simulation */
		// bas_notify();
	}
}
