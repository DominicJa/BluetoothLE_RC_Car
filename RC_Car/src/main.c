/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include "uart_async_adapter.h"

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>

#include <zephyr/logging/log.h>

/* includes from Robot Project*/
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

/* Include header for nrfx drivers */
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/***** Motor Stuff *****/

/*New! My attempt at retrieveing device structure for gpio pins*/
//static const struct gpio_dt_spec M1_Pin = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(user_dbg_pin), gpios, {0});
static const struct gpio_dt_spec M1_Pin = GPIO_DT_SPEC_GET_BY_IDX_OR(DT_NODELABEL(user_dbg_pin), gpios, 0, {0}); // Test to see if this works just like before
static const struct gpio_dt_spec M2_Pin = GPIO_DT_SPEC_GET_BY_IDX_OR(DT_NODELABEL(user_dbg_pin), gpios, 1, {0}); // Test to see if this stores to second gpio info
static const struct gpio_dt_spec M3_Pin = GPIO_DT_SPEC_GET_BY_IDX_OR(DT_NODELABEL(user_dbg_pin), gpios, 2, {0});
static const struct gpio_dt_spec M4_Pin = GPIO_DT_SPEC_GET_BY_IDX_OR(DT_NODELABEL(user_dbg_pin), gpios, 3, {0});

/*New! My attempt at retrieveing the device sturcture for the dc motor*/
#define DC_MOTOR    DT_NODELABEL(motor)
//Should probably switch from PWM_DT_SPEC_GET_BY_IDX to PWM_DT_SPEC_GET_BY_IDX_OR
//Because its the same thing but gives the option to set a default value
static const struct pwm_dt_spec pwm_motor = PWM_DT_SPEC_GET_BY_IDX(DC_MOTOR, 0); // Test to see if this works just like before (It does)
static const struct pwm_dt_spec pwm_motor2 = PWM_DT_SPEC_GET_BY_IDX(DC_MOTOR, 1); // Test to see if this stores the second channel info
static const struct pwm_dt_spec pwm_motor3 = PWM_DT_SPEC_GET_BY_IDX(DC_MOTOR, 2);
static const struct pwm_dt_spec pwm_motor4 = PWM_DT_SPEC_GET_BY_IDX(DC_MOTOR, 3);

/*New! My attemp at uisng DT_PROP() to obtain the minimum and maximum duty cycle for the dc motor*/
#define PWM_MOTOR_MIN_DUTY_CYCLE    DT_PROP(DC_MOTOR, min_pulse)
#define PWM_MOTOR_MAX_DUTY_CYCLE    DT_PROP(DC_MOTOR, max_pulse)

/* Some common PWM duty cycle values */
#define PWM_50P_DUTY_CYCLE     500000
#define PWM_80P_DUTY_CYCLE     800000

/*New! My attempt at setting the PWM motor period*/
#define PWM_DC_PERIOD   PWM_MSEC(1)

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif

/*NEW! My attempt at creating a function to set the pwm for the p0.06 pin that will output to the dc motor*/
int set_motor_speed(uint32_t duty_cycle_ns, const struct pwm_dt_spec *pwm_motor)
{
    int err;

    err = pwm_set_dt(pwm_motor, PWM_DC_PERIOD, duty_cycle_ns);
    if (err){
        LOG_ERR("pwm_set_dt_returned %d", err);
    }
    return err;
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data[0]);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
					   data[0]);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
		buf->len += evt->data.rx.len;

		if (disable_req) {
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data[0]);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t,
				   data);

		uart_tx(uart, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		err = usb_enable(NULL);
		if (err && (err != -EALREADY)) {
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);


	if (IS_ENABLED(CONFIG_BT_NUS_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}

	return err;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr,
			level, err);
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	uint16_t average_AN0 = 1700;
	uint16_t average_AN1 = 1700;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	//New code for extracting the two uint16_t values from the uint8_t data array 
	if(len == 4){

	// Reconstruct the first uint16_t value from the first two bytes
	average_AN0 = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

	// Reconstruct the second uint16_t value from the next two bytes
	average_AN1 = (uint16_t)data[2] | ((uint16_t)data[3] << 8);

	// Print the reconstructed values
	
	printf("Received uint16_t values:\n");
	printf("Value 1: %u\n", average_AN0);
	printf("Value 2: %u\n", average_AN1);
	}
	else{
		printk("Error: Received data length (%d) is too short to contain two uint16_t values.\n", len);
		return;
	}

	/*
            * pwm_motor & M1_pin are for the front left motor
            * pwm2_motor & M2_pin are for the front right motor
            * pwm3_motor & M3_pin are for the back left motor
            * pwm4_motor & M4_pin are for the back right motor
            */

            /* x joystick position also determines PWM and GPIO changes but as the outer if statement */

            /* This is done if the x joystick is in the right position */
            if(average_AN1 > 1800){
                if(average_AN0 > 3300){ // Move the motors backwards and to the right at full speed
                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 0);
                }
                else if((average_AN0 <= 3300) && (average_AN0 > 1800)){ // Move the motos backwards and to the right at 60% speed
                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 0);

                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 0);
                }
                else if((average_AN0 <= 1800) && (average_AN0 > 1650)){ // Don't move motors (even though x joystick is positioned right)
                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 0);
                }
                else if((average_AN0 <= 1650) && (average_AN0 > 40)){ // Move the motors forward and to the right at 60% speed
                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 0);

                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 0);
                }
                else if(average_AN0 <= 40){ // Move the motors forward at full speed
                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 0);
                }
                else{
                LOG_ERR("ERROR: Average_AN0 is equal to %lld", average_AN0);
                }
            }
            else if((average_AN1 <= 1800) && (average_AN1 > 1600)){ /* This is done if the x joystick is in the middle position */
                /* Change PWM and GPIO according to y joystick position */
                if(average_AN0 > 3300){ // Move the motor backwards at full speed
                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 1);
                }
                else if((average_AN0 <= 3300) && (average_AN0 > 1800)){ // Move the motor backwards at 60% speed
                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 0);

                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 1);

                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 0);

                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 1);
                }
                else if((average_AN0 <= 1800) && (average_AN0 > 1650)){ // Don't move motors
                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 0);
                }
                else if((average_AN0 <= 1650) && (average_AN0 > 40)){ // Move the motors forward at 60% speed
                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 1);

                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 0);

                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 1);

                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 0);
                }
                else if(average_AN0 <= 40){ // Move the motors forward at full speed
                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 0);
                }
                else{
                    LOG_ERR("ERROR: Average_AN0 is equal to %lld", average_AN0);
                }
            }
            else if(average_AN1 <= 1600){ // Do this when x joystick is in the left position
                if(average_AN0 > 3300){ // Move the motor backwards and to the left at full speed
                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 1);
                }
                else if((average_AN0 <= 3300) && (average_AN0 > 1800)){ // Move the motor backwards and to the left at 60% speed
                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 1);

                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 0);

                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 1);
                }
                else if((average_AN0 <= 1800) && (average_AN0 > 1650)){ // Don't move motors (even though the x joystick is in the left position)
                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 0);
                }
                else if((average_AN0 <= 1650) && (average_AN0 > 40)){ // Move the motors forward and to the left at 60% speed
                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 1);

                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 0);

                    set_motor_speed(PWM_50P_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 0);
                }
                else if(average_AN0 <= 40){ // Move the motors forward and to the left at full speed
                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor);
                    gpio_pin_set_dt(&M1_Pin, 1);

                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor2);
                    gpio_pin_set_dt(&M2_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MIN_DUTY_CYCLE, &pwm_motor3);
                    gpio_pin_set_dt(&M3_Pin, 0);

                    set_motor_speed(PWM_MOTOR_MAX_DUTY_CYCLE, &pwm_motor4);
                    gpio_pin_set_dt(&M4_Pin, 0);
                }
                else{
                    LOG_ERR("ERROR: Average_AN0 is equal to %lld", average_AN0);
                }
            }
            else{
                LOG_ERR("ERROR: Average_AN1 is equal to %lld", average_AN1);
            }

}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

int main(void)
{
	int blink_status = 0;
	int err = 0;

	configure_gpio();

	/*err = uart_init();
	if (err) {
		error();
	}*/

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			printk("Failed to register authorization callbacks.\n");
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			printk("Failed to register authorization info callbacks.\n");
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}

	/*New. Added to check if the dc motor device is ready */
    if(!pwm_is_ready_dt(&pwm_motor)){
        LOG_ERR("Error: PWM device %s is not ready", pwm_motor.dev->name);
        return 0;
    }

    /*New.  Added to check if the dc motor device is ready (second channel) */
    if(!pwm_is_ready_dt(&pwm_motor2)){
        LOG_ERR("Error: PWM device %s is not ready", pwm_motor2.dev->name);
        return 0;
    }

    if(!pwm_is_ready_dt(&pwm_motor3)){
        LOG_ERR("Error: PWM device %s is not ready", pwm_motor3.dev->name);
        return 0;
    }

    if(!pwm_is_ready_dt(&pwm_motor4)){
        LOG_ERR("Error: PWM device %s is not ready", pwm_motor4.dev->name);
        return 0;
    }

    /*New. Added to checkifthe GPIO device is ready */
    if(!gpio_is_ready_dt(&M1_Pin)){
        LOG_ERR("Error: GPIO pin %d is not ready", M1_Pin.pin);
        return 0;
    }

    /*New. Added to check if the GPIO pin 2 device is ready */
    if(!gpio_is_ready_dt(&M2_Pin)){
        LOG_ERR("Error: GPIO pin %d is not ready", M2_Pin.pin);
        return 0;
    }

    if(!gpio_is_ready_dt(&M3_Pin)){
        LOG_ERR("Error: GPIO pin %d is not ready", M3_Pin.pin);
        return 0;
    }

    if(!gpio_is_ready_dt(&M4_Pin)){
        LOG_ERR("Error: GPIO pin %d is not ready", M4_Pin.pin);
        return 0;
    }

    /*New. Used to configure a pin */
    gpio_pin_configure_dt(&M1_Pin, GPIO_OUTPUT);
    gpio_pin_configure_dt(&M2_Pin, GPIO_OUTPUT);
    gpio_pin_configure_dt(&M3_Pin, GPIO_OUTPUT);
    gpio_pin_configure_dt(&M4_Pin, GPIO_OUTPUT);

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		if (bt_nus_send(NULL, buf->data, buf->len)) {
			LOG_WRN("Failed to send data over BLE connection");
		}

		k_free(buf);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);
