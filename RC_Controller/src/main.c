/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Service Client sample
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <zephyr/settings/settings.h>

#include <zephyr/drivers/uart.h>

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

#define LOG_MODULE_NAME central_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/**** ADC Stuff ****/

/* Define the SAADC sample interval in microseconds */
#define SAADC_SAMPLE_INTERVAL_US 90 // Was 50 (right now it was 100)

/* Define the buffer size for the SAADC */
#define SAADC_BUFFER_SIZE   8000 // Was 8000 (right now it was 3500)(10000)

/* Declaring an instance of nrfx_timer for TIMER2. */
const nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(2);

/* Declare the buffers for the SAADC */
static int16_t saadc_sample_buffer[2][SAADC_BUFFER_SIZE];

/* Declare variable used to keep track of which buffer was last assigned to the SAADC driver */
static uint32_t saadc_current_buffer = 0;

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
#define PWM_60P_DUTY_CYCLE     600000
#define PWM_80P_DUTY_CYCLE     800000

/*New! My attempt at setting the PWM motor period*/
#define PWM_DC_PERIOD   PWM_MSEC(1)

/* UART payload buffer element size. */
#define UART_BUF_SIZE 20

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define NUS_WRITE_TIMEOUT K_MSEC(150)
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_RX_TIMEOUT 50000 /* Wait for RX complete event time in microseconds. */

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static struct k_work_delayable uart_work;

K_SEM_DEFINE(nus_write_sem, 0, 1);

struct uart_data_t {
	void *fifo_reserved;
	uint8_t  data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static struct bt_conn *default_conn;
static struct bt_nus_client nus_client;

int set_motor_speed(uint32_t duty_cycle_ns, const struct pwm_dt_spec *pwm_motor)
{
	int err;

	err = pwm_set_dt(pwm_motor, PWM_DC_PERIOD, duty_cycle_ns);
    if (err){
        LOG_ERR("pwm_set_dt_returned %d", err);
    }
    return err;
}

static void configure_timer(void)
{
    nrfx_err_t err;

    /* Declaring timer config and intialize nrfx_timer instance. */
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000);
    err = nrfx_timer_init(&timer_instance, &timer_config, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_timer_init error: %08x", err);
        return;
    }

    /* Set compare channel 0 to generate event every SAADC_SAMPLE_INTERVAL_US. */
    uint32_t timer_ticks = nrfx_timer_us_to_ticks(&timer_instance, SAADC_SAMPLE_INTERVAL_US);
    nrfx_timer_extended_compare(&timer_instance, NRF_TIMER_CC_CHANNEL0, timer_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

}

static void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    nrfx_err_t err;
    switch (p_event->type)
    {
        case NRFX_SAADC_EVT_READY:
        
           /* Buffer is ready, timer (and sampling) can be started. */
            nrfx_timer_enable(&timer_instance);
            break;                        
            
        case NRFX_SAADC_EVT_BUF_REQ:
        
            /* Set up the next available buffer. Alternate between buffer 0 and 1 */
            err = nrfx_saadc_buffer_set(saadc_sample_buffer[(saadc_current_buffer++)%2], SAADC_BUFFER_SIZE);
            //err = nrfx_saadc_buffer_set(saadc_sample_buffer[((saadc_current_buffer == 0 )? saadc_current_buffer++ : 0)], SAADC_BUFFER_SIZE);
            if (err != NRFX_SUCCESS) {
                LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
                return;
            }
            break;

        case NRFX_SAADC_EVT_DONE:

            /* Buffer has been filled. Do something with the data and proceed */
            int64_t average_AN0 = 0;
            int16_t max_AN0 = INT16_MIN;
            int16_t min_AN0 = INT16_MAX;
            int16_t current_value;

            int64_t average_AN1 = 0;
            int16_t max_AN1 = INT16_MIN;
            int16_t min_AN1 = INT16_MAX; 

            for(int i=0; i < p_event->data.done.size; i++){
                current_value = ((int16_t *)(p_event->data.done.p_buffer))[i];
                average_AN0 += current_value;
                if(current_value > max_AN0){
                    max_AN0 = current_value;
                }
                if(current_value < min_AN0){
                    min_AN0 = current_value;
                }

                i++;
                current_value = ((int16_t *)(p_event->data.done.p_buffer))[i];
                average_AN1 += current_value;
                if(current_value > max_AN1){
                    max_AN1 = current_value;
                }
                if(current_value < min_AN1){
                    min_AN1 = current_value;
                }
            }
            //average = average/p_event->data.done.size;
            average_AN0 = average_AN0 / (p_event->data.done.size / 2);

            average_AN1 = average_AN1 / (p_event->data.done.size / 2);

            //LOG_INF("SAADC buffer at 0x%x filled with %d samples", (uint32_t)p_event->data.done.p_buffer, p_event->data.done.size);
            //LOG_INF("AVG=%d, MIN=%d, MAX=%d", (int16_t)average, min, max);

			if(average_AN0 < 0){
				average_AN0 = 0;
			}

			if(average_AN1 < 0){
				average_AN1 = 0;
			}

            //LOG_INF("AVG=%d, MIN=%d, MAX=%d", (int16_t)average_AN0, min_AN0, max_AN0);
			printk("AVG=%d, MIN=%d, MAX=%d\n", (int16_t)average_AN0, min_AN0, max_AN0);

            //LOG_INF("AVG=%d, MIN=%d, MAX=%d", (int16_t)average_AN1, min_AN1, max_AN1);
			printk("AVG=%d, MIN=%d, MAX=%d\n", (int16_t)average_AN1, min_AN1, max_AN1);


			struct uart_data_t *buf;
			buf = k_malloc(sizeof(*buf));
			if(buf){
				buf->len = 0;
			}
			else{
				printk("Failed to allocate memory for buf in button_handler\n");
				return;
			}

			// Store the first uint16_t value into data[0] and data[1]
			buf->data[0] = (uint8_t)(average_AN0 & 0xFF);
			buf->data[1] = (uint8_t)((average_AN0 >> 8) & 0xFF);

			// Store the second uint16_t value into data[2] and data[3]
			buf->data[2] = (uint8_t)(average_AN1 & 0xFF);
			buf->data[3] = (uint8_t)((average_AN1 >> 8) & 0xFF);

			// Update the Length to reflect the number of bytes used 
			buf->len = 4; // Two uint16_t values = 4 bytes
			k_fifo_put(&fifo_uart_rx_data, buf);


            

            break;
        default:
            LOG_INF("Unhandled SAADC evt %d", p_event->type);
            break;
    }
}

static nrfx_saadc_channel_t m_multiple_channels[] =
{
    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 0),
    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN1, 1)
};

static void configure_saadc(void)
{
    nrfx_err_t err;

    /* Connect ADC interrupt to nrfx interrupt handler */
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
                DT_IRQ(DT_NODELABEL(adc), priority),
                nrfx_isr, nrfx_saadc_irq_handler, 0);

    
    /* Initialize the nrfx_SAADC driver */
    err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_init error: %08x", err);
        return;
    }

    
    /* Declare the struct to hold the configuration for the SAADC channel used to sample the battery voltage */
    //nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 0);
    
    #define NRFX_SAADC_CHANNEL_COUNT 2

    /* Change gain config in default config and apply channel configuration */
    //channel.channel_config.gain = NRF_SAADC_GAIN1_6;
    m_multiple_channels[0].channel_config.gain = NRF_SAADC_GAIN1_6;
    m_multiple_channels[1].channel_config.gain = NRF_SAADC_GAIN1_6;

    err = nrfx_saadc_channels_config(m_multiple_channels, NRFX_SAADC_CHANNEL_COUNT);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_channels_config error: %08x", err);
        return;
    }

    uint32_t channels_mask = nrfx_saadc_channels_configured_get();

    /* Configure channel 0 in advanced mode with event handler (non-blocking mode) */
    nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    err = nrfx_saadc_advanced_mode_set(channels_mask,
                                        NRF_SAADC_RESOLUTION_12BIT,
                                        &saadc_adv_config,
                                        saadc_event_handler);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_advanced_mode_set error: %08x", err);
        return;
    }
                                            
    /* Configure two buffers to make use of double-buffering feature of SAADC */
    err = nrfx_saadc_buffer_set(saadc_sample_buffer[0], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
        return;
    }
    err = nrfx_saadc_buffer_set(saadc_sample_buffer[1], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
        return;
    }

    /* Trigger the SAADC. This will not start sampling, but will prepare buffer for sampling triggered through PPI */
    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_mode_trigger error: %08x", err);
        return;
    }

}

static void configure_ppi(void)
{
    nrfx_err_t err;
    /* Declare variables used to hold the (D)PPI channel number */
    uint8_t m_saadc_sample_ppi_channel;
    uint8_t m_saadc_start_ppi_channel;

    /* Trigger task sample from timer */
    err = nrfx_gppi_channel_alloc(&m_saadc_sample_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gppi_channel_alloc error: %08x", err);
        return;
    }

    err = nrfx_gppi_channel_alloc(&m_saadc_start_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gppi_channel_alloc error: %08x", err);
        return;
    }

    /* Trigger task sample from timer */
    nrfx_gppi_channel_endpoints_setup(m_saadc_sample_ppi_channel, 
                                      nrfx_timer_compare_event_address_get(&timer_instance, NRF_TIMER_CC_CHANNEL0),
                                      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

    /* Trigger task start from end event */
    nrfx_gppi_channel_endpoints_setup(m_saadc_start_ppi_channel, 
                                      nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
                                      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

    /* Enable both (D)PPI channels */ 
    nrfx_gppi_channels_enable(BIT(m_saadc_sample_ppi_channel));
    nrfx_gppi_channels_enable(BIT(m_saadc_start_ppi_channel));
}

static void ble_data_sent(struct bt_nus_client *nus, uint8_t err,
					const uint8_t *const data, uint16_t len)
{
	ARG_UNUSED(nus);

	k_sem_give(&nus_write_sem);

	if (err) {
		LOG_WRN("ATT error code: 0x%02X", err);
	}
}

static uint8_t ble_data_received(struct bt_nus_client *nus,
						const uint8_t *data, uint16_t len)
{
	ARG_UNUSED(nus);

	int err;

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return BT_GATT_ITER_CONTINUE;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}

	return BT_GATT_ITER_CONTINUE;
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
			buf = CONTAINER_OF(evt->data.tx.buf,
					   struct uart_data_t,
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
			       UART_RX_TIMEOUT);

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
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
				   data[0]);

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

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_RX_TIMEOUT);
}

static int uart_init(void)
{
	int err;
	struct uart_data_t *rx;

	if (!device_is_ready(uart)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		return err;
	}

	return uart_rx_enable(uart, rx->data, sizeof(rx->data),
			      UART_RX_TIMEOUT);
}

static void discovery_complete(struct bt_gatt_dm *dm,
			       void *context)
{
	struct bt_nus_client *nus = context;
	LOG_INF("Service discovery completed");

	bt_gatt_dm_data_print(dm);

	bt_nus_handles_assign(dm, nus);
	bt_nus_subscribe_receive(nus);

	bt_gatt_dm_data_release(dm);
}

static void discovery_service_not_found(struct bt_conn *conn,
					void *context)
{
	LOG_INF("Service not found");
}

static void discovery_error(struct bt_conn *conn,
			    int err,
			    void *context)
{
	LOG_WRN("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found       = discovery_error,
};

static void gatt_discover(struct bt_conn *conn)
{
	int err;

	if (conn != default_conn) {
		return;
	}

	err = bt_gatt_dm_start(conn,
			       BT_UUID_NUS_SERVICE,
			       &discovery_cb,
			       &nus_client);
	if (err) {
		LOG_ERR("could not start the discovery procedure, error "
			"code: %d", err);
	}
}

static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) {
		LOG_INF("MTU exchange done");
	} else {
		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		LOG_INF("Failed to connect to %s (%d)", addr, conn_err);

		if (default_conn == conn) {
			bt_conn_unref(default_conn);
			default_conn = NULL;

			err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
			if (err) {
				LOG_ERR("Scanning failed to start (err %d)",
					err);
			}
		}

		return;
	}

	LOG_INF("Connected: %s", addr);

	static struct bt_gatt_exchange_params exchange_params;

	exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_WRN("MTU exchange failed (err %d)", err);
	}

	err = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (err) {
		LOG_WRN("Failed to set security: %d", err);

		gatt_discover(conn);
	}

	err = bt_scan_stop();
	if ((!err) && (err != -EALREADY)) {
		LOG_ERR("Stop LE scan failed (err %d)", err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (default_conn != conn) {
		return;
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)",
			err);
	}
}

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

	gatt_discover(conn);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed
};

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	LOG_INF("Filters matched. Address: %s connectable: %d",
		addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_WRN("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	default_conn = bt_conn_ref(conn);
}

static int nus_client_init(void)
{
	int err;
	struct bt_nus_client_init_param init = {
		.cb = {
			.received = ble_data_received,
			.sent = ble_data_sent,
		}
	};

	err = bt_nus_client_init(&nus_client, &init);
	if (err) {
		LOG_ERR("NUS Client initialization failed (err %d)", err);
		return err;
	}

	LOG_INF("NUS Client module initialized");
	return err;
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
		scan_connecting_error, scan_connecting);

static int scan_init(void)
{
	int err;
	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1,
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_NUS_SERVICE);
	if (err) {
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		return err;
	}

	LOG_INF("Scan module initialized");
	return err;
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

	LOG_WRN("Pairing failed conn: %s, reason %d", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	if(button_state & has_changed & BIT(0)){

		// Create and allocate a uart_data_t buffer
		/*struct uart_data_t *buf;
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			printk("Failed to allocate memory for buf in button_handler\n");
			return;
		}

		// Fill the buffer with "Hello, World" and a carriage return
		const char *message = "Hello, World";
		size_t message_len = strlen(message);

		// Ensure the message fits into the buffer
		if(message_len + 1 > UART_BUF_SIZE){
			printk("Message is too large for the buffer.\n");
			free(buf);
			return;
		}

		strncpy((char *)buf->data, message, UART_BUF_SIZE -1);
		buf->data[message_len] = '\r';
		buf->len = message_len + 1;

		k_fifo_put(&fifo_uart_rx_data, buf);*/
		//free(buf);
		//int err;
		//struct data_to_send_out *data_to_send;
		//data_to_send->data = 1;

		//err = bt_nus_client_send(&nus_client, data_to_send->data, sizeof(data_to_send->data));

		//New program to try to send over two uint16_t values over bluetooth le
		struct uart_data_t *buf;
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			printk("Failed to allocate memory for buf in button_handler\n");
			return;
		}

		// Two 16-bit unsigned intergers to store in the data array
		uint16_t value1 = 12345;
		uint16_t value2 = 54321;

		// Store the first uint16_t value into data[0] and data[1]
		buf->data[0] = (uint8_t)(value1 & 0xFF);
		buf->data[1] = (uint8_t)((value1 >> 8) & 0xFF);

		// Store the second uint16_t value into data[2] and data[3]
		buf->data[2] = (uint8_t)(value2 & 0xFF);
		buf->data[3] = (uint8_t)((value2 >> 8) & 0xFF);

		// Update the Length to reflect the number of bytes used 
		buf->len = 4; // Two uint16_t values = 4 bytes
		k_fifo_put(&fifo_uart_rx_data, buf);

	}
}

int main(void)
{
	int err;

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization callbacks.");
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}
	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/*err = uart_init();
	if (err != 0) {
		LOG_ERR("uart_init failed (err %d)", err);
		return 0;
	}*/

	err = scan_init();
	if (err != 0) {
		LOG_ERR("scan_init failed (err %d)", err);
		return 0;
	}

	err = nus_client_init();
	if (err != 0) {
		LOG_ERR("nus_client_init failed (err %d)", err);
		return 0;
	}

	err = dk_buttons_init(button_handler);
	if (err) {
		printk("Failed to initialize buttons (err %d)\n", err);
		return 0;
	}

	printk("Starting Bluetooth Central UART example\n");

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return 0;
	}

	LOG_INF("Scanning successfully started");

	if(!pwm_is_ready_dt(&pwm_motor)){
        LOG_ERR("Error: PWM device %s is not ready", pwm_motor.dev->name);
        return 0;
    }

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

	if(!gpio_is_ready_dt(&M1_Pin)){
        LOG_ERR("Error: GPIO pin %d is not ready", M1_Pin.pin);
        return 0;
    }

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

	k_sleep(K_MSEC(2000)); // Delay to give the controller time to connect to RC car.

	/*New. Used to configure a pin */
    gpio_pin_configure_dt(&M1_Pin, GPIO_OUTPUT);
    gpio_pin_configure_dt(&M2_Pin, GPIO_OUTPUT);
    gpio_pin_configure_dt(&M3_Pin, GPIO_OUTPUT);
    gpio_pin_configure_dt(&M4_Pin, GPIO_OUTPUT);

    configure_timer();
    configure_saadc();  
    configure_ppi();

	for (;;) {
		/* Wait indefinitely for data to be sent over Bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		err = bt_nus_client_send(&nus_client, buf->data, buf->len);
		if (err) {
			LOG_WRN("Failed to send data over BLE connection"
				"(err %d)", err);
		}

		err = k_sem_take(&nus_write_sem, NUS_WRITE_TIMEOUT);
		if (err) {
			LOG_WRN("NUS send timeout");
		}

		k_free(buf);
	}
}
