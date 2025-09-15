#include "ble.h"
#include <zephyr/types.h>
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/gpio.h>

#include <zephyr/sys/ring_buffer.h>

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart20));

uint8_t request_scan[2] = {0xA5, 0x20};
uint8_t request_stop[2] = {0xA5, 0x25};

const struct uart_config uart_cfg = {
	.baudrate = 460800,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static uint8_t ring_buf_data[RING_BUFF_SIZE];
struct ring_buf uart_ring_buf;

static void bt_repeat_work_handler(struct k_work *work);

static K_WORK_DEFINE(repeat_uart_on_bt, bt_repeat_work_handler);

static void bt_repeat_work_handler(struct k_work *work) {
	k_sem_take(&ble_init_ok, K_FOREVER);
	
	static uint8_t data[BT_TX_PKG_SIZE];
	static uint16_t len;

	static uint8_t peek_buf[2];

	if (ring_buf_size_get(&uart_ring_buf) > 1) {
		ring_buf_peek(&uart_ring_buf, peek_buf, sizeof(peek_buf));
		if ((peek_buf[0] == 0xA5) && (peek_buf[1] == 0x5A)) {
			// Discard descriptor
			len = ring_buf_get(&uart_ring_buf, data, 7);
			if ((len == 7) && (ring_buf_size_get(&uart_ring_buf) >= BT_TX_PKG_SIZE)) {
				len = ring_buf_get(&uart_ring_buf, data, BT_TX_PKG_SIZE);
				if (bt_nus_send(NULL, data, len)) {
					LOG_WRN("Failed to send data over BLE connection");
				}
			}
		}
	}
}

static uint8_t rx_buf[RECEIVE_BUFF_SIZE];

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
	switch (evt->type) {
	case UART_RX_RDY:
		if (evt->data.rx.len > 0) {
			ring_buf_put(&uart_ring_buf, &evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
			k_work_submit(&repeat_uart_on_bt);
		}
		break;
    case UART_RX_DISABLED:
        uart_rx_enable(dev, (uint8_t *)user_data, RECEIVE_BUFF_SIZE, RX_TIMEOUT);
        break;
    default:
        break;
	}
}

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;
static struct k_work adv_work;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

static void recycled_cb(void)
{
	LOG_INF("Connection object available from previous conn. Disconnect is complete!");
	advertising_start();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled_cb,
};

static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	char string_data[len + 1];

    for (size_t i = 0; i < len; i++) {
        string_data[i] = (char)data[i];
    }
    string_data[len] = '\0';

	LOG_INF("Data received: %s", string_data);
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

int initiate_bt_conn(void)
{
	int err;
	int ret;

	ring_buf_init(&uart_ring_buf, RING_BUFF_SIZE, ring_buf_data);

	err = uart_configure(uart, &uart_cfg);

	if (err) {
		LOG_ERR("UART Configuration Error: %d", -err);
		return -ENOSYS;
	} else {
		LOG_INF("UART Configured\n");
	}

	ret = uart_callback_set(uart, uart_cb, rx_buf);

	if (ret) {
		LOG_ERR("Couldn't set UART Callback");
		return 1;
	} else {
		LOG_INF("UART Callback Set\n");
	}

	ret = uart_rx_enable(uart, rx_buf, sizeof rx_buf, RX_TIMEOUT);

	if (ret) {
		LOG_ERR("Couldn't enable UART RX");
		return 1;
	} else {
		LOG_INF("UART RX Enabled\n");
	}

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization callbacks. (err: %d)", err);
			return 1;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization info callbacks. (err: %d)", err);
			return 2;
		}
	}

	err = bt_enable(NULL);
	if (err) {
        LOG_ERR("Failed to enable Bluetooth (err: %d)", err);
        return 3;
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 4;
	}

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	// Initiate LiDAR scan
	ret = uart_tx(uart, request_stop, sizeof(request_stop), SYS_FOREVER_MS);
	if (ret) {
		printk("TX Error: %d\n", ret);
		return 1;
	}
	
	k_msleep(1000);
	
	ret = uart_tx(uart, request_scan, sizeof(request_scan), SYS_FOREVER_MS);
	if (ret) {
		printk("TX Error: %d\n", ret);
		return 1;
	}

    return 0;
}