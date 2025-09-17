#include "ble.h"
#include <zephyr/types.h>
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/drivers/uart.h>
// #include <zephyr/bluetooth/conn.h>
// #include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <sdc_hci_vs.h>

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

const struct bt_conn_le_phy_param phy_param = {
    .options = BT_CONN_LE_PHY_OPT_NONE,
    .pref_rx_phy = BT_GAP_LE_PHY_2M,
    .pref_tx_phy = BT_GAP_LE_PHY_2M,
};

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static K_SEM_DEFINE(uart_init_ok, 0, 1);

static uint8_t ring_buf_data[RING_BUFF_SIZE];
struct ring_buf uart_ring_buf;

static void bt_repeat_work_handler(struct k_work *work);

static K_WORK_DEFINE(repeat_uart_on_bt, bt_repeat_work_handler);

static void bt_repeat_work_handler(struct k_work *work) {
	static uint8_t data[BT_TX_PKG_SIZE];
	static uint16_t len;

	len = ring_buf_size_get(&uart_ring_buf);

	if (len >= BT_TX_PKG_SIZE) {
		len = ring_buf_get(&uart_ring_buf, data, BT_TX_PKG_SIZE);
		int err = bt_nus_send(NULL, data, len);
		if (err != 0) {
			LOG_WRN("Failed to send data over BLE connection (err: %d)", err);
		}
	} else {
		return;
	}
	len = ring_buf_size_get(&uart_ring_buf);
	if (len >= BT_TX_PKG_SIZE) {
		k_work_submit(&repeat_uart_on_bt);
	}
	
}

static uint8_t rx_buf[RECEIVE_BUFF_SIZE];

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
	if (evt->type != UART_RX_RDY) {
        if (evt->type == UART_RX_DISABLED) {
            uart_rx_enable(dev, rx_buf, sizeof(rx_buf), RX_TIMEOUT);
		}
		return;
    }

	ring_buf_put(&uart_ring_buf, &evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
	k_work_submit(&repeat_uart_on_bt);
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

static void adv_work_handler(struct k_work *work) {
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}

static void advertising_start(void) {
	k_work_submit(&adv_work);
}

static void auth_cancel(struct bt_conn *conn) {
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded) {
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
		bt_security_err_to_str(reason));
}

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey) {
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static bool con_active = false;
static void connected(struct bt_conn *conn, uint8_t err) {
	char addr[BT_ADDR_LE_STR_LEN];

    if (err) {
		LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}

    current_conn = bt_conn_ref(conn);
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);


    /* 2 Mbit PHY */
    bt_conn_le_phy_update(conn, &phy_param);

    /* connection parameters – 7.5 ms interval */
    bt_conn_le_param_update(conn,
            BT_LE_CONN_PARAM(0x0006, 0x0006, 0, 400));   /* <-- address of compound literal */

    /* data length + MTU */
    bt_conn_le_data_len_update(conn, BT_LE_DATA_LEN_PARAM_MAX);

	con_active = true;
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
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

	con_active = false;
}

static void recycled_cb(void) {
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

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len) {
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

static void lidar_handler_thread() {
	int ret;
	bool lidar_is_on = false;

	k_sem_take(&uart_init_ok, K_FOREVER);
	while (1) {
		if (!con_active) {
			LOG_INF("Awaiting bluetooth connection...");
			if (lidar_is_on) {
				ret = uart_tx(uart, request_stop, sizeof(request_stop), SYS_FOREVER_MS);
				if (ret) {
					printk("TX Error: %d\n", ret);
					return;
				} else {
					lidar_is_on = false;
				}
			}
		} else if (!lidar_is_on) {
			ret = uart_tx(uart, request_scan, sizeof(request_scan), SYS_FOREVER_MS);
			if (ret) {
				printk("TX Error: %d\n", ret);
				return;
			} else {
				lidar_is_on = true;
			}
		}
		k_msleep(1000);
	}
}

int initiate_bt_conn(void) {
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

	ret = uart_callback_set(uart, uart_cb, rx_buf);

	if (ret) {
		LOG_ERR("Couldn't set UART Callback");
		return 1;
	} else {
		LOG_INF("UART Callback Set\n");
	}

	k_msleep(500);

	ret = uart_rx_enable(uart, rx_buf, sizeof rx_buf, RX_TIMEOUT);

	if (ret) {
		LOG_ERR("Couldn't enable UART RX");
		return 1;
	} else {
		LOG_INF("UART RX Enabled\n");
		k_sem_give(&uart_init_ok);
	}

    return 0;
}

K_THREAD_DEFINE(lidar_handler_thread_id, STACKSIZE, lidar_handler_thread, NULL, NULL, NULL, 7, 0, 0);