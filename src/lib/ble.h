#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/uart.h>

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RX_TIMEOUT 5000
#define RECEIVE_BUFF_SIZE 1024
#define RING_BUFF_SIZE 4096

#define BT_TX_PKG_SIZE 200
#define EXPRESS_SCAN_PKG_SIZE 84
#define SCAN_PKG_SIZE 5
#define RESPONSE_DESCRIPTOR_SIZE 7

typedef struct {
	bool con_active;
	bool transmit;
	bool reset_lidar;
    bool stop_lidar;
	bool expect_response_descriptor;
    uint8_t request_id;
} StatusAdmin;

int initiate_bt_conn(void);
