#include <zephyr/drivers/uart.h>

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RX_TIMEOUT 5000
#define RECEIVE_BUFF_SIZE 2600
#define RING_BUFF_SIZE 8192

#define BT_TX_PKG_SIZE 2500

int initiate_bt_conn(void);