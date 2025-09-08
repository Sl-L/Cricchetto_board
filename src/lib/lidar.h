#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#define LOG_MODULE_NAME lidar
#define RX_TIMEOUT 5000
#define SLEEP_TIME_MS 5000

#define RECEIVE_BUFF_SIZE 32

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart20));

const struct uart_config uart_cfg = {
	.baudrate = 460800,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

const uint8_t response_start_flag1 = 0xA5;
const uint8_t response_start_flag2 = 0x5A;

typedef struct __attribute__((packed)) {
	uint8_t start_flag1 : 8;
	uint8_t start_flag2 : 8;
	uint32_t data_length : 30;
	uint8_t send_mode : 2;
	uint8_t data_type : 8;
} LiDAR_response_descriptor_t;

const uint8_t request_start_flag = 0xA5;

// No response
const uint8_t LiDAR_STOP = 0x25;
const uint32_t LiDAR_STOP_COOLDOWN_MS = 10;

const uint8_t LiDAR_RESET = 0x40;
const uint32_t LiDAR_RESET_COOLDOWN_MS = 500;

// Single response
const uint8_t LiDAR_GET_INFO = 0x50;
const uint8_t LiDAR_GET_HEALTH = 0x52;
const uint8_t LiDAR_GET_SAMPLERATE = 0x59;

// Multiple response
const uint8_t START_SCAN = 0x20;