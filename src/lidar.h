#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#define LOG_MODULE_NAME lidar
#define RECEIVE_BUFF_SIZE 20
#define RX_TIMEOUT 5000
#define SLEEP_TIME_MS 5000

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart20));


const struct uart_config uart_cfg = {
	.baudrate = 460800,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};