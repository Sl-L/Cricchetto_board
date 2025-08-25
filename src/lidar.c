#include "lidar.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	
	case UART_RX_RDY:
		printk("Response\n");
		if (evt->data.rx.len > 0) {
			printk("Message received: \n");
			printk("	");
			for (int i = 0; i < evt->data.rx.len; i++) {
				printk("%d ", evt->data.rx.buf[i]);
			}
			printk("\n");
		}
		break;
	case UART_RX_DISABLED:
		uart_rx_enable(dev, rx_buf, sizeof rx_buf, RX_TIMEOUT);
		break;

	default:
		break;
	}
}