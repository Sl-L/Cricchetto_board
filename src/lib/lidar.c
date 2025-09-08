#include "lidar.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

static bool expect_response_descriptor = false;
static bool expect_data = false;

static LiDAR_response_descriptor_t response_descriptor;

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	static uint8_t rx_buf[RECEIVE_BUFF_SIZE];
    static size_t rx_count = 0;

	#if defined(LOG_LEVEL) && (LOG_LEVEL == LOG_LEVEL_DBG)
		static char response[RECEIVE_BUFF_SIZE];
	#endif

	switch (evt->type) {
	
	case UART_RX_RDY:
		LOG_INF("UART Timeout");

		static char s[RECEIVE_BUFF_SIZE] = "00000000000000000000000000000000";

		for (int i = 0; i < evt->data.rx.len; i++) {
			s[i] = evt->data.rx_buf.buf[i];
		}

		LOG_INF("%s", s);
		
		break;
	case UART_RX_DISABLED:
		uart_rx_enable(dev, rx_buf, sizeof rx_buf, RX_TIMEOUT);
		break;

	default:
		break;
	}

	// while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
    //     uart_fifo_read(dev, &rx_buf[rx_count], 1);
    //     rx_count++;
        
	// 	if (expect_response_descriptor) {
	// 		if (rx_count == sizeof(LiDAR_response_descriptor_t)) {
	// 			memcpy(&response_descriptor, rx_buf, sizeof(LiDAR_response_descriptor_t));
	// 			rx_count = 0;
	// 		}
	// 	} else if (expect_data) {
	// 		if (rx_count == response_descriptor.data_length) {
				
	// 		}
	// 	}
    // }
}