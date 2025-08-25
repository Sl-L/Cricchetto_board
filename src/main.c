/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/logging/log.h>

#include "observer.h"

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

int main(void)
{
	int err;

	LOG_INF("Main Start\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	(void)observer_start();

	while(1) {
		for (int i = 0; i < BEACON_COUNT; i++) {
			LOG_DBG("Beacon %d RSSI: %d\n", i, get_beacon_rssi(i));
		}
		k_msleep(10000);
	}

	return 0;
}