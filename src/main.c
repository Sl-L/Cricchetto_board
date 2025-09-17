/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/logging/log.h>

#include "lib/observer.h"
// #include "lib/particle_filter.h"
#include "lib/ble.h"

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

int main(void)
{
	LOG_INF("Main Start\n");

	initiate_bt_conn();


	while (1) {
		LOG_INF("Working...");
		k_msleep(2000);
	}

	return 0;
}