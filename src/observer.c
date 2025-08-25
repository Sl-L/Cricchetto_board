/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "observer.h"

#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME observer
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

#define BEACON_COUNT 5

static bt_beacon beacon0;
static bt_beacon beacon1;
static bt_beacon beacon2;
static bt_beacon beacon3;
static bt_beacon beacon4;

static bt_beacon *beacon_list [BEACON_COUNT] = {
	&beacon0,
	&beacon1,
	&beacon2,
	&beacon3,
	&beacon4,
};

const char *beacon_addr_list [BEACON_COUNT] = {
	"C2:0B:8C:0D:9D:6A",
	"CD:5C:CD:C7:6C:4F",
	"CC:C1:EC:A1:E7:52",
	"F3:9B:07:E2:52:44",
	"E2:AF:31:03:EC:52"
};

const char *beacon_addr_type = "public";

void parse_beacons() {
	for (int i = 0; i < BEACON_COUNT; i++) {

		int err = bt_addr_le_from_str(beacon_addr_list[i], beacon_addr_type, &beacon_list[i]->addr);
		beacon_list[i]->rssi = 0;
		if (err) {
			LOG_ERR("Error parsing address of Beacon %d\n", i);
		} else {
			LOG_INF("Successfully parsed address of Beacon %d\n", i);
		}
	}
}

int get_beacon_rssi(int id) {
	return beacon_list[id]->rssi;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	for (int i = 0; i < BEACON_COUNT; i++) {
		if (bt_addr_le_cmp(addr, &beacon_list[i]->addr) == 0) {
			char addr_str[BT_ADDR_LE_STR_LEN];
			bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

			beacon_list[i]->rssi = rssi;

			LOG_DBG("Beacon %d found (RSSI %d)\n", i, rssi);

			break;
		}
	}
}

int observer_start(void)
{
	parse_beacons();
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_PASSIVE,
		.options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};
	int err;

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		LOG_ERR("Start scanning failed (err %d)\n", err);
		return err;
	}
	LOG_INF("Started scanning...\n");

	return 0;
}