#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#define BEACON_COUNT 5

typedef struct
{
	const char *addr_type;
	int pos_x;
	int pos_y;
	bt_addr_le_t addr;
	int rssi;
} bt_beacon;

int get_beacon_rssi(int id);

int observer_start(void);
