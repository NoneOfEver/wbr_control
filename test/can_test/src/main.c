#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define TEST_CAN_ID 0x123U

K_SEM_DEFINE(g_tx_done_sem, 0, 1);
static volatile int g_tx_result;

static const char *CanStateToStr(enum can_state state)
{
	if (state == CAN_STATE_ERROR_ACTIVE) {
		return "error-active";
	}
	if (state == CAN_STATE_ERROR_WARNING) {
		return "error-warning";
	}
	if (state == CAN_STATE_ERROR_PASSIVE) {
		return "error-passive";
	}
	if (state == CAN_STATE_BUS_OFF) {
		return "bus-off";
	}
	if (state == CAN_STATE_STOPPED) {
		return "stopped";
	}

	return "unknown";
}

static void PrintCanState(const struct device *can_dev, const char *tag)
{
	enum can_state state;
	struct can_bus_err_cnt err_cnt;
	int rc = can_get_state(can_dev, &state, &err_cnt);

	if (rc == 0) {
		printk("[STATE] %s: %s tec=%u rec=%u\n", tag, CanStateToStr(state),
		       err_cnt.tx_err_cnt, err_cnt.rx_err_cnt);
	} else {
		printk("[STATE] %s: can_get_state rc=%d\n", tag, rc);
	}
}

static void OnTxDone(const struct device *dev, int error, void *user_data)
{
	(void)dev;
	(void)user_data;
	g_tx_result = error;
	k_sem_give(&g_tx_done_sem);
}

const struct device *FindCanDevice(void)
{
	const struct device *dev = NULL;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mcan4), okay)
	dev = DEVICE_DT_GET(DT_NODELABEL(mcan4));
	if (device_is_ready(dev)) {
		return dev;
	}
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mcan3), okay)
	dev = DEVICE_DT_GET(DT_NODELABEL(mcan3));
	if (device_is_ready(dev)) {
		return dev;
	}
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mcan2), okay)
	dev = DEVICE_DT_GET(DT_NODELABEL(mcan2));
	if (device_is_ready(dev)) {
		return dev;
	}
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mcan1), okay)
	dev = DEVICE_DT_GET(DT_NODELABEL(mcan1));
	if (device_is_ready(dev)) {
		return dev;
	}
#endif

	return NULL;
}

int main(void)
{
	printk("can_test booted\n");

	const struct device *can_dev = FindCanDevice();
	if (can_dev == NULL) {
		printk("[FAIL] CAN device not found\n");
		return -ENODEV;
	}

	printk("using can device: %s\n", can_dev->name);

	int rc = can_set_mode(can_dev, CAN_MODE_NORMAL);
	if (rc != 0) {
		printk("[FAIL] can_set_mode(normal) rc=%d\n", rc);
		return rc;
	}

	rc = can_start(can_dev);
	if ((rc != 0) && (rc != -EALREADY)) {
		printk("[FAIL] can_start rc=%d\n", rc);
		return rc;
	}

	PrintCanState(can_dev, "after start");

	printk("[INFO] physical CAN TX test started (id=0x%x)\n", TEST_CAN_ID);
	printk("[INFO] connect CAN transceiver and a peer/analyzer at same baud\n");

	uint32_t seq = 0U;
	bool tx_blocked = false;
	while (true) {
		if (tx_blocked) {
			PrintCanState(can_dev, "waiting (tx blocked)");
			k_sleep(K_MSEC(1000));
			continue;
		}

		struct can_frame tx = {
			.flags = 0U,
			.id = TEST_CAN_ID,
			.dlc = 8U,
			.data = {
				(uint8_t)(seq & 0xFFU),
				(uint8_t)((seq >> 8) & 0xFFU),
				0x32U,
				0x43U,
				0x54U,
				0x65U,
				0x76U,
				0x87U,
			},
		};

		while (k_sem_take(&g_tx_done_sem, K_NO_WAIT) == 0) {
			/* Drain stale completion events before next frame. */
		}

		g_tx_result = -EINPROGRESS;
		rc = can_send(can_dev, &tx, K_MSEC(50), OnTxDone, NULL);
		if (rc != 0) {
			printk("[ERR] can_send enqueue rc=%d (mailbox/state/wiring)\n", rc);
			PrintCanState(can_dev, "enqueue failed");
		} else {
			rc = k_sem_take(&g_tx_done_sem, K_MSEC(200));
			if (rc == 0) {
				if (g_tx_result == 0) {
					printk("[TX] seq=%u id=0x%x dlc=%u\n", seq, tx.id, tx.dlc);
				} else {
					printk("[ERR] tx callback error=%d (likely ACK/bus issue)\n", g_tx_result);
					PrintCanState(can_dev, "tx callback error");
				}
			} else {
				printk("[WARN] tx completion timeout (likely no ACK / transceiver missing)\n");
				PrintCanState(can_dev, "tx timeout");
				printk("[WARN] stop re-sending to avoid driver wait lock, keep monitoring state only\n");
				tx_blocked = true;
			}
		}

		seq++;
		k_sleep(K_MSEC(500));
	}
}
