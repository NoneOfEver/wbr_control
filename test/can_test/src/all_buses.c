/* Physical CAN0-CAN3 probe for the Dust HPM5E3Y board. */
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <hpm_mcan_drv.h>
#include <hpm_gpio_drv.h>
#include <hpm_soc.h>

struct can_target {
	const char *name;
	const struct device *dev;
	MCAN_Type *regs;
	uint8_t gpio_port;
	uint8_t tx_pin;
	uint8_t rx_pin;
};

static const struct can_target targets[] = {
	{ "CAN0", DEVICE_DT_GET(DT_NODELABEL(can0)), (MCAN_Type *)DT_REG_ADDR(DT_NODELABEL(can0)), GPIO_DI_GPIOD, 0, 1 },
	{ "CAN1", DEVICE_DT_GET(DT_NODELABEL(can1)), (MCAN_Type *)DT_REG_ADDR(DT_NODELABEL(can1)), GPIO_DI_GPIOD, 5, 4 },
	{ "CAN2", DEVICE_DT_GET(DT_NODELABEL(can2)), (MCAN_Type *)DT_REG_ADDR(DT_NODELABEL(can2)), GPIO_DI_GPIOC, 8, 9 },
	{ "CAN3", DEVICE_DT_GET(DT_NODELABEL(can3)), (MCAN_Type *)DT_REG_ADDR(DT_NODELABEL(can3)), GPIO_DI_GPIOC, 15, 14 },
};

K_SEM_DEFINE(tx_done, 0, 1);
static volatile int tx_result;

static void on_tx_done(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	tx_result = error;
	k_sem_give(&tx_done);
}

static void on_rx(const struct device *dev, struct can_frame *frame, void *user_data)
{
	const struct can_target *target = user_data;
	ARG_UNUSED(dev);
	printk("[%s RX] id=0x%x dlc=%u\n", target->name, frame->id, frame->dlc);
}

static void print_status(const struct can_target *target, const char *phase)
{
	enum can_state state;
	struct can_bus_err_cnt errors;
	int rc = can_get_state(target->dev, &state, &errors);
	MCAN_Type *can = target->regs;
	uint32_t rx_pad_index = (target->gpio_port * 32U) + target->rx_pin;
	uint32_t tx_pad_index = (target->gpio_port * 32U) + target->tx_pin;

	printk("[%s %s] state=%d rc=%d tec=%u rec=%u activity=%u LEC=%u RXD=%u GPIO_RX=%u GPIO_TX=%u\n",
	       target->name, phase, state, rc, errors.tx_err_cnt, errors.rx_err_cnt,
	       MCAN_PSR_ACT_GET(can->PSR), MCAN_PSR_LEC_GET(can->PSR),
	       MCAN_TEST_RX_GET(can->TEST),
	       gpio_read_pin(HPM_GPIO0, target->gpio_port, target->rx_pin),
	       gpio_read_pin(HPM_GPIO0, target->gpio_port, target->tx_pin));
	printk("[%s REG] CCCR=%08x PSR=%08x ECR=%08x IR=%08x TXBRP=%08x TXBTO=%08x\n",
	       target->name, can->CCCR, can->PSR, can->ECR, can->IR, can->TXBRP, can->TXBTO);
	printk("[%s PAD] TX_FUNC=%08x TX_CTL=%08x RX_FUNC=%08x RX_CTL=%08x\n",
	       target->name, HPM_IOC->PAD[tx_pad_index].FUNC_CTL,
	       HPM_IOC->PAD[tx_pad_index].PAD_CTL,
	       HPM_IOC->PAD[rx_pad_index].FUNC_CTL,
	       HPM_IOC->PAD[rx_pad_index].PAD_CTL);
}

static void probe_one(const struct can_target *target)
{
	int rc;
	struct can_filter filter = { .id = 0, .mask = 0, .flags = 0 };
	struct can_frame frame = {
		.id = 0x123,
		.dlc = 8,
		.data = { 0x43, 0x41, 0x4e, target->name[3], 0x54, 0x45, 0x53, 0x54 },
	};

	printk("[%s] device=%s ready=%u\n", target->name, target->dev->name,
	       device_is_ready(target->dev));
	if (!device_is_ready(target->dev)) {
		return;
	}
	rc = can_set_mode(target->dev, CAN_MODE_NORMAL);
	if (rc == 0) {
		rc = can_set_bitrate(target->dev, 1000000);
	}
	if (rc == 0) {
		rc = can_add_rx_filter(target->dev, on_rx, (void *)target, &filter);
		if (rc >= 0) {
			rc = 0;
		}
	}
	if (rc == 0) {
		rc = can_start(target->dev);
	}
	if (rc != 0) {
		printk("[%s] setup failed rc=%d\n", target->name, rc);
		return;
	}

	print_status(target, "before TX");
	k_sem_reset(&tx_done);
	tx_result = -EINPROGRESS;
	rc = can_send(target->dev, &frame, K_MSEC(50), on_tx_done, NULL);
	printk("[%s] can_send rc=%d\n", target->name, rc);
	if (rc == 0) {
		rc = k_sem_take(&tx_done, K_MSEC(300));
		printk("[%s] completion wait rc=%d callback=%d\n", target->name, rc, tx_result);
	}
	print_status(target, "after TX");
	rc = can_stop(target->dev);
	printk("[%s] stopped rc=%d\n", target->name, rc);
}

int main(void)
{
	printk("CAN0-CAN3 physical probe: 1 Mbps, one frame per controller\n");
	for (size_t i = 0; i < ARRAY_SIZE(targets); ++i) {
		probe_one(&targets[i]);
		k_sleep(K_MSEC(300));
	}
	printk("CAN0-CAN3 probe complete; RTT shell remains available\n");
	return 0;
}
