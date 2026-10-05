/* Brief GPIO-level TXD/RXD probe for HPM5E3Y CAN0/1, with CAN2 as a control. */
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <hpm_gpio_drv.h>
#include <hpm_soc.h>

struct pin_probe_target {
	const char *name;
	uint8_t port;
	uint8_t tx_pin;
	uint8_t rx_pin;
};

static const struct pin_probe_target targets[] = {
	{ "CAN0", GPIO_DI_GPIOD, 0, 1 },
	{ "CAN1", GPIO_DI_GPIOD, 5, 4 },
	{ "CAN2", GPIO_DI_GPIOC, 8, 9 },
};

static void probe_one(const struct pin_probe_target *target)
{
	const uint32_t port = target->port;
	const uint32_t tx_pad = (port * 32U) + target->tx_pin;
	const uint32_t rx_pad = (port * 32U) + target->rx_pin;
	const uint32_t tx_mask = 1UL << target->tx_pin;
	uint32_t saved_func = HPM_IOC->PAD[tx_pad].FUNC_CTL;
	uint32_t saved_rx_ctl = HPM_IOC->PAD[rx_pad].PAD_CTL;
	uint32_t saved_oe = HPM_GPIO0->OE[port].VALUE & tx_mask;
	uint32_t saved_out = HPM_GPIO0->DO[port].VALUE & tx_mask;
	uint8_t idle_rx = gpio_read_pin(HPM_GPIO0, port, target->rx_pin);
	uint8_t high_tx, high_rx, low_tx, low_rx, release_rx;
	uint8_t pull_up_rx, pull_down_rx, restored_rx;

	/* Start recessive, then switch only TXD to GPIO. RXD stays on MCAN mux. */
	gpio_write_pin(HPM_GPIO0, port, target->tx_pin, 1);
	gpio_set_pin_output(HPM_GPIO0, port, target->tx_pin);
	HPM_IOC->PAD[tx_pad].FUNC_CTL = IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
	k_busy_wait(100);
	high_tx = gpio_read_pin(HPM_GPIO0, port, target->tx_pin);
	high_rx = gpio_read_pin(HPM_GPIO0, port, target->rx_pin);

	/* A 100 us dominant pulse is enough to check the transceiver echo. */
	gpio_write_pin(HPM_GPIO0, port, target->tx_pin, 0);
	k_busy_wait(100);
	low_tx = gpio_read_pin(HPM_GPIO0, port, target->tx_pin);
	low_rx = gpio_read_pin(HPM_GPIO0, port, target->rx_pin);
	gpio_write_pin(HPM_GPIO0, port, target->tx_pin, 1);
	k_busy_wait(100);
	release_rx = gpio_read_pin(HPM_GPIO0, port, target->rx_pin);

	HPM_IOC->PAD[tx_pad].FUNC_CTL = saved_func;
	if (saved_out == 0U) {
		gpio_write_pin(HPM_GPIO0, port, target->tx_pin, 0);
	}
	if (saved_oe == 0U) {
		gpio_set_pin_input(HPM_GPIO0, port, target->tx_pin);
	}
	/* Disable the pad keeper while applying weak pulls; otherwise its held
	 * value can hide a floating RXD. Restore the original pad setting below.
	 */
	HPM_IOC->PAD[rx_pad].PAD_CTL =
		(saved_rx_ctl & ~(IOC_PAD_PAD_CTL_PE_MASK | IOC_PAD_PAD_CTL_PS_MASK |
		                  IOC_PAD_PAD_CTL_PRS_MASK | IOC_PAD_PAD_CTL_KE_MASK)) |
		IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
	k_busy_wait(1000);
	pull_up_rx = gpio_read_pin(HPM_GPIO0, port, target->rx_pin);
	HPM_IOC->PAD[rx_pad].PAD_CTL =
		(saved_rx_ctl & ~(IOC_PAD_PAD_CTL_PE_MASK | IOC_PAD_PAD_CTL_PS_MASK |
		                  IOC_PAD_PAD_CTL_PRS_MASK | IOC_PAD_PAD_CTL_KE_MASK)) |
		IOC_PAD_PAD_CTL_PE_SET(1);
	k_busy_wait(1000);
	pull_down_rx = gpio_read_pin(HPM_GPIO0, port, target->rx_pin);
	HPM_IOC->PAD[rx_pad].PAD_CTL = saved_rx_ctl;
	k_busy_wait(100);
	restored_rx = gpio_read_pin(HPM_GPIO0, port, target->rx_pin);
	printk("[%s PIN] idle RX=%u; TXD high: TX=%u RX=%u; TXD low: TX=%u RX=%u; released RX=%u\n",
	       target->name, idle_rx, high_tx, high_rx, low_tx, low_rx, release_rx);
	printk("[%s BIAS] pull-up RX=%u pull-down RX=%u restored RX=%u\n",
	       target->name, pull_up_rx, pull_down_rx, restored_rx);
	printk("[%s PIN] TX_FUNC restored=%08x RX_FUNC=%08x\n", target->name,
	       HPM_IOC->PAD[tx_pad].FUNC_CTL,
	       HPM_IOC->PAD[rx_pad].FUNC_CTL);
}

int main(void)
{
	printk("CAN0/CAN1/CAN2 TXD-to-RXD pin probe (100 us pulse per channel)\n");
	for (size_t i = 0; i < ARRAY_SIZE(targets); ++i) {
		probe_one(&targets[i]);
		k_sleep(K_MSEC(20));
	}
	printk("CAN pin probe complete; RTT shell remains available\n");
	return 0;
}
