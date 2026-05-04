#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	uint32_t counter = 0U;
	const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));
	const struct device *uart1 = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(uart1));
	const int console_ready = device_is_ready(console);
	const int uart0_ready = device_is_ready(uart0);
	const int uart1_ready = (uart1 != NULL) && device_is_ready(uart1);

	printk("printk_test booted\n");
	printk("ready: console=%d uart0=%d uart1=%d\n", console_ready, uart0_ready, uart1_ready);

	while (1) {
		printk("printk_test heartbeat: %u\n", counter);
		counter++;
		k_sleep(K_MSEC(1000));
	}

	return 0;
}
