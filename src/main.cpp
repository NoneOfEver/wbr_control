/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/app_main.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	const int rc = rm_test::app::Main();

	printk("main exited unexpectedly: rc=%d\n", rc);
	while (true) {
		k_sleep(K_SECONDS(1));
	}
}
