/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdio.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

extern void int_test_init(uint8_t busid, uint32_t reg_base);

int main(void)
{
	uint32_t usb_base = DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0));

	printf("cherryusb interrupt test booted.\n");

	int_test_init(0, usb_base);

	while (true) {
		k_sleep(K_SECONDS(1));
	}
}
