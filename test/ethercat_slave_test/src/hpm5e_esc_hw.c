/* SPDX-License-Identifier: Apache-2.0 */

#include "hpm5e_esc_hw.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

static const struct device *const esc = DEVICE_DT_GET(DT_NODELABEL(esc0));
static uint32_t timer_origin_ms;
static unsigned int critical_key;
static unsigned int critical_depth;

uint16_t hpm5e_ssc_hw_init(void)
{
	return device_is_ready(esc) ? 0U : 1U;
}

void hpm5e_ssc_hw_release(void)
{
	hpm_ethercat_stop(esc);
}

void hpm5e_ssc_esc_read(volatile void *data, uint16_t address, uint16_t length)
{
	if (hpm_ethercat_read(esc, address, (void *)data, length) != 0) {
		memset((void *)data, 0, length);
	}
}

void hpm5e_ssc_esc_write(const volatile void *data, uint16_t address, uint16_t length)
{
	(void)hpm_ethercat_write(esc, address, (const void *)data, length);
}

uint16_t hpm5e_ssc_al_event(void)
{
	return hpm_ethercat_get_al_event(esc);
}

uint32_t hpm5e_ssc_timer_get(void)
{
	return hpm_ethercat_timer_get() - timer_origin_ms;
}

void hpm5e_ssc_timer_clear(void)
{
	timer_origin_ms = hpm_ethercat_timer_get();
}

uint16_t hpm5e_ssc_eeprom_reload(void)
{
	return hpm_ethercat_process_eeprom(esc) == 0 ? 0U : 1U;
}

void hpm5e_ssc_irq_enable(void)
{
	hpm_ethercat_irq_enable(esc, HPM_ETHERCAT_IRQ_PDI, true);
}

void hpm5e_ssc_irq_disable(void)
{
	hpm_ethercat_irq_enable(esc, HPM_ETHERCAT_IRQ_PDI, false);
}

void hpm5e_ssc_enter_critical(void)
{
	const unsigned int key = irq_lock();

	if (critical_depth++ == 0U) {
		critical_key = key;
	}
}

void hpm5e_ssc_leave_critical(void)
{
	if (critical_depth == 0U) {
		return;
	}
	if (--critical_depth == 0U) {
		irq_unlock(critical_key);
	}
}
