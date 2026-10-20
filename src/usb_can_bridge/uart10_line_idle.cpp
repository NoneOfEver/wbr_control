/* SPDX-License-Identifier: Apache-2.0 */
#include <drivers/uart_hpmicro.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <hpm_clock_drv.h>
#include <hpm_gptmr_drv.h>
#include <hpm_trgm_drv.h>
#include <hpm_trgmmux_src.h>
#include <cerrno>

namespace {
constexpr unsigned kCompare = 0, kCapture = 2;
constexpr unsigned kIrq = DT_IRQN(DT_NODELABEL(uart10_idle_timer));
const device *receiver;
uint32_t idle_ticks;
volatile uint32_t idle_errors;

void IdleIsr(const void *)
{
	const unsigned key = irq_lock();
	constexpr uint32_t cmp = GPTMR_CH_CMP_STAT_MASK(kCompare, 0);
	constexpr uint32_t cap = GPTMR_CH_CAP_STAT_MASK(kCapture);
	if (gptmr_check_status(HPM_GPTMR2, cmp)) {
		// A new RX edge may have reset the counter after compare asserted.
		// Defer instead of freezing DMA while the next frame is active.
		if (gptmr_channel_get_counter(HPM_GPTMR2, kCompare,
			gptmr_counter_type_normal) + 1 < idle_ticks) {
			gptmr_clear_status(HPM_GPTMR2, cmp);
			irq_unlock(key);
			return;
		}
		gptmr_stop_counter(HPM_GPTMR2, kCompare);
		gptmr_disable_irq(HPM_GPTMR2, GPTMR_CH_CMP_IRQ_MASK(kCompare, 0));
		gptmr_clear_status(HPM_GPTMR2, cmp | cap);
		int result = uart_hpm_rx_line_idle_flush(receiver);
		if (result < 0 && result != -EAGAIN) ++idle_errors;
		// Keep RX DMA running throughout the gap. Only the detector rearms.
		gptmr_enable_irq(HPM_GPTMR2, GPTMR_CH_CAP_IRQ_MASK(kCapture));
	} else if (gptmr_check_status(HPM_GPTMR2, cap)) {
		gptmr_clear_status(HPM_GPTMR2, cap | cmp);
		gptmr_disable_irq(HPM_GPTMR2, GPTMR_CH_CAP_IRQ_MASK(kCapture));
		gptmr_channel_reset_count(HPM_GPTMR2, kCompare);
		gptmr_start_counter(HPM_GPTMR2, kCompare);
		gptmr_enable_irq(HPM_GPTMR2, GPTMR_CH_CMP_IRQ_MASK(kCompare, 0));
	}
	irq_unlock(key);
}
}

int Uart10LineIdleStart(const device *dev, uint32_t timeout_us)
{
	receiver = dev;
	clock_add_to_group(clock_gptmr2, 0);
	const uint64_t ticks = uint64_t(clock_get_frequency(clock_gptmr2)) * timeout_us / 1000000;
	if (!ticks || ticks >= UINT32_MAX - 1) return -EINVAL;
	idle_ticks = ticks;
	gptmr_channel_config_t cfg;
	gptmr_channel_get_default_config(HPM_GPTMR2, &cfg);
	cfg.enable_cmp_output = true;
	cfg.cmp[0] = idle_ticks;
	cfg.synci_edge = gptmr_synci_edge_both;
	if (gptmr_channel_config(HPM_GPTMR2, kCompare, &cfg, false) != status_success)
		return -EIO;
	gptmr_channel_get_default_config(HPM_GPTMR2, &cfg);
	cfg.enable_cmp_output = false;
	cfg.mode = gptmr_work_mode_capture_at_falling_edge;
	if (gptmr_channel_config(HPM_GPTMR2, kCapture, &cfg, false) != status_success)
		return -EIO;
	trgm_output_update_source(HPM_TRGM1, HPM_TRGM1_OUTPUT_SRC_GPTMR2_SYNCI,
		HPM_TRGM1_INPUT_SRC_TRGM1_P3);
	trgm_output_update_source(HPM_TRGM1, HPM_TRGM1_OUTPUT_SRC_GPTMR2_IN2,
		HPM_TRGM1_INPUT_SRC_TRGM1_P3);
	gptmr_clear_status(HPM_GPTMR2, GPTMR_CH_CMP_STAT_MASK(kCompare, 0) |
		GPTMR_CH_CAP_STAT_MASK(kCapture));
	gptmr_channel_reset_count(HPM_GPTMR2, kCompare);
	gptmr_channel_reset_count(HPM_GPTMR2, kCapture);
	gptmr_enable_irq(HPM_GPTMR2, GPTMR_CH_CMP_IRQ_MASK(kCompare, 0));
	IRQ_CONNECT(kIrq, 2, IdleIsr, nullptr, 0);
	irq_enable(kIrq);
	gptmr_start_counter(HPM_GPTMR2, kCapture);
	gptmr_start_counter(HPM_GPTMR2, kCompare);
	return 0;
}

bool Uart10LineIdleHealthy() { return idle_errors == 0; }
