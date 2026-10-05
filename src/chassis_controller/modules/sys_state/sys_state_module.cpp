/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file src/chassis_controller/modules/sys_state/sys_state_module.cpp
 * @ingroup wbr_modules
 * @brief 管理系统状态、使能条件与故障状态。
 * @details 实现运行在模块自有 Zephyr 线程或其驱动回调中。回调路径只完成有界的数据搬运和通知，耗时解析与控制计算留在线程上下文执行。
 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include "sys_state_module.h"

#include <hpm_clock_drv.h>
#include <hpm_gpio_drv.h>
#include <periodic_schedule.h>
#include <chassis_controller/scheduling/thread_phases.h>
#include <chassis_controller/scheduling/thread_priorities.h>
#include <msg/system_status_message.hpp>
#include <sys_state_params_generated.h>

LOG_MODULE_REGISTER(sys_state_module, LOG_LEVEL_INF);

namespace
{

#if DT_NODE_EXISTS(DT_ALIAS(led0))
const gpio_dt_spec kLedR = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
const gpio_dt_spec kLedG = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
const gpio_dt_spec kLedB = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(ws2812))
const gpio_dt_spec kWs2812 = GPIO_DT_SPEC_GET(DT_ALIAS(ws2812), gpios);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(pwm_buzzer))
#define WBR_CONTROL_BUZZER_NODE DT_ALIAS(pwm_buzzer)
#elif DT_NODE_EXISTS(DT_ALIAS(buzzer_pwm))
#define WBR_CONTROL_BUZZER_NODE DT_ALIAS(buzzer_pwm)
#elif DT_NODE_EXISTS(DT_ALIAS(buzzer))
#define WBR_CONTROL_BUZZER_NODE DT_ALIAS(buzzer)
#endif

#ifdef WBR_CONTROL_BUZZER_NODE
const pwm_dt_spec kBuzzer = PWM_DT_SPEC_GET(WBR_CONTROL_BUZZER_NODE);
#endif

K_THREAD_STACK_DEFINE(g_sys_state_module_stack, 1024);

constexpr uint8_t kPwmLevels = modules::sys_state_params::kSystemStatePwmLevels;
constexpr uint32_t kLedFrameMs = modules::sys_state_params::kSystemStateLedFrameMs;
constexpr uint16_t kBreathSteps = modules::sys_state_params::kSystemStateBreathSteps;
constexpr uint32_t kBuzzerPeriodUs =
	modules::sys_state_params::kSystemStateBuzzerPeriodUs;
constexpr uint8_t kBuzzerTickPercent =
	modules::sys_state_params::kSystemStateBuzzerTickPercent;
constexpr uint16_t kBuzzerTickSteps =
	modules::sys_state_params::kSystemStateBuzzerTickSteps;

#if DT_NODE_EXISTS(DT_ALIAS(ws2812))
constexpr uint32_t kWs2812Port = GPIO_DO_GPIOD;
uint32_t g_ws2812_period_cycles;
uint32_t g_ws2812_zero_high_cycles;
uint32_t g_ws2812_one_high_cycles;

uint32_t ReadCoreCycles()
{
	uint32_t cycles;
	__asm__ volatile("csrr %0, mcycle" : "=r"(cycles));
	return cycles;
}

void WaitUntilCycle(uint32_t deadline)
{
	while (static_cast<int32_t>(ReadCoreCycles() - deadline) < 0) {
		__asm__ volatile("nop");
	}
}

void SendWs2812Byte(uint8_t value)
{
	for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
		gpio_write_pin(HPM_GPIO0, kWs2812Port, kWs2812.pin, 1U);
		const uint32_t start = ReadCoreCycles();
		WaitUntilCycle(start + ((value & mask) != 0U ? g_ws2812_one_high_cycles
							       : g_ws2812_zero_high_cycles));
		gpio_write_pin(HPM_GPIO0, kWs2812Port, kWs2812.pin, 0U);
		WaitUntilCycle(start + g_ws2812_period_cycles);
	}
}

void SendWs2812Color(uint8_t r, uint8_t g, uint8_t b)
{
	const unsigned int key = irq_lock();
	/* WS2812B wire order is GRB. One pixel holds interrupts for about 30 us. */
	SendWs2812Byte(g);
	SendWs2812Byte(r);
	SendWs2812Byte(b);
	irq_unlock(key);
	/* Keep DIN low beyond the latch interval before a future update. */
	k_busy_wait(80U);
}
#endif

uint8_t TriangleBrightnessPercent(uint16_t step)
{
	const uint16_t half = kBreathSteps / 2U;
	if (step < half) {
		return static_cast<uint8_t>((step * 100U) / half);
	}

	return static_cast<uint8_t>(((kBreathSteps - step) * 100U) / half);
}

uint8_t PercentToDuty(uint8_t pct)
{
	if (pct >= 100U) {
		return static_cast<uint8_t>(kPwmLevels - 1U);
	}

	return static_cast<uint8_t>((pct * kPwmLevels) / 100U);
}

} // namespace

namespace modules
{

volatile uint32_t g_sys_state_diag_state = kSysStateDiagBoot;

int SysStateModule::Start()
{
	if (started_) {
		g_sys_state_diag_state = kSysStateDiagStartSkippedAlreadyStarted;
		return 0;
	}

	g_sys_state_diag_state = kSysStateDiagInitEnter;

#if DT_NODE_EXISTS(DT_ALIAS(ws2812))
	ws2812_ = kWs2812;
	const uint32_t cpu_frequency_hz = clock_get_frequency(clock_cpu0);
	g_ws2812_period_cycles = cpu_frequency_hz / 800000U;
	g_ws2812_zero_high_cycles =
		static_cast<uint32_t>((static_cast<uint64_t>(cpu_frequency_hz) * 350U) /
				      1000000000ULL);
	g_ws2812_one_high_cycles =
		static_cast<uint32_t>((static_cast<uint64_t>(cpu_frequency_hz) * 700U) /
				      1000000000ULL);
	if (gpio_is_ready_dt(&ws2812_)) {
		const int rc = gpio_pin_configure_dt(&ws2812_, GPIO_OUTPUT_INACTIVE);
		if (rc == 0) {
			ws2812_ready_ = true;
			led_ready_ = true;
			SendWs2812Color(0U, 0U, 0U);
		} else {
			LOG_WRN("WS2812 GPIO configuration failed (%d)", rc);
		}
	} else {
		LOG_WRN("WS2812 GPIO controller not ready");
	}
#endif

#if DT_NODE_EXISTS(DT_ALIAS(led0)) && DT_NODE_EXISTS(DT_ALIAS(led1)) &&                            \
	DT_NODE_EXISTS(DT_ALIAS(led2))
	led_r_ = kLedR;
	led_g_ = kLedG;
	led_b_ = kLedB;

	if (!gpio_is_ready_dt(&led_r_) || !gpio_is_ready_dt(&led_g_) ||
	    !gpio_is_ready_dt(&led_b_)) {
		g_sys_state_diag_state = kSysStateDiagInitGpioNotReady;
		LOG_WRN("sys_state init skipped: gpio not ready");
		return 0;
	}

	int rc = gpio_pin_configure_dt(&led_r_, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		g_sys_state_diag_state = kSysStateDiagInitConfigRFail;
		LOG_WRN("sys_state init skipped: led_r config failed (%d)", rc);
		return 0;
	}

	rc = gpio_pin_configure_dt(&led_g_, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		g_sys_state_diag_state = kSysStateDiagInitConfigGFail;
		LOG_WRN("sys_state init skipped: led_g config failed (%d)", rc);
		return 0;
	}

	rc = gpio_pin_configure_dt(&led_b_, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		g_sys_state_diag_state = kSysStateDiagInitConfigBFail;
		LOG_WRN("sys_state init skipped: led_b config failed (%d)", rc);
		return 0;
	}

	led_ready_ = true;
	led_r_ready_ = true;
	led_g_ready_ = true;
	led_b_ready_ = true;
#else
	if (!ws2812_ready_) {
		LOG_WRN("sys_state init skipped: led aliases not found");
	}
#endif

#ifdef WBR_CONTROL_BUZZER_NODE
	buzzer_ = kBuzzer;
	if (device_is_ready(buzzer_.dev)) {
		const uint32_t period = PWM_USEC(kBuzzerPeriodUs);
		const int rc = pwm_set_dt(&buzzer_, period, 0U);
		if (rc == 0) {
			buzzer_ready_ = true;
		}
	}
#else
	LOG_WRN("status_indicator init skipped: buzzer alias not found");
#endif

	ready_ = led_ready_ || buzzer_ready_;

	if (!ready_) {
		g_sys_state_diag_state = kSysStateDiagInitNoOutputs;
		LOG_WRN("status_indicator init skipped: no output device ready");
		return 0;
	}

	g_sys_state_diag_state = kSysStateDiagInitReady;

	const int start_result = CreateThread(
		g_sys_state_module_stack, K_THREAD_STACK_SIZEOF(g_sys_state_module_stack),
		K_PRIO_PREEMPT(modules::scheduling::thread_priority::kSystemState),
		"sys_state_module");
	if (start_result != 0) {
		g_sys_state_diag_state = kSysStateDiagThreadCreateFail;
		return start_result;
	}
	g_sys_state_diag_state = kSysStateDiagThreadCreated;
	return 0;
}

void SysStateModule::ApplyDuty(uint8_t r, uint8_t g, uint8_t b)
{
	if (!led_ready_) {
		return;
	}

#if DT_NODE_EXISTS(DT_ALIAS(ws2812))
	if (ws2812_ready_) {
		const auto scale = [](uint8_t value) {
			return static_cast<uint8_t>((static_cast<uint16_t>(value) * 255U) /
						    (kPwmLevels - 1U));
		};
		const uint8_t scaled_r = scale(r);
		const uint8_t scaled_g = scale(g);
		const uint8_t scaled_b = scale(b);
		/* Refresh the latched pixel periodically.  The LED's 5 V rail may become
		 * valid after the MCU, in which case a one-shot boot frame is lost.
		 */
		SendWs2812Color(scaled_r, scaled_g, scaled_b);
		ws2812_r_ = scaled_r;
		ws2812_g_ = scaled_g;
		ws2812_b_ = scaled_b;
		ws2812_color_valid_ = true;
		return;
	}
#endif

	const uint8_t peak = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
	if (peak == 0U) {
		(void)gpio_pin_set_dt(&led_r_, 0);
		(void)gpio_pin_set_dt(&led_g_, 0);
		(void)gpio_pin_set_dt(&led_b_, 0);
		return;
	}

	const uint32_t on_ms = (static_cast<uint32_t>(peak) * kLedFrameMs) / (kPwmLevels - 1U);

	(void)gpio_pin_set_dt(&led_r_, (r > 0U) ? 1 : 0);
	(void)gpio_pin_set_dt(&led_g_, (g > 0U) ? 1 : 0);
	(void)gpio_pin_set_dt(&led_b_, (b > 0U) ? 1 : 0);
	if (on_ms > 0U) {
		k_sleep(K_MSEC(on_ms));
	}

	(void)gpio_pin_set_dt(&led_r_, 0);
	(void)gpio_pin_set_dt(&led_g_, 0);
	(void)gpio_pin_set_dt(&led_b_, 0);
}

void SysStateModule::ApplyBuzzerPercent(uint8_t pct)
{
	if (!buzzer_ready_) {
		return;
	}

	if (pct > 100U) {
		pct = 100U;
	}

	if (pct == buzzer_duty_pct_) {
		return;
	}

	buzzer_duty_pct_ = pct;
	const uint32_t period = PWM_USEC(kBuzzerPeriodUs);
	const uint32_t pulse = static_cast<uint32_t>((static_cast<uint64_t>(period) * pct) / 100U);
	(void)pwm_set_dt(&buzzer_, period, pulse);
}

void SysStateModule::RunLoop()
{
	g_sys_state_diag_state = kSysStateDiagRunLoopEnter;
	LOG_INF("status_indicator module started");

	uint16_t breathe_step = 0U;
	msg::SystemStatusMessage system_status = { msg::kBooting, 0U };
	wbr_control::scheduling::AbsolutePeriodicSchedule release(
		kLedFrameMs, modules::scheduling::thread_phase_ms::kSystemState);

	while (true) {
		(void)release.WaitForNextRelease();
		missed_release_count_ = release.total_missed_releases();
		const uint8_t brightness_pct = TriangleBrightnessPercent(breathe_step);
		const uint8_t duty = PercentToDuty(brightness_pct);
		(void)zbus_chan_read(&wbr_control_system_status_chan, &system_status, K_NO_WAIT);

		if (led_ready_) {
			if (system_status.phase == msg::kRunning) {
				ApplyDuty(0U, static_cast<uint8_t>(kPwmLevels - 1U), 0U);
			} else {
				ApplyDuty(duty, duty / 3U, 0U);
			}
		}

		const uint8_t buzzer_pct = system_status.phase == msg::kBooting &&
						    breathe_step < kBuzzerTickSteps
						? kBuzzerTickPercent
						: 0U;
		ApplyBuzzerPercent(buzzer_pct);

		++breathe_step;
		if (breathe_step >= kBreathSteps) {
			breathe_step = 0U;
		}
	}
}

} // namespace modules
