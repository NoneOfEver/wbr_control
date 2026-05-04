/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/sys/printk.h>

#include <app/bootstrap/thread_utils.h>
#include <app/modules/referee/referee_module.h>
#include <app/channels/uart_raw_frame_queue.h>
#include <platform/drivers/devices/system/referee_client.h>

namespace {

K_THREAD_STACK_DEFINE(g_referee_module_stack, 1024);

}  // namespace

namespace rm_test::app::modules::referee {

int RefereeModule::Initialize()
{
	started_ = false;
	sequence_ = 0U;
	return rm_test::platform::drivers::devices::system::referee_client::Initialize();
}

int RefereeModule::Start()
{
	if (started_) {
		return 0;
	}

	bootstrap::StartMemberThread<RefereeModule, &RefereeModule::RunLoop>(
		&thread_,
		g_referee_module_stack,
		K_THREAD_STACK_SIZEOF(g_referee_module_stack),
		this,
		K_PRIO_PREEMPT(8),
		"referee_module");

	started_ = true;
	return 0;
}

void RefereeModule::RunLoop()
{
	printk("referee module started\n");

	rm_test::app::channels::RefereeStateMessage state = {};
	while (true) {
		DecodeUartFramesInQueue();

		if (rm_test::platform::drivers::devices::system::referee_client::GetLatestState(&state) == 0) {
			state.sequence = ++sequence_;
			(void)zbus_chan_pub(&rm_test_referee_state_chan, &state, K_NO_WAIT);
		}
		k_sleep(K_MSEC(20));
	}
}

void RefereeModule::DecodeUartFramesInQueue()
{
	while (true) {
		rm_test::app::channels::uart_raw_frame_queue::UartRawFrameMessage frame = {};
		if (rm_test::app::channels::uart_raw_frame_queue::DequeueForReferee(&frame) != 0) {
			break;
		}

		(void)rm_test::platform::drivers::devices::system::referee_client::FeedBytes(
			frame.data,
			frame.len);
	}
}

}  // namespace rm_test::app::modules::referee
