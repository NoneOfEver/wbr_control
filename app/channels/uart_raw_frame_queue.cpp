/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <zephyr/kernel.h>

#include <app/channels/uart_raw_frame_queue.h>

namespace {

constexpr size_t kQueueDepth = 32U;

K_MSGQ_DEFINE(g_remote_input_uart_raw_msgq,
	     sizeof(rm_test::app::channels::uart_raw_frame_queue::UartRawFrameMessage),
	     kQueueDepth,
	     4);
K_MSGQ_DEFINE(g_referee_uart_raw_msgq,
	     sizeof(rm_test::app::channels::uart_raw_frame_queue::UartRawFrameMessage),
	     kQueueDepth,
	     4);
K_MSGQ_DEFINE(g_mavlink_uart_raw_msgq,
	     sizeof(rm_test::app::channels::uart_raw_frame_queue::UartRawFrameMessage),
	     kQueueDepth,
	     4);

int Enqueue(struct k_msgq *msgq,
	    const rm_test::app::channels::uart_raw_frame_queue::UartRawFrameMessage *frame)
{
	if ((msgq == nullptr) || (frame == nullptr)) {
		return -EINVAL;
	}

	return k_msgq_put(msgq, frame, K_NO_WAIT);
}

int Dequeue(struct k_msgq *msgq,
	    rm_test::app::channels::uart_raw_frame_queue::UartRawFrameMessage *frame)
{
	if ((msgq == nullptr) || (frame == nullptr)) {
		return -EINVAL;
	}

	return k_msgq_get(msgq, frame, K_NO_WAIT);
}

}  // namespace

namespace rm_test::app::channels::uart_raw_frame_queue {

int EnqueueForRemoteInput(const UartRawFrameMessage *frame)
{
	return Enqueue(&g_remote_input_uart_raw_msgq, frame);
}

int DequeueForRemoteInput(UartRawFrameMessage *frame)
{
	return Dequeue(&g_remote_input_uart_raw_msgq, frame);
}

int EnqueueForReferee(const UartRawFrameMessage *frame)
{
	return Enqueue(&g_referee_uart_raw_msgq, frame);
}

int DequeueForReferee(UartRawFrameMessage *frame)
{
	return Dequeue(&g_referee_uart_raw_msgq, frame);
}

int EnqueueForMavlink(const UartRawFrameMessage *frame)
{
	return Enqueue(&g_mavlink_uart_raw_msgq, frame);
}

int DequeueForMavlink(UartRawFrameMessage *frame)
{
	return Dequeue(&g_mavlink_uart_raw_msgq, frame);
}

}  // namespace rm_test::app::channels::uart_raw_frame_queue
