/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <zephyr/kernel.h>

#include <app/channels/can_raw_frame_queue.h>

namespace {

constexpr size_t kQueueDepth = 32U;

K_MSGQ_DEFINE(g_chassis_can_raw_msgq,
	     sizeof(rm_test::app::channels::can_raw_frame_queue::CanRawFrameMessage),
	     kQueueDepth,
	     4);
K_MSGQ_DEFINE(g_arm_can_raw_msgq,
	     sizeof(rm_test::app::channels::can_raw_frame_queue::CanRawFrameMessage),
	     kQueueDepth,
	     4);
K_MSGQ_DEFINE(g_gimbal_can_raw_msgq,
	     sizeof(rm_test::app::channels::can_raw_frame_queue::CanRawFrameMessage),
	     kQueueDepth,
	     4);
K_MSGQ_DEFINE(g_gantry_can_raw_msgq,
	     sizeof(rm_test::app::channels::can_raw_frame_queue::CanRawFrameMessage),
	     kQueueDepth,
	     4);

int Enqueue(struct k_msgq *msgq,
	    const rm_test::app::channels::can_raw_frame_queue::CanRawFrameMessage *frame)
{
	if ((msgq == nullptr) || (frame == nullptr)) {
		return -EINVAL;
	}

	return k_msgq_put(msgq, frame, K_NO_WAIT);
}

int Dequeue(struct k_msgq *msgq,
	    rm_test::app::channels::can_raw_frame_queue::CanRawFrameMessage *frame)
{
	if ((msgq == nullptr) || (frame == nullptr)) {
		return -EINVAL;
	}

	return k_msgq_get(msgq, frame, K_NO_WAIT);
}

}  // namespace

namespace rm_test::app::channels::can_raw_frame_queue {

int EnqueueForChassis(const CanRawFrameMessage *frame)
{
	return Enqueue(&g_chassis_can_raw_msgq, frame);
}

int DequeueForChassis(CanRawFrameMessage *frame)
{
	return Dequeue(&g_chassis_can_raw_msgq, frame);
}

int EnqueueForArm(const CanRawFrameMessage *frame)
{
	return Enqueue(&g_arm_can_raw_msgq, frame);
}

int DequeueForArm(CanRawFrameMessage *frame)
{
	return Dequeue(&g_arm_can_raw_msgq, frame);
}

int EnqueueForGimbal(const CanRawFrameMessage *frame)
{
	return Enqueue(&g_gimbal_can_raw_msgq, frame);
}

int DequeueForGimbal(CanRawFrameMessage *frame)
{
	return Dequeue(&g_gimbal_can_raw_msgq, frame);
}

int EnqueueForGantry(const CanRawFrameMessage *frame)
{
	return Enqueue(&g_gantry_can_raw_msgq, frame);
}

int DequeueForGantry(CanRawFrameMessage *frame)
{
	return Dequeue(&g_gantry_can_raw_msgq, frame);
}

}  // namespace rm_test::app::channels::can_raw_frame_queue
