/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_CHANNELS_CAN_RAW_FRAME_QUEUE_H_
#define RM_TEST_APP_CHANNELS_CAN_RAW_FRAME_QUEUE_H_

#include <stdint.h>

namespace rm_test::app::channels::can_raw_frame_queue {

struct CanRawFrameMessage {
	uint8_t bus;
	uint16_t can_id;
	uint8_t dlc;
	uint8_t data[8];
};

int EnqueueForChassis(const CanRawFrameMessage *frame);
int DequeueForChassis(CanRawFrameMessage *frame);

int EnqueueForArm(const CanRawFrameMessage *frame);
int DequeueForArm(CanRawFrameMessage *frame);

int EnqueueForGimbal(const CanRawFrameMessage *frame);
int DequeueForGimbal(CanRawFrameMessage *frame);

int EnqueueForGantry(const CanRawFrameMessage *frame);
int DequeueForGantry(CanRawFrameMessage *frame);

}  // namespace rm_test::app::channels::can_raw_frame_queue

#endif /* RM_TEST_APP_CHANNELS_CAN_RAW_FRAME_QUEUE_H_ */
