/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_CHANNELS_UART_RAW_FRAME_QUEUE_H_
#define RM_TEST_APP_CHANNELS_UART_RAW_FRAME_QUEUE_H_

#include <stddef.h>
#include <stdint.h>

namespace rm_test::app::channels::uart_raw_frame_queue {

constexpr size_t kUartRawChunkSize = 32U;

struct UartRawFrameMessage {
	uint8_t len;
	uint8_t data[kUartRawChunkSize];
};

int EnqueueForRemoteInput(const UartRawFrameMessage *frame);
int DequeueForRemoteInput(UartRawFrameMessage *frame);

int EnqueueForReferee(const UartRawFrameMessage *frame);
int DequeueForReferee(UartRawFrameMessage *frame);

int EnqueueForMavlink(const UartRawFrameMessage *frame);
int DequeueForMavlink(UartRawFrameMessage *frame);

}  // namespace rm_test::app::channels::uart_raw_frame_queue

#endif /* RM_TEST_APP_CHANNELS_UART_RAW_FRAME_QUEUE_H_ */
