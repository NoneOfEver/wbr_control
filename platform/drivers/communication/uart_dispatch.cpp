/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <channels/uart_raw_frame_queue.h>

#include "uart_dispatch.h"

namespace {

#if DT_HAS_CHOSEN(zephyr_uart_mcumgr)
#define RM_TEST_UART_DISPATCH_NODE DT_CHOSEN(zephyr_uart_mcumgr)
#else
#define RM_TEST_UART_DISPATCH_NODE DT_CHOSEN(zephyr_shell_uart)
#endif

#if defined(CONFIG_UART_ASYNC_API) && CONFIG_UART_ASYNC_API
constexpr size_t kUartRxBufSize = 128;
constexpr int32_t kRxIdleTimeoutUs = 1000;
#endif

const struct device *g_uart_dev = nullptr;
bool g_started = false;

#if defined(CONFIG_UART_ASYNC_API) && CONFIG_UART_ASYNC_API
uint8_t g_uart_rx_buf_a[kUartRxBufSize];
uint8_t g_uart_rx_buf_b[kUartRxBufSize];

void RouteUartBytesToModuleQueues(const uint8_t *data, size_t len)
{
	if ((data == nullptr) || (len == 0U)) {
		return;
	}

	size_t offset = 0U;
	while (offset < len) {
		channels::uart_raw_frame_queue::UartRawFrameMessage msg = {};
		const size_t step =
			((len - offset) > channels::uart_raw_frame_queue::kUartRawChunkSize)
				? channels::uart_raw_frame_queue::kUartRawChunkSize
				: (len - offset);
		msg.len = static_cast<uint8_t>(step);
		memcpy(msg.data, data + offset, step);
		offset += step;
		(void)channels::uart_raw_frame_queue::EnqueueForRemoteInput(&msg);
		(void)channels::uart_raw_frame_queue::EnqueueForReferee(&msg);
		(void)channels::uart_raw_frame_queue::EnqueueForMavlink(&msg);
	}
}

void UartRxCallback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case UART_RX_RDY: {
		const uint8_t *src = evt->data.rx.buf + evt->data.rx.offset;
		RouteUartBytesToModuleQueues(src, evt->data.rx.len);
		break;
	}
	case UART_RX_BUF_REQUEST: {
		uint8_t *next_buf = (evt->data.rx_buf.buf == g_uart_rx_buf_a)
					 ? g_uart_rx_buf_b
					 : g_uart_rx_buf_a;
		(void)uart_rx_buf_rsp(g_uart_dev, next_buf, sizeof(g_uart_rx_buf_a));
		break;
	}
	case UART_RX_DISABLED:
		(void)uart_rx_enable(g_uart_dev,
				    g_uart_rx_buf_a,
				    sizeof(g_uart_rx_buf_a),
				    kRxIdleTimeoutUs);
		break;
	default:
		break;
	}
}
#endif

}  // namespace

namespace platform::drivers::communication::uart_dispatch {

int Initialize()
{
	if (g_started) {
		return 0;
	}

#if DT_SAME_NODE(RM_TEST_UART_DISPATCH_NODE, DT_CHOSEN(zephyr_console))
	/* Avoid attaching async RX callback on the same UART used by printk/console. */
	printk("uart_dispatch skipped: dispatch uart is console\n");
	return 0;
#endif

	g_uart_dev = DEVICE_DT_GET(RM_TEST_UART_DISPATCH_NODE);
	if ((g_uart_dev == nullptr) || !device_is_ready(g_uart_dev)) {
		return -ENODEV;
	}

#if defined(CONFIG_UART_ASYNC_API) && CONFIG_UART_ASYNC_API
	if (uart_callback_set(g_uart_dev, UartRxCallback, nullptr) != 0) {
		return -ENOTSUP;
	}

	if (uart_rx_enable(g_uart_dev,
			   g_uart_rx_buf_a,
			   sizeof(g_uart_rx_buf_a),
			   kRxIdleTimeoutUs) != 0) {
		return -EIO;
	}
#else
	printk("uart_dispatch skipped: async UART API disabled\n");
	g_started = true;
	return 0;
#endif
	g_started = true;
	return 0;
}

}  // namespace platform::drivers::communication::uart_dispatch
