/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <hpm_l1c_drv.h>

#include <channels/uart_raw_frame_queue.h>

#include "uart_dispatch.h"

LOG_MODULE_REGISTER(uart_dispatch, LOG_LEVEL_INF);

namespace {

#if !DT_HAS_CHOSEN(rm_test_remote_input_uart)
#error "rm-test,remote-input-uart must be selected for uart_dispatch"
#endif

#define RM_TEST_REMOTE_INPUT_UART_NODE DT_CHOSEN(rm_test_remote_input_uart)
#define RM_TEST_CONSOLE_UART_NODE DT_CHOSEN(zephyr_console)
#if DT_HAS_CHOSEN(zephyr_shell_uart)
#define RM_TEST_SHELL_UART_NODE DT_CHOSEN(zephyr_shell_uart)
#else
#define RM_TEST_SHELL_UART_NODE RM_TEST_CONSOLE_UART_NODE
#endif

#if defined(CONFIG_UART_ASYNC_API) && CONFIG_UART_ASYNC_API
constexpr size_t kUartRxBufSize = 128;
constexpr int32_t kRxIdleTimeoutUs = 1000;
#define UART_DMA_ALIGN __attribute__((aligned(HPM_L1C_CACHELINE_SIZE)))
#define UART_DISPATCH_ASYNC 1
#else
#define UART_DISPATCH_ASYNC 0
#endif
#if !UART_DISPATCH_ASYNC
constexpr size_t kUartPollStackSize = 1024;
constexpr size_t kUartPollRxChunkSize = 64;
constexpr int kUartPollThreadPriority = 10;
#endif

enum class UartRoute {
  kMavlink,
  kReferee,
  kRemoteInput,
};

struct UartRxPort {
  const struct device *dev;
  UartRoute route;
#if defined(CONFIG_UART_ASYNC_API) && CONFIG_UART_ASYNC_API
  uint8_t *rx_buf_a;
  uint8_t *rx_buf_b;
  uint8_t *next_rx_buf;
#endif
  bool started;
};

#if defined(CONFIG_UART_ASYNC_API) && CONFIG_UART_ASYNC_API
uint8_t g_remote_input_rx_buf_a[kUartRxBufSize] UART_DMA_ALIGN;
uint8_t g_remote_input_rx_buf_b[kUartRxBufSize] UART_DMA_ALIGN;
#if DT_HAS_CHOSEN(rm_test_referee_uart)
uint8_t g_referee_rx_buf_a[kUartRxBufSize] UART_DMA_ALIGN;
uint8_t g_referee_rx_buf_b[kUartRxBufSize] UART_DMA_ALIGN;
#endif
#if DT_HAS_CHOSEN(rm_test_mavlink_uart)
uint8_t g_mavlink_rx_buf_a[kUartRxBufSize] UART_DMA_ALIGN;
uint8_t g_mavlink_rx_buf_b[kUartRxBufSize] UART_DMA_ALIGN;
#endif
#endif

UartRxPort g_remote_input_uart = {
    .dev = nullptr,
    .route = UartRoute::kRemoteInput,
#if defined(CONFIG_UART_ASYNC_API) && CONFIG_UART_ASYNC_API
    .rx_buf_a = g_remote_input_rx_buf_a,
    .rx_buf_b = g_remote_input_rx_buf_b,
    .next_rx_buf = g_remote_input_rx_buf_b,
#endif
    .started = false,
};

#if DT_HAS_CHOSEN(rm_test_referee_uart)
UartRxPort g_referee_uart = {
    .dev = nullptr,
    .route = UartRoute::kReferee,
#if defined(CONFIG_UART_ASYNC_API) && CONFIG_UART_ASYNC_API
    .rx_buf_a = g_referee_rx_buf_a,
    .rx_buf_b = g_referee_rx_buf_b,
    .next_rx_buf = g_referee_rx_buf_b,
#endif
    .started = false,
};
#endif

#if DT_HAS_CHOSEN(rm_test_mavlink_uart)
UartRxPort g_mavlink_uart = {
    .dev = nullptr,
    .route = UartRoute::kMavlink,
#if defined(CONFIG_UART_ASYNC_API) && CONFIG_UART_ASYNC_API
    .rx_buf_a = g_mavlink_rx_buf_a,
    .rx_buf_b = g_mavlink_rx_buf_b,
    .next_rx_buf = g_mavlink_rx_buf_b,
#endif
    .started = false,
};
#endif

bool g_started = false;
uint32_t g_remote_input_rx_events = 0U;
#if !UART_DISPATCH_ASYNC
K_THREAD_STACK_DEFINE(g_remote_input_poll_stack, kUartPollStackSize);
struct k_thread g_remote_input_poll_thread;
#endif

#if UART_DISPATCH_ASYNC
UartRxPort *FindPort(const struct device *dev) {
  if (dev == g_remote_input_uart.dev) {
    return &g_remote_input_uart;
  }

#if DT_HAS_CHOSEN(rm_test_referee_uart)
  if (dev == g_referee_uart.dev) {
    return &g_referee_uart;
  }
#endif

#if DT_HAS_CHOSEN(rm_test_mavlink_uart)
  if (dev == g_mavlink_uart.dev) {
    return &g_mavlink_uart;
  }
#endif

  return nullptr;
}
#endif

void RouteRemoteInputBytes(const uint8_t *data, size_t len) {
  if ((data == nullptr) || (len == 0U)) {
    return;
  }

  const uint32_t written =
      ring_buf_put(&channels::uart_raw_frame_queue::remote_input_ring_buf, data,
                   static_cast<uint32_t>(len));
  if (written > 0U) {
    k_sem_give(&channels::uart_raw_frame_queue::remote_input_sem);
  }
}

void RouteMessageQueueBytes(struct k_msgq *queue, const uint8_t *data,
                            size_t len) {
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
    (void)k_msgq_put(queue, &msg, K_NO_WAIT);
  }
}

void RouteUartBytes(const UartRxPort &port, const uint8_t *data, size_t len) {
  switch (port.route) {
  case UartRoute::kMavlink:
    RouteMessageQueueBytes(
        &channels::uart_raw_frame_queue::mavlink_uart_raw_msgq, data, len);
    break;
  case UartRoute::kReferee:
    RouteMessageQueueBytes(
        &channels::uart_raw_frame_queue::referee_uart_raw_msgq, data, len);
    break;
  case UartRoute::kRemoteInput:
    RouteRemoteInputBytes(data, len);
    break;
  }
}

void LogRemoteInputRxSample(const uint8_t *data, size_t len) {
  if ((data == nullptr) || (len == 0U)) {
    return;
  }

  ++g_remote_input_rx_events;
  if ((g_remote_input_rx_events > 10U) &&
      ((g_remote_input_rx_events % 30U) != 0U)) {
    return;
  }

  const uint8_t b0 = (len > 0U) ? data[0] : 0U;
  const uint8_t b1 = (len > 1U) ? data[1] : 0U;
  const uint8_t b2 = (len > 2U) ? data[2] : 0U;
  const uint8_t b3 = (len > 3U) ? data[3] : 0U;
  // LOG_INF("remote uart rx event=%u len=%u head=%02x %02x %02x %02x",
  //         g_remote_input_rx_events, static_cast<unsigned int>(len), b0, b1, b2,
  //         b3);
}

#if UART_DISPATCH_ASYNC
void InvalidateDmaRxCache(const uint8_t *data, size_t len) {
  if ((data == nullptr) || (len == 0U)) {
    return;
  }

  const uint32_t start = HPM_L1C_CACHELINE_ALIGN_DOWN(
      reinterpret_cast<uint32_t>(data));
  const uint32_t end = HPM_L1C_CACHELINE_ALIGN_UP(
      reinterpret_cast<uint32_t>(data) + static_cast<uint32_t>(len));
  l1c_dc_invalidate(start, end - start);
}

void ResetAsyncRxBuffers(UartRxPort *port) {
  if (port != nullptr) {
    port->next_rx_buf = port->rx_buf_b;
  }
}

uint8_t *TakeNextAsyncRxBuffer(UartRxPort *port) {
  if ((port == nullptr) || (port->next_rx_buf == nullptr)) {
    return nullptr;
  }

  uint8_t *next_buf = port->next_rx_buf;
  port->next_rx_buf =
      (next_buf == port->rx_buf_a) ? port->rx_buf_b : port->rx_buf_a;
  return next_buf;
}

void UartRxCallback(const struct device *dev, struct uart_event *evt,
                    void *user_data) {
  ARG_UNUSED(user_data);

  UartRxPort *port = FindPort(dev);
  if (port == nullptr) {
    return;
  }

  switch (evt->type) {
  case UART_RX_RDY: {
    const uint8_t *src = evt->data.rx.buf + evt->data.rx.offset;
    InvalidateDmaRxCache(src, evt->data.rx.len);
    if (port->route == UartRoute::kRemoteInput) {
      LogRemoteInputRxSample(src, evt->data.rx.len);
    }
    RouteUartBytes(*port, src, evt->data.rx.len);
    break;
  }
  case UART_RX_BUF_REQUEST: {
    uint8_t *next_buf = TakeNextAsyncRxBuffer(port);
    if (next_buf != nullptr) {
      (void)uart_rx_buf_rsp(port->dev, next_buf, kUartRxBufSize);
    }
    break;
  }
  case UART_RX_DISABLED:
    ResetAsyncRxBuffers(port);
    (void)uart_rx_enable(port->dev, port->rx_buf_a, kUartRxBufSize,
                         kRxIdleTimeoutUs);
    break;
  default:
    break;
  }
}
#endif

bool IsProtectedUartNode(const struct device *dev) {
  const struct device *console = DEVICE_DT_GET(RM_TEST_CONSOLE_UART_NODE);
  const struct device *shell = DEVICE_DT_GET(RM_TEST_SHELL_UART_NODE);

  return (dev == console) || (dev == shell);
}

bool IsAlreadyStarted(const struct device *dev) {
  return (g_remote_input_uart.started && (dev == g_remote_input_uart.dev))
#if DT_HAS_CHOSEN(rm_test_referee_uart)
         || (g_referee_uart.started && (dev == g_referee_uart.dev))
#endif
#if DT_HAS_CHOSEN(rm_test_mavlink_uart)
         || (g_mavlink_uart.started && (dev == g_mavlink_uart.dev))
#endif
      ;
}

#if UART_DISPATCH_ASYNC
int StartAsyncRx(UartRxPort *port) {
  if ((port == nullptr) || (port->dev == nullptr) || !device_is_ready(port->dev)) {
    return -ENODEV;
  }

  if (IsProtectedUartNode(port->dev)) {
    return 0;
  }

  if (IsAlreadyStarted(port->dev)) {
    return 0;
  }

  if (uart_callback_set(port->dev, UartRxCallback, nullptr) != 0) {
    return -ENOTSUP;
  }

  ResetAsyncRxBuffers(port);
  if (uart_rx_enable(port->dev, port->rx_buf_a, kUartRxBufSize,
                     kRxIdleTimeoutUs) != 0) {
    return -EIO;
  }

  port->started = true;
  LOG_INF("uart async rx started for %s", port->dev->name);
  return 0;
}
#endif

#if !UART_DISPATCH_ASYNC
void UartPollThread(void *port_arg, void *, void *) {
  UartRxPort *port = static_cast<UartRxPort *>(port_arg);
  if ((port == nullptr) || (port->dev == nullptr)) {
    return;
  }

  uint32_t chunk_len = 0U;
  uint8_t chunk[kUartPollRxChunkSize];
  while (true) {
    uint8_t byte = 0U;
    if (uart_poll_in(port->dev, &byte) == 0) {
      chunk[chunk_len++] = byte;
      if (chunk_len >= sizeof(chunk)) {
        if (port->route == UartRoute::kRemoteInput) {
          LogRemoteInputRxSample(chunk, chunk_len);
        }
        RouteUartBytes(*port, chunk, chunk_len);
        chunk_len = 0U;
      }
      continue;
    }

    if (chunk_len > 0U) {
      if (port->route == UartRoute::kRemoteInput) {
        LogRemoteInputRxSample(chunk, chunk_len);
      }
      RouteUartBytes(*port, chunk, chunk_len);
      chunk_len = 0U;
    }

    k_sleep(K_USEC(500));
  }
}

int StartPollingRx(UartRxPort *port, k_thread_stack_t *stack,
                   struct k_thread *thread) {
  if ((port == nullptr) || (port->dev == nullptr) || !device_is_ready(port->dev)) {
    return -ENODEV;
  }

  if (IsProtectedUartNode(port->dev)) {
    return 0;
  }

  if (IsAlreadyStarted(port->dev)) {
    return 0;
  }

  uint8_t dummy;
  while (uart_poll_in(port->dev, &dummy) == 0) {
  }

  k_thread_create(thread, stack, kUartPollStackSize, UartPollThread, port,
                  nullptr, nullptr, kUartPollThreadPriority, 0, K_NO_WAIT);
  port->started = true;
  LOG_INF("uart polling rx started for %s", port->dev->name);
  return 0;
}
#endif

} // namespace

namespace platform::drivers::communication::uart_dispatch {

int Initialize() {
  if (g_started) {
    return 0;
  }

  g_remote_input_uart.dev = DEVICE_DT_GET(RM_TEST_REMOTE_INPUT_UART_NODE);
#if DT_HAS_CHOSEN(rm_test_referee_uart)
  g_referee_uart.dev = DEVICE_DT_GET(DT_CHOSEN(rm_test_referee_uart));
#endif
#if DT_HAS_CHOSEN(rm_test_mavlink_uart)
  g_mavlink_uart.dev = DEVICE_DT_GET(DT_CHOSEN(rm_test_mavlink_uart));
#endif

  if ((g_remote_input_uart.dev == nullptr) ||
      !device_is_ready(g_remote_input_uart.dev)) {
    return -ENODEV;
  }

#if defined(CONFIG_UART_ASYNC_API) && CONFIG_UART_ASYNC_API
  int rc = StartAsyncRx(&g_remote_input_uart);
  if (rc != 0) {
    return rc;
  }

#if DT_HAS_CHOSEN(rm_test_referee_uart)
  rc = StartAsyncRx(&g_referee_uart);
  if (rc != 0) {
    return rc;
  }
#endif

#if DT_HAS_CHOSEN(rm_test_mavlink_uart)
  rc = StartAsyncRx(&g_mavlink_uart);
  if (rc != 0) {
    return rc;
  }
#endif
#else
  int rc = StartPollingRx(&g_remote_input_uart, g_remote_input_poll_stack,
                          &g_remote_input_poll_thread);
  if (rc != 0) {
    return rc;
  }
#endif
  g_started = true;
  return 0;
}

} // namespace platform::drivers::communication::uart_dispatch
