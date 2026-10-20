/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdatomic.h>

struct device { void *data; };
typedef struct {
    uint32_t current_mode, current_total_size, current_offset, current_size;
    uint8_t *current_buf;
} dma_info_t;
struct uart_hpm_data {
    bool rx_line_idle_mode, rx_circular_mode;
    uint32_t rx_generation;
    struct { const struct device *dma_dev; uint32_t channel; } dma_rx;
    dma_info_t dma_rxinfo;
};
struct dma_status { bool busy; uint32_t pending_length; };
#define UART_RX_RDY 1
struct uart_event {
    int type;
    union { struct { uint8_t *buf; uint32_t len, offset; } rx; } data;
};
static uint8_t buffer[128], next_buffer[128], published[128];
static struct uart_hpm_data state;
static struct device device = { &state };
static struct dma_status statuses[2];
static int status_errors[2], status_calls, suspend_calls, resume_calls;
static int events, bytes, callback_action, lock_depth;
static unsigned int irq_lock(void) { return lock_depth++; }
static void irq_unlock(unsigned int key) { assert(lock_depth == (int)key+1); --lock_depth; }
static int dma_get_status(const struct device *dev, uint32_t channel, struct dma_status *status)
{
    (void)dev; (void)channel;
    assert(status_calls < 2);
    *status = statuses[status_calls];
    return status_errors[status_calls++];
}
static int dma_suspend(const struct device *dev, uint32_t channel)
{ (void)dev; (void)channel; ++suspend_calls; return 0; }
static int dma_resume(const struct device *dev, uint32_t channel)
{ (void)dev; (void)channel; ++resume_calls; return 0; }
static void async_user_callback(const struct device *dev, struct uart_event *event)
{
    assert(dev == &device && event->type == UART_RX_RDY);
    assert(event->data.rx.len && event->data.rx.offset + event->data.rx.len <= 128);
    assert(state.dma_rxinfo.current_offset == event->data.rx.offset + event->data.rx.len);
    assert(bytes + (int)event->data.rx.len <= 128);
    memcpy(published + bytes, event->data.rx.buf + event->data.rx.offset, event->data.rx.len);
    bytes += event->data.rx.len;
    ++events;
    if (callback_action) {
        ++state.rx_generation;
        state.dma_rxinfo.current_mode = callback_action == 2;
        state.dma_rxinfo.current_buf = next_buffer;
        state.dma_rxinfo.current_offset = 0;
    }
}
#include "driver_functions.inc"
static void reset(void)
{
    memset(&state, 0, sizeof(state));
    state.rx_line_idle_mode = true;
    state.rx_generation = 10;
    state.dma_rxinfo.current_mode = 1;
    state.dma_rxinfo.current_total_size = 128;
    state.dma_rxinfo.current_buf = buffer;
    for (int i=0; i<128; ++i) buffer[i] = i;
    memset(status_errors, 0, sizeof(status_errors));
    status_calls = suspend_calls = resume_calls = events = bytes = callback_action = lock_depth = 0;
}
static void pending(uint32_t count)
{
    status_calls = 0;
    statuses[0] = statuses[1] = (struct dma_status){true, count};
}
int main(void)
{
    reset(); pending(103); // True idle with a 25-byte tail; no future bytes are needed.
    assert(uart_hpm_rx_line_idle_flush(&device) == 25);
    assert(events == 1 && bytes == 25 && !memcmp(buffer, published, 25));
    pending(103); // Repeated idle must not publish duplicate bytes.
    assert(uart_hpm_rx_line_idle_flush(&device) == 0);
    assert(events == 1 && bytes == 25);
    pending(78);
    assert(uart_hpm_rx_line_idle_flush(&device) == 25);
    assert(events == 2 && bytes == 50 && !memcmp(buffer, published, 50));
    for (int action=1; action<=2; ++action) {
        reset(); pending(103); callback_action = action;
        assert(uart_hpm_rx_line_idle_flush(&device) == 25);
        assert(resume_calls == 0 && state.rx_generation == 11);
        assert(state.dma_rxinfo.current_buf == next_buffer);
    }
    reset(); pending(0); // DMA completion owns release/switch, not this flush.
    assert(uart_hpm_rx_line_idle_flush(&device) == -EAGAIN);
    assert(!suspend_calls && !events);
    reset(); pending(1); statuses[1].pending_length = 0;
    assert(uart_hpm_rx_line_idle_flush(&device) == -EAGAIN);
    assert(suspend_calls == 1 && !resume_calls && !events);
    reset(); pending(129);
    assert(uart_hpm_rx_line_idle_flush(&device) == -EIO && !events);
    reset(); pending(100); state.dma_rxinfo.current_offset = 40;
    assert(uart_hpm_rx_line_idle_flush(&device) == -EIO && !events);
    reset(); pending(103); status_errors[1] = -EIO;
    assert(uart_hpm_rx_line_idle_flush(&device) == -EIO);
    assert(suspend_calls == 1 && !resume_calls && !events);
    reset(); pending(103); status_errors[0] = -EIO;
    assert(uart_hpm_rx_line_idle_flush(&device) == -EIO && !suspend_calls);
    reset(); state.dma_rxinfo.current_mode = 0;
    assert(uart_hpm_rx_line_idle_flush(&device) == -EAGAIN && !status_calls);
    reset(); state.rx_circular_mode = true;
    assert(uart_hpm_rx_line_idle_flush(&device) == -EAGAIN && !status_calls);
    assert(!lock_depth);
    puts("Current driver: final-tail, duplicate idle, callback disable/restart, TC race and DMA-error contracts passed");
}
