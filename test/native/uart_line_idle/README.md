# Current-driver UART line-idle contracts

From the Zephyr workspace:

```bash
python3 wbr_control/test/native/uart_line_idle/run.py
```

Requires Python 3 and a native C11 compiler (`CC`, or `cc`). The runner extracts
`uart_hpm_rx_line_idle_flush` and `async_evt_rx_rdy` directly from the current
sibling HPM driver, rather than maintaining a second implementation. A minimal
DMA/callback backend controls pending lengths, completion and failures.
The RISC-V MMIO fence is replaced only for host compilation by a native atomic
fence. No claim about actual DMA visibility or peripheral timing follows.

Covered: a final 25-byte partial buffer with no later data; repeated idle without
duplicate delivery; contiguous publication of a subsequent range; synchronous
callback disable and restart generation changes; TC before/during suspend;
invalid remaining counts/offsets; DMA status errors before/after suspend; and
inactive/circular-mode rejection.

The callback lifecycle cases exercise the flush's response to state changes,
not the complete real `uart_rx_disable`/`uart_rx_enable` APIs. Timer/TRGM edge
handling, actual UART silence, full hardware lifecycle and physical USB detach
remain hardware tests. These tests must not be cited as proof of a real final
wire tail being delivered.
