# UART0 asynchronous TX test

This test validates asynchronous UART0 DMA transmission and abort/recovery.
All diagnostic text uses RTT, matching the main application.
UART0 sends only VOFA JustFloat data at 921600 baud every 10 ms.

Each frame is 112 bytes: 27 little-endian floats followed by `00 00 80 7f`.
Channels 0–2 are sequence, TX done count, and TX abort count; the rest are zero.
The startup abort self-test may leave a partial frame before complete frames resume.

For dust-hpm5e3y, from the workspace root:

```sh
west build -p always -b dust-hpm5e3y -s wbr_control/test/uart0_async_tx_test -d wbr_control/build-hpm5e3y-uart-probe
west flash -d wbr_control/build-hpm5e3y-uart-probe
west rtt -d wbr_control/build-hpm5e3y-uart-probe
```

RTT should report `UART0 async abort/recovery OK`. Failure and timeout messages
also appear on RTT. Use VOFA JustFloat decoding on UART0 to inspect the counters.

For dust-hpm6750, replace the board with `dust-hpm6750` and use a separate
build directory. Its app overlay selects XDMA, while dust-hpm5e3y uses the
board's HDMA assignment.
