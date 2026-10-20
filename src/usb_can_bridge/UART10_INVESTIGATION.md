# UART10 receive batching investigation — 2026-10-21

The saved NUC `uart_forwarding.csv` contains 1460 SBUS chunks (46720 bytes).
Grouping consecutive MCU observation timestamps within 200 us gives exactly
365 groups of `[40,40,40,8]` bytes: every group is one 128-byte DMA buffer.
MCU observation gaps themselves have p99 84493 us and maximum 84593 us.
Thus the long gaps already exist before USB publication, rather than appearing
only at the NUC callback. Chunk sequences have no gaps, which does not establish
that UART wire reception itself has lost no bytes.

## Source-level cause to verify on hardware

In `hpm_support/drivers/serial/uart_hpmicro.c`:

1. `UART_HPMICRO_IRQ_FUNC_DEFINE_SYNC` connects the ISR but enables its PLIC
   source only when `!cfg->async_capable`. UART10 has DMA and is async capable.
2. `uart_hpm_rx_enable` enables `uart_intr_rx_data_avail_or_timeout` in the UART,
   but never calls `irq_enable(config->irq_num)`.
3. A masked PLIC source prevents `uart_hpm_isr` from calling
   `uart_hpm_handle_rx_timeout`, which would flush the partial DMA buffer.
4. The separate HDMA interrupt remains enabled. A full DMA buffer reaches
   `uart_hpm_dma_callback`, flushes 128 bytes and switches buffers.
5. The gateway splits these bytes into 40/40/40/8, matching all 365 groups.

This is a concrete interrupt-enable omission and a strong explanation of the
capture. It is not yet an on-hardware A/B confirmation of the flashed image.
The RX-enable API's `1000` argument is only stored by this driver, not used to
program the timeout; the ISR uses the UART's hardware timeout condition.

## Controlled A/B experiment

The normal build keeps `CONFIG_WBR_UART10_PLIC_PROBE=n`. The probe sets only the
remote UART PLIC enable bit after successful async RX start. It changes no DMA
buffer size, FIFO threshold, baud, SBUS parser, USB format or forwarding policy.
It does not modify the shared HPM driver or enable the HI91 UART interrupt.

Build from the Zephyr workspace:

```bash
west build -b dust-hpm6750 -s wbr_control/src/usb_can_bridge \
  -d /tmp/wbr-uart10-plic-baseline -- -DCONFIG_WBR_UART10_PLIC_PROBE=n
west build -b dust-hpm6750 -s wbr_control/src/usb_can_bridge \
  -d /tmp/wbr-uart10-plic-probe -- -DCONFIG_WBR_UART10_PLIC_PROBE=y
```

Both configurations have compiled successfully. Keep the robot disabled and the
same transmitter operating; do not run the chassis controller during this test.
First preserve the baseline CSV, flash the probe with the usual board programmer,
and run the same UART capture. On the Mac workspace the already built probe is:

```bash
west flash -d /tmp/wbr-uart10-plic-probe
```

On NUC, from its `wbr_control_companion` checkout:

```bash
sudo ./scripts/run_uart_forwarding.sh --seconds 60 \
  --csv ~/wbr_realtime_lab/results/uart_plic_enabled.csv
python3 chassis_controller/tools/analyze_uart_capture.py \
  ~/wbr_realtime_lab/results/uart_plic_enabled.csv
```

Expected evidence if this explanation is correct: partial-buffer publications
appear; the exclusive 128-byte pattern disappears; MCU observation gaps and
host parser-update gaps move toward the actual SBUS frame cadence. No specific
cadence is assumed before measuring the transmitter. Check sequence gaps,
continued HI91 reception and actual capture duration as well. A persistent long
gap or failed acquisition disproves the sufficiency of this change and needs
further UART/IIR/DMA tracing. Do not promote this probe to the default yet.
To restore the comparison baseline: `west flash -d /tmp/wbr-uart10-plic-baseline`.

## Other findings from the same CSV

HI91 queue maximum 6987 us is at the beginning of capture. After excluding only
the first second for a separately labelled steady-state statistic, p50/p99/max
are 46/132/279 us. Keep the original full-capture maximum; this is not proof
that the startup delay can be ignored in control.

The host data span is 26.658 s, despite a requested duration of 60 s. The old
capture did not report elapsed time or stop reason, so the exact cause cannot
be recovered from that CSV. The host tool now reports duration/signal/error and
fails when Send returns false even if UsbLink has no stored error string.

Total UART-wire-to-NUC latency remains unmeasured: pre-observation buffering and
post-submit USB transit cannot be recovered by subtracting unsynchronized clocks.
