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

## A/B result and comparison with HI91

The first PLIC-enabled run completed 60.0006 s. SBUS still had 84.6-ms
parser-update gaps and 3292 chunks/105344 bytes, so unmasking PLIC alone has
not removed batching. The prior explanation is insufficient; verify the actual
image and execution path before changing reception policy.

HI91 uses continuously running circular DMA and a 1-ms software poll of the
DMA producer position. Publication therefore does not require a UART timeout
ISR or a full DMA buffer. It retains 16 outstanding bytes as a DMA-write guard.
SBUS uses finite 128-byte double buffers and publishes only on RX_RDY events,
which originate from full DMA completion or timeout-driven partial flush.
The structural defect is the missing effective partial-publication mechanism;
PLIC enable was only one prerequisite, not proof of timeout delivery.
Do not copy the 16-byte retention guard into SBUS unchanged: its lower baud and
inter-frame gaps can postpone the tail of a frame until the next transmission.

The opt-in probe now adds a URT2 marker and cumulative counters to otherwise
unused Ready bytes 28..63. Counts are ISR entries, RX_RDY callbacks, callbacks
shorter than 128 bytes, callbacks exactly 128 bytes, bytes, maximum callback gap,
and last callback length. PLIC status is sampled with the snapshot. A registered
user IRQ callback counts entries after the driver's ISR without reading IIR again.
Counters do not identify the IRQ source; an ISR count alone is not timeout proof.
No periodic diagnostic packets are emitted; the production build leaves these
Ready bytes zero and compiles out counters.

Use the updated host tool with `--probe-status` to read status before capture
and via a second stop/Hello handshake after capture. URT2 absent means this
instrumented image is not verified. URT2 + PLIC on + no ISR increment points to
interrupt delivery/peripheral conditions; ISR increment + full-only RX_RDY
points to source identification/partial-DMA flush. Partial RX_RDY with continued
host grouping moves investigation to publication/queue timing. These are
branching observations, not asserted outcomes.
