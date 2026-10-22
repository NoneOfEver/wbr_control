> Historical investigation: the probes and diagnostic pages below were removed on 2026-10-22. UART10 line-idle is now production-default; use README.md and patches/hpm_uart_line_idle.patch, not the old probe configuration.

# UART10 receive batching investigation — 2026-10-21

**Current result:** the opt-in `CONFIG_WBR_UART10_LINE_IDLE` candidate uses the
confirmed PB13/TRGM1/GPTMR2 route and removes the observed 128-byte batching.
Two 60-second enabled runs show approximately 14-ms SBUS updates, maximum
about 15.5 ms, with no idle-handler error; the second also checks RX offsets.
It retains 128-byte double buffers and defaults off. Earlier hypotheses and
experiments below are preserved as history; current implementation and limits
are described in the final section. Wire losslessness and true tail-after-stop
delivery are not established by the length-only captures.

The saved NUC `uart_forwarding.csv` contains 1460 SBUS chunks (46720 bytes).
Grouping consecutive MCU observation timestamps within 200 us gives exactly
365 groups of `[40,40,40,8]` bytes: every group is one 128-byte DMA buffer.
MCU observation gaps themselves have p99 84493 us and maximum 84593 us.
Thus the long gaps already exist before USB publication, rather than appearing
only at the NUC callback. Chunk sequences have no gaps, which does not establish
that UART wire reception itself has lost no bytes.

## Initial hypothesis (insufficient; retained as experiment history)

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

## URT3: classify the driver path

The confirmed URT2 capture had ISR +8434, RX_RDY +824, partial +0,
full128 +824. This establishes IRQ delivery but not timeout delivery.
URT3 observes the driver's existing IIR read (no second register read) and
instruments only the timeout-driven DMA flush, keeping reception unchanged.

The local HPM UART driver patch is packaged at
`diagnostics/hpm_uart_rx_probe.patch`. It is separate from unrelated HPM CAN
changes and is not pushed to the upstream module repository. On a fresh module
checkout, from the Zephyr workspace apply it once before building the probe:

```bash
git -C hpm_support apply --check ../wbr_control/src/usb_can_bridge/diagnostics/hpm_uart_rx_probe.patch
git -C hpm_support apply ../wbr_control/src/usb_can_bridge/diagnostics/hpm_uart_rx_probe.patch
```

Already patched: `git -C hpm_support apply --reverse --check PATH_TO_PATCH`
will succeed instead. Do not apply twice. The local Mac tree is already patched.
Normal builds compile out the hooks. The probe requires the driver's version
function, so a missing driver patch causes a link error rather than a false
URT3 marker with empty counters.

Driver hook events: 1 = IIR ID; 2 = entry into timeout flush;
3 = DMA total, previously published offset, and pending length after successful
`dma_get_status`; 4 = calculated new length and DMA status return code.
The application filters these events to UART10. Snapshot fields are:

- `irq_rx_available`, `irq_timeout`, `irq_other`: source counts; `last_irq_id`
  is the last observed ID, in decimal (4 = RX available, 12 = timeout).
- `timeout_flush`, `zero_flush`, `dma_status_errors`: timeout path entry,
  successful status with zero new bytes, and failed DMA status calls.
- `last_total`, `last_offset`, `last_pending`, `last_added`: most recent
  timeout-flush values, not a histogram. Before any such flush these are zero;
  after a status failure the last successful lengths remain. Values pack into
  16-bit fields, sufficient for this unchanged 128-byte buffer. Flags bit 2
  reports an oversized calculated length; it must not be interpreted as valid.

Marker is now URT3. The host still decodes URT2 with its old field meanings.
Run with `--probe-status` and use a new CSV name such as `uart_probe_v3.csv`.
Compare before/after counters: timeout zero means no timeout identified; timeout
nonzero but flush unchanged suggests the circular-mode skip; flush increases
with zero/status errors explains lack of new publication. A positive new-byte
count requires checking downstream RX_RDY and host data. Counts include short
stop/handshake intervals outside capture, as in URT2.

## URT4: hardware semantics and disjoint IIR classes

Sources checked against the local HPM6700 SDK and original manufacturer PDFs:

- [HPM6700/6400 User Manual Rev2.3](https://www.hpmicro.com/Public/Uploads/uploadfile/files/20240822/HPM67006400UMV23.pdf),
  section 55.4.8, printed page 799 (PDF page 842): FIFO timeout requires at least
  one unread character and four character times without a FIFO read/write.
  RBR reads emptying the FIFO clear timeout. **Reading IIR clears TX empty,
  not RX timeout.** The earlier driver comment claiming an IIR timeout ACK was
  incorrect and has been corrected.
- [HPM6700/6400 Errata Rev2.3](https://doc.hpmicro.com/Public/Uploads/uploadfile/files/20240923/HPM67006400ErrataV23.pdf),
  E00018, printed page 16 (PDF page 20), affects silicon 1.0 and 2.0; no fix
  listed. DMAE does not gate requests as expected. Nonzero RFIFOT does not
  reliably prevent RX DMA requests below threshold. Official workaround:
  RFIFOT=0, SRCBURSTSIZE=0 (one transfer). FIFO reset does not clear an already
  generated DMA request; resetting FIFO can therefore leave a stale request.
- Local `soc/HPM6700/HPM6750/hpm_soc_ip_feature.h` has neither UART RX-idle
  detection nor timeout-DMA-disable support. APIs for newer UART IP cannot be
  assumed to exist on this chip.

The driver uses source/destination width 1 and source burst length 1, mapped
by `hpm_dma_get_transfer_burst()` to hardware SRCBURSTSIZE=0. That part already
matches E00018. Its RFIFOT=3 configuration does not. The opt-in B experiment
changes only UART10 RFIFOT to 0 with the SDK FIFO-control API before RX start;
FIFO reset flags stay false. UART2/HI91 and the DMA driver are unchanged.
`fifo_erratum_probe` reports the compiled experiment, not FCR readback: FCR is
write-only, and reading its address returns IIR.

### Source-read and PLIC audit

The UART ISR's existing `uart_get_irq_id()` is still the only IIR read in that
execution path. `uart_hpm_irq_is_pending()` can also read IIR, but the gateway
does not call it. The gateway's registered IRQ callback only increments a
counter, after the driver's IIR read. The DMA driver does not read UART IIR.
The earlier UART reset/RBR-drain loop is at RX start, not in steady reception.

The active controller is `hpm_support/drivers/interrupt_controller/intc_hpm_plic.c`,
not Zephyr's generic SiFive PLIC driver. It reads claim, dispatches the table
entry, restores the nesting threshold, and writes completion. It does not touch
UART IIR/RBR. A PLIC request can have been accepted while the UART condition
was asserted, although DMA removes that condition before ISR inspection.
Thus IIR=1 at ISR entry is compatible with a real earlier peripheral request;
it does not establish a corrupt PLIC dispatch. These snapshots do not identify
whether each earlier request was RX-available or timeout; a waveform/trace
would be required to reconstruct that history.

The confirmed problem is using a transient **FIFO** timeout as the sole partial
publication event for a buffer filled by DMA. DMA is itself a reader that can
remove the required FIFO condition. E00018 additionally invalidates the assumed
nonzero-threshold protection. Increasing IRQ priority, enabling PLIC alone,
or interpreting every IIR=1 as timeout does not make this a reliable idle API.

### URT4 wire layout (opt-in; no additional IIR reads)

`CONFIG_WBR_UART10_IIR_HISTOGRAM=y` requires the existing PLIC probe and changes
only the Ready snapshot to URT4. It retains the driver version link dependency.
Normal builds and URT3 remain supported; the host decodes URT2/3/4.

| Ready offset | URT4 field |
| --- | --- |
| 28 | `0x34545255` (URT4) |
| 32 | existing flags; bit 3 = FIFO erratum experiment |
| 36 | IIR ID 1: no pending interrupt |
| 40 | IIR ID 4: RX available |
| 44 | IIR ID 12: RX timeout |
| 48 | IIR ID 2: TX empty |
| 52 | IIR ID 6: RX line status |
| 56 | IIR ID 0: modem |
| 60 | all other IDs |

All seven classes are disjoint cumulative 32-bit counters. URT4 intentionally
replaces the URT3 flush/position payload; do not decode those words as URT3.
The application still maintains flush diagnostics internally. The histogram
does not read LSR/RBR or add periodic USB diagnostic traffic.

### Initial captures (ordinary user, `--no-rt`)

Files below are under `~/wbr_realtime_lab/results` on `wbr-nuc`.
The capture executable sends empty 1-kHz heartbeats and explicit Stop; it does
not execute a chassis algorithm or send motor-enable commands. No controller
process was running when checked. These are not real-time-host benchmarks.

| Capture | Actual duration / stop | IIR=1 delta | Timeout delta | SBUS bytes / 128-byte groups | MCU observation gap p99 / max |
| --- | --- | ---: | ---: | ---: | --- |
| `uart_probe_investigation_urt3.csv` | 60.0002 s / duration | not classified; other +8436 | 0 | 105344 / 823 | 84.486 / 84.660 ms |
| `uart_probe_urt4_A.csv` (RFIFOT=3) | 46.5443 s / error | 6547 | 0 | 81792 / 639 | 84.393 / 84.683 ms |
| `uart_probe_urt4_A_repeat.csv` | 55.7681 s / error | 7846 | 0 | 98048 / 766 | 84.391 / 84.599 ms |
| `uart_probe_urt4_B.csv` (RFIFOT=0) | 60.0001 s / duration | 105500 | 0 | 105344 / 823 | 83.462 / 83.764 ms |

Every group in these captures is exactly `[40,40,40,8]`. All have zero missing
forwarding chunks, zero duplicates/backwards, and zero capture overflow.
This still does not prove UART wire losslessness. In both URT4 A runs all six
non-1 ID classes have zero delta. In B, RX-available=1 and timeout=1390 were
already present at the first handshake and stayed unchanged; all observed
steady ISR increments were ID 1. Startup counters must not be treated as
capture-period timeout delivery.

Both incomplete A runs terminated with `USB session unhealthy or heartbeat
submission failed`; retain them as failed captures. Their cause has not been
isolated. The absence of host RT permissions was verified, but is not proof
that scheduling caused these failures. They are not full 60-second A tests.

RFIFOT=0 raises the IRQ rate from about 141/s to about 1758/s, close to the
received byte rate. This strongly supports FIFO-condition/DMA competition;
it does not prove the identity of any request already cleared before IIR read.
The official hardware workaround **does not fix partial publication**: B still
waits for 128 bytes. Its ~1-ms improvement in maximum gap is not a latency fix.

Timeout-handler suspend/resume is not implicated in the measured steady A/B
batching: no timeout was recognized in those intervals, so that handler did
not run then. It can affect startup: disabling the DMA consumer while unread
FIFO data remains does not ACK timeout, and repeated zero-growth flushes can
occur until a DMA RBR read actually removes the condition. This is consistent
with URT3's 6170 zero flushes, but the startup register sequence was not traced
and this is not asserted as the uniquely proven cause of that storm.

### Minimum correction and remaining publication work

The hardware correction is RFIFOT=0 with one-byte source bursts, without
resetting FIFO during normal buffer replacement. The missing PLIC enable and
ignored `uart_rx_enable()` timeout are separate driver defects. Only the
UART10 FIFO correction is offered as a default-off experiment here; no change
has been promoted to all UARTs.

A reliable latency fix needs an independent partial-publication trigger.
For a DMA-preserving implementation, use the supplied finite timeout to drive
a per-buffer progress/idle timer (or a separate RX-line timer), serialize it
with DMA completion/disable, and keep the same 128-byte double buffers. It
must publish only `[current_offset, committed_end)` and advance offset before
callback. Reject pending > total or end < offset rather than underflowing.
Cancel/rebase timers with a buffer/session generation so stale callbacks
cannot report a replacement buffer. A zero-growth flush emits no event.

**Do not implement this by calling the current suspend/flush from a new timer
and declaring it safe.** `dma_hpm_dma_suspend()` merely clears ENABLE;
`dma_get_status()` reads TRANSIZE, a source-side remaining count. CHEN is an
alias of ENABLE, not a documented outstanding-write drain indication. Neither
API establishes that counted bytes are committed to RAM. Cache invalidation
also does not drain another bus master's writes. The manual checked here does
not establish a sufficient drain guarantee for this use. A destination-commit
or documented quiescence mechanism must be verified before publishing the tail.
Do not replace that proof with a fixed 16-byte holdback or an arbitrary delay.

The official SDK `samples/drivers/uart/uart_software_rx_idle` demonstrates
TRGMUX + GPTMR RX-line detection for chips without UART idle hardware. It is
an alternative trigger, not a drop-in board change: the sample requires RX
signal routing to TRGM input and uses a stop/restart scheme. Board routing,
timer ownership, continuous reception and write visibility still need review.
An interrupt-driven FIFO receiver with RX DMAMUX disconnected is another
candidate for this low byte rate; DMAE=0 alone is insufficient under E00018.
Either would be an explicit reception-engine change, not a proven one-line fix.

Further acceptance requires partial RX_RDY events, no duplicate published
offsets, correct tail delivery when the transmitter stops, buffer-boundary
and disable/re-enable stress, plus a known byte stream/logic-analyzer check
for UART loss and parity/overrun. Length-only CSVs and forwarding sequence
numbers cannot supply that proof. No partial-publication implementation is
claimed fixed by this investigation.

### Reproduction

Build A from the workspace using the existing diagnostic build directory:

```bash
CCACHE_DISABLE=1 /Users/panpoming/Documents/zephyr_projects/.venv/bin/west \
  build -d /tmp/wbr-uart10-plic-probe -- \
  -DCONFIG_WBR_UART10_PLIC_PROBE=y \
  -DCONFIG_WBR_UART10_IIR_HISTOGRAM=y \
  -DCONFIG_WBR_UART10_FIFO_ERRATUM_PROBE=n
```

For B, use the same command with `FIFO_ERRATUM_PROBE=y`. Flash only after the
current capture exits. Source/headers/CAN changes outside this investigation
must not be reset or included in a UART patch. Driver patch was regenerated
from `git -C hpm_support diff -- drivers/serial/uart_hpmicro.c`; its reverse
apply check passes. It includes no CAN changes and is not committed upstream.

The NUC capture tool is a standalone CMake project in
`build/chassis_controller`, not a target of the historical root build:

```bash
cmake --build build/chassis_controller --target uart_forwarding -j4
ctest --test-dir build/chassis_controller -R uart_forwarding_capture --output-on-failure
```

The existing capture regression passes with URT4 decoding added. Rebuilding
the executable can remove file capabilities. The user subsequently granted
`cap_sys_nice=ep` on the NUC executable to permit the intended RT capture;
avoid rebuilding it during the RT comparison.

### Capture-health race found during RT retries

Granting real-time scheduling did not by itself fix premature capture exit:
`uart_probe_urt4_B_rt.csv` stopped at 11.7542 s and
`uart_probe_urt4_B_rt_repeat.csv` at 54.7199 s, both with the same generic
health/heartbeat error. IIR=1 deltas were 20775 and 96175; timeout/RX-available
had zero delta. Both still showed only full-buffer publication. Preserve these
as failed measurements, not full-duration RT B tests.

Source audit found an independent race in host `UsbLink::healthy()`:

```cpp
MonotonicTimeNs() / 1000 - last_receive_us_.load() < kCommandTimeoutUs
```

The existing Release binary was disassembled: it called MonotonicTimeNs first,
then loaded last_receive_us_. If the USB thread publishes a newer timestamp
between them, unsigned subtraction underflows and returns false despite a
fresh receive. This is a concrete valid interleaving, though earlier captures
did not record operands and cannot prove every prior exit used this path.

The host now snapshots last_receive_us_ first in a separate statement and
samples monotonic time afterward. Post-fix disassembly confirms that order.
The 20-ms timeout, failed/ready gates, Stop behavior and MCU code are unchanged.
The existing `uart_forwarding_capture` regression passed after relink. Relink
cleared cap_sys_nice, so restoring it requires the same user sudo command.

### Completed comparison after the host health fix

Both runs below use the same fixed host executable and **`--no-rt`**. They are
full 60-second MCU buffering comparisons, not completed RT-host benchmarks.
Each has a matching `.log` containing the handshake counters and capture exit
reason. Machine-readable results, including before/after counters, are saved
in [`diagnostics/uart10_urt4_ab.json`](diagnostics/uart10_urt4_ab.json).

| Capture | Elapsed / termination | IIR=1 delta | All six other ID classes | SBUS bytes / groups | MCU observation gap p99 / max |
| --- | --- | ---: | --- | ---: | --- |
| `uart_probe_urt4_A_healthfix.csv` | 60.001 s / duration | 8440 | zero delta | 105472 / 824 | 84.400 / 84.599 ms |
| `uart_probe_urt4_B_healthfix.csv` | 60.0008 s / duration | 105525 | zero delta | 105472 / 824 | 83.560 / 83.775 ms |

Both contain exclusively `[40,40,40,8]`, with no forwarding-sequence gaps,
duplicates/backwards, or capture overflow. HI91 continued with 59999 parser
update events in each run; its host parser-update p99/max were respectively
1.21592/1.33724 ms (A) and 1.22249/1.35366 ms (B). This is a continuity check,
not a wire-to-host latency measurement or proof of wire losslessness.

These runs confirm the same negative result with a comparable host program:
the FIFO hardware correction does not restore partial publication. Startup
timeout counts differ across resets and remain constant during each capture.
One successful run per configuration after the host fix is consistent with
removing the host race, but is not proof that every earlier failure was that
race or that no other host transport failures exist.

At the end, the board is restored to **URT4 A**: PLIC probe on, histogram on,
FIFO erratum experiment off. `/tmp/wbr-uart10-plic-probe` matches that state.
The final 0.3-second `uart_probe_urt4_restored_check.csv` handshake confirms
URT4, PLIC enabled, FIFO experiment off, and clean duration termination.
The production/default-off configuration also builds successfully in
`/tmp/wbr-uart10-plic-baseline`; its ELF contains no probe hook symbols.
No control process was launched, no motor-enable commands were sent, and no
UART, DMA, or CAN changes were committed. Unrelated HPM CAN changes remain.

## Official SDK and supplied Rev2.6 manual review

Reviewed the user's local SDK at
`/Users/panpoming/Documents/zephyr_projects/sdk_env/hpm_sdk`, revision
`7b8cb8bb2` with pre-existing unrelated modifications. No SDK files were changed.
The supplied Desktop PDF `HPM67006400用户手册（拖移项目）.pdf` is a 25-page
extract from Rev2.6, covering DMA/DMAMUX and MBX, not the UART chapter. This is
newer than the Rev2.3 full manual used earlier.

### What the official examples actually implement

| SDK-relative source | Relevant code | Consequence for UART10 |
| --- | --- | --- |
| `samples/drivers/uart/uart_rx_timeout/src/uart.c` | lines 64-86: CPU reads RBR; RX-available handler deliberately leaves FIFO bytes; timeout drains FIFO. DMAMUX setup connects TX only. | Not an example of reliable timeout with active RX DMA. `config.dma_enable=true` does not mean RX DMA is configured. |
| `samples/drivers/uart/uart_software_rx_idle/src/uart.c` | lines 132-166: RX signal via TRGMUX to GPTMR capture/SYNCI; edges reset idle timing. Lines 193-204: idle event disables DMA, then lines 176-177 read TRANSIZE and copy. | Official independent line-idle trigger and partial DMA readout pattern for this generation. Does not wait for UART IIR=0xC. |
| `samples/drivers/uart/uart_hardware_rx_idle/src/uart.c` | lines 122-128: disable DMA after hardware RX-idle, then count/copy. `app.yaml` requires `ip_feature_uart_rx_idle_detect`. | Similar readout sequence, but HPM6750 lacks the required RX-idle feature; cannot port its UART register setup. |
| `samples/drivers/dma/dma_uart_rx_circle_transfer/src/dma_uart_circle_rx.c` | lines 118-139: read TRANSIZE, derive producer index and copy all new bytes. Line 159: 5-ms timer. Buffers/descriptors are noncacheable. | Official example of publishing before buffer completion without RX timeout. It contains no fixed 16-byte holdback. |
| `samples/cherryusb/device/cdc_acm/cdc_acm_uart_com/src/uart_dma.c` | `uart_task_entry_5ms`: cyclic RX DMA progress -> copy -> USB. | Also avoids RX timeout. This is an example, not a production backpressure guarantee: consumer index advances before USB busy/configured checks. Do not copy its queue handling unchanged. |

Both software-idle and cyclic examples configure not-empty RX FIFO threshold,
byte source/destination widths and one-transfer source bursts. The examples
therefore agree with the E00018 workaround. They do not establish that the
gateway's cacheable DMA buffers are interchangeable with noncacheable buffers.

### DMA completion evidence and limits

Rev2.6 section 31.3.3 (printed p549) says DMA ACK follows successful transfer
of a burst. Section 31.3.6, figure 46 (p550), depicts source reads followed by
destination writes within each burst; TC follows the final write. With the
current one-byte source burst this is a one-transfer burst, not a documented
mandatory 16-byte software retention rule. The figure does not specify the
exact cycle when TRANSIZE decrements or when software-cleared ENABLE takes
effect relative to an in-flight bus transfer.

Sections 31.5.5 (p556) and 31.5.7 (p558) still describe CHEN as enable state
and TRANSIZE as source transfer amount. The excerpt does not define either as
a standalone destination-write completion fence. SRCADDR/DSTADDR descriptions
also do not supply an explicit live committed-byte API.

The SDK implementation confirms the primitives: `drivers/inc/hpm_dma_drv.h`
line 186 clears CTRL.ENABLE; line 283 reads TRANSIZE. The software-idle sample
**does use disable -> remaining count -> memcpy after independent RX-line
idle**, with no additional drain API. This is a manufacturer reference pattern
that supports proceeding with a controlled implementation and verification;
the absence of a separate drain function should not by itself halt all work.
It does not prove safe readout at an arbitrary instant during an active burst.

### Concrete driver direction

Use the software-idle example as the reference for the first DMA-preserving
candidate: independent RX-line idle -> suspend current DMA -> snapshot only
new bytes -> publish -> resume the same current DMA. Retain 128-byte double
buffers, full-buffer DMA completion and per-buffer offsets. The sample's
stop/reconfigure-on-next-start flow and print loop must not be copied into the
gateway; they are not continuous async buffer management. Reset FIFO only for
explicit recovery with the E00018 stale-request caveat.

Before implementing TRGM/GPTMR setup, verify the dust board RX signal routing
and free timer channels. The official hpm6750evk2 maps an **additional PD19
TRGM2 input** to GPTMR4; its README explicitly requests an RX-to-TRGM signal
connection. Dust pinctrl selects PC04/UART10_RXD and PB13/TRGM1_P03, but pinmux
alone does not establish that both pads receive the same SBUS wire signal.
Do not switch PC04 away from UART to manufacture a timer route.

If that route is unavailable, periodic finite-DMA progress/idle sampling is
the alternative suggested by the cyclic examples, but it has different idle
semantics and needs explicit design and tests. It must not be presented as
the same hardware-line detector or implemented as a permanent 16-byte tail.

This review changes no receive policy, driver runtime behavior, or flashed
firmware. It supplies the official implementation basis for the next candidate;
the final acceptance still needs known byte patterns, tail-after-stop delivery,
buffer-boundary races, and the existing stopped/zero-output A/B procedure.

## Dust V3 schematic: RX-to-TRGM connection confirmed

Reviewed the user-supplied `/Users/panpoming/Downloads/HPM6750 V3.pdf` visually,
including enlarged MCU pins, network aliases and DBUS inverter. This resolves
the routing question above **for the supplied V3 design**:

- PDF page 4, UART & DBUS block: H1 pin 3 is `DBUS`, feeding Q2 PMBT3904.
  Its emitter goes to ground. The collector has R18 4.7k pull-up to 3.3 V and
  net label `UART10_RXD`. Both MCU consumers below therefore see the same
  post-inversion, pulled-up signal, not the raw DBUS polarity.
- PDF page 3, U3D: `UART10_RXD` connects to PC04, ball A2, configured as
  UART10_RXD. Keep this pin in the UART function.
- PDF page 3, U3C: `TRGM1_P3_UART10` connects to PB13, ball B2. Its pin
  function list includes TRGM1.P03.
- PDF page 3, upper-right network-alias block: the global labels
  `TRGM1_P3_UART10` and `UART10_RXD` meet electrically at the same point.
  The block is annotated `用于触发串口空闲中断` (used to trigger UART idle
  interrupt). This explicit connection, not merely similar naming, establishes
  the RX signal fanout.

Signal path in the drawing:

```text
H1.3 DBUS -> Q2 inversion -> UART10_RXD -> PC04/A2 (UART10 RX)
                                      -> PB13/B2 (TRGM1_P03)
```

No added RX-to-TRGM jumper is required by this schematic. The existing dust
`pinmux_uart10` already selects PC04 ALT2 and PB13 ALT16 with input enabled,
matching both functions. SDK `hpm_iomux.h` confirms PB13 ALT16 is
`IOC_PB13_FUNC_CTL_TRGM1_P_03`. SDK `hpm_trgmmux_src.h` defines the input
`HPM_TRGM1_INPUT_SRC_TRGM1_P3` (0x5), with destinations for GPTMR2/3 SYNCI
and IN2/IN3. Thus the official software-idle example has a concrete board
route; replace its EVK-specific TRGM2/PD19/GPTMR4 mapping with the dust
TRGM1/PB13 mapping after choosing and checking timer ownership.

The drawing establishes connectivity, not runtime GPTMR configuration or
measured idle/byte accuracy. Existing source still does not initialize this
idle detector. This review changes only the investigation record; no firmware
was modified or flashed, and actual board assembly correspondence has not been
electrically measured.

## Configured candidate: independent UART10 RX-line idle

`CONFIG_WBR_UART10_LINE_IDLE` defaults to n and is restricted to dust-hpm6750.
It is enabled in the existing `/tmp/wbr-uart10-plic-probe` candidate build.
The runtime source is `uart10_line_idle.cpp`:

- PB13/TRGM1_P03 -> TRGM1 -> GPTMR2 SYNCI and channel 2 capture input.
  GPTMR2 channel 0 compares against 1000 us since the last line edge. Channel
  2 detects the next falling edge to rearm the comparator. The compare ISR
  rejects a stale compare if a newer edge has already reset the counter.
- GPTMR2 source 62 is connected through a DT node and Zephyr IRQ_CONNECT,
  with PLIC priority 2. It does not use SDK ISR registration outside Zephyr.
  The current build uses the RISC-V machine timer for its system tick, has
  CONFIG_COUNTER disabled, and no other GPTMR2 owner was found in the gateway.
- Initial compare timing starts immediately so a partial first buffer can
  flush even if its first edge preceded detector initialization. Between
  frames, only the detector stops/rearms; RX DMA continues on the same buffer.
- The UART RX API argument and detector setup share `kUartRxTimeoutUs=1000`.
  This is a board-specific fixed-timeout integration, not general support for
  dynamically changing all UART async timeout arguments.

The shared UART driver adds `uart_hpm_rx_line_idle_enable()` (before finite RX
start) and `uart_hpm_rx_line_idle_flush()` (only after independent line idle).
The mode selects RFIFOT=0 and disables UART FIFO RX IRQs, including subsequent
RX enables. TX and HI91 circular DMA are unchanged. There is no fallback that
interprets IIR=1 as timeout.

The flush follows the official software-idle sequence: IRQ lock, check active
finite DMA, suspend, snapshot remaining count, report only unpublished bytes,
resume the same transfer. Pending=0/completed DMA returns -EAGAIN and leaves
buffer release/replacement to the TC callback. Invalid lengths return errors;
offset advances before synchronous RX_RDY. A session generation prevents a
callback's disable/restart from being followed by a stale DMA resume. On a
status-query error after suspend, the candidate leaves DMA suspended and
reports an error, rather than resuming an unknown terminal state. A CPU fence
orders MMIO; it is not claimed as a separate DMA write-drain guarantee.

DMA completion now unconditionally releases its current buffer after flushing,
even when a previous partial publication left no new bytes to report. Normal
buffer replacement still supplies the spare buffer before the next switch.
The ordinary flush also rejects pending > total and end < offset before
unsigned subtraction.

Only the remote port's two 128-byte buffers move into `.nocache`, with cache
invalidation skipped for that port in this mode. The linked ELF places them at
0x01083800, inside the configured noncacheable PMA range
0x01080000..0x01088000. No fixed tail holdback is introduced. Other UART
buffers retain their existing cache handling.

URT4 Ready flags add bit 4 = line-idle configured, bit 5 = idle-handler error,
and bit 6 = RX offset error. The offset diagnostic checks nonzero bounded
lengths, contiguous offsets within a buffer and release before a buffer change.
The host prints these fields. `fifo_erratum_probe=0` with `line_idle=1` is
expected: the line-idle driver mode itself selects RFIFOT=0, independently of
the old FIFO-only experiment switch. FIFO-IIR counters remain zero because
the new mode suppresses those IRQs; they are not GPTMR IRQ counters.

### Enabled measurements

All runs below used the fixed host health-check implementation, `--no-rt`,
empty heartbeats and Stop. Both completed their requested 60 seconds. Host
capture regression passed after the Ready decoder update.

| Capture | SBUS bytes / chunks | Parser-update p99 / max | MCU observation-gap p99 / max | Idle / offset error |
| --- | ---: | --- | --- | --- |
| `uart_probe_line_idle_C.csv` | 105400 / 5006 | 14.6069 / 15.4666 ms | 14.491 / 15.464 ms | 0 / not yet instrumented |
| `uart_probe_line_idle_tail.csv` | 105825 / 5027 | 14.6219 / 15.4317 ms | 14.502 / 15.394 ms | 0 / 0 |

Both have zero forwarding gaps, duplicates/backwards and capture overflow;
HI91 continued with 59999 parser updates per run. Most SBUS publications are
25 bytes. Frames crossing a 128-byte DMA boundary are split into a TC prefix
and idle-published remainder; this is expected. A byte-sum and frame-sized
publication pattern are not a proof of valid wire bytes.

The user closed/reopened the transmitter for the second run. However, the
recorded stream has no substantial UART gap (max host parser interval only
15.43 ms). Whether the receiver continued failsafe output or timing did not
overlap sufficiently is not independently established. **This run does not
verify a truly stopped UART transmitter or delivery of its final tail.** It
does verify offset continuity while the user performed the operation.

The shared driver/header changes remain packaged in
`diagnostics/hpm_uart_rx_probe.patch`, regenerated using only those two files;
reverse-apply check passes. Unrelated CAN changes are excluded. The default-off
production build in `/tmp/wbr-uart10-plic-baseline` also compiles successfully.

### Same-source off/on control and final state

`uart_probe_line_idle_off_A.csv` uses the same driver and offset diagnostic,
with only `WBR_UART10_LINE_IDLE=n` (the old FIFO-only experiment remains off).
It completed 60.0002 s, returned to exclusively 823 groups of
`[40,40,40,8]` / 105344 bytes, and SBUS parser-update p99/max returned to
84.3599/84.6391 ms. IIR=1 increased by 8438, timeout did not increase, and
offset error remained zero. This separates the independent idle path from
the other driver/diagnostic changes. All three full runs are stored in
[`diagnostics/uart10_line_idle_results.json`](diagnostics/uart10_line_idle_results.json),
with source CSV/log paths, handshake snapshots and timing summaries.

The enabled candidate is restored at the end, with defaults still off in
Kconfig. The final candidate binary matches the second enabled run byte for
byte (SHA256 `0c2809235070309f65209f01591b7b457a30e212f2eb8dc0723ebb4629b96ea0`).
No control algorithm or motor-enable commands were run, no UART/DMA/CAN
changes were committed, and unrelated HPM CAN modifications remain intact.

## Acceptance follow-up — software boundaries and remaining hardware checks

The current driver's actual line-idle flush and RX_RDY functions pass the native
contracts in `test/native/uart_line_idle`: final partial payload without later
input, no repeated-idle duplicates, contiguous next publication, generation
changes from callback disable/restart, pending TC, invalid DMA counts and status
failures. The native fence is not a hardware DMA-drain model. In particular this
is a callback reentrancy test, not an end-to-end hardware RX disable/re-enable test.

Host regression passes seven tests on Orb, including five mock USB sessions in
one process, state reset, old-session rejection, new command sequences, clean
transfer drain and SBUS partial-stream reset across reconnect. These are mocked
USB sessions, not a physical detach test.

An attempted NUC realtime capture was not started: `sudo -n` requires a password.
The current NUC candidate tool already supports the following manual capture:

```bash
cd ~/wbr_realtime_lab/src/wbr_control_companion
sudo ./scripts/run_uart_forwarding.sh --probe-status --seconds 60 \
  --csv ~/wbr_realtime_lab/results/uart_line_idle_rt_acceptance.csv
```

Omitting `--no-rt` enables the USB event owner's configured CPU/FIFO scheduling.
This tool still sends only empty heartbeats; it does not execute the chassis
control algorithm or prove its final scheduling behavior. Preserve all previous
CSV files. Require duration completion, line_idle=1, no idle/offset flags or
stream gaps, and no regression in HI91 before accepting this configuration.

Still pending: genuine transmitter silence with a known final incomplete DMA
buffer, hardware RX close/restart, and physical USB detach/reconnect. Disabling
RX is not a substitute for testing the final idle-driven tail: RX disable has
its own flush and can mask a failure in the timer path. Turning the transmitter
off was already shown not to establish UART silence. A controlled UART source
or verified receiver-output interruption is needed, keeping motors disabled.

Do not enable the production default or remove the acceptance diagnostics yet.
The candidate remains opt-in; unrelated HPM CAN changes are not part of its patch.
