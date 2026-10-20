# dust-hpm6750 USB acquisition gateway

Independent forwarding firmware. It starts shared CAN/CherryUSB drivers and
hardware acquisition, not chassis control, AHRS, VOFA, RTT or pressure tests.
USB is HS Bulk with a 128-byte OUT buffer and 64-byte IN messages. Motor commands
are submitted in W/B/D FIFO order separately on CAN3 (left) and CAN2 (right).
CAN/USB acquisition callbacks are operational subscriptions; the forwarding build
compiles out the trace tap and legacy local feedback routing. No sampled latency
recorder is linked. It uses the existing standard batch driver API, without
requiring the experimental hardware-trigger trace extension.

Build from the existing Zephyr workspace:

```bash
west build -b dust-hpm6750 -s wbr_control/src/usb_can_bridge \
  -d wbr_control/build/usb_can_bridge
west flash -d wbr_control/build/usb_can_bridge
```

The matching Linux host is the independent `chassis_controller/` target in
`wbr_control_companion`, branch `feature/usb-can-host-controller`. It rejects
legacy test firmware through a 64-byte version/capability handshake before
sending 128-byte command packets.

## Peripherals

- Standard CAN data on configured CAN controllers is accepted; known motor
  feedback uses latest-state mailboxes, other accepted IDs are forwarded through
  a bounded queue. Host commands are restricted to the six production motor IDs.
- UART10 retains the board's SBUS 100000 8E2 configuration and async double-buffer
  reception. UART2 retains HI91 921600 and the proven continuous-DMA ring/16-byte
  outstanding-write guard. Protocol parsing now runs on NUC.
- Board-mounted SPI IMU transport is reused to forward raw bursts and temperature,
  without an MCU estimator/controller. A failing sensor is absent from capability
  bits instead of fabricating valid data.
- An optional `wbr-control,referee-uart` chosen node enables a third forwarded UART.
  This board has no such declaration currently; no pin assignment is invented.

Capability bits in Ready: CAN=1, remote UART=2, HI91 UART=4, raw onboard IMU=8,
referee UART=16. The host requires CAN, remote UART and HI91 UART to initialize,
then independently requires fresh valid data before accepting remote enable.

## Failure behavior

Hello performs a local stop and starts a new session. Entire command packets are
validated for version, session, sequence, motor address and slots before any
publication. Invalid/replayed packets do not renew the lease. A disconnect or
20 ms without a valid new command submits zero wheel currents and DM exit frames,
invalidates the session, and requires a new handshake. The same shutdown runs
on explicit Stop. This is software loss-of-host handling, not an independent
hardware emergency-stop guarantee if the MCU or CAN hardware itself fails.

No per-cycle ACK, stress command, profiling query, timestamp export, VOFA,
printk or logging backend is enabled. Small acquisition timestamps remain in
sensor messages because freshness is a control input requirement.

Compilation and native protocol tests have passed; physical acquisition, board
capabilities, host/MCU reconnection and real control behavior remain to be tested
when NUC and the robot are available. Default host operation keeps outputs disabled.

UART10's receive batching is under controlled investigation. See
[the evidence and opt-in PLIC A/B experiment](UART10_INVESTIGATION.md).
The probe remains disabled in the normal build pending hardware confirmation.

`CONFIG_WBR_GATEWAY_FAULT_DIAGNOSTICS` adds a default-off FLT1 Hello page for
stop-only process exit/kill/freeze testing. The snapshot is taken before that
Hello resets the old session. It records stop CAN group submission, not hardware
completion or physical motor stop. See the matching companion repository's
`chassis_controller/measurements/fault_recovery/README.md` for the test sequence.
