# Migration Baseline Checklist

This checklist defines the baseline observation points before deeper migration work.

## Goal

- Ensure each module has visible runtime signals for command, feedback and output.
- Provide a fixed regression checklist used after each migration step.

## Runtime Observation Points

1. Chassis
- Verify periodic trace includes target omega, measured omega and output current.
- Verify `idle` field increases when no command arrives and wheel targets return to zero after timeout.
- Confirm CAN bus is CAN1 and current group is 0x200.

2. Arm
- Verify periodic trace includes claw/elbow virtual angles and wrist target/measured speed.
- Confirm DM and wrist commands are sent on CAN3.

3. Gantry
- Verify periodic trace includes X/Y/Z virtual distances and command enable.
- Current phase only confirms command path, not motor output.

4. Gimbal
- Verify periodic trace includes yaw/pitch virtual angles and enable state.
- Confirm servo ready state is stable.

## Per-Step Regression Checks

1. Build check
- Run `cmake --build build -j8` and ensure success.

2. Boot check
- Confirm module startup logs are present.

3. Trace check
- Observe `[baseline][chassis]`, `[baseline][arm]`, `[baseline][gantry]`, `[baseline][gimbal]` logs.
- Ensure values evolve with input commands and do not stay stuck unexpectedly.

4. Safety check
- With command enable disabled, verify outputs do not grow unexpectedly.

## Notes

- Baseline traces are intentionally throttled and used for migration only.
- Keep this checklist updated when a module transitions from skeleton to full closed-loop control.
