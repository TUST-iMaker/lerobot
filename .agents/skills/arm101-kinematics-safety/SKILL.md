---
name: arm101-kinematics-safety
description: Trigger whenever modifying FK/IK kinematics, calibration, trajectory replay, real-machine synchronization, E-Stop, or any SO-ARM101 6-DOF joint control code.
---

# SO-ARM101 Kinematics & Hardware Safety Contract

## 1. Canonical 6-DOF Joint Definitions & Soft Limits
Never rename, reorder, or invert the sign of these 6 joints (aligned with `repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf` and `lerobot v0.7.1`):

| ID | Joint Key | Unit | Soft Limit Range | Homing Default | Rest Pose | Link Length |
|---|---|---|---|---|---|---|
| 1 | `shoulder_pan` | deg (`°`) | `[-110.0, +110.0]` | `0.0°` | `0.0°` | `L1 = 116.6 mm` |
| 2 | `shoulder_lift` | deg (`°`) | `[-100.0, +100.0]` | `0.0°` | `-95.0°` | `L2 = 116.0 mm` |
| 3 | `elbow_flex` | deg (`°`) | `[-97.0, +97.0]` | `0.0°` | `+92.0°` | `L3 = 135.0 mm` |
| 4 | `wrist_flex` | deg (`°`) | `[-95.0, +95.0]` | `0.0°` | `-90.0°` | `L4 = 160.5 mm` |
| 5 | `wrist_roll` | deg (`°`) | `[-157.21, +162.79]` | `0.0°` | `0.0°` | Roll axis |
| 6 | `gripper` | pct (`%`) | `[0.0, 100.0]` | `20.0%` | `2.0%` | Parallel jaw |

- **Official Homing Pose (`all 5 body joints = 0.0°`)**: Upper arm vertical, forearm and gripper horizontal forward, `TCP = (391.4, 0.0, 226.5) mm`.
- **STS3215 12-Bit Encoder Mapping**: `0..4095` steps, center `2047` (`4095 / 360.0` steps/deg for body joints; linear `0..100%` for `gripper`).

## 2. Mandatory Hardware Safety Invariants
1. **E-STOP Highest Interrupt Priority**: When `estop_latched == True`, immediately block all follower joint writes, IK commands, preset poses, and routines (`HTTP 409 ESTOP_LATCHED`), terminate active subprocesses, and release servo torque (`torque_enabled = False`).
2. **Sim-to-Real Gate**: Switching `sync_direction` to `sim_to_real` MUST require `safety_confirmed == True` and active real hardware (`HTTP 409` otherwise).
3. **Calibration Torque Release**: Starting or stopping real-arm calibration MUST immediately release follower/leader servo torque so the operator can move joints by hand without stiffness.
4. **Honest Sensor Reporting**: In `dry_run` mode (or when 0 serial ports are connected), never fabricate physical bus voltage, temperature, or EEPROM lock values; display `--` and label as simulated.
