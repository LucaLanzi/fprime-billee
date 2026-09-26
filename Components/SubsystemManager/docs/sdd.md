# fprime-billee::SubsystemManager

Controls and monitors the switched rover subsystems: DRIVETRAIN (six wheel enables switched together), ARM and SCIENCE.

## Introduction

SubsystemManager owns the eight power-enable GPIOs (RP2350 GPIO4–11). Each goes through the TXS0108E buffer and an SN74LVC08 AND gate, whose other input is the E-STOP line (GPIO12, `BUFF_E_STOP_STATUS`), and then to an LTC7001 high-side driver. The hardware AND gate is the primary interlock. This component is the software layer on top of it.

## Requirements

| Name | Description | Rationale | Validation |
|---|---|---|---|
| SSM-001 | Read E_STOP_STATUS before enabling any controlled rail, and refuse power-ON while the E-STOP is engaged or unreadable. | Rev H §11.4 | UT S1, S2; bench T3 |
| SSM-002 | When the E-STOP engages, force every controlled subsystem OFF in software. Releasing the E-STOP restores nothing. | No automatic restart after an E-STOP | UT S4; bench T4 |
| SSM-003 | Refuse power-ON on a subsystem while FPManager holds a fault inhibit on it. Releasing the inhibit never turns anything on. | Fault latch (see FPManager) | UT S5 |
| SSM-004 | Always accept power-OFF. | Safe direction must never be blocked | UT S7 |
| SSM-005 | If any drivetrain GPIO write fails, drive all six enables LOW and report OFF. | Enables must stay consistent | UT S6 |
| SSM-006 | Report every actual power-state change to FPManager. | Gates FPManager's undervoltage check (switched-side VBUS reads ~0 V when off) | UT S3, S4, S5 |
| SSM-007 | Never drive or pull GPIO12. | It is shared with all 8 AND-gate inputs | Overlay review; bench T1 |

## Design

- **Single choke point:** `applyState()` is the only path that writes enables. It writes the GPIO(s), updates state, emits the event and telemetry, and calls `powerStateOut`.
- **E-STOP polling:** `run` is on the 10 Hz rate group. A failed read is treated as engaged.
- **Power-ON:** every power-ON does a fresh E-STOP read and never trusts the cached value.
- **Fault inhibits:** `faultInhibitIn` (from FPManager) sets or clears a per-subsystem inhibit. Setting it forces OFF through `applyState()`.
- **No AUX:** there is no AUX channel in hardware (`+12V_AUX_VCC` is always on). The old `SET_AUX_POWER_STATE` (opcode 2) and `AuxSet` port were removed. Invoking the unconnected `AuxSet` port asserted and rebooted the board.

## Configuration

None. GPIO wiring is in the deployment topology. Early-boot safe outputs are set by `SYS_INIT` in the deployment's `Main.cpp`.
