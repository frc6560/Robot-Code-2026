# Intake current-based end-stop tracking

The new robot has one reciprocating intake motor: ID 26 on `Canivore`. The intake no longer uses encoder position, roller velocity control, or timed rack positions. Positive voltage extends and negative voltage retracts, subject to the configured motor inversion.

## Current status

`Constants.IntakeConstants.ENABLED` is **false** while the intake is broken. Motor current, connection status, voltage, and temperature are still logged; the subsystem and hardware adapter both block motor output. Both end-stop thresholds are **0.0**, meaning unconfigured. Changing `ENABLED` alone cannot run the intake until valid thresholds are supplied.

## Operating behavior

- Driver 2 button 4 starts continuous back-and-forth cycling while held. Releasing it stops the motor.
- Shooting requests also cycle the intake and restore the manual operating request afterward.
- With an unknown endpoint, cycling first retracts toward the rear stop.
- A current rise must persist after the startup-ignore interval. Once confirmed, the motor stops, records an inferred endpoint, pauses, and reverses on the next leg.
- A retract/reset request homes toward the rear stop and stops there instead of cycling. Driver 2 button 6 clears a latched fault and requests this homing operation.
- Disabling the robot, disabling the intake, or entering pit coast mode stops output and invalidates the endpoint reference.
- Missing/stale current feedback or exceeding the travel timeout stops the motor and latches a fault. Faults do not reverse or retry automatically.

## Calibration after repair

Settings are in `src/main/java/frc/robot/Constants.java`, in `IntakeConstants`:

| Setting | Purpose |
| --- | --- |
| `EXTEND_END_STOP_CURRENT_AMPS` | Sustained stator-current threshold at the front stop |
| `RETRACT_END_STOP_CURRENT_AMPS` | Sustained stator-current threshold at the rear stop |
| `STARTUP_IGNORE_SECONDS` | Ignore initial acceleration current after each motion starts |
| `END_STOP_CONFIRM_SECONDS` | Required uninterrupted time above the threshold |
| `REVERSAL_PAUSE_SECONDS` | Zero-output interval before reversing |
| `TRAVEL_TIMEOUT_SECONDS` | Maximum time for a leg before a latched fault |
| `EXTEND_VOLTS`, `RETRACT_VOLTS` | Motion voltage magnitudes; currently 2 V starting values |
| `STATOR_CURRENT_LIMIT_AMPS` | Motor torque/current ceiling; thresholds must be below this |
| `MOTOR_INVERTED` | Direction mapping; verify extension and retraction on the repaired mechanism |

Measure normal moving stator current and contact current at both stops using a controlled motor test after repair. Select thresholds above normal load but below the current limiter. Check startup timing, reversal pause, travel timeout, and motion voltage against actual travel. The initial timing/voltage settings are not hardware-calibrated. Supply current is logged separately and is not used for end-stop detection.

## AdvantageScope fields

Under `Intake`, graph `motorStatorCurrentAmps`, `motorSupplyCurrentAmps`, `motorAppliedVolts`, and `CommandedVolts`. Inspect `motorConnected`, `motorCurrentValid`, `motorConfigurationStatus`, `OutputsEnabled`, `DetectionConfigured`, `InferredEndpoint`, `EndpointKnown`, `RequestedDirection`, `MovingDirection`, `EndStopCount`, `CompletedCycles`, and `Fault`.

`InferredEndpoint` is a current-based inference, not a measured continuous position. A mid-travel jam can resemble an end stop. The count records detections, including the initial rear homing event; it does not prove a complete mechanical stroke. `CompletedCycles` counts an inferred front-to-rear pair during cycling. There is no retained position reference after power cycling.
