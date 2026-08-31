# State-space intake integration

This package is the command-ready version of the intake model from `frc6560/State-Space-Control`. It is wired into `RobotContainer` and `AutoCommands` on the `offszn` line.

- `IntakeStateSpaceController` generates extension and roller voltage commands from measured position, velocity, and roller speed.
- `IntakeStateSpaceSubsystem` runs the controller in the WPILib 20 ms loop and publishes `Intake/StateSpace/*` signals through AdvantageKit.
- `TalonFXIntakeStateSpaceIO` maps CAN 15 to the extension X44 and CAN 16 to the roller X60, with a 64:14 rack reduction, 1.751 in pinion, ±6 V cap, and Phoenix software limits.

The existing button-board behavior is preserved: button 3 activates the intake and releasing it leaves the rack extended while stopping the roller. Code can request `OSCILLATING`, `RETRACTED`, `EXTENDED`, or `OUTTAKE` through `setMode(...)`. Call `zeroExtensionEncoder()` only after physically retracting the mechanism.

All constants are intentionally conservative pending SysId and mechanical verification. Check motor inversion, encoder sign/zero, rack travel, current limits, and hard stops with the robot secured before enabling full operation.
