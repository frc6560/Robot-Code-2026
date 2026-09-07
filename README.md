# StateSpaceSwerve

`StateSpaceSwerve` is deployable WPILib Java code for the 6560 swerve drivetrain. It is not an AdvantageKit swerve library: AdvantageKit only records telemetry. The drive-motor velocity loop is the state-space portion of this project: every module uses a velocity plant, Kalman observer, and LQR controller to calculate a bounded voltage command.

## Hardware map

The map was copied from `frc6560/Robot-Code-2026`, branch `bline-transition`.

| Module | Drive Kraken X60 | Steer Kraken X44 | CANcoder | Offset |
| --- | ---: | ---: | ---: | ---: |
| Front left | 4 | 6 | 5 | 0.967° |
| Front right | 1 | 3 | 2 | 112.500° |
| Back left | 7 | 9 | 8 | 250.313° |
| Back right | 10 | 12 | 11 | 203.730° |

All devices use the `Canivore` bus. The Pigeon 2 is CAN ID 13. Drive and steer supply-current limits are 40 A and open-loop voltage ramps are 0.25 s.

## Driver controls

- Left stick: field-relative translation
- Right stick X: rotation
- Start: zero gyro and odometry
- X: hold an X-lock wheel stance

## Before enabling a robot

The project compiles to a roboRIO artifact and uses real hardware I/O, but it is not safe to compete with until these checks are completed on blocks:

1. Confirm every motor and CANcoder ID and the Pigeon 2 are visible on the `Canivore` bus. The program aligns each steering TalonFX's integrated position to its CANcoder at startup; verify that behavior while disabled.
2. Verify all four absolute encoder offsets and steering directions. The imported offsets are from the BLine transition branch; they must be re-checked after any module service.
3. Run drivetrain SysId and replace `DRIVE_KV_VOLTS_PER_MPS` and `DRIVE_KA_VOLTS_PER_MPS_SQUARED` in `Constants.java`. The checked-in values are conservative values inherited from the existing drivetrain configuration, not a new characterization.
4. Confirm each wheel's forward direction and gyro sign before a full-speed test.

Deploy with `./gradlew deploy` from this directory after setting the team number in `.wpilib/wpilib_preferences.json`.
