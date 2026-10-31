# Fuel shooting: setup, measurement, and tuning

This is the **driver auto-turn shooting** path with translation lead compensation (see the latest trigger mapping below). It does not use the old video calibrator,
its files, or its fitted distance policy. `FuelShotModel` integrates gravity, drag and backspin lift
from ideal wheel/hood launch conditions. All flight aerodynamics are provisional until basket tests.

## Driver behavior

- Hold **driver right bumper (RB)**: auto-turn to the alliance HUB and immediately accelerate the
  flywheel while approaching. With the formula model, RPM follows the calculated trajectory.
  Without it, use the explicit fixed-speed fallback: `Fuel/Test Flywheel RPM` defaults to **3000**,
  and `Fuel/Test Tunnel RPM` defaults to **1500**. `Fuel/Mode` identifies the uncalibrated fallback;
  its range/trajectory has not been validated.
- Once actual heading error is within the configured aim tolerance, actual turn rate is within **5 deg/s**
  of the required tracking rate (zero for stationary shots),
  measured flywheel speed is within **60 RPM** of the request, and those conditions remain true
  for **0.10 s**, the command runs the real wall/tunnel and the floor indexer (**800 RPM**).
- With the formula model, measured RPM must also produce a scoring trajectory at the current range. Losing any
  release condition closes the gate immediately; regaining it requires another settling period.
- Release RB: stop the tunnel and shooter. Both indexers remain stopped. No second button is needed.
- Translation remains driver controlled. Formula shots compensate aim and RPM while translating;
  the fixed-RPM calibration/fallback and alliance pass still pause feeding above 0.15 m/s.
  At/below 0.05 m/s, formula shots retain the original stationary calculation and release checks.
- Driver Station alliance and a finite, correctly localized pose are required. Only shots from
  your own alliance zone are allowed. Start by resetting localization at a known position.
- Pit coast, disconnected flywheel motors, or a disconnected/misconfigured tunnel block feeding.

Neither indexer runs automatically on enable. The floor is enabled by
`FuelShotConfig.FLOOR_ENABLED=true` and runs alongside the tunnel once the shot is ready.
Driver left bumper runs the floor/tunnel manually when RB is not held; Y is no longer the shooting binding. Operator A /
button-board 11 still provide explicit manual spin-up, and board 3 still provides manual floor/tunnel
feed outside RB. After RB release, any held operator control must release before manual feeding
can resume. LT selects rack retraction; RT selects extension. The rack holds its selected
position after trigger release, but intake rollers run only while RT remains held and a valid, homed rack
reaches the requested full-extension position (9.5 motor rotations, within 0.25 rotations) and settles
for 150 ms below 0.1 motor rotations/s. Releasing RT or selecting retract immediately stops the rollers; losing
position/home feedback also stops them. The motor-output layer independently enforces extension,
request, homing, fresh feedback and speed checks. Rollers use brake neutral mode to reduce coasting.

This change replaces the driver shooting binding with a held-RB shot path. Legacy button-board shot actions and autonomous
shooting routines are not wired to this new calibrated model. In particular, legacy auto shooting
that relied on the always-running tunnel will now need an explicit tunnel-owning shot command;
do not rely on those routines for calibrated firing. The separate timed center-auto still explicitly
owns its tunnel. Shooting does not request intake roller motion; rollers require the held RT request.

## Formula shooting and tuning

No measurement CSV is required. RB uses an ideal powered wheel against a fixed hood:

```
surface_speed = pi * wheel_diameter * rpm / 60
exit_speed = EXIT_SPEED_MULTIPLIER * surface_speed / 2
backspin_rps = BACKSPIN_MULTIPLIER * surface_speed / (2 * pi * ball_diameter)
```

Tune `FuelShotConfig.java` and redeploy/restart. Speed and spin multipliers default to 1.
Start with repeated shots at one reachable distance. Consistently short shots suggest reducing
EXIT_SPEED_MULTIPLIER so the solver requests more RPM; long shots suggest increasing it.
Change one value at a time and verify at near and far ranges.
LAUNCH_ANGLE_OFFSET_DEGREES adjusts the nominal 48.36-degree elevation.
BACKSPIN_MULTIPLIER, DRAG_COEFFICIENT, LIFT_SLOPE and SPIN_DECAY_PER_SECOND tune flight behavior.
Measure RELEASE_FORWARD_METERS and RELEASE_LEFT_METERS from the chassis pose origin.
TUNNEL_RPM controls feed speed. The model integrates gravity, drag and lift; its predictions
need field tuning. Feeding still waits for a reachable trajectory, aim and measured RPM.

For fixed-speed experiments enable Fuel/Calibration Mode and set Fuel/Test Flywheel RPM and
Fuel/Test Tunnel RPM. Release RB before changing values; disable calibration mode to restore
formula shooting. No data submission or external tool is needed.

## Shooting while translating

Formula shooting converts measured robot-relative X/Y velocity into field coordinates and computes
`lead = field_velocity * (flight_time + release_delay)`. Both aim and RPM use the hub displacement
minus that lead. Six bounded cache-only iterations estimate the flight time; the final selected RPM
uses the existing trajectory validation. Measured-RPM readiness integrates a 3D trajectory using
measured heading, RPM and inherited field velocity. Drag and lift act on the combined launch and chassis velocity, so a simple translated
stationary flight cannot incorrectly clear a marginal moving release. A failed or nonconverging solution blocks feeding. Velocity changes
update the moving solution every command cycle, even when the physical range is unchanged.

The aim controller tracks the changing bearing while translating. Its angular readiness gate compares
measured turn rate with the required tracking rate. Pure rotation does not activate compensation;
zero translation keeps the original stationary RPM, aiming, dwell, and scoring checks. A 0.05 m/s
noise threshold prevents tiny velocity estimates from activating lead. Transitions between stationary
and moving solutions restart the normal 0.10-second feed dwell. The backup autonomous routine keeps
its separate stopped-at-endpoint requirement.

Aim/RPM selection uses a constant-velocity lead approximation with the cached stationary trajectories.
Before each moving release, one bounded 3D integration checks the actual predicted landing, near-rim
clearance and entry angle. This validation does not optimize aim/RPM again; marginal solutions pause
feeding. All aerodynamic coefficients remain provisional. Validate approach, retreat, and sideways
shots on the robot, especially at higher speeds or during acceleration. Set
`FuelShotConfig.MOVING_SHOT_RELEASE_DELAY_SECONDS` to the measured delay from a feed command to
ball exit; it defaults to zero until measured and affects only moving shots. Existing stationary
calibration and speed multipliers are unchanged. Calibration mode, missing-model fallback, and
alliance passing retain their existing motion restrictions.

`Fuel/MovingShot/Active`, `Valid`, `LeadX`, `LeadY`, and `EffectiveRange` in the recorded telemetry
show whether compensation is active and the lead used (meters in field coordinates). The cache is
reused rather than regenerated for each velocity. Each moving cycle uses seven cache estimates,
one final solve (two stationary trajectory integrations), and one 3D measured-RPM integration when the other
release gates pass. Measure loop time on the roboRIO before relying on a particular CPU budget.

## Floor and intake current limits

The floor target remains **800 RPM**. Only the intake rack has a startup boost.

| Mechanism | Stator limit | Supply limit |
|---|---|---|
| Each floor motor | 40 A | 40 A |
| Wall/tunnel motor | 40 A | 40 A |
| Intake rack | 60 A during first second of travel, then 40 A | 30 A |
| Each intake roller | 40 A | 40 A |

Both shooter motors stay at 40 A stator and 40 A supply per motor, with no startup
boost. The intake rack retains its one-second 60 A stator allowance; it also returns
to 40 A on arrival. Configuration confirmation/retry handling
remains, including restoring fixed limits after a controller reset.

Current limit telemetry: `Indexer/FloorStatorLimitAmps`, `Indexer/TunnelStatorLimitAmps`,
and `Intake/DeployStatorLimitAmps`. Their corresponding `FloorCurrentLimitOK`,
`TunnelCurrentLimitOK`, and `DeployCurrentLimitOK` flags indicate whether the
controller configuration was confirmed.

Automatic floor and rack stall cutoffs have been removed by operator request. Sustained high-current,
low-speed operation no longer cancels the requested RPM/position or requires a disable to reset.
The continuous current limits above, rack soft limits, disable/pit-coast behavior, and roller
extension interlocks remain. Legacy `Indexer/FloorStallFault` and `Intake/DeployStallFault` dashboard values are cleared to
false to avoid stale fault indicators. This removes a software cutoff; it does not resolve binding.

Watch `Intake/FullyExtended` / `Intake/RollerPermitted` for the roller interlock.
The existing `Intake/DeployCurrentAmps` is **supply current**, whereas floor
current readings are **stator current**. New `Intake/DeploySupplyAmps` and `Intake/DeployStatorAmps`
make that distinction explicit. `Intake/LeftRollerVolts` / `Intake/RightRollerVolts` help investigate
one roller failing to spin. Separate `Intake/LeftRollerSupplyAmps`, `LeftRollerStatorAmps`,
`RightRollerSupplyAmps`, and `RightRollerStatorAmps` are available on SmartDashboard
and in saved logs. Legacy RollerCurrentAmps remains the combined supply current.
`Intake/RollerTriggerHeld`, `RollerCommandedRPM`, and `RollerBlockedReason` explain
whether the rollers were requested or stopped. For a noisy left roller, compare
its current, voltage and measured RPM against the right side. A high current with
little motion calls for checking binding, loading, wiring and motor feedback;
it does not establish which fault is present. The rack assumes physically retracted at code startup; its motor encoder
cannot independently verify the physical endpoint. Verify that 9.5 rotations matches full extension
before relying on the roller interlock, since the earlier measured-travel calculation was 7.81.


## When the wall does not run

Release both bumpers, then hold **left bumper alone** to request manual floor (800 RPM) and tunnel
(1500 RPM) without alliance/aim/flywheel gates. This is an explicit ungated manual control; do not
use it to judge automatic shot readiness. Right bumper still owns the mechanisms when held and
waits for its release conditions. After releasing RB, release any held manual input before retrying.

With RB held, `Fuel/Status` gives the blocking condition (for example Aiming, No alliance, Outside
own alliance zone, Waiting for measured flywheel RPM, or Stop translating to shoot). No automatic
feed is requested until those conditions pass; the timed current limits are independent of that decision.

`Indexer/TunnelTargetRPM=0` means the command is not requesting feed. A nonzero target with
`Indexer/TunnelRPM` near zero needs the accompanying `Indexer/TunnelVolts`,
`Indexer/TunnelStatorAmps`, `Indexer/TunnelConfigurationStatus`, and `Indexer/TunnelControlStatus`
to distinguish a motor/controller problem from a stalled mechanism.

## Live dashboard coefficient

`Shooter/Launch Speed Coefficient` defaults to 1.00 and accepts 0.5–1.5.
It multiplies modeled exit speed and updates trajectories and scoring checks live, including
while RB is held. Too far: try 1.03; too short: try 0.97. Restart resets it to 1.00.
Keep `Fuel/Calibration Mode=false` for automatic formula shooting; fixed-RPM mode bypasses
trajectory selection. The spin multiplier remains independently configured in FuelShotConfig.


## Persistent live launch-speed adjustment

Edit the numeric NetworkTables input `/SmartDashboard/Shooter/Launch Speed Coefficient`
from Elastic and commit the edit. Do not edit the read-only logged trajectory
coefficient or the separate Applied/Saved displays. Supported range: 0.5 to 1.5.
The last accepted value now has its own Preferences key,
`ShooterLaunchSpeedCoefficient`, separate from the dashboard request. Robot startup
restores that saved value; a valid old dashboard value is migrated on first use.
Edits are validated and saved every cycle, even while disabled and before shooting.
Invalid or missing requests do not overwrite the saved value or force the input to 1.

Check `Shooter/Saved Launch Speed Coefficient` while disabled; it should follow an
accepted edit. Check `Shooter/Applied Launch Speed Coefficient` while shooting.
`Shooter/Launch Speed Coefficient Status` describes request validation.
`Fuel/Tuning` records requested/saved values and validity continuously.
The NetworkTables server persists Preferences to disk; abrupt power loss before
it saves a recent edit may still lose that edit.

For intermittent shutdowns, inspect `Power/BatteryVoltage`, `Power/BrownedOut`,
`Power/ControllerTemperatureCelsius`, and `Runtime/CodeUptimeSeconds`, together
with Driver Station errors. Code uptime restarting indicates a new Robot instance;
a graph simply freezing can also be a lost connection. These logs do not prove
that a battery, connection, or noisy mechanism is electrically healthy.

`Runtime/ProgramSession` changes for every new Robot instance. After reconnecting,
a changed session and restarted CodeUptimeSeconds establish a program restart;
a matching session and continuing uptime indicate the program survived the lost
connection. Battery voltage, brownout flags and Driver Station messages are still
needed to distinguish electrical failure, an exception, and a network problem.

## Top current users

On Elastic, add text displays for SmartDashboard `Power/Top1CurrentUser`,
`Power/Top2CurrentUser`, and `Power/Top3CurrentUser`. Each shows a physical motor
label, CAN ID and current supply amps. Corresponding numeric `...Amps` keys can
be graphed. The same keys are recorded under AdvantageKit RealOutputs.

All 16 installed Talon FX controllers are tracked together on hardware: eight
swerve drive/steering motors, two shooter motors, three intake motors, and three
indexer motors. No new motor-controller objects are created. Supply-current
signals are requested at 20 Hz and ranked every robot loop. Refresh calls are
grouped by CAN connection; a Phoenix refresh cannot mix rio and CANivore signals. Failed/stale signals
are excluded; check Power/TrackedMotorCount and Power/FreshMotorCount for coverage.
Per-motor readings/freshness are under Power/Motors. TotalTrackedSupplyAmps sums
only fresh monitored motor readings, not all electrical loads on the robot.
This is an instantaneous battery-draw ranking, not an overheating or fault diagnosis.


Hottest-controller displays are `Power/Top1HottestDevice`, `Power/Top2HottestDevice`,
and `Power/Top3HottestDevice`, with separate numeric `...Celsius` keys. These rank
fresh device-temperature readings from the same 16 motor controllers, requested
at 10 Hz. Temperatures more than 2 seconds old are excluded. Device temperature
is the controller's reported sensor temperature, not a direct measurement of the
motor windings, bearings or power connections. `Power/MonitorStatus` and
`Power/RefreshStatus` expose diagnostic errors. A runtime exception in this monitor
is reported without propagating into the robot loop; this does not catch crashes
elsewhere or native/process failures. A stale device-temperature warning by itself
does not establish an overtemperature condition. Preserve the Driver Station error
and stack trace to investigate an actual program termination.

## Driver control swap

Right trigger now aims/shoots (formerly RB); left trigger manually feeds floor/tunnel
(formerly LB). Right bumper extends the intake and requests rollers while held;
left bumper retracts the intake. Release right trigger before resuming held manual feed.
This mapping supersedes the older button names above.

## Closed-loop desktop simulation

Run `./gradlew movingShotSimulation` to exercise the
production aim command, shot solver, feed gates, and shooter controller with the configured gains.
The harness uses the YAGSL/MapleSim physical drivetrain and WPILib FlywheelSim with a 1 ms
P/D motor-controller stand-in (the stock ShooterIOSim ignores D). It exports nine motion/readiness scenarios plus a stationary control
case to `build/reports/moving-shot-simulation`, with actual RPM, heading error, feed state,
command timing, and sampled ball landings. A separate 3D RK4 ball integration adds measured field
velocity at release and applies drag/lift to the resulting ground velocity; a stationary cross-check
verifies it against the existing trajectory model. It samples releases every 0.2 s while feeding;
it does not model actual ball inventory, spacing, flywheel loading or indexer dynamics.
It reports stationary releases after stops separately from moving releases, so the preserved
stationary heading tolerance is also visible in the flight check.

These runs use a flat field, 12 V supply, the saved-default launch coefficient, ideal indexer
readiness, and no vision noise or release delay. YAGSL's module motor loops are simplified; the
production hub-aim PID/profile and shooter P/D gains/feedforward/ramp are exercised. The motor
stand-in omits Talon firmware filtering, CAN latency and supply-current limiting. This is
software validation, not a measured success probability or a roboRIO CPU benchmark. The desktop
results and plots can be regenerated with `tools/analyze-moving-shot-simulation.py`.

### Additional curved-path simulation

`./gradlew movingShotStressSimulation` runs eight 10-second cases in a separate process: stationary,
curved arcs and figure-eight motion on both alliances, near weaving at 2.7 m, far weaving at 4 m,
and a duplicate stationary control. Translation runs from 1.5 to 8.5 seconds. The harness exports
all results before checking them, so predicted misses remain inspectable when a scenario fails.
Every moving case must produce releases, and every sampled moving release must pass the independent
3D flight check. Reports are under `build/reports/moving-shot-stress-simulation`.

The flywheel gains in `Constants.ShooterConstants` are P=2.0, I=0.0, D=0.0. These shared constants
are used by the production Talon configuration and shooter subsystem output as well as simulation.
The cached stationary selection and stationary release gates are unchanged.

### Selected aim-at-hub gains

The production hub-aim controller uses P=6.0, I=0.0, D=0.0. In the nine-configuration desktop
sweep, this setting gave the highest mean moving feed availability (about 53%) and 109/109
sampled moving releases passed the independent flight check. These are single-run simulation
results, not measured hardware performance. Flywheel gains remain P=2.0, I=0.0, D=0.0.

A repeat run at the selected aim gains produced one marginal moving predicted miss (43.5 cm
from center against a 42.9 cm usable limit). This reinforces that the single-run 109/109 result
is not a guarantee; the modeled release gate still has edge sensitivity to simulated pose/motion.
The original straight-line simulation also failed its moving-release assertion in the final
P=6, D=0 verification. Both failures are recorded rather than weakening the assertions or
silently changing the selected gains. Shooting/aim unit regression checks passed (36 tests).
