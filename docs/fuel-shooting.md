# Fuel shooting: setup, measurement, and tuning

This is the new stationary **driver right-bumper auto-turn** path. It does not use the old video calibrator,
its files, or its fitted distance policy. `FuelShotModel` integrates gravity, drag and backspin lift
from ideal wheel/hood launch conditions. All flight aerodynamics are provisional until basket tests.

## Driver behavior

- Hold **driver right bumper (RB)**: auto-turn to the alliance HUB and immediately accelerate the
  flywheel while approaching. With the formula model, RPM follows the calculated trajectory.
  Without it, use the explicit fixed-speed fallback: `Fuel/Test Flywheel RPM` defaults to **3000**,
  and `Fuel/Test Tunnel RPM` defaults to **1500**. `Fuel/Mode` identifies the uncalibrated fallback;
  its range/trajectory has not been validated.
- Once actual heading error is within **1.5 degrees**, actual turn rate is within **5 deg/s**,
  measured flywheel speed is within **60 RPM** of the request, and those conditions remain true
  for **0.10 s**, the command runs the real wall/tunnel and the floor indexer (**1200 RPM**).
- With the formula model, measured RPM must also produce a scoring trajectory at the current range. Losing any
  release condition closes the gate immediately; regaining it requires another settling period.
- Release RB: stop the tunnel and shooter. Both indexers remain stopped. No second button is needed.
- Translation remains driver controlled, but **feeding pauses above 0.15 m/s**. This version does
  not claim moving-shot compensation. Measured release delay and flight validation come first.
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

## Floor and intake current limits

The floor target remains **1200 RPM**. Only the intake rack has a startup boost.

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

The live readings `Intake/DeployCurrentAmps` and the two `RollerSupplyAmps`
fields report supply current. `Intake/DeployStatorAmps` and the indexer current
fields report stator current. Compare RPM, target RPM, voltage, and current to
check which motor responds to a request. The rack assumes a physically retracted
position at startup and uses an integrated encoder.

## When the wall does not run

Release both bumpers, then hold **left bumper alone** to request manual floor (1200 RPM) and tunnel
(1500 RPM) without alliance/aim/flywheel gates. This is an explicit ungated manual control; do not
use it to judge automatic shot readiness. Right bumper still owns the mechanisms when held and
waits for its release conditions. After releasing RB, release any held manual input before retrying.

With RB held, `Fuel/Status` gives the blocking condition (for example Aiming, No alliance, Outside
own alliance zone, Waiting for measured flywheel RPM, or Stop translating to shoot). No automatic
feed is requested until those conditions pass; the timed current limits are independent of that decision.

`Indexer/TunnelMotorTargetRPM=0` means the command is not requesting feed. A nonzero target with
`Indexer/TunnelMotorRPM` near zero needs the accompanying `Indexer/TunnelVolts`,
and `Indexer/TunnelStatorAmps`
to distinguish a motor/controller problem from a stalled mechanism.

## Fixed launch-speed coefficient

The modeled launch-speed coefficient is fixed in code at **1.025**. Dashboard edits and legacy
saved Preferences cannot override it, including during an active shot or after restart.
`Shooter/Launch Speed Coefficient` is now display-only; `Shooter/Applied Launch Speed Coefficient`
shows the fixed value while shooting. Changing this coefficient requires editing code and redeploying.
This applies to both right-trigger and Y calculator shots. Fixed-RPM calibration mode still bypasses
trajectory selection as before.

## Live motor and shooting telemetry

Connect to the robot and look under `SmartDashboard`. The swerve, shooter,
intake, and indexer folders contain position, RPM, target, voltage, and current
readings. Swerve also reports each steering absolute encoder. On the rack and
steering motors, a position request supplies the target and the motion profile
supplies target RPM when available. `NaN` indicates no active speed reference.
Shooter and hub-aim diagnostics remain under `Shooter`, `Fuel`, and `HubAim`.

## Driver control swap

Right trigger now aims/shoots (formerly RB); left trigger manually feeds floor/tunnel
(formerly LB). Right bumper extends the intake and requests rollers while held;
left bumper retracts the intake. Release right trigger before resuming held manual feed.
This mapping supersedes the older button names above.

## Driver Y: calculator shot with manual alignment

Hold Y to run the same distance/RPM calculator, flywheel readiness, stationary-speed gate,
feed dwell, indexers and intake retraction as the right-trigger shot. Y assumes the driver has
already pointed the shooter at the hub: it skips automatic turning and the heading-alignment
check, and leaves the drivetrain under normal joystick control. Release Y to stop feeding.
Right trigger takes priority while both are held; outtake and pit-coast still block shooting.

## Driver B: raw fixed-speed shooting

Hold B to request flywheel **2700 RPM** immediately. Floor **1200 RPM** and tunnel **1500 RPM**
start only once measured flywheel speed is within **60 RPM** of the goal, and pause immediately
if it leaves that band. The same 60-RPM requirement applies to Y/RT, including passing shots.
B does not run the calculator, aim, check heading/range/alliance/translation speed, retract the
intake, or take drivetrain control. B overrides RT/Y calculator shots.
Release B to stop its outputs. Teleop enable, pit-coast and outtake priority remain in force.
The shared floor running-speed constant is now 1200 RPM everywhere, including reverse outtake.
