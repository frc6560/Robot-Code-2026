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
  for **0.10 s**, the command runs the real wall/tunnel and the floor indexer (**800 RPM**).
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
can resume. The driver's LT/RT rack controls are unchanged. Intake rollers run only after a valid, homed rack
reaches the requested full-extension position (9.5 motor rotations, within 0.25 rotations) and settles
for 150 ms below 0.1 motor rotations/s. Selecting retract immediately stops the rollers; losing
position/home feedback also stops them. The motor-output layer independently enforces extension,
request, homing, fresh feedback and speed checks. Rollers use brake neutral mode to reduce coasting.

This change replaces the driver shooting binding with a held-RB shot path. Legacy button-board shot actions and autonomous
shooting routines are not wired to this new calibrated model. In particular, legacy auto shooting
that relied on the always-running tunnel will now need an explicit tunnel-owning shot command;
do not rely on those routines for calibrated firing. The separate timed center-auto still explicitly
owns its tunnel. The intake rack and intake rollers retain their existing automatic behavior.

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
one roller failing to spin. The rack assumes physically retracted at code startup; its motor encoder
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
