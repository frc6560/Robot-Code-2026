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

The floor target remains **800 RPM**. Startup stator-current allowances are:

| Mechanism | First second of a new move | Continuing / stopped | Supply limit |
|---|---|---|---|
| Each floor motor | 40 A | 40 A | 40 A |
| Wall/tunnel motor | 40 A | 40 A | 40 A |
| Intake rack | 60 A while traveling | 40 A (also on arrival) | 30 A |
| Each intake roller | 40 A | 40 A | 40 A |

The indexers have no startup boost: both floor motors and the wall/tunnel stay at 40 A stator and
40 A supply. Only the intake rack retains its startup window.

The rack startup window begins with the nonzero motor command, not after an encoder detects motion;
a jam cannot keep the 60 A allowance indefinitely. Repeating the same command each loop, or
changing a running indexer's RPM has no effect on its fixed 40 A ceiling. Stopping restores 40 A;
a subsequent stop-to-start receives a new startup window. Selecting a different rack endpoint
starts a new rack move; repeatedly holding the same endpoint does not. The rack also returns to
40 A as soon as its measured position reaches the selected endpoint tolerance.

The robot checks expiry every control cycle (normally 20 ms), independently of new button events.
Only current-limit changes write controller configuration; repeated unchanged limits do not.
Configuration writes use bounded timeouts. If a limit change cannot be confirmed, motor output
is stopped while the limit update is retried, rather than continuing with an unconfirmed limit.
This is a controller-configuration failure response, not the removed stall cutoff.

Current limit telemetry: `Indexer/FloorStatorLimitAmps`, `Indexer/TunnelStatorLimitAmps`, and
`Intake/DeployStatorLimitAmps`. Their corresponding `FloorCurrentLimitOK`, `TunnelCurrentLimitOK`,
and `DeployCurrentLimitOK` flags indicate whether the controller update was confirmed. Stator and
supply limits are independent; allowing 60 A stator does not increase the listed supply limits.

Automatic floor and rack stall cutoffs have been removed by operator request. Sustained high-current,
low-speed operation no longer cancels the requested RPM/position or requires a disable to reset.
The timed startup/current limits above, rack soft limits, disable/pit-coast behavior, and roller
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
