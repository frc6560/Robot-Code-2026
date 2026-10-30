# Fuel shooting: setup, measurement, and tuning

This is the new stationary **driver Y auto-turn** path. It does not use the old video calibrator,
its files, or its fitted distance policy. `FuelShotModel` integrates gravity, drag and backspin lift
from measured launch conditions. All flight aerodynamics are provisional until basket tests.

## Driver behavior

- Hold **driver Y**: auto-turn to the alliance HUB and spin the flywheel to the calculated RPM.
- Once actual heading error is within **1.5 degrees**, actual turn rate is within **5 deg/s**,
  measured flywheel speed is within **60 RPM** of the request, and those conditions remain true
  for **0.10 s**, the command runs the floor indexer and real tunnel.
- The measured RPM must also produce a scoring trajectory at the current range. Losing any
  release condition closes the gate immediately; regaining it requires another settling period.
- Release Y: stop the tunnel and stop the shooter. The normal floor-only default resumes.
- Translation remains driver controlled, but **feeding pauses above 0.15 m/s**. This version does
  not claim moving-shot compensation. Measured release delay and flight validation come first.
- Driver Station alliance and a finite, correctly localized pose are required. Only shots from
  your own alliance zone are allowed. Start by resetting localization at a known position.
- Pit coast, disconnected flywheel motors, or a disconnected/misconfigured tunnel block feeding.

The tunnel no longer runs automatically on enable. Existing explicit manual controls (bumper /
operator A / button-board 11 spin-up; trigger / board 3 feed) remain ungated overrides outside Y.
When Y is released with a manual control still held, all those controls must release before manual
feeding can resume. Do not use the manual override to evaluate the automatic release gate.

This change replaces the driver-Y shot path. Legacy button-board shot actions and autonomous
shooting routines are not wired to this new calibrated model. In particular, legacy auto shooting
that relied on the always-running tunnel will now need an explicit tunnel-owning shot command;
do not rely on those routines for calibrated firing. The separate timed center-auto still explicitly
owns its tunnel. The intake rack and intake rollers retain their existing automatic behavior.

## First measurement session (no calibration file needed)

1. Deploy the code normally. In Driver Station select the correct alliance and establish a known
   pose so auto-turn points in the intended direction. Use a clear shooting area and remote filming;
   place/secure rulers or reference boards only while the robot is disabled.
2. In the dashboard set:
   - `Fuel/Calibration Mode` = **true**
   - `Fuel/Test Flywheel RPM` = **2500** to start
   - `Fuel/Test Tunnel RPM` = **1500**
3. Load one ball for each measurement. Hold **Y** to aim and spin up. In measurement mode,
   **Y alone never feeds**: additionally hold the **right trigger** to feed after alignment/speed settle.
   Release the trigger between balls; release Y before changing test values. RPM settings and mode
   are sampled on Y press, not changed in the middle of a shot.
4. Watch `Fuel/Status` for the reason a shot is waiting. `Fuel/Measured RPM` is flywheel shaft RPM,
   already corrected for 12:18 gearing. Do not multiply it by the gear ratio again.
5. Measure at least **three repeated shots at each of at least three speeds**. A useful first session
   is **2000, 2500, 3000, 3500, 3800 RPM**, keeping the tunnel at 1500 throughout. Start with the lower
   speeds and verify behavior before increasing. Use the stable observed shaft RPM averaged over
   each speed group as the group's `rpm` in the measurement file. Record pre-contact RPM as well as
   RPM drop/recovery in the robot log. If pre-shot RPM differs substantially across repetitions,
   resolve speed control first instead of averaging it away.

A ruler/plank is a **visual scale beside the trajectory**, not an added ball-contact surface. Do not
flatten the trajectory by making the ball hit a plank. A horizontal test fixture is usable only if
it preserves the same flywheel/hood/compression/feed contact geometry; otherwise its calibration
will not describe the installed shooter. The easiest useful recording is the installed shooter
at its confirmed 48.36-degree angle, with the camera perpendicular to the trajectory plane.

## Measure horizontal exit speed

Keep the camera still. Place the reference scale in the same depth plane as the ball path.
Track the **ball center** over a short interval immediately after it stops touching the shooter;
measure true horizontal displacement, not distance along an inclined plank or along the arc.

Use the original camera **capture FPS**, e.g. 240, not 30 FPS slow-motion playback/export timing:

```
time_seconds = frame_intervals / capture_fps
horizontal_velocity_mps = horizontal_distance_m / time_seconds
exit_speed_mps = horizontal_velocity_mps / cos(actual_launch_angle)
```

From frame 100 to frame 112 is **12 frame intervals**, not 13. Fractional frame intervals are allowed
if you interpolate crossings of reference lines. Use several visible frames to reduce timing error,
while staying close enough to release that aerodynamic deceleration is small. If a long interval is
needed because of blurry/low-resolution video, report that limitation rather than treating its
average velocity as exact exit velocity. Record the actual camera frame rate for each clip.

Optional spin: follow an existing seam/mark over the same near-exit region. Enter signed turns and
frame intervals. Positive `spin_turns` means **backspin giving upward lift**; negative means topspin.
Measure all shots' spin, or leave both spin columns blank for every shot. Blank spin uses the
explicit estimate `omega = exit_speed / ball_radius`; this is an assumption, not measured Magnus lift.

## Enter measurements and build the robot file

Fill `tools/fuel-calibration/measurements.csv`, one row per shot. Its header is:

```
rpm,tunnel_rpm,horizontal_m,frames,capture_fps,launch_angle_deg,spin_turns,spin_frames
```

- `rpm`: stable observed flywheel shaft RPM for the test group, in the range 1000–3800.
- `tunnel_rpm`: the fixed tunnel setting used during all these tests, e.g. 1500.
- `horizontal_m`: ball-center horizontal travel in meters near release.
- `frames`: corresponding frame intervals (can be fractional).
- `capture_fps`: original camera frame rate.
- `launch_angle_deg`: 48.36 for the installed shooter unless you measure a different actual exit
  angle. A tangent angle of the hood alone may differ from the actual trajectory.
- `spin_turns`, `spin_frames`: optional signed rotations and timing; both blank if unmeasured.

Run from the repository root, with ordinary Python 3; no packages or video processing are required:

```sh
python3 tools/fuel-calibration/calibrate.py tools/fuel-calibration/measurements.csv
```

It averages repeated measurements at each RPM, reports speed scatter, and writes
`src/main/deploy/fuel-launch.csv` only after validation. It rejects fewer than three repeats per
speed, mixed tunnel settings, mixed spin/no-spin data, nonmonotonic mean speed, invalid timing,
and speed scatter above 5% within a group. The previous calibration is preserved on a rejected fit.
There are no invented measured rows in the repository.

The robot linearly interpolates **measured launch speed vs shaft RPM**, never extrapolates. This
small launch calibration is not a distance-to-RPM shot table. The flight solver determines the shot
from the physics and picks the midpoint of the widest scoring RPM band it finds. It checks ball
radius, rim margin, ascent clearance and descending entry angle. A 10 RPM search grid can miss very
narrow scoring bands; such shots are deliberately not supported reliably.

Redeploy/restart after generating the file. `Fuel/Calibration Status` reports whether it loaded and
whether spin is measured or estimated. Set `Fuel/Calibration Mode` to **false** (also reset to false
at startup). Holding Y now automatically feeds after readiness; the extra trigger is only required
in measurement mode. Missing/invalid calibration leaves auto-turn available but blocks normal shots.

## Field validation and hand tuning

Start stationary at about **3.5 m from the ball release point to HUB center**, provided the dashboard
finds a reachable shot. Shoot at least 10 balls before judging accuracy. Repeat at a nearer and a
farther reachable range, then at different lateral positions at the same range. Record:

| Range | Requested/measured shaft RPM | Tunnel RPM | Makes / attempts | Short/long/left/right | RPM dip |
|---|---|---|---|---|---|

- **Scattered shots at one setting:** check compression, contact repeatability, pre-shot speed,
  feed spacing, and ball condition before changing the trajectory model.
- **Consistent distance error:** check measured launch speed, actual exit angle, release height,
  and range origin first. Keep the flywheel and tunnel settings used for calibration consistent.
- **Error changes with range:** tune effective drag/lift against multiple ranges. Do not fit all
  aerodynamic parameters from one hit/miss or one trajectory.
- **Only volleys miss:** tune tunnel rate and flywheel recovery. Automatic feeding pauses on measured
  RPM loss, but a ball already beyond the tunnel cannot be recalled. There is no ball-presence sensor
  in this implementation, so exact per-ball spacing is not measured. Lower tunnel speed and repeat
  launch calibration at that new fixed feed speed if necessary.
- **Different lateral positions miss differently:** check localization and shooter origin/yaw before
  fitting drag. `FuelShotConfig.RELEASE_FORWARD_METERS` / `RELEASE_LEFT_METERS` start at zero;
  measure from the drivetrain pose origin to the actual release point and enter these offsets.
  Positive is forward/left. The heading model assumes the shooter fires along chassis forward.

In `src/main/java/frc/robot/utility/shooting/FuelShotConfig.java`, `DRAG_COEFFICIENT=0.47`, `LIFT_SLOPE=0.75`, and spin decay 0.10/s
are initial approximations, not verified FUEL properties. `LAUNCH_ANGLE_OFFSET_DEGREES` adjusts
actual launch elevation relative to 48.36 degrees. Change it to match the measured launch angle if
needed. Missing spin uses an ideal rolling estimate. Speed measurements alone cannot validate all
these aerodynamic assumptions. Confirm basket performance before relying on multiple ranges.

Useful dashboard entries: `Fuel/Status`, `Fuel/Calibration Status`, `Fuel/Feeding`, `Fuel/Range M`,
`Fuel/Target RPM`, `Fuel/Measured RPM`, minimum/maximum scoring RPM; `Hub Aim/Heading Error Deg`,
`Hub Aim/At Target`; `Indexer/TunnelRPM`, `Indexer/TunnelReady`. Logs under `Fuel/` include tunnel
requested/measured RPM, predicted flight time, and feeding state. Old LED/calculator indicators are
legacy; use these new entries to judge this path.

Software verification:

```sh
./gradlew test
python3 -m unittest discover -s tools/fuel-calibration -v
```
