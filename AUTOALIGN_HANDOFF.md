# AutoAlign / Autopilot Handoff — for the next agent

You're continuing an AprilTag-relative **auto-align** feature on a 2026 FRC swerve robot. The human's
context window ended. Read this top-to-bottom before touching anything.

Repo: `/Users/kevinchou/Alpha-Code-2026/Robot-Code-2026`   Branch: **`autopilot`**   (compiles; tree committed)

---

## 0. BUILD (do this or waste an hour)
System Java is 24 and breaks Gradle. **Always build with the WPILib JDK 17:**
```bash
cd /Users/kevinchou/Alpha-Code-2026/Robot-Code-2026
JAVA_HOME="$HOME/wpilib/2026/jdk" ./gradlew compileJava --offline
```
Exit 0 + `BUILD SUCCESSFUL` = good. Everything is cached; use `--offline`.
Do NOT spawn subagents/workflows for this — it's single-file work with full context here.

⚠️ **Branch confusion warning:** a different branch has `ClimbCommand.java` / `ClimbCommandauto.java`
and NO `Autoalign.java`. If files look "deleted," you're on the wrong branch — `git checkout autopilot`.

---

## 1. There are TWO separate aligners — don't mix them up

| File | Purpose | Frame it drives in | Status |
|---|---|---|---|
| `commands/Autoalign.java` | **Intake** align to a tag (the ACTIVE work) | tag-relative pose, **gyro** heading for output | Being tuned on-robot |
| `commands/automations/AutoAlignCommandFactory.java` | Reef align + `getAlignHeadOnToTag(tagId, standoff)` | tag-anchored fake-pose sharing gyro orientation | More complete/reviewed; also in use |

Both use the **Autopilot** library (`com.github.therekrab:autopilot:1.5.0`, a vendordep) and are
tag-relative (never trust global field pose). `AutoAlignCommandFactory.getAlignHeadOnToTag` is the
"gold" reference — if `Autoalign.java` gets messy, mirror that one.

Bindings (`RobotContainer.configureBindings`):
- **Y** `whileTrue` → `autoAlign.getAlignHeadOnToTag(21, feet(2.0))`  (AutoAlignCommandFactory)
- **B** `onTrue` → `new Autoalign(drivebase)`  (the intake align)
- ⚠️ **B IS DOUBLE-BOUND**: also `driverXbox.b().onTrue(...)` / `.onFalse(...)` around lines 216/220 for
  something else. Pressing B fires BOTH. Pick a free button for one of them (A is open).

---

## 2. `Autoalign.java` — how it works NOW (current, correct design)

Goal: drive the robot to a spot a fixed distance in front of a tag and face it, so the **intake**
goes into it. Everything is derived from the tag via the Limelight (`limelight-br`), not odometry.

Per-tick loop in `getDriveToTarget()`:
1. `currentPose = getTagRelativeRobotPose("limelight-br")` — robot pose **in the tag frame**.
2. `target = new APTarget(kTargetPose).withEntryAngle(kEntryAngle)` — goal in the tag frame.
3. `output = kAutopilot.calculate(currentPose, robotRelativeSpeeds, target)` → field-…er, **tag-frame**
   vx/vy + a heading reference.
4. **GYRO-HEADING output conversion (the key fix):**
   ```java
   Rotation2d gyro = drivetrain.getPose().getRotation();
   Rotation2d tagToField = gyro.minus(currentPose.getRotation());     // tag-frame -> field frame
   Translation2d velField = new Translation2d(xVel, yVel).rotateBy(tagToField);
   drivetrain.driveFieldOriented(new ChassisSpeeds(velField.getX(), velField.getY(), rotVel));
   ```
5. `until(atTarget(getTagRelativeRobotPose(cam), target))`, `finallyDo(stop)`.

The tag-relative pose helper (the axis remap — REQUIRED, see §4):
```java
private Pose2d getTagRelativeRobotPose(String limelightName) {
    Pose3d r = LimelightHelpers.getBotPose3d_TargetSpace(limelightName);
    Translation3d fwd = new Translation3d(1,0,0).rotateBy(r.getRotation());
    return new Pose2d(r.getZ(), -r.getX(), new Rotation2d(Math.atan2(-fwd.getX(), fwd.getZ())));
}
```

Constants:
- `kCamera = "limelight-br"` (the back-right cam; per Constants.LIMELIGHT_NAMES).
- `kStandoffMeters = 0.5` — TUNE (tag-face → robot-center distance).
- `kTargetPose = new Pose2d(kStandoffMeters, 0.0, new Rotation2d(0))` — tag frame: x=out from face,
  y=left, heading `0`. **Heading 0 = intake/front pointed at the tag IN THIS SETUP** (see §3).
- `kEntryAngle = kTargetPose.getRotation()` — tied to target heading (front-first intake ⇒ facing == travel dir).
- Autopilot: `withVelocity(0.3).withAcceleration(0.3).withJerk(1.0)` — **intentionally very low for testing**;
  raise once it behaves. `withErrorXY(2cm).withErrorTheta(0.5deg).withBeelineRadius(8cm)`.
- Heading PID `rotationController = new PIDController(5.5, 0.15, 0.05)` (continuous input).

---

## 3. THE TWO BUGS WE HIT (so you don't reintroduce them)

**A. "Spun 180° and didn't translate."** Root cause was two-fold:
- Target heading was `π` while the robot's tag-frame heading was ≈`0` → 180° error → hard spin.
- The old output conversion used `fromFieldRelativeSpeeds(vx,vy,rot, currentPose.getRotation())` where
  `currentPose.getRotation()` is the **vision tag-frame heading**. While spinning, that angle whips
  around, so the robot-relative translation flips every tick and **averages to zero** → pure spin.
- Fixes applied: (1) target heading set to `0` (so ~no rotation error), (2) **switched to gyro-heading
  output** (§2 step 4): `tagToField = gyro − tagHeading` is constant even mid-spin, so field velocity
  stays pointed at the goal and `driveFieldOriented` maintains it through rotation. **Use gyro heading
  always** — don't revert to the vision-heading conversion.

**B. `rotationController` NPE.** A prior edit deleted the `new PIDController(...)` init but kept
`.enableContinuousInput(...)`. It's fixed; keep the init in the constructor.

---

## 4. Why the axis remap (recurring question)
`getBotPose3d_TargetSpace` is in Limelight **camera** convention: **X=right, Y=down, Z=out of the tag**.
`Pose2d`/Autopilot need **X=forward, Y=left, yaw about vertical**. So `.toPose2d()` on it is the wrong
plane (y would be "down"). The remap maps forward=`Z`, left=`-X`, heading from the projected forward
vector. There is NO pre-made call that returns a floor-plane tag-relative Pose2d — the ~3-line remap is
required. (Do not reintroduce the old `1/sqrt(ta)`+`tan(tx)` trig; that was deleted on purpose.)

---

## 5. Debug logging (NetworkTables, `Climb/Prescore/…` — labels only, this is intake not climb)
View in Glass/AdvantageScope. Read while triggering:
- **`HasTarget`** (getTV), **`Tid`**, **`BotposeNorm`** ← if 0, the Limelight isn't publishing
  `botpose_targetspace`; everything downstream is garbage (fix the LL first).
- `Current_X/Y/HeadingDeg` (robot in tag frame), `Target_X/Y/HeadingDeg`.
- `Error_X/Y/Rot_Deg`, `Distance`.
- `Output_X_Vel/Y_Vel` (tag frame), **`Output_Field_X_Vel/Y_Vel`** (should stay ~steady pointing at the
  goal even while rotating — proof the gyro-heading fix works), `GyroDeg`, `TargetAngleDeg`, `At_Target`.

---

## 6. OPEN ITEMS / NEXT STEPS
1. **Confirm target heading** empirically: position the robot exactly how it should FINISH (intake into
   the tag), read `Climb/Prescore/Current_HeadingDeg`, set `kTargetPose` heading to that number. It's
   `0` now based on the 180-spin observation; verify on the real robot.
2. **Tune `kStandoffMeters`** (0.5 m) to the real intake distance (tag-face → robot-center; subtract
   bumper if needed).
3. **Raise `withVelocity`/`withAcceleration`** from 0.3/0.3 once motion looks correct.
4. **Fix the B double-binding** (§1) — move one binding to a free button (A).
5. Verify translation now happens with rotation (watch `Output_Field_X/Y_Vel`).

## 7. HARDWARE-VERIFY (must be true on the robot)
- **Gyro field-zeroed** (driver **Start** = `zeroNoAprilTagsGyro`). The gyro-heading conversion needs it.
- **`limelight-br`**: exact NT name, AprilTag pipeline, **3D / `botpose_targetspace` output enabled**,
  tag size set. The tuner's 3D box is NOT the same as the `botpose_targetspace` NT output.
- **No no-tag guard**: `getTagRelativeRobotPose` returns `~(0,0)` if the tag is lost (checks were removed
  on request) → it would drive toward a phantom goal. Only run it while the tag is visible, or re-add a
  guard (return early if `getBotPose3d_TargetSpace(cam).getTranslation().getNorm() < 1e-3`).

## 8. KEY DESIGN DECISIONS (don't undo)
- **Tag-relative, not global pose** (team distrusts field-map calibration).
- **Gyro for heading + the output-frame conversion**, tag/vision for position only (PnP yaw is
  flip-prone; gyro is stable). This is why translation survives rotation.
- **Autopilot in the tag frame** with the anchor cancelling; output rotated to field via the constant
  `gyro − tagHeading` offset.
- Prefer **pre-made** Limelight/WPILib calls over custom trig.
- If `Autoalign.java` degrades, the correct, reviewed pattern is
  `AutoAlignCommandFactory.getAlignHeadOnToTag` — copy from it.
