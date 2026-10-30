# Testing camera position and shooting readiness

The saved AdvantageKit `.wpilog` now contains the vision diagnostics. Open it in
AdvantageScope, or connect AdvantageScope to the robot live. Look under RealOutputs
in a saved log (or AdvantageKit/RealOutputs live). Start with these folders:

- `Vision/Front`: physical front camera, network name `limelight-front`.
- `Vision/Left`: physical left camera, network name `limelight-left`.
- `Vision/Right`: physical right camera, network name `limelight-right`.
- `Swerve/Pose`: the combined robot position and heading used for driving/aiming.
- `Fuel/Status`: first reason feeding is blocked. `Fuel/Readiness` shows individual
  conditions; `Fuel/Feeding` is the actual automatic feed decision.

## A short test

1. Release automatic aim. With the robot stationary and a tag visible, check the
   physical camera's `Connected`, `TargetVisible`, `PrimaryTagID`, and `TagCount`.
   `TagIDs` is the full list contributing to the MegaTag2 estimate when supplied
   by the camera. The primary target can be a different tag from that full list.
2. Check `ConfiguredMount` against `ReportedMount`. The configured Pose3d is a
   container for Limelight forward/right/up values, NOT a WPILib left-positive
   transform. ReportedMount is [forward, right, up, roll, pitch, yaw] in meters/degrees.
   Code writes mounting configuration at startup, overriding previous web settings.
3. Put `Swerve/Pose`, `Vision/Front/MegaTag1Pose`, and
   `Vision/Front/MegaTag2Pose` on the same 2D field. Camera poses are arrays with
   one robot pose, or an empty array when no tagged finite estimate is available.
   A missing reading is not drawn as a robot at (0,0). Check frame ages too:
   even a finite tagged pose may be stale. Camera poses show estimates, including
   rejected ones; only Swerve/Pose is the combined result used for driving.
4. Rotate gently without translating. Check Swerve/Pose rotation and
   `Vision/Front/HeadingSentDeg`. Both should track the actual robot rotation.
   MegaTag2 uses the heading supplied by the robot; it is not an independent
   check that the robot's direction sensor is correct. Normal vision fusion
   effectively ignores camera heading corrections.
5. Read `Status`. `Used` means a new reading was inserted this loop; `Waiting for
   new frame` between camera frames is normal. Use `SecondsSinceUsed` to judge
   whether updates continue. -1 means none has been used since program startup.
   Disconnected cameras, absent tags, invalid data, timestamps more than 0.5 s
   old, repeated frames, rapid rotation, and >2 m position differences are rejected.
6. If the camera pose matches the real robot but Swerve/Pose is >2 m away, the
   jump guard prevents automatic recovery. The current driver X button explicitly
   resets to the FRONT camera's fresh tagged MegaTag1 pose. It changes heading
   as well as position: first verify that pose while stationary, then use the
   reset in a controlled teleop test. `Vision/Front/ResetStatus` records the result.
   Do not use a reset to hide a wrong field map or a jumping camera estimate.
7. Test shooting only after the combined pose is believable. Hold the aimed-shot
   control and read Fuel/Status, Fuel/Readiness, Fuel/RangeMeters, and HubAim logs.
   "Settling" is the short required readiness dwell. "Ready" and Feeding=true
   should command nonzero Indexer/FloorTargetRPM and Indexer/TunnelTargetRPM.
   Nonzero targets with no physical motion indicate an output/hardware issue;
   zero targets with Feeding=false indicate a release condition has not passed.
   Readiness booleans describe the last active command execution; Fuel/Status
   "Released" and Feeding=false identify an inactive command.

Driver Start sets the current heading to 0 degrees. In the blue-origin field
frame, 0 means facing the red alliance wall, not facing whichever tag is visible.

## A single practice tag is a miniature field-map test

Tag numbers identify fixed field landmarks, not robot cameras. Any camera can
see any tag. In WPILib's official 2026 welded field map, tag 25 is on the blue
hub's face toward the blue wall, at approximately X=4.02186 m, Y=4.39024 m,
Z=1.12395 m with yaw=180 degrees. Its center is about 44.25 inches above the
floor. Tag 26 is beside it on the same face.

If a loose tag 25 is placed somewhere else, the camera STILL assumes it occupies
that mapped location and orientation. Either reproduce the mapped arrangement
(including tag size, height and upright orientation), interpreting room positions
as a virtual competition field, or upload a measured practice map to each camera.
A practice map used with hub auto-aim must share the robot code's field coordinate
frame and hub location. One tag can support localization, but viewing multiple
correctly placed tags generally gives a stronger geometric constraint. A tag
number alone does not tell the camera where the actual practice hub is.

Official map:
https://raw.githubusercontent.com/wpilibsuite/allwpilib/v2026.2.1/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2026-rebuilt-welded.json

## Camera names and configuration

Hardware hostnames and code now both use limelight-front, limelight-left, and
limelight-right. Deploy this version after renaming the cameras. The dashboard
must show Vision/ConfigurationID = three-ll4-named-2026-10-30 and
Vision/ConfiguredCameraCount = 3. Limelights communicate over
the robot's Ethernet/IP network, using NetworkTables to exchange data with the
roboRIO; they are not motor-controller devices listed in Phoenix Tuner.

For future hostname changes, connect the laptop to the robot network and use
Limelight Hardware Manager to discover/open each camera, or use its IP address
with port 5801 in a browser. Settings contains the hostname and team number.
Give each camera a unique name, such as limelight-front, limelight-left, and
limelight-right. Change the robot code's camera list, pose switch, logging-role
mapping, and reset references in the same deployment. Renaming hardware alone
would disconnect the old code from that camera's NetworkTables entries.

Quick-start:
https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-4

## Drawing the robot and diagnosing unreachable shots

In AdvantageScope live, create a 2D Field tab. Drag the entire
`AdvantageKit/RealOutputs/Swerve/Pose` structured value onto it, not its individual
X/Y members. Alternatively use the `SmartDashboard/Swerve/Field` Field2d value.
While the aimed shot is held, `Fuel/RobotPose`, `Fuel/HubPose`, and `Fuel/ReleasePose`
are also structured Pose2d outputs suitable for overlaying on the same field.
The hub is a reference marker, not another robot. These Fuel poses describe the
last active shot execution; Swerve/Pose updates continuously.

When Fuel/Status says "No reachable shot", read `Fuel/Trajectory/Reason`
(or SmartDashboard/Fuel/TrajectoryReason). The trajectory folder records the
range last solved, RPM search limits, launch angle/speed scale and progressive
candidate counts: reached hub height, cleared near rim, landed in opening, and
passed minimum entry angle. A zero at one stage identifies the first stage
where all RPM candidates failed. No model trajectory is bypassed by these logs.
Compare `Fuel/RangeMeters` with the actual distance to hub center, not its edge.
