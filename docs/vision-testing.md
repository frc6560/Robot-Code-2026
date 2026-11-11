# Testing camera position and shooting readiness

Connect AdvantageScope or Elastic to the robot. The retained telemetry is under
`SmartDashboard`. Add `Swerve/Pose` and `Vision/Front/RawPose`,
`Vision/Left/RawPose`, and `Vision/Right/RawPose` to the same 2D field. The
camera poses show the last finite estimate from each camera. `Swerve/Pose`
shows the combined robot pose used for driving and aiming.

1. With the robot stationary and a tag visible, compare the camera estimate and
   the combined pose to the robot's actual field position. The camera's web UI
   shows its visible tags and mounting configuration. Code writes the configured
   camera mount at startup.
2. Rotate gently without translating. The fused heading should track the robot.
   MegaTag2 uses the heading and yaw rate supplied by the robot. Normal fusion
   gives the camera heading very little weight.
3. The code can reuse a published camera estimate. It skips missing or zero
   poses, non-finite positions, poses more than 1 m from the field rectangle,
   turns faster than 360 degrees per second, and jumps of more than 2 m from odometry.
4. If the combined pose is more than 2 m away, the jump guard prevents automatic
   recovery. Driver X tries left, front, then right and resets to the first valid
   MegaTag1 pose, including heading. The 1 m field margin applies to that reset.
5. Hold the aimed-shot control and check `Fuel/Status`, `Fuel/Readiness`,
   `Fuel/RangeMeters`, and `HubAim`. `Settling` is the short readiness dwell.
   `Feeding=true` should coincide with nonzero `Indexer/FloorMotorTargetRPM` and
   `Indexer/TunnelMotorTargetRPM`.

Driver Start sets heading to 0 degrees on blue or 180 degrees on red; an unknown
alliance skips the reset. In the blue-origin field
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
limelight-right. Deploy this version after renaming the cameras. Limelights communicate over
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

In AdvantageScope live, drag `SmartDashboard/Swerve/Pose` or
`SmartDashboard/Fuel/RobotPose` onto a 2D Field tab. Both show the combined pose.

When Fuel/Status says "No reachable shot", read `Fuel/Trajectory/Status`. The trajectory folder records the
range last solved, RPM search limits, launch angle/speed scale and progressive
candidate counts: reached hub height, cleared near rim, landed in opening, and
passed minimum entry angle. A zero at one stage identifies the first stage
where all RPM candidates failed. No model trajectory is bypassed by these logs.
Compare `Fuel/RangeMeters` with the actual distance to hub center, not its edge.
