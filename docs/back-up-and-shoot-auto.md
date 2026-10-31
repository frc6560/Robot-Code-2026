# Back Up and Shoot autonomous

Select **Backup - Back Up and Shoot** in **Auto Chooser** before enabling autonomous. Set the robot's field pose correctly using the existing vision/odometry workflow. This routine uses the measured pose, without a hardcoded starting-pose reset.

The command aims at the alliance hub, then uses the actual **B-Line FollowPath** follower to back away in a straight line. It uses the same translation, rotation and cross-track PID gains from `BLineConstants` as the other B-Line autos. The generated path is already in field coordinates, so it is not alliance-flipped a second time. Translation is capped at **1.0 m/s**, with **0.5 m/s²** acceleration/deceleration limits.

The path targets a **total retreat of 2.4 meters** along the direction away from the hub. It does not add distance or search farther out. The endpoint must be inside the field/alliance zone and provide a reachable shot with 0.10 m of shooting-range margin; otherwise the routine aborts. The shooting-range margin does not increase the retreat distance.

The flywheel stays at 1000 RPM during aim and retreat, then accelerates for the shot. After B-Line finishes, the measured retreat must be within the 0.03 m endpoint tolerance of 2.4 m, and translation must be below 0.05 m/s before feeding. Hub aim and all existing measured-RPM, hardware, trajectory and readiness-dwell checks still apply. Dashboard calibration mode cannot bypass autonomous trajectory checks.

Unloading runs for six seconds of actual gated feeding. There is no basket-empty sensor, so adjust `FEED_SECONDS` in `BackUpAndShootCommand` after measuring a full basket on the robot. Feeding pauses if readiness drops. The routine has a 15-second overall timeout, even if unloading is incomplete.

The routine stops on unknown/changed alliance, invalid pose/velocity, leaving the alliance zone, reaching the 0.5 m field-edge margin, pit coast, or leaving autonomous. Cancellation stops B-Line, hub aim, the shooter and both indexers. Start with a clear retreat corridor; the routine does not detect obstacles.

The dashboard reports phase, measured retreat and accumulated feeding time under **Auto/Back Up and Shoot**. Right bumper retains its existing teleop behavior. This is a generated B-Line path wrapped in a command that plans the endpoint and sequences shooting; it does not require an editor-exported path file.
