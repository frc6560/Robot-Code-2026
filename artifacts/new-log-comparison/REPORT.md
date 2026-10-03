# New robot log analysis

Source: `/Users/andrei/Documents/Untitled.csv`, SHA256 `8e6cb68743fc137535a70223f57d74a39b999b1e97cf5961693b08e85b4b6b60`.

## Log integrity

The CSV spans timestamp 491.502–494.200 seconds, a continuous 2.698-second export with no gaps over one second. Every row is enabled teleop (`Enabled=true`, `Autonomous=false`) and repeats an already-completed BLine history. The normal Field/Robot pose is stationary during this export. Therefore, this is a post-run snapshot rather than a timestamped recording of the auto itself.

The BLine position history has exactly 144 positions in every CSV row. Its eleven logged waypoints exactly match the current updated path. BLine v0.9.1 keeps `robotTranslations` on each FollowPath command instance and clears it in `initialize()`, so there is no evidence in this file that several command histories were appended together. The strongest interpretation is one completed command's stored history. It still cannot authenticate which physical attempt it came from because there are no timestamps or run identifier inside that array.

The final snapshot says BLine finished on the final translation and rotation elements, and both endpoint controllers were at setpoint. Remaining path distance is 1.10 cm. The last stored point is also 1.10 cm from the final waypoint by the robot's estimated pose.

## Updated GUI comparison

| Approximate geometry metric | New log | Previous supplied log |
| --- | ---: | ---: |
| RMS deviation | 10.05 cm | 9.84 cm |
| Mean deviation | 7.35 cm | 7.49 cm |
| 95th percentile | 17.78 cm | 21.11 cm |
| Maximum sampled deviation | 45.56 cm | 29.46 cm |
| Last stored point to final waypoint | 1.10 cm | 0.13 cm |
| Reconstructed robot path length | 19.61 m | 19.18 m |

The new trace is better through most of the path: its P95 deviation is 3.33 cm lower and mean deviation is slightly lower. Overall RMS is nearly unchanged. A single intermediate excursion makes the maximum substantially worse.

The largest GUI deviation occurs around reconstructed point `(4.129, 6.886)` m, approximately 9.38 m along the stored history, immediately after the first return toward the initial waypoint and before the second circuit. The current path deliberately contains two circuits in one command.

There is also a 78.65 cm gap between adjacent stored positions 53 and 54, from approximately `(5.670, 7.519)` to `(4.905, 7.338)` m. BLine stores every third `execute()` call, not every fixed 60 ms: a delayed robot loop or missing execution interval can increase elapsed time between stored points. Because the array contains no point timestamps, the graph cannot determine whether this gap represents rapid physical motion, a long control-loop/logging gap, or missing samples. Straight-line interpolation across it should not be interpreted as the exact physical trajectory.

## Simulation comparison

| Approximate order-aligned difference | Result |
| --- | ---: |
| RMS | 10.69 cm |
| Mean | 6.76 cm |
| 95th percentile | 22.13 cm |
| Maximum sampled difference | 50.91 cm |

The recorded path and current simulation are generally close over most of both circuits, but differ strongly at the same intermediate excursion. These are path-shape differences, not simultaneous position errors: the robot-history points lack timestamps, so timing and heading comparisons are unavailable.

## Method and limits

Both graphs use the same established comparison: 2 cm distance-uniform resampling, monotone XY dynamic-time-warping correspondence, and continuous projection onto locally matched reference segments. No coordinate fitting, translation, rotation, scale, or endpoint snapping is applied. Red dashed circles show the requested waypoint tolerance regions at field scale: 0.30 m intermediate waypoint tolerance and 0.03 m final translation tolerance. Because the path repeats coordinates, identical circles are drawn once; the final 0.03 m circle is also drawn inside the 0.30 m intermediate circle at the shared start/end coordinate.

The result measures the robot's estimated pose, not independent physical ground truth. It cannot distinguish PID behavior from carpet movement, wheel slip, localization corrections, delayed loops, or drivetrain interference. The order alignment is approximate on a path whose outbound and return portions overlap, and can make peak correspondence ambiguous. The updated GUI reference uses path SHA256 `6db4c6319e7eba25192ea2c288d4d59eb48523cfad93657232e24d13d9090df0`; the simulation reference is the previously saved 440-pose current-code run.

Outputs:

- `untitled-vs-gui/path-comparison-tolerance-30cm-final.png` and `path-comparison.svg`
- `untitled-vs-simulation/path-comparison-tolerance-30cm-final.png` and `path-comparison.svg`
- Detailed summaries and distance samples in each output directory

No source log, path, PID constants, or robot behavior was modified for this analysis.
