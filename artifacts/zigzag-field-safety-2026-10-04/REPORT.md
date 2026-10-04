# Zigzag field-boundary simulation check

## Result

The actual **Zigzag** autonomous selector was simulated three times. All three runs met the requested constraint: the robot-center Y coordinate never exceeded its starting Y coordinate of **7.40708 m**. The check includes autonomous motion and the logged stopping interval after the path completed.

| Run | Duration | Maximum center-Y overshoot | Estimated minimum bumper-to-border margin | Final position error |
|---:|---:|---:|---:|---:|
| 1 | 6.555 s | 0.00 cm | 22.50 cm | 3.22 cm |
| 2 | 6.483 s | 0.00 cm | 22.47 cm | 3.12 cm |
| 3 | 6.518 s | 0.00 cm | 22.48 cm | 3.17 cm |

The largest center Y in every trace was exactly the starting value. The generated waypoint geometry also has no waypoint above the starting value.

## Footprint check

The bumper clearance estimate accounts for the robot's heading and the full rotated rectangular footprint, not merely its center. It assumes the project's default robot footprint of **0.812 m by 0.812 m** and a field upper boundary at **Y = 8.07 m**. Under those assumptions, the smallest simulated bumper-to-boundary clearance was **22.47 cm**.

## Safety limitation

This is simulation evidence, not a collision guarantee. Before a full-speed field run, measure the real bumper length and width and confirm the robot's physical starting placement. The 22.47 cm result can shrink because of pose error, wheel slip, carpet behavior, delayed stopping, mechanical differences, or a mismatch between the configured and real footprint. The path also has zero center-line buffer at its return endpoint because it returns to the starting Y.

## Files

- `summary/boundary-check.svg`: path and boundary visualization
- `summary/run-summary.csv`: per-run measurements
- `summary/boundary-summary.json`: machine-readable aggregate result
- `run-01/robot-trace.csv`, `run-02/robot-trace.csv`, `run-03/robot-trace.csv`: extracted simulated robot traces
- Each run directory also contains the WPILib log and simulator console output.

No robot, PID, or path configuration was changed for this check. The only source file added is the repeatable boundary-analysis utility in `tools/analyze-zigzag-boundary.mjs`.
