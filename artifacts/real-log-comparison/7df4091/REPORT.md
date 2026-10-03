# Logged robot path versus BLine GUI preview

Analyzed `LogsFrThePath.csv` from commit `7df4091`. Reconstructed 169 ordered x/y positions from `BLine/FollowPath/robotTranslations`, not from rigid path waypoints. All eleven logged path translations match both currently checked-in `phase-1-canvas-draft.json` and `zigzag.json` (these files are identical). The filename/launcher cannot be distinguished from this export.

Regenerated the reference directly with BLine-Web's preview simulation code at revision `e7bad71d071e2a485f0170bf2ced32e3c370ad45`, using current path/config, 20 ms timestep. It contains 340 projected poses. This matches the previously generated preview, but the export cannot establish that historical constraints/config were exactly the checked-in ones.

## Results

| Metric | Result |
| --- | ---: |
| Order-aligned RMS position deviation | 10.77 cm |
| Mean position deviation | 7.31 cm |
| 95th-percentile position deviation | 24.65 cm |
| Maximum sampled position deviation | 34.97 cm |
| Last stored position versus intended endpoint | 0.133 cm (1.33 mm) |
| Reconstructed robot polyline length | 19.18 m |
| GUI preview polyline length | 17.63 m |

The greatest sampled deviation is near robot position **(7.518 m, 5.488 m)**, approximately 14.56 m into the reconstructed traveled path. The matched GUI curve point is approximately **(7.800 m, 5.695 m)**. The robot trace overshoots/retraces around turns and the intermediate return, despite its last stored position being very close to the final waypoint. There is no end-of-command marker, so this is last-stored-position error, not confirmed stopped-pose error.

`path-comparison.png` / `.svg` show the overlay and deviation versus traveled distance. `reconstructed-positions.csv` preserves the 169 extracted positions. `deviation-by-distance.csv` contains the detailed comparison.

## Method

Linearly interpolate the stored robot history and resample it and the GUI curve at 2 cm traveled-distance intervals. Align the full sequences monotonically with XY dynamic time warping, preserving path order. Project each robot sample onto continuous GUI preview segments in its local matched interval. Report RMS, mean, interpolated 95th percentile, and maximum of those distances. No translation, rotation, scaling, pose fitting, or endpoint snapping is performed.

This is an approximate geometric comparison, not a time-indexed tracking error. Distance-uniform sampling prevents stationary samples from dominating. The fine grid does not increase the information in the original 169 robot positions. Repeat calculations at 1 cm and 4 cm spacing give RMS **10.83 cm** and **10.71 cm**, respectively, and maximum approximately **34.97 cm** in both cases.

As a deliberately optimistic cross-check, distance to the nearest point anywhere on the GUI curve gives RMS **6.72 cm**, P95 **14.51 cm**, and maximum **34.97 cm**. It can match a first-lap robot point against a second-lap GUI segment, hiding wrong-leg deviations, so it is not the headline metric. Even this lower-bound comparison shows a substantial worst excursion.

## What this export cannot establish

- `Swerve/Pose`, controller error/finished signals, and most BLine numeric outputs are all null in the exported CSV. The SmartDashboard robot pose stream contains stationary snapshots rather than the moving run.
- The stored position history contains neither heading nor a timestamp for each position. BLine v0.9.1 adds a position every third command execution. Assuming an exact 60 ms interval would fabricate timing, so heading RMS, timing lag, run duration, and settling behavior are unavailable.
- These are the robot's estimated positions, not independent physical ground truth. The analysis measures estimated-path versus GUI-preview agreement and does not isolate PID performance or validate localization.
- The current path/config are used as the historical reference assumption. Matching waypoints alone cannot prove historical speeds, acceleration limits, handoff settings, or other constraints matched.
- Interpolation may miss excursions between stored samples. The stored list has 169 points, below BLine's memory-trimming threshold, and starts at the expected initial waypoint.

Conclusion: the available recorded geometry is useful evidence, but it does **not** demonstrate exact GUI parity or acceptable trench safety. The last stored position is excellent by the robot's estimate, while the route deviates by as much as approximately 35 cm. This export is insufficient to conclude the deviations are caused by PID alone.

## Provenance

- Input CSV SHA256: `61ce592a9c09640f57f0349d1a7f44f01f653c554d291e92b96bf94e05d7463b`.
- Path SHA256: `824734f04901d113ed13fa466323e19b2f19a701ae9b7dd96a83ab94c0ce6be7`.
- Config SHA256: `f31e2ec5d1216a48ffb1ff1415dd1ab13da1a1661fa9a1b72eb4ed9f3f6301db`.
- BLine position-history behavior verified against the locally cached v0.9.1 `FollowPath.java` source, initialization lines 656–665 and execution lines 912–921.
- Tracked robot source and original CSV were not modified. Stash was not restored. No deployment or gain change.
