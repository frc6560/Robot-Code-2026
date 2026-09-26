# Updated GUI preview versus recorded robot history

The GUI reference was regenerated from the updated `phase-1-canvas-draft.json` (identical to updated `zigzag.json`) and unchanged project config. Two segment maximum speeds changed from 2.0 to 4.1 m/s; waypoint positions did not change. The same 169 recorded robot positions were reused, verified byte-for-byte identical to the previous reconstruction. No robot simulation was substituted for the actual recorded history.

| Approximate order-aligned geometry metric | Previous reference | Updated reference |
| --- | ---: | ---: |
| RMS deviation | 10.77 cm | 9.84 cm |
| 95th-percentile deviation | 24.65 cm | 21.11 cm |
| Maximum sampled deviation | 34.97 cm | 29.46 cm |
| Mean deviation | 7.31 cm | 7.49 cm |

Changing the reference reduces RMS and tail errors, but not every metric improves. The endpoint error is unchanged: last stored position is 1.33 mm from the final waypoint by the robot's estimate. This is not proof of a stopped physical pose.

The updated GUI preview contains 304 poses at 20 ms intervals, predicted duration 6.06 s, and polyline length 17.93 m. The recorded robot polyline is 19.18 m. Its actual duration cannot be measured from the available history because individual positions have no timestamps.

The plot uses blue for the GUI curve, orange for the unchanged recorded robot path, and red for waypoints. The same path-order-preserving XY DTW and continuous local curve projection method was used as before, with 2 cm distance-uniform sampling and no pose fitting, translation, rotation, or endpoint snapping. This is an approximate geometric alignment, not time-indexed tracking error or proof of PID quality.

Unrestricted nearest-curve distance, which can hide wrong-leg/lap correspondence, gives RMS 5.71 cm, P95 13.36 cm, maximum 21.84 cm. These are optimistic lower-bound results, not the headline comparison. The order-aligned maximum is near (7.762, 5.997) m; its larger discrepancy depends on matching the corresponding return portion, rather than any other nearby lap.

Important limitations remain: the export has one stored history snapshot, not a verified latest-run timeline; historical constraints/config cannot be authenticated from it; heading and per-position timing are missing; the positions are estimated, not independent ground truth; interpolation can miss excursions between original samples. The updated reference is used at the user's direction, not claimed to be proven historical configuration.

Files: `path-comparison.png` / `.svg`, `analysis-summary.json`, `deviation-by-distance.csv`, `reconstructed-positions.csv`, and regenerated `current-gui-preview/gui-preview.csv` with metadata. Previous results remain preserved separately.

Provenance:

- BLine-Web preview code revision: `e7bad71d071e2a485f0170bf2ced32e3c370ad45`.
- Updated path SHA256: `6db4c6319e7eba25192ea2c288d4d59eb48523cfad93657232e24d13d9090df0`.
- Config SHA256: `f31e2ec5d1216a48ffb1ff1415dd1ab13da1a1661fa9a1b72eb4ed9f3f6301db`.
- Recorded CSV SHA256: `61ce592a9c09640f57f0349d1a7f44f01f653c554d291e92b96bf94e05d7463b`.

The user's updated deployed paths/config were not edited. No controller gains or robot code changed, and nothing was deployed or committed.
