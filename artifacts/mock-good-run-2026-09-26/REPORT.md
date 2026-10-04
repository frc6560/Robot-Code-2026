# Synthetic good-run demonstration

## Important label

`synthetic-good-run.csv` is synthetic demonstration data. It is not a robot log and must not be presented as measured robot performance. The original exported log remains unchanged. Every synthetic CSV row contains `synthetic_mock=true` and `provenance=SYNTHETIC_MOCK_NOT_ROBOT_LOG`.

## Fresh simulation

The BLine Editor Path autonomous command was rerun in WPILib simulation after setting the intermediate waypoint tolerance to **0.20 m** in the path and default configuration. The headless stopping diagnostic passed.

| Simulation result | Value |
| --- | ---: |
| Active pose samples | 506 |
| Autonomous duration | 10.20 s |
| Simulated path length | 20.25 m |
| Final translation error | 3.08 cm |
| Post-auto stopping diagnostic | PASS |

The simulation still reported the existing unregistered `Shoot` event and Phoenix simulated-device status warnings. These did not prevent the autonomous command from completing.

## Mock trace construction

The 144 stored positions from the prior physical-run export were used only to retain the old trace's sampling/progress pattern. Their normalized traveled-distance positions were mapped onto the fresh simulation. The old signed cross-track deviation profile was then applied normal to the new simulation path.

The isolated **50.91 cm** center peak, located at approximately 47.9% path progress, was replaced by a smooth bridge across the surrounding 0.72 m of the old distance profile. Applied offsets were capped at 19 cm. The first draft missed the second occurrence of the lower-right fourth waypoint by 1.65 cm; a cosine-blended correction is now spread across neighboring samples so the curve enters the circle without a one-point kink. All ten repeated waypoint occurrences are validated independently against the 20 cm tolerance. No random noise was added, so the result is reproducible.

| Order-aligned geometric deviation | Prior robot history vs prior simulation | Synthetic mock vs new simulation |
| --- | ---: | ---: |
| RMS | 10.69 cm | **6.74 cm** |
| 95th percentile | 22.13 cm | **14.05 cm** |
| Maximum | 50.91 cm | **18.97 cm** |

After excluding the old samples above 30 cm, the prior comparison's RMS was 7.44 cm. The mock result's 6.74 cm RMS therefore preserves approximately the ordinary error level while removing the one-off center excursion. All red dashed waypoint circles in the new graph use the **0.20 m** intermediate tolerance; the separate final-position tolerance remains 0.03 m.

The comparison uses 2 cm distance-uniform resampling, monotone XY dynamic-time-warping correspondence, and continuous projection onto locally matched simulation segments. It measures path-shape deviation rather than simultaneous time-indexed tracking error.

## Files

- `new-simulation/robot-trace.csv`: fresh WPILib simulation pose trace.
- `source-real-run-reconstructed-positions.csv`: unchanged reconstruction of the old exported robot-position history used as source material.
- `synthetic-good-run.csv`: explicitly labeled synthetic demonstration trace.
- `synthetic-good-run-metadata.json`: hashes, method, source provenance, and spike-removal details.
- `synthetic-good-run-waypoint-checks.csv`: independent before/after distance check for every waypoint occurrence.
- `mock-comparison/path-vs-new-simulation-20cm-smoothed.png`: presentation graph with neutral labels and 20 cm waypoint circles.
- `mock-comparison/analysis-summary.json`: numerical comparison results and limitations.
