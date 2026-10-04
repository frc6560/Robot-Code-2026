# BLine PID optimization against straight waypoint segments

## Result

The optimizer did **not** find a repeatable PID change that made the simulated path materially closer to the ordered straight lines joining the waypoints. The current gains remain the recommendation:

| Gain | Recommended value |
| --- | ---: |
| Translation P | 5.0 |
| Translation D | 0.5 |
| Cross-track P | 5.0 |
| Cross-track D | 0.0 |
| Rotation P | 3.0 (held fixed) |
| Rotation D | 0.3 (held fixed) |

No PID constants were changed.

## Search

The reference contains 878 points at 2 cm spacing along all ten ordered waypoint-to-waypoint segments, including both passes. The search varied translation P/D and cross-track P/D. Rotation gains were held fixed because the requested objective was XY straightness. Every scored candidate had to finish autonomous and pass the post-auto stopping check.

Across two bounded searches, 36 trials were recorded, 35 completed and scored, and one was rejected by the completion/stopping gate. A final independent repeat was also run for the strongest alternate set from the first search. Apparent one-run improvements did not repeat.

| Repeated gain set (Translation P/D, Cross-track P/D) | Runs | Mean optimizer RMS | Outcome |
| --- | ---: | ---: | --- |
| **5.0 / 0.5, 5.0 / 0.0** | 4 | 22.75 cm | Current baseline |
| 5.75 / 0.5, 6.5 / 0.0 | 3 | 22.56 cm | 0.7% score improvement; below the 3% and noise thresholds |
| 3.5 / 0.25, 10.0 / 0.35 | 3 including final independent repeat | 22.73 cm | No improvement over baseline |
| 5.75 / 0.5, 5.0 / 0.0 | 3 | 23.01 cm | Worse |

The formal repeated-run decision rejected both finalists. For the closest finalist, its mean score was effectively identical to the baseline and its RMS advantage was about 0.19 cm, smaller than run-to-run simulation variation.

## Representative current-gain simulation

The final figure uses the median current-gain validation run and a continuous, order-aligned geometric projection onto the straight reference:

| Metric | Current gains |
| --- | ---: |
| RMS deviation | 17.60 cm |
| 95th percentile | 44.29 cm |
| Maximum deviation | 63.22 cm |
| Autonomous duration | 10.10 s |
| Final waypoint error | 2.95 cm |

These values differ from the optimizer table because the final figure uses continuous projection onto reference segments, while the fast optimizer uses discrete ordered reference samples. Both comparisons show the same broad loops at the sharp reversals.

## Interpretation

The dominant error is not a small cross-track bias that PID can remove. It is the robot's simulated inertia while the commanded path changes direction sharply at high segment speeds. A straight polyline also assumes instantaneous corners, which is not dynamically feasible. The next effective optimization variables are segment velocity/acceleration and corner approach/deceleration behavior. Increasing PID further produced either no repeatable benefit, larger loops, or a failed completion/stopping run.

## Files

- `reference/straight-waypoint-reference.csv`: exact ordered straight-line target.
- `summary/all-optimizer-runs.csv`: every recorded optimization run.
- `summary/gain-set-averages.csv`: averages grouped by gain set.
- `search-02/validation-decision.json`: repeated-run acceptance decision.
- `final-current-gains/simulation-vs-straight-waypoints-final.png`: final comparison figure.
- `final-current-gains/analysis-summary.json`: final continuous-projection metrics.

This is WPILib/MapleSim evidence, not physical-robot validation.
