# Empirical PID tuning against the phase-1 GUI preview

The selected settings produce a somewhat closer XY curve, not an identical trajectory. Gains alone did not resolve the timing or heading mismatch. This is a simulation experiment, not a validated hardware tune.

## Selected constants

| Controller | Previous P / I / D | Selected P / I / D |
| --- | --- | --- |
| Translation | 5 / 0 / 0.5 | 5 / 0 / 0.5 |
| Rotation | 3 / 0 / 0.3 | 4 / 0 / 0.5 |
| Cross-track | 5 / 0 / 0 | 0 / 0 / 0 |

These values are now in `Constants.BLineConstants`. Hub aiming's independent controller, the path JSON, optimizer results, global speed/acceleration limits, completion tolerances, and handoff settings were not changed. Existing working-tree edits were preserved.

Zero cross-track P is deliberate: the GUI simulator has no extra straight-segment cross-track PID. The robot still commands closed-loop motion toward the active translation target. It does not blindly drive a prerecorded velocity sequence. Omitting the extra straight-segment correction gave a closer XY curve in these idealized simulation tests, but may reduce disturbance rejection on hardware. Retest conservatively before running near a trench, barrier, or people.

## Final verification versus the original baseline

| Measurement | Original baseline | Selected constants, no overrides |
| --- | ---: | ---: |
| Aligned XY RMS difference | 7.00 cm | 6.56 cm |
| Aligned XY maximum difference | 21.13 cm | 19.60 cm |
| Aligned absolute heading RMS difference | 18.11 degrees | 17.80 degrees |
| Aligned absolute heading maximum difference | 63.33 degrees | 69.11 degrees |
| Auto duration | 9.80 s | 9.82 s |
| Robot travel distance | 18.673 m | 18.448 m |
| GUI duration / distance | 6.78 s / 17.632 m | unchanged |

The final run has about 6% lower XY RMS difference and 7% lower peak XY difference than the original comparison. Heading RMS is essentially unchanged and peak heading error is worse. Timing is essentially unchanged. This is a modest geometric tradeoff, not proof of improved overall robot performance or a fix for the underlying model mismatch.

The selected gain set was simulated four times: its initial candidate, two explicit-gain repeats, and the final run using actual constants without overrides. XY RMS differences were 6.04, 6.38, 6.52, and 6.56 cm. All completed and passed the post-auto stopping check. Desktop timing variation is significant enough that the small average improvements should not be overstated.

## Trial process

23 new robot simulations were run: 18 initial/refinement trials (including a baseline repeat), two constant checks for an intermediate candidate, two repeats of the chosen zero-cross-track candidate, and one final no-override check after writing the chosen constants. 15 runs completed and passed the stopping check; eight were rejected for excessive coast or failure to finish within 30 seconds. Higher translation/rotation gains were not automatically better. Some aggressive candidates drifted roughly 25–28 cm after the command finished; those were not selected.

`all-trials.csv` and `all-trials.json` retain scores, diagnostics, and successful-run statistics. Raw logs and console output for the earlier candidates are in `trial-evidence.tar.gz`; final selected data are directly accessible in `selected/`.

The runner's exploratory score combines normalized XY RMS, heading RMS, and duration mismatch. Its single lowest combined-score candidate was rotation 4/0/0.5 with cross-track 3/0/0. It was checked twice against actual constants; XY RMS was 6.79 and 6.56 cm. The final zero-cross-track choice prioritizes the requested curve's spatial resemblance and peak XY deviation, rather than claiming the lowest combined score or a global optimum. Differences among these nearby candidates are small and noisy. Intermediate files named `final-constants-run1/2` refer to that cross-track-3 candidate, not the final selected settings.

## Why a residual mismatch remains

The preview was kept fixed: BLine-Web revision `e7bad71d071e2a485f0170bf2ced32e3c370ad45`, direct upstream simulator execution at 20 ms, original working-tree path/config hashes in `preview-metadata.json`. No optimizer was rerun and no GUI geometry was altered to make the results look better. The team's installed GUI version remains unverified.

The GUI uses square-root braking expressions for translation and rotation, then integrates an idealized limited chassis velocity. BLine-Lib uses PID outputs, its own segment/rotation selection, acceleration limiting, and the actual WPILib/YAGSL simulated swerve response. These are different controllers and dynamical models. The saved path also mixes profiled and unprofiled rotations, particularly on the second traversal. PID gains cannot make all resulting target transitions and movement timing identical.

An example from the baseline diagnostic: on the second return near X=7.60, Y=5.75, BLine is already commanding about -0.6 degrees while the XY-matched GUI pose is still about -86.5 degrees. The robot's measured heading there is about -25.6 degrees. That mismatch is not merely failure to follow the same commanded heading: the models differ in their progress and angular response. New diagnostic CSV columns expose commanded heading, requested/measured omega, and translation-element index for further inspection.

## Verification and reproducibility

The final auto completed and passed the existing stopping diagnostic: post-auto displacement 1.09 cm, initial coast peak 0.119 m/s, final speed effectively zero. PASS is a stopping check, not a GUI-parity check. Readback from its original `.wpilog` confirmed rotation P/D 4.0/0.5, cross-track P/D 0/0, and translation P/D 5.0/0.5. No experiment-gain environment variables were supplied to that final run. Integral gains remain zero.

`selected/robot-trace.csv`, `aligned-comparison.csv`, `comparison-summary.json`, `effective-gains.json`, the original `.wpilog`, and the rendered overlay retain the final result. Comparison uses the existing order-preserving XY dynamic-time-warping method; heading is evaluated after alignment. This removes timing differences and is an optimistic geometric comparison, not proof of identical execution.

Simulation tuning overrides in `AutoCommands` are ignored on hardware and outside the headless diagnostic. Actual effective P/D gains are logged under `BLine/Tuning`. Tools and trial definitions are in `tools/bline-tuning-*.json`, `tune-bline.mjs`, `summarize-bline-tuning.mjs`, `ExtractBLineTrace.java`, and `compare-bline.mjs`.

For a fresh no-override check from the repository root:

```sh
node tools/tune-bline.mjs tools/bline-tuning-selected-verification.json build/fresh-selected-check artifacts/bline-comparison/phase-1-2026-09-13/gui-preview.csv
```

Use a fresh output folder. The selected constants are global BLine defaults, so other BLine autos also use them; those paths were not independently hardware-validated. The existing unregistered `Shoot` event was not fixed. Phoenix initialization/deprecation warnings and expected headless-exit warnings remain.

Compilation and all successful headless runs completed normally. `./gradlew test` succeeded but reported `NO-SOURCE`; this checkout has no automated unit tests. `git diff --check` passed. No deployment, commit, or push was performed.
