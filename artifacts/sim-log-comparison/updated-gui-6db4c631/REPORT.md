# Updated path: WPILib simulation versus GUI preview

One full BLine Editor Path autonomous execution on Blue alliance, with the current updated `phase-1-canvas-draft.json`. Selected the editor command deliberately: the separate Zigzag finish-plane wrapper can finish immediately for a path whose final waypoint equals its initial waypoint. This tests the full path follower, not that wrapper.

Actual saved WPILOG poses were extracted, retaining one contiguous active-command interval plus the first post-completion pose. The comparison contains 440 poses; the raw CSV contains 590 including the post-auto stopping-monitor interval. The graph excludes that three-second monitor interval. Red markers are waypoints, blue is the exact regenerated BLine-Web preview, orange is the simulated robot pose trace.

| Approximate order-aligned deviation | Simulation | Recorded robot history |
| --- | ---: | ---: |
| RMS | 6.69 cm | 9.84 cm |
| 95th percentile | 13.58 cm | 21.11 cm |
| Maximum sampled deviation | 24.84 cm | 29.46 cm |

Simulation pose-trace duration: **8.90 s**, versus GUI prediction **6.06 s**. First post-completion position is **2.97 cm** from the final waypoint. The simulated polyline length is **18.97 m**, versus GUI **17.93 m**. The largest geometric deviation is near **(3.973 m, 7.404 m)** on the intermediate return/reversal.

The headless check printed PASS: maximum displacement after auto completion **0.0186 m**, initial coast peak **0.1327 m/s**, final speed **0.0000 m/s** after three seconds. This is a stopping check, not a trajectory-accuracy certificate.

Current gain P/I/D values: translation **5 / 0 / 0.5**, rotation **3 / 0 / 0.3**, cross-track **5 / 0 / 0**. The earlier manually tuned gains remain stashed and were not restored or used. The simulation uses the current robot code with its actual path constraints; no PID or constraint tuning was performed.

The metric and graph layout are the same as the recorded-history comparison: robot and GUI geometry resampled at 2 cm traveled-distance spacing, monotone XY DTW correspondence, continuous projection onto locally matched GUI preview segments, RMS/P95/max in centimeters. No pose fit, transform, or endpoint snapping. It is an approximate geometric comparison that removes timing differences, not a time-indexed controller error. The real export has sparse estimated x/y history; this simulation has denser timestamped estimated poses. Neither is independent real-world ground truth.

Important warnings: the path's `Shoot` event is unregistered in current robot code, so no shooting action fired. No event registration was changed. The simulation also emitted Phoenix simulated-device status warnings and some loop overruns; it completed despite those. YAGSL's bundled MapleSim defaults to a 2025 Reefscape arena, not a validated 2026 arena, so this is not proof of physical trench safety.

Reproduce from repository root with fresh output log directory:

```sh
BLINE_HEADLESS_DIAGNOSTIC=1 BLINE_HEADLESS_AUTO=editor BLINE_COMPARE_LOG_DIR=/absolute/fresh/log-directory ./gradlew simulateJava -Pheadless
```

Only simulation-gated pose logging, editor command selection, and the shared offline comparison utility were added. The user's path edits were preserved. No hardware behavior/gains changed, nothing deployed, committed, or pushed.

Provenance: path SHA256 `6db4c6319e7eba25192ea2c288d4d59eb48523cfad93657232e24d13d9090df0`; config SHA256 `f31e2ec5d1216a48ffb1ff1415dd1ab13da1a1661fa9a1b72eb4ed9f3f6301db`; GUI code revision `e7bad71d071e2a485f0170bf2ced32e3c370ad45`. Saved WPILOG: `logs/akit_26-09-13_16-41-36.wpilog`.
