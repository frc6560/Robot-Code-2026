# Automated BLine PID search — 2026-09-13

## Decision

Keep the existing gains. This bounded search found no repeatable improvement against the phase-1-canvas-draft GUI preview. It does not establish a global optimum, and preview matching is not hardware validation.

| Settings changed from current | Repeated runs | Mean weighted score (lower is better) | Change versus current |
| --- | ---: | ---: | ---: |
| None | 5 | 3.10845 | baseline |
| Rotation D: 0.5 → 0.7 | 3 | 3.15598 | 1.53% worse |
| Translation P: 5 → 5.5 | 3 | 3.17090 | 2.01% worse |

Current P/I/D: translation **5 / 0 / 0.5**, rotation **4 / 0 / 0.5**, cross-track **0 / 0 / 0**. No gains, deployed paths, motion constraints, or config values were changed by this optimization. Cross-track zero remains a preview-parity experiment from the earlier manual tune, not evidence of good real-world disturbance rejection. Do not treat these tests as clearance to drive near a wall.

## Method and evidence

20 initial evaluations using bounded coordinate polling and optional pattern extrapolation, then repeated finalist validation. Total: 28 main-path simulation executions, plus eight holdout executions (four collision-confounded originals and four relocated retests). Each execution used the actual WPILib robot autonomous command and YAGSL simulated drivetrain. Candidate P/D overrides are gated to headless simulation; hardware ignores them. Effective gains are extracted from the WPILOG to verify that each requested candidate actually ran.

The original campaign accidentally repeated the baseline as one finalist. Its five baseline runs are pooled explicitly; the independent review adds the missing repeats of the second distinct candidate. `adoption-decision.json` is the authoritative corrected decision, not the preliminary `validation-decision.json`.

Reference: upstream BLine-Web `e7bad71d071e2a485f0170bf2ced32e3c370ad45`, using its deserialize/config/simulate pipeline directly at 20 ms, without browser automation. Main-path reference: 340 samples, 6.78 s. Path SHA256: `824734f04901d113ed13fa466323e19b2f19a701ae9b7dd96a83ab94c0ce6be7`; config SHA256: `f31e2ec5d1216a48ffb1ff1415dd1ab13da1a1661fa9a1b72eb4ed9f3f6301db`. These deployed inputs were verified unchanged after the search.

Bounds, in coordinate order:

- Translation P: 3.5–8; D: 0.15–0.85.
- Rotation P: 2.8–8; D: 0–1.
- Cross-track P: 0–8; D: 0–0.15.
- All integral gains held at zero. Initial steps: 1, 0.15, 1, 0.2, 2, 0.05; halved after unsuccessful polls.

Predeclared objective:

`XY_RMS/0.07 + 0.5*XY_peak/0.21 + heading_RMS/18 + 0.5*heading_peak/65 + 0.25*abs(duration−GUI_duration)/3 + 0.05*chatter_count`.

Heading errors are degrees, XY errors meters, time seconds. Chatter counts requested angular-velocity sign reversals with both magnitudes above 0.2 rad/s and adjacent heading targets less than 2° apart during auto; this is only a proxy, not a vibration measurement. Timeout or failed post-command stopping rejects a candidate regardless of score. Adoption requires at least three successful repetitions, at least 3% lower mean score, and a score advantage larger than one combined standard error. This small-sample rule is a noise screen, not a formal confidence guarantee.

Geometry comparison uses order-preserving XY dynamic-time-warping alignment and heading at the matched GUI pose. It removes most timing differences and is approximate/optimistic; it is not proof of identical time-indexed trajectories. Timing is penalized separately. Current repeated XY RMS spans 5.74–6.54 cm; heading RMS spans 17.66–18.02°, with peak heading near 69°. Auto duration is about 9.8 s versus 6.78 s in the preview. The remaining heading/timing mismatch is substantial; this search did not resolve it.

## Holdout caveat: the simulator is not the 2026 field

YAGSL 2026.1.12's bundled MapleSim `SimulatedArena.getInstance()` defaults to `Arena2025Reefscape`. Its blue reef spans x≈3.66–5.32, y≈3.07–4.99. The first synthetic fixtures started at (4,4), inside that reef, and both baseline and candidate stalled at its boundary. Those failures cannot be attributed to PID. The fixtures/previews and original decision are preserved in `collision-confounded-holdouts/` and `collision-confounded-decision.json`.

Retests moved only the synthetic fixture geometry into the clear y=6–7 band and regenerated both GUI references. Both controllers completed and stopped on both retests; neither triggered the relative-regression guard. Straight-path XY RMS was 0.92 cm baseline versus 0.98 cm candidate. Corner-path XY RMS was 24.94 cm baseline versus 24.79 cm candidate, with heading RMS 11.45° versus 10.29°. Passing a relative guard does not mean close GUI parity: the corner mismatch is large for both controllers. These tests screen for gross regression, not physical trench safety. Final results and metrics are in `adoption-decision.json`. Arena behavior was not changed. Earlier assumptions that the YAGSL simulation lacks field collisions were incorrect; it includes collisions against an older season's field.

## Repeating the experiment

Run from the repository root with a JDK/WPILib simulation setup:

```sh
node tools/test-bline-pid-objective.mjs
node tools/optimize-bline-pid.mjs build/bline-pid-new-run artifacts/bline-comparison/phase-1-2026-09-13/gui-preview.csv
node tools/review-bline-pid.mjs build/bline-pid-new-run artifacts/bline-pid-optimization/2026-09-13
```

Use a fresh search directory; execution is sequential. Regenerate previews with `tools/bline-preview.ts` if paths/config/upstream revision change. `tools/tune-bline.mjs` and `tools/compare-bline.mjs` retain per-run console output, effective gains, robot trace, aligned comparison, and overlay. Full run evidence is archived alongside this report. The build check succeeds, but this repository has no Java test sources; the objective/noise-screen self-test is separate and passes. Nothing was deployed, committed, or pushed.
