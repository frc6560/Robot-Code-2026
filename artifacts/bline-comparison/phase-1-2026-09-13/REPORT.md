# phase-1-canvas-draft: GUI preview versus robot simulation

The curves are close, but they are not identical. Their timing and intermediate heading behavior differ substantially. This compares the current saved path without changing its optimizer constraints, PID gains, or geometry.

## Results

| Measurement | GUI preview | Logged robot simulation |
| --- | ---: | ---: |
| Duration | 6.78 s | 9.80 s |
| Travel distance | 17.632 m | 18.673 m |
| Compared samples | 340 | 489 |

After order-preserving XY alignment to remove timing differences:

- Position difference: mean 5.23 cm; RMS 7.00 cm; 95th percentile 16.63 cm; maximum 21.13 cm.
- Absolute heading difference: mean 11.40 degrees; RMS 18.11 degrees; 95th percentile 42.12 degrees; maximum 63.33 degrees.
- End-of-command position difference: 2.85 cm; heading difference: -1.83 degrees.
- At equal elapsed times, position RMS difference over the shared 6.78-second interval is 2.19 m. This largely reflects different progress/timing, not geometric tracking error.

The robot travels about 1.04 m farther in total and takes about 3.02 s longer (44.6%). The overlay shows different corner rounding and overshoot. A close final position does not imply an identical intermediate trajectory or heading.

## Data provenance

- Path and configuration: snapshots of the actual working-tree `phase-1-canvas-draft.json` and `config.json`, saved beside this report. Existing user edits were preserved.
- GUI source: BLine-Web revision `e7bad71d071e2a485f0170bf2ced32e3c370ad45`, package version `0.1.0-alpha.12`.
- Preview generation: upstream `deserializePath`, `createProjectConfig`, `projectConfigDefaultLookup`, and `simulatePath(..., {dt_s: 0.02})` were invoked directly. No optimizer was rerun. No polyline through waypoints was substituted for the preview.
- The actual GUI renderer draws its trail from `poses_by_time`; `gui-preview.csv` contains those exact simulated positions and headings. The overlay uses a different presentation, but the same coordinates, with equal XY scale.
- Runtime: project BLine-Lib v0.9.1, actual robot code through WPILib/YAGSL desktop simulation, Blue alliance, editor-path command rather than Zigzag's finish-plane wrapper.
- Capture: AdvantageKit WPILOGWriter saved `akit_26-09-13_14-56-13.wpilog`. The extractor reads binary Pose2d records from `Swerve/Pose`; it does not synthesize or reconstruct robot motion from path targets.
- Headless auto completed successfully. Post-completion displacement was 1.33 cm and final speed was effectively zero. The diagnostic's PASS checks stopping after auto, not agreement with the GUI.
- Compilation/simulation and `./gradlew test` completed successfully. Existing Phoenix constructor deprecation warnings remain.

## Files

- `gui-preview.csv`: time in seconds, field X/Y in metres, heading in radians.
- `robot-trace.csv`: saved robot timestamps, estimated X/Y/heading, and active-auto flag. Includes startup and three seconds after completion.
- `aligned-comparison.csv`: 489 active-run samples, including the first completion sample; elapsed robot time starts at the first recorded active pose. Each row has a matched GUI pose and position/heading difference.
- `comparison-summary.json`: full-precision statistics and method notes.
- `preview-metadata.json`: source revision and SHA-256 hashes of input files.
- `trajectory-overlay.png` / `.svg`: blue GUI curve, dashed orange robot curve.
- Original `.wpilog`, path JSON, and config JSON are retained for independent inspection.

## Method and limitations

The alignment uses dynamic time warping with XY-distance cost and preserves traversal order, rather than freely choosing the nearest point anywhere on a self-crossing/repeated route. One lowest-distance GUI sample on that ordered alignment is selected for each robot sample. Heading is compared afterward and does not influence the alignment. This is an optimistic geometric comparison; timing differences are intentionally removed. It does not prove identical segment handoffs or events.

The GUI samples are at 20 ms; the robot uses logged timestamps and real desktop loop timing. Some loop overruns occurred. The GUI's time-zero pose is already its first integrated sample, which is a small sampling convention difference retained rather than manually corrected.

The GUI preview uses an idealized controller, not the robot's PID controllers or swerve-module response. Matching PID gains cannot make those two models intrinsically identical. This run cannot establish real-hardware PID quality, wheel slip, vision accuracy, or safe trench clearance.

The team's installed desktop GUI version has not been verified. These samples exactly reproduce the inspected upstream revision for these inputs, not necessarily an older installed application.

The path contains `Shoot`, but runtime reports `Unregistered event trigger key: Shoot`. This run therefore tests driving, not a successful shooting event. No fix was made because this task requested comparison rather than changes to autonomous behavior.

## Repeat the comparison

From the repository root, use a checkout of BLine-Web at the revision above:

```sh
npx --yes tsx@4.23.13 tools/bline-preview.ts /path/to/BLine-Web src/main/deploy/autos/paths/phase-1-canvas-draft.json src/main/deploy/autos/config.json build/bline-comparison/phase-1
BLINE_HEADLESS_DIAGNOSTIC=1 BLINE_HEADLESS_AUTO=editor BLINE_COMPARE_LOG_DIR=/absolute/output/logs ./gradlew simulateJava -Pheadless
java -cp build/libs/Robot-Code-2026.jar tools/ExtractBLineTrace.java /absolute/output/logs/actual-log-name.wpilog build/bline-comparison/phase-1/robot-trace.csv
node tools/compare-bline.mjs build/bline-comparison/phase-1
```

Choose a fresh output/log folder for each run and extract that run's actual filename. If npm's existing cache has permission problems, set `npm_config_cache` to a new writable task-specific directory rather than altering the global cache.

## Code changes made for this task

Only simulation instrumentation changed: an environment-selected editor auto, optional simulation file logging, and an active-auto log marker in the headless diagnostic. Ordinary roboRIO logging, chooser defaults, drivetrain constants, and path files were not changed. Three reusable tools were added under `tools/`. Nothing was committed or pushed.
