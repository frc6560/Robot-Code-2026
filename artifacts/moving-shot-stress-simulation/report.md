# Shoot-on-the-move closed-loop simulation

These are simulated outcomes, not measured robot hit rates. The production command and controller code ran at 20 ms against YAGSL/MapleSim drivetrain physics and WPILib FlywheelSim. No alignment or flywheel speed was mocked. Indexer readiness was idealized.

## Controller values and setup

```text
Aim PID: 1.200, 0.000, 0.200; angular limits: 12.500 rad/s, 24.000 rad/s^2
Aim tolerance: 7.500 deg; tracking-rate tolerance: 5.000 deg/s
Shooter PID: 2.000, 0.000, 0.000; FF kS=0.150000 kV=0.027932 kA=0.003836
Launch coefficient: 1.050; battery: 12.0 V; robot loop: 20 ms; motor loop: 1 ms; MapleSim substeps: 5
Indexer idealized ready; flat field; no vision noise, wheel impacts, supply sag, release delay or ball loading. Motor P/D stand-in omits firmware filtering and CAN latency.
```

Each case lasts 10 seconds, with translation from 1.5 to 8.5 seconds and a 0.3-second initial ramp. Curved arcs reverse smoothly around the hub; figure-eight paths vary both X and Y velocity continuously. Near and far weaving cases start 2.7 m and 4.0 m from the hub; all other cases start 3.5 m away with a 35-degree heading error. Both alliances are exercised. The extra stationary control disables D only in the simulator. All production gains are read from the current robot configuration.

YAGSL uses simplified module motor loops rather than reproducing Talon firmware. The production hub-aim PID, angular profile, tracking feedforward, shooter P/D gains, voltage feedforward, current-limited flywheel plant with a 1 ms unfiltered software motor-loop stand-in, acceleration limiter, and feed dwell all run directly. The simulated field is flat with no obstacles. Battery is fixed at 12 V; ball impacts, inventory, flywheel droop from shots, vision noise, and release-delay uncertainty are absent.

## Results

In the final second of the stationary test, the requested speed averaged 2575 RPM. The current P/D model averaged 2575 RPM; the previous d = 0 run averaged 2575 RPM. The feed gate requires measured speed within 60 RPM of the goal. The simulation does not reproduce Talon firmware filtering or CAN latency.

![Effect of D in the motor-loop model](flywheel-pid-comparison.png)

| Scenario (commanded speed) | Actual moving speed, median | First feed | Feed open while moving | All sampled scores / releases | Moving scores / releases | Largest miss from center |
|---|---:|---:|---:|---:|---:|---:|
| Stationary | 0 m/s | 1.02 s | — | 44 / 44 | — | 12.5 cm |
| Blue alliance, curved arc | 0.84 m/s | 1.02 s | 26% | 15 / 15 | 8 / 8 | 34.2 cm |
| Red alliance, curved arc | 0.84 m/s | 1.00 s | 27% | 15 / 15 | 8 / 8 | 36.6 cm |
| Blue alliance, figure eight | 0.62 m/s | 1.00 s | 24% | 16 / 16 | 9 / 9 | 33.8 cm |
| Red alliance, figure eight | 0.62 m/s | 1.02 s | 24% | 16 / 16 | 9 / 9 | 34.7 cm |
| Near weaving, start 2.7 m | 0.52 m/s | 0.94 s | 59% | 33 / 33 | 19 / 19 | 34.0 cm |
| Far weaving, start 4.0 m | 0.90 m/s | 1.06 s | 29% | 18 / 18 | 11 / 11 | 42.1 cm |
| Control: stationary with D disabled | 0 m/s | 1.02 s | — | 44 / 44 | — | 0.8 cm |

Feed-open fraction uses only the 2–8.5 second interval and samples with actual speed above 0.15 m/s. It includes pauses for aiming, dwell, RPM, or an unreachable trajectory; it is not a ball throughput or success percentage. If present, first feed can occur during the initial stationary spin-up.

Releases are sampled at most once every 0.2 seconds while the feed gate is open. Every scenario samples releases before, during, and after translation. Moving releases are classified by actual speed above 0.15 m/s; all other sampled releases are stationary. The independent RK4 model integrates 3D gravity, quadratic drag and backspin lift after adding ground velocity at release. It shares the provisional aero/launch constants, so it can detect algorithm inconsistencies but cannot validate those constants against real balls. A separate stationary cross-check verifies the independent integrator against the existing model. The usable center-distance limit is 42.9 cm, with near-rim clearance and entry-angle checks also required.

![Physical paths and feed state](field-paths.png)

![PID tracking and feed state](tracking.png)

![Sampled landing error](landing-errors.png)

## Findings

This second motion suite uses the production P=2, I=0, D=0 flywheel gains and the production hub-aim controller. An initial run found four marginal moving misses with the translated stationary trajectory check. The release gate now integrates 3D drag/lift with inherited velocity, measured heading and RPM. Cache-based aim/RPM selection and all stationary behavior are unchanged. The initial traces are saved in `before-3d-release-check/`.

The configured P=2, I=0, D=0 run produced 64/64 passing moving samples and 157/157 passing samples overall, excluding the duplicate stationary control. Releases after stopping can miss because the preserved stationary 7.5-degree heading tolerance allows too much angular error at longer ranges. Feed continuity still drops during reversals and faster sideways motion. These results are simulated checks, not a physical success rate.

## Timing

CSV traces contain wall time of command execution on this desktop, including trajectory selection, readiness, aiming, telemetry and JVM scheduling/JIT effects. These are not roboRIO timings or CPU-utilization percentages. Warmed later scenarios are more representative than the first stationary scenario; do not compare those directly as an overhead ratio.

## Reproduce

```sh
./gradlew movingShotStressSimulation
uv run --with matplotlib tools/analyze-moving-shot-simulation.py --source build/reports/moving-shot-stress-simulation --output artifacts/moving-shot-stress-simulation --comparison artifacts/moving-shot-simulation
```
