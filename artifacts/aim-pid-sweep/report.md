# Aim-at-hub PID sweep

Nine single-run configurations; six moving cases per configuration, 10 seconds each. Shooter P=2, I=0, D=0 throughout. Aim I=0 throughout. Feed fraction is the unweighted mean of per-case moving feed fractions over 2–8.5 s with measured speed above 0.15 m/s. Cases: blue/red curved arcs, blue/red figure eights, near/far weaving. Same production controller and simplified drivetrain/ball models as the prior diagnostic.

| Aim P | Aim D | Feed enabled | Moving samples passing | Mean of per-case 95th-percentile heading errors |
|---:|---:|---:|---:|---:|
| 1.2 | 0.2 | 31.8% | 65/66 | 8.72° |
| 2.0 | 0.2 | 32.4% | 71/71 | 7.94° |
| 3.0 | 0.2 | 38.1% | 83/84 | 7.58° |
| 4.0 | 0.2 | 41.6% | 87/87 | 7.31° |
| 6.0 | 0.2 | 48.3% | 104/104 | 7.23° |
| 3.0 | 0 | 37.5% | 80/80 | 7.54° |
| 4.0 | 0 | 43.8% | 92/92 | 7.35° |
| 6.0 | 0 | 52.6% | 109/109 | 7.15° |
| 4.0 | 0.5 | 42.9% | 88/88 | 7.45° |

Best tested: P=6, I=0, D=0. This is a single-pass simulation comparison, not a global optimum or hardware validation. Two settings, including the baseline, produced one marginal predicted miss, demonstrating that earlier perfect sample counts were not a guarantee.

Higher P reduces lag, but the controller switches to tracking feedforward alone once inside the existing 7.5-degree alignment tolerance. Heading error can persist within that band; the trajectory gate and tracking-rate gate then pause feeding. PID tuning alone does not remove this behavior.

Runtime aim gains were restored to P=1.2, I=0, D=0.2 after the sweep. Shooter remains P=2, I=0, D=0. Every configuration directory includes its console log, configuration, trace, landing samples and summary.