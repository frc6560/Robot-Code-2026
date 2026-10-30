# Manual fuel launch calibration

Fresh, standard-library-only conversion of hand-measured slow-motion data. Does not read video or
use the old calibrator. See [the complete robot and measurement procedure](../../docs/fuel-shooting.md).

```sh
python3 tools/fuel-calibration/calibrate.py tools/fuel-calibration/measurements.csv
```

Output is `src/main/deploy/fuel-launch.csv`. No measured data is provided until you collect it.
