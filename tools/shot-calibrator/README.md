# Charging Champions Shot Model Lab

Streamlit MVP for optimizing a robot shot, tracking a measured side-view trajectory from MOV/MP4 video, and fitting the empirical projectile-model parameters.

## Workflow

1. **Model**: load the deployed shot constants and runtime policy from `src/main/java/frc/robot/Constants.java`, then review or adjust the target, ball, shooter geometry, limits, environment, and current empirical coefficients. The app samples the bounded RPM/hood-command space, displays the valid trajectory envelope, and selects an interior command that remains farthest from sampled scoring failures. Every valid sample clears the near rim and descends through the usable scoring opening at the configured minimum entry angle.
2. **Measure**: upload a fixed-camera side-view video, enter a release-point hint and pixel scale, and run physics-validated yellow-ball tracking.
3. **Calibrate**: first fit identifiable physics parameters so the calibrated model reproduces the measured shot at the recorded controls. Then solve a second bounded RPM/hood problem so the calibrated model's next-shot prediction follows the Stage 01 reference optimum. Optionally request a constrained LLM review of the fitted parameters. Each numerical and LLM candidate is recorded with its controls, model values, before/after RMSE, Java revision, and source video.

The deterministic bounded least-squares fit is the numerical baseline. Its orange graph trace should approach the black recording because both use the same controls. The separate corrected-next-shot trace changes RPM and the rear-referenced hood command and should approach the teal reference optimum. The LLM receives only numeric summaries and returns a candidate inside explicit bounds. The app evaluates that candidate and disables application when it does not improve trajectory RMSE.

## Loaded robot profile

Stage 01 safely parses numeric constants from `src/main/java/frc/robot/Constants.java`; it does not execute Java or use text substitution. The synchronization panel shows the source hash, loaded values, and the robot's quadratic RPM and rear-referenced hood policies. **Reload from robot code** rereads the file, updates the model controls, records a baseline for a new Java revision, and invalidates stale fits and shot maps.

Values that are not part of `Constants.java` remain documented in `shotlab/robot_profile.py`. Wheel tread, gearing, compression, contact path, robot footprint, and drive-wheel diameter come from the supplied robot information form and deploy configuration. The form describes independently powered wheels, while the current code configures one opposed follower; the app follows the current code-facing mechanism RPM and records that mismatch in the profile table.

## Calibration history

Stage 03 plots before/after trajectory RMSE and all empirical parameters across recorded attempts. A Java baseline appears once per unique `Constants.java` revision, while every completed numerical fit or LLM candidate adds a new attempt. The attempt table includes target range, recorded RPM, rear-referenced hood command, fitted parameter names, and fit quality, and it can be downloaded as CSV.

History persists locally in `.data/calibration_history.json` and is intentionally ignored by Git. Resetting history preserves a fresh baseline for the currently loaded Java revision.

## Hood-angle convention

The robot code's hood value is a command measured from the back/vertical reference. It is not the projectile elevation above horizontal. The physics model converts it at the release boundary:

```text
launch_elevation_deg = 90 - hood_command_from_back_deg + launch_angle_offset_deg
```

With zero fitted offset, the code limits of 25.1-45.0 degrees correspond to launch elevations of 64.9-45.0 degrees. A smaller robot hood command therefore creates a steeper launch. Optimization and calibration continue to report robot command values; only trajectory simulation uses the converted launch elevation.

## Robust shot map

Stage 01 samples 29 rear-referenced hood commands by 41 mechanism RPM commands. Each of the 1,189 candidates uses the full drag, Magnus, spin-decay, robot-velocity, and HUB-clearance model. The dark trajectory chart shows representative lower-bound, interior, and upper-bound scoring paths. Its inset plots the valid launch-angle versus launch-speed band.

The selected command maximizes normalized distance from failure in command space, where one robustness unit represents 0.5 degrees of hood-command error or 100 RPM of flywheel error. The flight-time, entry-angle, and mechanism-effort objective breaks ties between equally robust cells. Reported RPM and hood windows are contiguous sampled scoring ranges, not statistical confidence intervals.

The Stage 01 recommendation is the best point found on this finite grid, not a proof of a continuous global minimum. The robot cannot run the full 1,189-candidate search every control-loop cycle, so `PhysicsShotSolver.solveRuntime()` evaluates a smooth quadratic RPM/hood policy generated from 89 robust maps and then re-simulates that command with the full equation. Robot tests sweep 1,001 ranges across the configured envelope, require every runtime command to satisfy all scoring constraints, and bound its difference from representative full-map solutions. A Python cross-language regression independently parses the deployed Java constants and verifies the same policy with the app model.

Both teleoperated and autonomous release are fail-closed. The feeder remains off unless localization is finite, the range is inside the configured envelope, the calculated command scores, the turret and hood are at target, the flywheel is at target, and the **measured** mechanism RPM and rear-referenced hood angle still score when re-evaluated by the full drag/Magnus equation. Changing any empirical physics constant requires regenerating the policy coefficients and passing both Java and Python suites before deployment.

Every trajectory graph draws the robot behind the release point and the 2026 REBUILT HUB at the entered center range. The HUB drawing separates the 41.7 in upper funnel entrance at 72 in from the 23.95 in lower throat at 56.44 in and draws the sloped GE-26329 funnel panels between them. The 47 in body footprint and upper-rim dimensions come from Game Manual section 5.4; the lower profile comes from FIRST's TE-26300 STEP model, whose build instructions specify the same funnel panels as the field. The scoring test derives the ball-center plane from the upper rim plus the current ball radius, shrinks the horizontal entrance by the ball radius and configurable safety margin, checks near-rim clearance, and enforces a configurable minimum entry angle. The lower throat guides a ball after entry and is not treated as a second no-contact clearance gate.

## Run

```bash
cd tools/shot-calibrator
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m streamlit run app.py
```

Set `OPENAI_API_KEY` to enable Stage 03 LLM review. `OPENAI_MODEL` is optional and defaults to `gpt-5.6-luna`.

## Camera setup

- Use a stationary near-side camera. It does not need to be perfectly perpendicular when HUB-reference calibration is used.
- Enter the known release-to-HUB-center distance, mark the robot release X, and mark the HUB base and 72 in upper funnel-rim center in Stage 02.
- The three reference points define an affine image-to-field transform that corrects horizontal scale, vertical scale, image tilt, and mild camera obliqueness.
- If the ball is first visible after launch, the tracker fits the accepted parabola backward to the release X and preserves that hidden time in the measured trajectory.
- Use short clips containing one shot. Trim the clip in the app when necessary.
- Direct pixels-per-meter mode remains available as a fallback, but it cannot correct camera obliqueness.

## Tracking pipeline

The detector uses the production defaults from the companion `ball-tracker` project: HSV hue 20-35, saturation 120+, value 100+, circularity 0.60, an 80-pixel association gate, 15-frame track memory, and motion assistance disabled. It maintains multiple provisional tracks with frame-aware prediction and one-to-one candidate association. Tracks are stopped at a bounce or reversal and robustly fitted to `x = vx*t` and `y = vy*t + 0.5*g*t^2`; paths with poor direction, acceleration, displacement, R², RMSE, or inlier retention are rejected before one shot is selected for calibration.

## Calibration limits

A single 2D path usually cannot identify RPM transfer, launch-angle offset, drag, spin transfer, Magnus lift, and spin decay simultaneously. Start with velocity transfer and hood offset. Add drag across shots at different speeds. Fit Magnus terms only when wheel-speed difference or measured ball spin changes across controlled tests.

## Tests

```bash
.venv/bin/python -m pytest -q
```
