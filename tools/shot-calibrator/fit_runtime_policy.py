"""Regenerate the robot's runtime RPM policy from the shot model in Constants.java.

The robot can't search for the scoring RPM band every loop, so it evaluates a quadratic in
distance and re-simulates that one command. This fits the quadratic through the middle of each
distance's scoring band, using the same model the app uses, and prints the three Java lines to
paste into ShotModelConstants. Run it after copying a new calibration fit into Constants.java:

    .venv/bin/python fit_runtime_policy.py
"""

from __future__ import annotations

import numpy as np

from shotlab.flywheel import scoring_rpm_band
from shotlab.robot_code import app_models, hub_target, load_robot_shot_config

SAMPLES = 64


def main() -> None:
    config = load_robot_shot_config()
    values = config.values
    ball, environment, shooter = app_models(config)
    hood = values["FIXED_HOOD_COMMAND_DEGREES"]
    rpm_bounds = (shooter.min_rpm, shooter.max_rpm)

    distances = np.linspace(values["MIN_DISTANCE_METERS"], values["MAX_DISTANCE_METERS"], SAMPLES)
    lows, highs = [], []
    for distance in distances:
        band = scoring_rpm_band(
            hood, shooter, ball, environment, hub_target(config, distance), rpm_bounds, refine_steps=18
        )
        if band is None:
            raise SystemExit(
                f"No scoring RPM at {distance:.3f} m. Raise MIN_DISTANCE_METERS or lower "
                "MAX_DISTANCE_METERS until every distance in range can score."
            )
        lows.append(band[0])
        highs.append(band[1])

    lows, highs = np.array(lows), np.array(highs)
    centers = 0.5 * (lows + highs)
    coefficients = np.polyfit(distances, centers, 2)
    fitted = np.polyval(coefficients, distances)
    margin = np.minimum(fitted - lows, highs - fitted)
    worst = int(np.argmin(margin))

    print(f"Band width {np.min(highs - lows):.0f}-{np.max(highs - lows):.0f} RPM across range")
    print(f"Fit stays >= {margin[worst]:.1f} RPM inside the band (tightest at {distances[worst]:.3f} m)")
    if margin[worst] <= 0.0:
        print("WARNING: the quadratic leaves the scoring band somewhere; narrow the distance range.")
    a, b, c = (float(value) for value in coefficients)
    print()
    print(f"    public static final double RPM_POLICY_DISTANCE_SQUARED = {a!r};")
    print(f"    public static final double RPM_POLICY_DISTANCE = {b!r};")
    print(f"    public static final double RPM_POLICY_CONSTANT = {c!r};")


if __name__ == "__main__":
    main()
