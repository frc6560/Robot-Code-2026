from pathlib import Path

import numpy as np
import pytest

from shotlab.models import BallSpec, Environment, ShooterModel, ShotControls, Target
from shotlab.physics import _is_scoring_entry, hub_entry_metrics, simulate_shot
from shotlab.robot_code import (
    RobotCodeParseError,
    _evaluate_java_numeric_expression,
    _parse_constant_class,
    default_robot_constants_path,
    load_robot_shot_config,
)


def test_loads_current_robot_shot_configuration():
    config = load_robot_shot_config()

    assert config.source_path == default_robot_constants_path()
    assert config.values["DRAG_COEFFICIENT"] == 0.47
    assert config.values["TOP_WHEEL_DIAMETER_METERS"] == pytest.approx(0.0635)
    assert config.values["HUB_BALL_CENTER_HEIGHT_METERS"] == pytest.approx(1.9038)
    assert config.values["FLYWHEEL_IDLE_RPM"] == 500.0
    assert config.values["MAX_RPM"] == 5000.0
    assert config.empirical_parameters["velocity_transfer"] == 0.72
    assert config.app_state_values["model_rim_margin_in"] == pytest.approx(1.0)
    assert len(config.source_hash) == 64


def test_numeric_parser_supports_units_references_and_arithmetic():
    source = """
    public static final class ExampleConstants {
      public static final double WIDTH = Units.inchesToMeters(41.7);
      public static final double RADIUS = WIDTH / 2.0;
      public static final double OFFSET = -RADIUS + 0.5;
    }
    """

    values = _parse_constant_class(source, "ExampleConstants")

    assert values["WIDTH"] == pytest.approx(1.05918)
    assert values["RADIUS"] == pytest.approx(0.52959)
    assert values["OFFSET"] == pytest.approx(-0.02959)
    assert _evaluate_java_numeric_expression("Units.feetToMeters(15.0)", {}) == pytest.approx(4.572)


def test_missing_robot_classes_raise_clear_error(tmp_path: Path):
    path = tmp_path / "Constants.java"
    path.write_text("public final class Constants {}", encoding="utf-8")

    with pytest.raises(RobotCodeParseError, match="ShotModelConstants"):
        load_robot_shot_config(path)


def test_java_runtime_policy_scores_in_python_equation_across_range():
    values = load_robot_shot_config().values
    ball = BallSpec(
        mass_kg=values["BALL_MASS_KG"],
        diameter_m=values["BALL_DIAMETER_METERS"],
        drag_coefficient=values["DRAG_COEFFICIENT"],
    )
    environment = Environment(
        air_density_kg_m3=values["AIR_DENSITY_KG_PER_CUBIC_METER"],
        gravity_m_s2=values["GRAVITY_METERS_PER_SECOND_SQUARED"],
        wind_x_m_s=values["WIND_X_METERS_PER_SECOND"],
    )
    shooter = ShooterModel(
        top_wheel_diameter_m=values["TOP_WHEEL_DIAMETER_METERS"],
        bottom_wheel_diameter_m=values["BOTTOM_WHEEL_DIAMETER_METERS"],
        min_rpm=values["FLYWHEEL_IDLE_RPM"],
        max_rpm=values["MAX_RPM"],
        min_hood_deg=values["HOOD_MIN_ANGLE"],
        max_hood_deg=values["HOOD_MAX_ANGLE"],
        bottom_to_top_rpm_ratio=values["BOTTOM_TO_TOP_RPM_RATIO"],
        release_height_m=values["RELEASE_HEIGHT_METERS"],
        velocity_transfer=values["VELOCITY_TRANSFER"],
        spin_transfer=values["SPIN_TRANSFER"],
        hood_offset_deg=values["HOOD_OFFSET_DEGREES"],
        drag_scale=values["DRAG_SCALE"],
        lift_slope=values["LIFT_SLOPE"],
        max_lift_coefficient=values["MAX_LIFT_COEFFICIENT"],
        spin_decay_per_s=values["SPIN_DECAY_PER_SECOND"],
    )
    rpm_policy = (
        values["RPM_POLICY_DISTANCE_SQUARED"],
        values["RPM_POLICY_DISTANCE"],
        values["RPM_POLICY_CONSTANT"],
    )
    hood_policy = (
        values["HOOD_POLICY_DISTANCE_SQUARED"],
        values["HOOD_POLICY_DISTANCE"],
        values["HOOD_POLICY_CONSTANT"],
    )

    for distance_m in np.linspace(
        values["MIN_DISTANCE_METERS"],
        values["MAX_DISTANCE_METERS"],
        101,
    ):
        top_rpm = float(np.clip(np.polyval(rpm_policy, distance_m), shooter.min_rpm, shooter.max_rpm))
        hood_command_deg = float(
            np.clip(np.polyval(hood_policy, distance_m), shooter.min_hood_deg, shooter.max_hood_deg)
        )
        controls = ShotControls(
            top_rpm=top_rpm,
            bottom_rpm=top_rpm * shooter.bottom_to_top_rpm_ratio,
            hood_angle_deg=hood_command_deg,
        )
        target = Target(
            distance_m=float(distance_m),
            center_height_m=values["HUB_BALL_CENTER_HEIGHT_METERS"],
            opening_height_m=values["BALL_DIAMETER_METERS"],
            opening_span_m=values["HUB_OPENING_SPAN_METERS"],
            min_entry_angle_deg=values["MIN_ENTRY_ANGLE_DEGREES"],
            rim_margin_m=values["HUB_RIM_MARGIN_METERS"],
        )
        trajectory = simulate_shot(controls, ball, shooter, environment, target, dt_s=0.004)
        metrics = hub_entry_metrics(trajectory, target, ball)

        assert _is_scoring_entry(metrics, target), f"Java runtime policy missed at {distance_m:.4f} m"
