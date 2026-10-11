from pathlib import Path

import numpy as np
import pytest

from shotlab.physics import (
    _is_scoring_entry,
    controls_from_top_rpm,
    hub_entry_metrics,
    release_state,
    simulate_shot,
)
from shotlab.robot_code import (
    RobotCodeParseError,
    _evaluate_java_numeric_expression,
    _parse_constant_class,
    app_models,
    default_robot_constants_path,
    hub_target,
    load_robot_shot_config,
)


def test_loads_current_robot_shot_configuration():
    config = load_robot_shot_config()

    assert config.source_path == default_robot_constants_path()
    assert config.values["DRAG_COEFFICIENT"] == 0.47
    assert config.values["FLYWHEEL_DIAMETER_METERS"] == pytest.approx(0.1016)
    assert config.values["LAUNCH_ELEVATION_DEGREES"] == pytest.approx(48.36)
    assert config.values["RELEASE_HEIGHT_METERS"] == pytest.approx(27.875 * 0.0254)
    assert config.values["HUB_BALL_CENTER_HEIGHT_METERS"] == pytest.approx(1.9038)
    assert config.values["FLYWHEEL_IDLE_RPM"] == 500.0
    assert config.values["MAX_RPM"] == 3800.0
    assert config.empirical_parameters["velocity_transfer"] == pytest.approx(1.0)
    assert config.empirical_parameters["hood_offset_deg"] == pytest.approx(0.0)
    assert config.app_state_values["model_rim_margin_in"] == pytest.approx(1.0)
    assert len(config.source_hash) == 64


def test_fixed_hood_robot_maps_onto_app_wheel_and_hood_terms():
    values = load_robot_shot_config().values

    assert values["TOP_WHEEL_DIAMETER_METERS"] == values["FLYWHEEL_DIAMETER_METERS"]
    assert values["BOTTOM_TO_TOP_RPM_RATIO"] == 0.0
    assert values["FIXED_HOOD_COMMAND_DEGREES"] == pytest.approx(90.0 - 48.36)
    assert values["HOOD_MIN_ANGLE"] < values["FIXED_HOOD_COMMAND_DEGREES"] < values["HOOD_MAX_ANGLE"]


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
    config = load_robot_shot_config()
    values = config.values
    ball, environment, shooter = app_models(config)
    rpm_policy = config.runtime_policy["rpm"]
    hood_command_deg = values["FIXED_HOOD_COMMAND_DEGREES"]

    for distance_m in np.linspace(
        values["MIN_DISTANCE_METERS"],
        values["MAX_DISTANCE_METERS"],
        101,
    ):
        top_rpm = float(np.clip(np.polyval(rpm_policy, distance_m), shooter.min_rpm, shooter.max_rpm))
        controls = controls_from_top_rpm(top_rpm, hood_command_deg, shooter)
        target = hub_target(config, distance_m)
        trajectory = simulate_shot(controls, ball, shooter, environment, target, dt_s=0.004)
        metrics = hub_entry_metrics(trajectory, target, ball)

        assert _is_scoring_entry(metrics, target), f"Java runtime policy missed at {distance_m:.4f} m"


def test_fixed_hood_ball_leaves_with_backspin():
    config = load_robot_shot_config()
    ball, _, shooter = app_models(config)
    controls = controls_from_top_rpm(3000.0, config.values["FIXED_HOOD_COMMAND_DEGREES"], shooter)

    speed, _, spin = release_state(controls, ball, shooter)

    assert spin > 0.0
    assert spin == pytest.approx(speed / ball.radius_m)
