import numpy as np

from shotlab.models import BallSpec, Environment, OptimizationWeights, ShotControls, ShooterModel, Target, Trajectory
from shotlab.physics import (
    build_shot_map,
    hub_entry_metrics,
    launch_elevation_deg,
    optimize_shot,
    release_state,
    shot_objective,
    simulate_shot,
)
from shotlab.robot_profile import (
    BALL_DIAMETER_M,
    DRIVE_MODULE_LONGITUDINAL_OFFSET_M,
    HUB_BALL_CENTER_HEIGHT_M,
    HUB_BODY_DEPTH_M,
    HUB_FUNNEL_RIM_HEIGHT_M,
    HUB_FUNNEL_RIM_SPAN_M,
    HUB_FUNNEL_THROAT_HEIGHT_M,
    HUB_FUNNEL_THROAT_SPAN_M,
    HUB_OPENING_PLANE_HEIGHT_M,
    HUB_OPENING_SPAN_M,
)


def test_2026_hub_dimensions_match_game_manual():
    np.testing.assert_allclose(HUB_BODY_DEPTH_M, 1.1938)
    np.testing.assert_allclose(HUB_OPENING_SPAN_M, 1.05918)
    np.testing.assert_allclose(HUB_OPENING_PLANE_HEIGHT_M, 1.8288)
    np.testing.assert_allclose(HUB_FUNNEL_RIM_SPAN_M, 1.05918)
    np.testing.assert_allclose(HUB_FUNNEL_RIM_HEIGHT_M, 1.8288)
    np.testing.assert_allclose(HUB_FUNNEL_THROAT_SPAN_M, 0.608441194809816)
    np.testing.assert_allclose(HUB_FUNNEL_THROAT_HEIGHT_M, 1.4336)
    np.testing.assert_allclose(HUB_BALL_CENTER_HEIGHT_M, HUB_OPENING_PLANE_HEIGHT_M + BALL_DIAMETER_M / 2.0)


def test_drive_module_offset_matches_swerve_configuration():
    np.testing.assert_allclose(DRIVE_MODULE_LONGITUDINAL_OFFSET_M, 0.276225)


def test_rear_referenced_hood_command_converts_to_launch_elevation():
    shooter = ShooterModel(hood_offset_deg=0.0)
    minimum_command = ShotControls(2000.0, 2000.0, 25.1)
    maximum_command = ShotControls(2000.0, 2000.0, 45.0)

    np.testing.assert_allclose(launch_elevation_deg(minimum_command, shooter), 64.9)
    np.testing.assert_allclose(launch_elevation_deg(maximum_command, shooter), 45.0)
    np.testing.assert_allclose(np.degrees(release_state(minimum_command, BallSpec(), shooter)[1]), 64.9)


def test_no_aerodynamics_matches_closed_form_projectile():
    ball = BallSpec(drag_coefficient=0.0)
    shooter = ShooterModel(
        velocity_transfer=0.75,
        spin_transfer=0.0,
        lift_slope=0.0,
        spin_decay_per_s=0.0,
        release_height_m=0.8,
    )
    controls = ShotControls(top_rpm=3000.0, bottom_rpm=2550.0, hood_angle_deg=42.0)
    environment = Environment()
    trajectory = simulate_shot(controls, ball, shooter, environment, dt_s=0.002, max_time_s=0.7)
    speed, angle, _ = release_state(controls, ball, shooter)
    time = trajectory.time_s
    expected_x = speed * np.cos(angle) * time
    expected_z = shooter.release_height_m + speed * np.sin(angle) * time - 0.5 * environment.gravity_m_s2 * time**2
    np.testing.assert_allclose(trajectory.x_m, expected_x, atol=2e-7)
    np.testing.assert_allclose(trajectory.z_m, expected_z, atol=2e-6)


def test_faster_lower_wheel_creates_positive_backspin():
    ball = BallSpec()
    shooter = ShooterModel(
        top_wheel_diameter_m=0.0635,
        bottom_wheel_diameter_m=0.1016,
        bottom_to_top_rpm_ratio=1.0,
    )
    controls = ShotControls(top_rpm=2000.0, bottom_rpm=2000.0, hood_angle_deg=30.0)

    _, _, spin = release_state(controls, ball, shooter)

    assert spin > 0.0


def test_optimizer_returns_descending_shot_inside_target():
    ball = BallSpec()
    shooter = ShooterModel()
    environment = Environment()
    target = Target(distance_m=4.0, center_height_m=2.0, opening_height_m=0.45)
    result = optimize_shot(ball, shooter, environment, target, OptimizationWeights(), seed=42)
    entry = hub_entry_metrics(result.trajectory, target, ball)
    assert result.success
    assert entry is not None
    assert entry["inside"]
    assert entry["near_rim_clearance_m"] >= target.rim_margin_m
    assert entry["entry_angle_deg"] >= target.min_entry_angle_deg
    assert shooter.min_rpm <= result.controls.top_rpm <= shooter.max_rpm
    assert shooter.min_hood_deg <= result.controls.hood_angle_deg <= shooter.max_hood_deg


def test_shot_map_selects_a_valid_robust_candidate():
    ball = BallSpec()
    shooter = ShooterModel()
    target = Target(distance_m=4.0)
    shot_map = build_shot_map(
        ball,
        shooter,
        Environment(),
        target,
        OptimizationWeights(),
        hood_steps=15,
        rpm_steps=25,
        max_trajectories=12,
        boundary_refinement_steps=0,
    )

    assert 0 < shot_map.valid_count < shot_map.evaluated_count
    assert shot_map.robust_result is not None
    assert shot_map.robust_result.success
    assert shot_map.robustness_radius > 0.0
    assert shot_map.rpm_tolerance_lower >= 0.0
    assert shot_map.rpm_tolerance_upper >= 0.0
    assert shot_map.hood_tolerance_lower_deg >= 0.0
    assert shot_map.hood_tolerance_upper_deg >= 0.0
    assert np.any(np.isfinite(shot_map.lower_speed_m_s))
    assert np.any(np.isfinite(shot_map.upper_speed_m_s))


def test_shallow_center_crossing_fails_minimum_entry_constraint():
    ball = BallSpec()
    shooter = ShooterModel()
    target = Target(distance_m=4.572, min_entry_angle_deg=20.0)
    controls = ShotControls(top_rpm=3000.0, bottom_rpm=3000.0, hood_angle_deg=35.0)
    shallow_entry_deg = 10.0
    vx_m_s = 5.0
    vz_m_s = -vx_m_s * np.tan(np.radians(shallow_entry_deg))
    x_m = np.array([target.distance_m - 0.6, target.distance_m, target.distance_m + 0.4])
    z_m = target.center_height_m + (x_m - target.distance_m) * vz_m_s / vx_m_s
    trajectory = Trajectory(
        time_s=(x_m - x_m[0]) / vx_m_s,
        x_m=x_m,
        z_m=z_m,
        vx_m_s=np.full(3, vx_m_s),
        vz_m_s=np.full(3, vz_m_s),
        spin_rad_s=np.zeros(3),
    )

    entry = hub_entry_metrics(trajectory, target, ball)
    score, _ = shot_objective(controls, trajectory, ball, target, shooter, OptimizationWeights())

    assert entry is not None
    assert entry["inside"]
    assert entry["near_rim_clearance_m"] >= target.rim_margin_m
    assert entry["entry_angle_deg"] < target.min_entry_angle_deg
    assert score >= 20_000.0
