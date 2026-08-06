import numpy as np

from shotlab.calibration import fit_parameters, match_reference_controls, model_with_candidate, trajectory_rmse
from shotlab.models import BallSpec, Environment, OptimizationWeights, ShotControls, ShooterModel, Target
from shotlab.physics import optimize_shot, simulate_shot
from shotlab.tracking import TrackedShot


def tracked_from_trajectory(trajectory):
    indices = np.arange(0, len(trajectory.time_s), 4)
    return TrackedShot(
        time_s=trajectory.time_s[indices],
        frame_numbers=indices,
        x_px=trajectory.x_m[indices] * 400.0,
        y_px=700.0 - trajectory.z_m[indices] * 400.0,
        x_m=trajectory.x_m[indices],
        z_m=trajectory.z_m[indices],
        fps=120.0,
        frame_width=1920,
        frame_height=1080,
        annotated_frame_rgb=np.zeros((10, 10, 3), dtype=np.uint8),
        detected_points=len(indices),
        decoded_frames=len(indices),
    )


def test_fit_reduces_error_and_recovers_primary_parameters():
    ball = BallSpec()
    environment = Environment()
    target = Target()
    controls = ShotControls(3200.0, 2720.0, 43.0)
    true_model = ShooterModel(velocity_transfer=0.80, hood_offset_deg=2.5, drag_scale=1.0)
    starting_model = ShooterModel(velocity_transfer=0.67, hood_offset_deg=-1.0, drag_scale=1.0)
    truth = simulate_shot(controls, ball, true_model, environment, target, max_time_s=0.75)
    observed = tracked_from_trajectory(truth)
    fit = fit_parameters(
        observed,
        controls,
        ball,
        starting_model,
        environment,
        target,
        ["velocity_transfer", "hood_offset_deg"],
        regularization=0.01,
    )
    assert fit.after_rmse_m < fit.before_rmse_m * 0.1
    assert abs(fit.fitted_model.velocity_transfer - true_model.velocity_transfer) < 0.02
    assert abs(fit.fitted_model.hood_offset_deg - true_model.hood_offset_deg) < 0.6


def test_llm_candidate_bounds_are_enforced():
    model = ShooterModel()
    updated = model_with_candidate(model, {"drag_scale": 1.3, "unknown": 999})
    assert updated.drag_scale == 1.3
    assert updated.velocity_transfer == model.velocity_transfer

    try:
        model_with_candidate(model, {"drag_scale": 8.0})
    except ValueError as error:
        assert "outside" in str(error)
    else:
        raise AssertionError("Out-of-bounds candidate should have failed")


def test_rmse_is_zero_for_matching_track():
    ball = BallSpec()
    model = ShooterModel()
    environment = Environment()
    target = Target()
    controls = ShotControls(3000.0, 2550.0, 40.0)
    trajectory = simulate_shot(controls, ball, model, environment, target, max_time_s=0.6)
    observed = tracked_from_trajectory(trajectory)
    assert trajectory_rmse(observed, trajectory) < 1e-12


def test_control_correction_matches_reference_path_with_fitted_model():
    ball = BallSpec()
    environment = Environment()
    target = Target(distance_m=4.0)
    original_model = ShooterModel()
    reference = optimize_shot(ball, original_model, environment, target, OptimizationWeights(), seed=42)
    fitted_model = original_model.updated(velocity_transfer=0.67, hood_offset_deg=5.0)

    correction = match_reference_controls(
        reference.trajectory,
        ball,
        fitted_model,
        environment,
        target,
        seed=42,
    )

    assert correction.success
    assert correction.objective < 0.01
    assert correction.controls.top_rpm > reference.controls.top_rpm
    assert correction.controls.hood_angle_deg > reference.controls.hood_angle_deg + 4.0
