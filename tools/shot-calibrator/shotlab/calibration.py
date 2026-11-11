from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import differential_evolution, least_squares

from .models import BallSpec, Environment, ShotControls, ShooterModel, Target, Trajectory
from .physics import OptimizationResult, controls_from_top_rpm, hub_entry_metrics, simulate_shot
from .tracking import TrackedShot


PARAMETER_BOUNDS: dict[str, tuple[float, float]] = {
    "velocity_transfer": (0.20, 1.20),
    "spin_transfer": (0.05, 1.50),
    "hood_offset_deg": (-15.0, 15.0),
    "drag_scale": (0.20, 3.00),
    "lift_slope": (0.0, 2.50),
    "spin_decay_per_s": (0.0, 2.00),
}

PRIOR_SCALES: dict[str, float] = {
    "velocity_transfer": 0.15,
    "spin_transfer": 0.30,
    "hood_offset_deg": 4.0,
    "drag_scale": 0.50,
    "lift_slope": 0.50,
    "spin_decay_per_s": 0.35,
}


@dataclass(frozen=True)
class FitResult:
    original_model: ShooterModel
    fitted_model: ShooterModel
    fitted_parameters: tuple[str, ...]
    before_trajectory: Trajectory
    after_trajectory: Trajectory
    before_rmse_m: float
    after_rmse_m: float
    optimizer_success: bool
    optimizer_message: str
    jacobian_condition: float

    @property
    def improvement_percent(self) -> float:
        if self.before_rmse_m <= 1e-12:
            return 0.0
        return 100.0 * (self.before_rmse_m - self.after_rmse_m) / self.before_rmse_m


def trajectory_rmse(observed: TrackedShot, predicted: Trajectory) -> float:
    predicted_x = np.interp(observed.time_s, predicted.time_s, predicted.x_m)
    predicted_z = np.interp(observed.time_s, predicted.time_s, predicted.z_m)
    squared_distance = (predicted_x - observed.x_m) ** 2 + (predicted_z - observed.z_m) ** 2
    return float(np.sqrt(np.mean(squared_distance)))


def trajectory_path_rmse(reference: Trajectory, candidate: Trajectory, max_x_m: float) -> float:
    """Compare two trajectory shapes by height at common horizontal positions."""
    end_x_m = min(float(max_x_m), float(np.max(reference.x_m)))
    if end_x_m <= 0.0 or float(np.max(candidate.x_m)) < end_x_m:
        return float("inf")
    sample_x_m = np.linspace(0.0, end_x_m, 90)
    reference_z_m = np.interp(sample_x_m, reference.x_m, reference.z_m)
    candidate_z_m = np.interp(sample_x_m, candidate.x_m, candidate.z_m)
    return float(np.sqrt(np.mean((candidate_z_m - reference_z_m) ** 2)))


def match_reference_controls(
    reference: Trajectory,
    ball: BallSpec,
    fitted_model: ShooterModel,
    environment: Environment,
    target: Target,
    *,
    seed: int = 6560,
) -> OptimizationResult:
    """Find the next RPM and hood command that reproduces a reference path."""
    reference_end_x_m = min(
        target.distance_m + target.opening_span_m / 2.0,
        float(np.max(reference.x_m)),
    )
    sample_x_m = np.linspace(0.0, reference_end_x_m, 90)
    reference_z_m = np.interp(sample_x_m, reference.x_m, reference.z_m)
    reference_entry = hub_entry_metrics(reference, target, ball)

    def objective(candidate: np.ndarray) -> float:
        controls = controls_from_top_rpm(candidate[0], candidate[1], fitted_model)
        trajectory = simulate_shot(
            controls,
            ball,
            fitted_model,
            environment,
            target,
            dt_s=0.008,
        )
        candidate_end_x_m = float(np.max(trajectory.x_m))
        candidate_z_m = np.interp(sample_x_m, trajectory.x_m, trajectory.z_m)
        path_rmse_m = float(np.sqrt(np.mean((candidate_z_m - reference_z_m) ** 2)))
        score = (path_rmse_m / 0.02) ** 2

        missing_range_m = max(0.0, reference_end_x_m - candidate_end_x_m)
        if missing_range_m > 0.0:
            score += 50_000.0 + 50_000.0 * (missing_range_m / max(reference_end_x_m, 0.1)) ** 2

        entry = hub_entry_metrics(trajectory, target, ball)
        if entry is None:
            return score + 100_000.0
        if not bool(entry["inside"]):
            score += 50_000.0 + 50_000.0 * (
                max(0.0, -float(entry["clearance_m"])) / max(target.opening_span_m / 2.0, 0.01)
            ) ** 2
        rim_deficit_m = target.rim_margin_m - float(entry["near_rim_clearance_m"])
        if rim_deficit_m > 0.0:
            score += 50_000.0 + 50_000.0 * (rim_deficit_m / max(target.rim_margin_m, 0.01)) ** 2
        entry_deficit_deg = target.min_entry_angle_deg - float(entry["entry_angle_deg"])
        if entry_deficit_deg > 0.0:
            score += 50_000.0 + 200.0 * entry_deficit_deg**2

        if reference_entry is not None:
            time_delta_s = float(entry["time_s"]) - float(reference_entry["time_s"])
            entry_delta_deg = float(entry["entry_angle_deg"]) - float(reference_entry["entry_angle_deg"])
            score += 0.15 * (time_delta_s / 0.10) ** 2
            score += 0.05 * (entry_delta_deg / 5.0) ** 2
        return float(score)

    result = differential_evolution(
        objective,
        bounds=[
            (fitted_model.min_rpm, fitted_model.max_rpm),
            (fitted_model.min_hood_deg, fitted_model.max_hood_deg),
        ],
        seed=seed,
        popsize=9,
        maxiter=34,
        polish=True,
        tol=1e-6,
        workers=1,
    )
    controls = controls_from_top_rpm(result.x[0], result.x[1], fitted_model)
    trajectory = simulate_shot(controls, ball, fitted_model, environment, target)
    metrics = hub_entry_metrics(trajectory, target, ball) or {
        "time_s": float("nan"),
        "height_m": float("nan"),
        "vx_m_s": float("nan"),
        "vz_m_s": float("nan"),
        "entry_angle_deg": float("nan"),
        "height_error_m": float("nan"),
        "crossing_x_m": float("nan"),
        "center_offset_m": float("nan"),
        "clearance_m": float("nan"),
        "near_rim_clearance_m": float("nan"),
        "usable_half_span_m": float("nan"),
        "inside": False,
    }
    path_rmse_m = trajectory_path_rmse(reference, trajectory, reference_end_x_m)
    success = (
        bool(metrics["inside"])
        and float(metrics["near_rim_clearance_m"]) >= target.rim_margin_m
        and float(metrics["entry_angle_deg"]) >= target.min_entry_angle_deg
        and path_rmse_m <= 0.08
    )
    return OptimizationResult(
        controls=controls,
        trajectory=trajectory,
        metrics=metrics,
        objective=path_rmse_m,
        success=bool(success),
        message=str(result.message),
    )


def residual_profile(observed: TrackedShot, predicted: Trajectory) -> dict[str, float]:
    predicted_x = np.interp(observed.time_s, predicted.time_s, predicted.x_m)
    predicted_z = np.interp(observed.time_s, predicted.time_s, predicted.z_m)
    x_error = observed.x_m - predicted_x
    z_error = observed.z_m - predicted_z
    midpoint = max(1, len(x_error) // 2)
    return {
        "mean_x_error_m": float(np.mean(x_error)),
        "mean_z_error_m": float(np.mean(z_error)),
        "early_z_error_m": float(np.mean(z_error[:midpoint])),
        "late_z_error_m": float(np.mean(z_error[midpoint:])),
        "final_x_error_m": float(x_error[-1]),
        "final_z_error_m": float(z_error[-1]),
    }


def model_with_candidate(current: ShooterModel, candidate: dict[str, float]) -> ShooterModel:
    clean: dict[str, float] = {}
    for name, value in candidate.items():
        if name not in PARAMETER_BOUNDS:
            continue
        low, high = PARAMETER_BOUNDS[name]
        numeric = float(value)
        if not low <= numeric <= high:
            raise ValueError(f"{name}={numeric} is outside the allowed range [{low}, {high}].")
        clean[name] = numeric
    if not clean:
        raise ValueError("The candidate did not contain any recognized empirical parameters.")
    return current.updated(**clean)


def fit_parameters(
    observed: TrackedShot,
    controls: ShotControls,
    ball: BallSpec,
    shooter: ShooterModel,
    environment: Environment,
    target: Target,
    parameter_names: list[str],
    *,
    regularization: float = 0.08,
) -> FitResult:
    names = tuple(name for name in parameter_names if name in PARAMETER_BOUNDS)
    if not names:
        raise ValueError("Select at least one empirical parameter to fit.")
    if len(observed.time_s) < 6:
        raise ValueError("At least six measured points are required for fitting.")

    original_values = np.asarray([getattr(shooter, name) for name in names], dtype=float)
    lower = np.asarray([PARAMETER_BOUNDS[name][0] for name in names], dtype=float)
    upper = np.asarray([PARAMETER_BOUNDS[name][1] for name in names], dtype=float)
    sample_indices = np.linspace(0, len(observed.time_s) - 1, min(90, len(observed.time_s))).astype(int)
    sample_times = observed.time_s[sample_indices]
    sample_x = observed.x_m[sample_indices]
    sample_z = observed.z_m[sample_indices]
    position_scale_m = 0.03

    def simulate(values: np.ndarray) -> Trajectory:
        updates = {name: float(value) for name, value in zip(names, values)}
        model = shooter.updated(**updates)
        return simulate_shot(
            controls,
            ball,
            model,
            environment,
            target,
            max_time_s=max(0.5, float(observed.time_s[-1]) + 0.15),
        )

    def residuals(values: np.ndarray) -> np.ndarray:
        trajectory = simulate(values)
        predicted_x = np.interp(sample_times, trajectory.time_s, trajectory.x_m)
        predicted_z = np.interp(sample_times, trajectory.time_s, trajectory.z_m)
        position_residuals = np.column_stack(
            ((predicted_x - sample_x) / position_scale_m, (predicted_z - sample_z) / position_scale_m)
        ).ravel()
        prior_residuals = np.asarray(
            [
                regularization * (value - original) / PRIOR_SCALES[name]
                for name, value, original in zip(names, values, original_values)
            ]
        )
        return np.concatenate((position_residuals, prior_residuals))

    before = simulate_shot(
        controls,
        ball,
        shooter,
        environment,
        target,
        max_time_s=max(0.5, float(observed.time_s[-1]) + 0.15),
    )
    result = least_squares(
        residuals,
        np.clip(original_values, lower + 1e-9, upper - 1e-9),
        bounds=(lower, upper),
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=180,
        x_scale="jac",
    )
    fitted = shooter.updated(**{name: float(value) for name, value in zip(names, result.x)})
    after = simulate_shot(
        controls,
        ball,
        fitted,
        environment,
        target,
        max_time_s=max(0.5, float(observed.time_s[-1]) + 0.15),
    )
    singular_values = np.linalg.svd(result.jac, compute_uv=False) if result.jac.size else np.array([])
    condition = (
        float(singular_values[0] / singular_values[-1])
        if len(singular_values) and singular_values[-1] > 1e-12
        else float("inf")
    )
    return FitResult(
        original_model=shooter,
        fitted_model=fitted,
        fitted_parameters=names,
        before_trajectory=before,
        after_trajectory=after,
        before_rmse_m=trajectory_rmse(observed, before),
        after_rmse_m=trajectory_rmse(observed, after),
        optimizer_success=bool(result.success),
        optimizer_message=str(result.message),
        jacobian_condition=condition,
    )
