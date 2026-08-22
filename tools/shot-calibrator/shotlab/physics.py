from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.optimize import differential_evolution

from .models import (
    BallSpec,
    Environment,
    OptimizationWeights,
    ShotControls,
    ShooterModel,
    Target,
    Trajectory,
)


@dataclass(frozen=True)
class OptimizationResult:
    controls: ShotControls
    trajectory: Trajectory
    metrics: dict[str, float | bool]
    objective: float
    success: bool
    message: str


@dataclass(frozen=True)
class ShotMapTrajectory:
    controls: ShotControls
    trajectory: Trajectory
    boundary: str


@dataclass(frozen=True)
class ShotMapResult:
    hood_commands_deg: np.ndarray
    launch_elevations_deg: np.ndarray
    top_rpms: np.ndarray
    launch_speeds_m_s: np.ndarray
    feasible_mask: np.ndarray
    lower_speed_m_s: np.ndarray
    upper_speed_m_s: np.ndarray
    representative_shots: tuple[ShotMapTrajectory, ...]
    robust_result: OptimizationResult | None
    robustness_radius: float
    rpm_tolerance_lower: float
    rpm_tolerance_upper: float
    hood_tolerance_lower_deg: float
    hood_tolerance_upper_deg: float

    @property
    def valid_count(self) -> int:
        return int(np.count_nonzero(self.feasible_mask))

    @property
    def evaluated_count(self) -> int:
        return int(self.feasible_mask.size)


def controls_from_top_rpm(top_rpm: float, hood_deg: float, shooter: ShooterModel) -> ShotControls:
    if shooter.mode == "fixed_hood":
        bottom_rpm = 0.0
    else:
        bottom_rpm = top_rpm * shooter.bottom_to_top_rpm_ratio
    return ShotControls(float(top_rpm), float(bottom_rpm), float(hood_deg))


def launch_elevation_deg(controls: ShotControls, shooter: ShooterModel) -> float:
    """Convert the robot's rear-referenced hood command to elevation above horizontal."""
    return float(90.0 - controls.hood_angle_deg + shooter.hood_offset_deg)


def release_state(
    controls: ShotControls,
    ball: BallSpec,
    shooter: ShooterModel,
) -> tuple[float, float, float]:
    top_surface_m_s = np.pi * shooter.top_wheel_diameter_m * controls.top_rpm / 60.0
    bottom_surface_m_s = np.pi * shooter.bottom_wheel_diameter_m * controls.bottom_rpm / 60.0
    exit_speed_m_s = shooter.velocity_transfer * (top_surface_m_s + bottom_surface_m_s) / 2.0
    launch_angle_rad = np.radians(launch_elevation_deg(controls, shooter))
    # Positive spin is defined as backspin, which produces upward Magnus lift.
    spin_rad_s = shooter.spin_transfer * (bottom_surface_m_s - top_surface_m_s) / (2.0 * ball.radius_m)
    return float(exit_speed_m_s), float(launch_angle_rad), float(spin_rad_s)


def simulate_shot(
    controls: ShotControls,
    ball: BallSpec,
    shooter: ShooterModel,
    environment: Environment,
    target: Target | None = None,
    *,
    dt_s: float = 0.004,
    max_time_s: float = 3.0,
) -> Trajectory:
    speed, angle, spin = release_state(controls, ball, shooter)
    state = np.array(
        [
            0.0,
            shooter.release_height_m,
            speed * np.cos(angle) + shooter.robot_velocity_x_m_s,
            speed * np.sin(angle),
            spin,
        ],
        dtype=float,
    )

    def derivative(current: np.ndarray) -> np.ndarray:
        _, _, vx, vz, omega = current
        relative_velocity = np.array([vx - environment.wind_x_m_s, vz], dtype=float)
        relative_speed = float(np.linalg.norm(relative_velocity))
        acceleration = np.array([0.0, -environment.gravity_m_s2], dtype=float)

        if relative_speed > 1e-8:
            dynamic_accel = 0.5 * environment.air_density_kg_m3 * ball.area_m2 / ball.mass_kg
            acceleration += (
                -dynamic_accel
                * ball.drag_coefficient
                * shooter.drag_scale
                * relative_speed
                * relative_velocity
            )

            spin_ratio = ball.radius_m * abs(omega) / relative_speed
            lift_coefficient = min(shooter.max_lift_coefficient, shooter.lift_slope * spin_ratio)
            lift_direction = np.array([-relative_velocity[1], relative_velocity[0]]) / relative_speed
            acceleration += (
                dynamic_accel
                * lift_coefficient
                * relative_speed**2
                * np.sign(omega)
                * lift_direction
            )

        spin_rate = -shooter.spin_decay_per_s * omega
        return np.array([vx, vz, acceleration[0], acceleration[1], spin_rate], dtype=float)

    states = [state.copy()]
    times = [0.0]
    max_x = max(10.0, (target.distance_m * 1.25) if target else 10.0)
    steps = int(np.ceil(max_time_s / dt_s))
    for step in range(steps):
        k1 = derivative(state)
        k2 = derivative(state + 0.5 * dt_s * k1)
        k3 = derivative(state + 0.5 * dt_s * k2)
        k4 = derivative(state + dt_s * k3)
        state = state + dt_s * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        states.append(state.copy())
        times.append((step + 1) * dt_s)
        if state[1] < 0.0 or state[0] > max_x:
            break

    values = np.asarray(states)
    return Trajectory(
        time_s=np.asarray(times),
        x_m=values[:, 0],
        z_m=values[:, 1],
        vx_m_s=values[:, 2],
        vz_m_s=values[:, 3],
        spin_rad_s=values[:, 4],
    )


def hub_entry_metrics(
    trajectory: Trajectory,
    target: Target,
    ball: BallSpec,
) -> dict[str, float | bool] | None:
    """Return the descending ball-center crossing through the upper funnel entrance."""
    plane_height_m = target.center_height_m
    usable_half_span_m = max(
        target.opening_span_m / 2.0 - ball.radius_m - target.rim_margin_m,
        0.0,
    )
    for index in range(len(trajectory.z_m) - 1):
        z_left = float(trajectory.z_m[index])
        z_right = float(trajectory.z_m[index + 1])
        if not (z_left >= plane_height_m > z_right):
            continue

        dz = z_right - z_left
        fraction = 0.0 if abs(dz) < 1e-12 else (plane_height_m - z_left) / dz

        def lerp(values: np.ndarray) -> float:
            return float(values[index] + fraction * (values[index + 1] - values[index]))

        crossing_x_m = lerp(trajectory.x_m)
        vx_m_s = lerp(trajectory.vx_m_s)
        vz_m_s = lerp(trajectory.vz_m_s)
        center_offset_m = abs(crossing_x_m - target.distance_m)
        near_rim_x_m = target.distance_m - target.opening_span_m / 2.0
        if near_rim_x_m < float(np.min(trajectory.x_m)) or near_rim_x_m > float(np.max(trajectory.x_m)):
            near_rim_clearance_m = float("-inf")
        else:
            near_rim_height_m = float(np.interp(near_rim_x_m, trajectory.x_m, trajectory.z_m))
            near_rim_clearance_m = near_rim_height_m - plane_height_m

        center_crossing = trajectory.target_crossing(target)
        return {
            "time_s": lerp(trajectory.time_s),
            "height_m": plane_height_m,
            "vx_m_s": vx_m_s,
            "vz_m_s": vz_m_s,
            "entry_angle_deg": float(np.degrees(np.arctan2(-vz_m_s, max(vx_m_s, 1e-9)))),
            "height_error_m": (
                float(center_crossing["height_error_m"]) if center_crossing is not None else float("nan")
            ),
            "crossing_x_m": crossing_x_m,
            "center_offset_m": center_offset_m,
            "clearance_m": usable_half_span_m - center_offset_m,
            "near_rim_clearance_m": near_rim_clearance_m,
            "usable_half_span_m": usable_half_span_m,
            "inside": center_offset_m <= usable_half_span_m,
        }
    return None


def shot_objective(
    controls: ShotControls,
    trajectory: Trajectory,
    ball: BallSpec,
    target: Target,
    shooter: ShooterModel,
    weights: OptimizationWeights,
) -> tuple[float, dict[str, float | bool] | None]:
    metrics = hub_entry_metrics(trajectory, target, ball)
    if metrics is None:
        final_gap = max(0.0, target.distance_m - float(np.max(trajectory.x_m)))
        height_gap = max(0.0, target.center_height_m - float(np.max(trajectory.z_m)))
        return 1_000_000.0 + 10_000.0 * (final_gap + height_gap), None

    usable_half_span_m = max(float(metrics["usable_half_span_m"]), 1e-6)
    center_fraction = float(metrics["center_offset_m"]) / usable_half_span_m
    score = 0.20 * center_fraction**2

    if float(metrics["clearance_m"]) < 0.0:
        outside_fraction = -float(metrics["clearance_m"]) / usable_half_span_m
        score += 20_000.0 + 20_000.0 * outside_fraction**2

    rim_deficit_m = target.rim_margin_m - float(metrics["near_rim_clearance_m"])
    if rim_deficit_m > 0.0:
        score += 20_000.0 + 20_000.0 * (rim_deficit_m / max(target.rim_margin_m, 0.01)) ** 2

    entry_deficit_deg = target.min_entry_angle_deg - float(metrics["entry_angle_deg"])
    if entry_deficit_deg > 0.0:
        score += 20_000.0 + 100.0 * entry_deficit_deg**2

    rpm_fraction = (controls.top_rpm - shooter.min_rpm) / max(shooter.max_rpm - shooter.min_rpm, 1.0)
    score += weights.flight_time * float(metrics["time_s"]) / 1.5
    score += weights.entry_angle * (1.0 - min(float(metrics["entry_angle_deg"]), 60.0) / 60.0)
    score += weights.mechanism_effort * rpm_fraction**2
    return float(score), metrics


def _is_scoring_entry(metrics: dict[str, float | bool] | None, target: Target) -> bool:
    return bool(
        metrics is not None
        and float(metrics["usable_half_span_m"]) > 0.0
        and float(metrics["vx_m_s"]) > 0.0
        and float(metrics["vz_m_s"]) < 0.0
        and bool(metrics["inside"])
        and float(metrics["near_rim_clearance_m"]) >= target.rim_margin_m
        and float(metrics["entry_angle_deg"]) >= target.min_entry_angle_deg
    )


def _contiguous_true_bounds(values: np.ndarray, index: int) -> tuple[int, int]:
    lower = index
    upper = index
    while lower > 0 and bool(values[lower - 1]):
        lower -= 1
    while upper + 1 < len(values) and bool(values[upper + 1]):
        upper += 1
    return lower, upper


def build_shot_map(
    ball: BallSpec,
    shooter: ShooterModel,
    environment: Environment,
    target: Target,
    weights: OptimizationWeights,
    *,
    hood_steps: int = 29,
    rpm_steps: int = 41,
    max_trajectories: int = 48,
    boundary_refinement_steps: int = 7,
) -> ShotMapResult:
    """Sample the bounded command space and select the center of its scoring region."""
    if hood_steps < 3 or rpm_steps < 3:
        raise ValueError("Shot-map dimensions must each contain at least three samples.")

    hood_commands = np.linspace(shooter.min_hood_deg, shooter.max_hood_deg, hood_steps)
    top_rpms = np.linspace(shooter.min_rpm, shooter.max_rpm, rpm_steps)
    launch_elevations = np.array(
        [launch_elevation_deg(controls_from_top_rpm(shooter.min_rpm, hood, shooter), shooter) for hood in hood_commands]
    )
    launch_speeds = np.array(
        [release_state(controls_from_top_rpm(rpm, hood_commands[0], shooter), ball, shooter)[0] for rpm in top_rpms]
    )
    feasible = np.zeros((hood_steps, rpm_steps), dtype=bool)
    objective_values = np.full((hood_steps, rpm_steps), np.inf, dtype=float)

    for hood_index, hood_command in enumerate(hood_commands):
        for rpm_index, top_rpm in enumerate(top_rpms):
            controls = controls_from_top_rpm(top_rpm, hood_command, shooter)
            trajectory = simulate_shot(
                controls,
                ball,
                shooter,
                environment,
                target,
                dt_s=0.012,
            )
            objective, metrics = shot_objective(controls, trajectory, ball, target, shooter, weights)
            if _is_scoring_entry(metrics, target):
                feasible[hood_index, rpm_index] = True
                objective_values[hood_index, rpm_index] = objective

    lower_speeds = np.full(hood_steps, np.nan, dtype=float)
    upper_speeds = np.full(hood_steps, np.nan, dtype=float)

    def scores_at(top_rpm: float, hood_command: float) -> bool:
        controls = controls_from_top_rpm(top_rpm, hood_command, shooter)
        trajectory = simulate_shot(controls, ball, shooter, environment, target, dt_s=0.012)
        return _is_scoring_entry(hub_entry_metrics(trajectory, target, ball), target)

    for hood_index in range(hood_steps):
        valid_rpms = np.flatnonzero(feasible[hood_index])
        if len(valid_rpms):
            first_valid = int(valid_rpms[0])
            last_valid = int(valid_rpms[-1])
            lower_boundary_rpm = float(top_rpms[first_valid])
            upper_boundary_rpm = float(top_rpms[last_valid])
            if first_valid > 0:
                invalid_rpm = float(top_rpms[first_valid - 1])
                valid_rpm = lower_boundary_rpm
                for _ in range(max(0, boundary_refinement_steps)):
                    midpoint = (invalid_rpm + valid_rpm) / 2.0
                    if scores_at(midpoint, float(hood_commands[hood_index])):
                        valid_rpm = midpoint
                    else:
                        invalid_rpm = midpoint
                lower_boundary_rpm = valid_rpm
            if last_valid + 1 < rpm_steps:
                valid_rpm = upper_boundary_rpm
                invalid_rpm = float(top_rpms[last_valid + 1])
                for _ in range(max(0, boundary_refinement_steps)):
                    midpoint = (valid_rpm + invalid_rpm) / 2.0
                    if scores_at(midpoint, float(hood_commands[hood_index])):
                        valid_rpm = midpoint
                    else:
                        invalid_rpm = midpoint
                upper_boundary_rpm = valid_rpm
            lower_speeds[hood_index] = release_state(
                controls_from_top_rpm(lower_boundary_rpm, float(hood_commands[hood_index]), shooter),
                ball,
                shooter,
            )[0]
            upper_speeds[hood_index] = release_state(
                controls_from_top_rpm(upper_boundary_rpm, float(hood_commands[hood_index]), shooter),
                ball,
                shooter,
            )[0]

    if not np.any(feasible):
        return ShotMapResult(
            hood_commands_deg=hood_commands,
            launch_elevations_deg=launch_elevations,
            top_rpms=top_rpms,
            launch_speeds_m_s=launch_speeds,
            feasible_mask=feasible,
            lower_speed_m_s=lower_speeds,
            upper_speed_m_s=upper_speeds,
            representative_shots=(),
            robust_result=None,
            robustness_radius=0.0,
            rpm_tolerance_lower=0.0,
            rpm_tolerance_upper=0.0,
            hood_tolerance_lower_deg=0.0,
            hood_tolerance_upper_deg=0.0,
        )

    hood_step_deg = float(abs(hood_commands[1] - hood_commands[0]))
    rpm_step = float(abs(top_rpms[1] - top_rpms[0]))
    # One distance unit corresponds to a 0.5 deg hood error or 100 RPM error.
    padded_mask = np.pad(feasible, 1, constant_values=False)
    robustness = distance_transform_edt(
        padded_mask,
        sampling=(hood_step_deg / 0.5, rpm_step / 100.0),
    )[1:-1, 1:-1]
    valid_indices = [tuple(map(int, index)) for index in np.argwhere(feasible)]
    ordered_indices = sorted(
        valid_indices,
        key=lambda index: (-float(robustness[index]), float(objective_values[index])),
    )

    selected_index: tuple[int, int] | None = None
    selected_controls: ShotControls | None = None
    selected_trajectory: Trajectory | None = None
    selected_metrics: dict[str, float | bool] | None = None
    selected_objective = float("inf")
    for hood_index, rpm_index in ordered_indices:
        candidate_controls = controls_from_top_rpm(top_rpms[rpm_index], hood_commands[hood_index], shooter)
        candidate_trajectory = simulate_shot(candidate_controls, ball, shooter, environment, target)
        candidate_objective, candidate_metrics = shot_objective(
            candidate_controls,
            candidate_trajectory,
            ball,
            target,
            shooter,
            weights,
        )
        if _is_scoring_entry(candidate_metrics, target):
            selected_index = (hood_index, rpm_index)
            selected_controls = candidate_controls
            selected_trajectory = candidate_trajectory
            selected_metrics = candidate_metrics
            selected_objective = candidate_objective
            break

    if selected_index is None or selected_controls is None or selected_trajectory is None or selected_metrics is None:
        robust_result = None
        robustness_radius = 0.0
        rpm_tolerances = (0.0, 0.0)
        hood_tolerances = (0.0, 0.0)
    else:
        hood_index, rpm_index = selected_index
        rpm_lower_index, rpm_upper_index = _contiguous_true_bounds(feasible[hood_index], rpm_index)
        hood_lower_index, hood_upper_index = _contiguous_true_bounds(feasible[:, rpm_index], hood_index)
        rpm_tolerances = (
            float(selected_controls.top_rpm - top_rpms[rpm_lower_index]),
            float(top_rpms[rpm_upper_index] - selected_controls.top_rpm),
        )
        hood_tolerances = (
            float(selected_controls.hood_angle_deg - hood_commands[hood_lower_index]),
            float(hood_commands[hood_upper_index] - selected_controls.hood_angle_deg),
        )
        robustness_radius = float(robustness[selected_index])
        robust_result = OptimizationResult(
            controls=selected_controls,
            trajectory=selected_trajectory,
            metrics=selected_metrics,
            objective=selected_objective,
            success=True,
            message="Selected from the interior of the sampled scoring region.",
        )

    valid_rows = np.flatnonzero(np.any(feasible, axis=1))
    row_budget = max(1, max_trajectories // 3)
    if len(valid_rows) > row_budget:
        row_positions = np.linspace(0, len(valid_rows) - 1, row_budget).round().astype(int)
        representative_rows = valid_rows[row_positions]
    else:
        representative_rows = valid_rows

    representative_specs: list[tuple[int, int, str]] = []
    seen_specs: set[tuple[int, int]] = set()
    for hood_index in representative_rows:
        valid_rpms = np.flatnonzero(feasible[int(hood_index)])
        candidates = (
            (int(valid_rpms[0]), "lower"),
            (int(valid_rpms[len(valid_rpms) // 2]), "interior"),
            (int(valid_rpms[-1]), "upper"),
        )
        for rpm_index, boundary in candidates:
            key = (int(hood_index), rpm_index)
            if key not in seen_specs:
                seen_specs.add(key)
                representative_specs.append((key[0], key[1], boundary))

    representatives: list[ShotMapTrajectory] = []
    for hood_index, rpm_index, boundary in representative_specs[:max_trajectories]:
        controls = controls_from_top_rpm(top_rpms[rpm_index], hood_commands[hood_index], shooter)
        trajectory = simulate_shot(controls, ball, shooter, environment, target, dt_s=0.01)
        metrics = hub_entry_metrics(trajectory, target, ball)
        if _is_scoring_entry(metrics, target):
            representatives.append(ShotMapTrajectory(controls, trajectory, boundary))

    return ShotMapResult(
        hood_commands_deg=hood_commands,
        launch_elevations_deg=launch_elevations,
        top_rpms=top_rpms,
        launch_speeds_m_s=launch_speeds,
        feasible_mask=feasible,
        lower_speed_m_s=lower_speeds,
        upper_speed_m_s=upper_speeds,
        representative_shots=tuple(representatives),
        robust_result=robust_result,
        robustness_radius=robustness_radius,
        rpm_tolerance_lower=rpm_tolerances[0],
        rpm_tolerance_upper=rpm_tolerances[1],
        hood_tolerance_lower_deg=hood_tolerances[0],
        hood_tolerance_upper_deg=hood_tolerances[1],
    )


def optimize_shot(
    ball: BallSpec,
    shooter: ShooterModel,
    environment: Environment,
    target: Target,
    weights: OptimizationWeights,
    *,
    seed: int = 6560,
) -> OptimizationResult:
    def objective(candidate: np.ndarray) -> float:
        controls = controls_from_top_rpm(candidate[0], candidate[1], shooter)
        trajectory = simulate_shot(controls, ball, shooter, environment, target, dt_s=0.008)
        score, _ = shot_objective(controls, trajectory, ball, target, shooter, weights)
        return score

    result = differential_evolution(
        objective,
        bounds=[(shooter.min_rpm, shooter.max_rpm), (shooter.min_hood_deg, shooter.max_hood_deg)],
        seed=seed,
        popsize=9,
        maxiter=34,
        polish=True,
        tol=1e-6,
        workers=1,
    )
    controls = controls_from_top_rpm(result.x[0], result.x[1], shooter)
    trajectory = simulate_shot(controls, ball, shooter, environment, target)
    objective_value, metrics = shot_objective(controls, trajectory, ball, target, shooter, weights)
    metrics = metrics or {
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
    hit = _is_scoring_entry(metrics, target)
    return OptimizationResult(
        controls=controls,
        trajectory=trajectory,
        metrics=metrics,
        objective=objective_value,
        success=bool(hit),
        message=str(result.message),
    )
