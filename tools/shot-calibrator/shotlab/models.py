from __future__ import annotations

from dataclasses import asdict, dataclass, replace

import numpy as np

from .robot_profile import (
    ASSUMED_TARGET_CENTER_HEIGHT_M,
    ASSUMED_TARGET_OPENING_HEIGHT_M,
    BALL_DIAMETER_M,
    BALL_MASS_KG,
    BOTTOM_WHEEL_DIAMETER_M,
    FLYWHEEL_IDLE_RPM,
    FLYWHEEL_MAX_RPM,
    FOLLOWER_TO_LEADER_RPM_RATIO,
    HUB_DEFAULT_MIN_ENTRY_ANGLE_DEG,
    HUB_DEFAULT_RIM_MARGIN_M,
    HUB_OPENING_SPAN_M,
    HOOD_MAX_DEG,
    HOOD_MIN_DEG,
    RELEASE_HEIGHT_M,
    TOP_WHEEL_DIAMETER_M,
)


@dataclass(frozen=True)
class BallSpec:
    mass_kg: float = BALL_MASS_KG
    diameter_m: float = BALL_DIAMETER_M
    drag_coefficient: float = 0.47

    @property
    def radius_m(self) -> float:
        return self.diameter_m / 2.0

    @property
    def area_m2(self) -> float:
        return np.pi * self.radius_m**2


@dataclass(frozen=True)
class Environment:
    air_density_kg_m3: float = 1.225
    gravity_m_s2: float = 9.80665
    wind_x_m_s: float = 0.0


@dataclass(frozen=True)
class Target:
    distance_m: float = 4.0
    center_height_m: float = ASSUMED_TARGET_CENTER_HEIGHT_M
    opening_height_m: float = ASSUMED_TARGET_OPENING_HEIGHT_M
    opening_span_m: float = HUB_OPENING_SPAN_M
    min_entry_angle_deg: float = HUB_DEFAULT_MIN_ENTRY_ANGLE_DEG
    rim_margin_m: float = HUB_DEFAULT_RIM_MARGIN_M

    @property
    def lower_edge_m(self) -> float:
        return self.center_height_m - self.opening_height_m / 2.0

    @property
    def upper_edge_m(self) -> float:
        return self.center_height_m + self.opening_height_m / 2.0


@dataclass(frozen=True)
class ShooterModel:
    mode: str = "dual_wheel"
    top_wheel_diameter_m: float = TOP_WHEEL_DIAMETER_M
    bottom_wheel_diameter_m: float = BOTTOM_WHEEL_DIAMETER_M
    min_rpm: float = FLYWHEEL_IDLE_RPM
    max_rpm: float = FLYWHEEL_MAX_RPM
    min_hood_deg: float = HOOD_MIN_DEG
    max_hood_deg: float = HOOD_MAX_DEG
    bottom_to_top_rpm_ratio: float = FOLLOWER_TO_LEADER_RPM_RATIO
    release_height_m: float = RELEASE_HEIGHT_M
    robot_velocity_x_m_s: float = 0.0
    velocity_transfer: float = 0.72
    spin_transfer: float = 0.70
    hood_offset_deg: float = 0.0
    drag_scale: float = 1.0
    lift_slope: float = 0.75
    max_lift_coefficient: float = 0.35
    spin_decay_per_s: float = 0.10

    def updated(self, **changes: float) -> "ShooterModel":
        return replace(self, **changes)

    def empirical_parameters(self) -> dict[str, float]:
        return {
            "velocity_transfer": self.velocity_transfer,
            "spin_transfer": self.spin_transfer,
            "hood_offset_deg": self.hood_offset_deg,
            "drag_scale": self.drag_scale,
            "lift_slope": self.lift_slope,
            "spin_decay_per_s": self.spin_decay_per_s,
        }

    def as_dict(self) -> dict[str, float | str]:
        return asdict(self)


@dataclass(frozen=True)
class ShotControls:
    top_rpm: float
    bottom_rpm: float
    # Robot command measured from the rear/vertical reference, not elevation above horizontal.
    hood_angle_deg: float

    @property
    def average_rpm(self) -> float:
        active = [self.top_rpm]
        if self.bottom_rpm > 0:
            active.append(self.bottom_rpm)
        return float(np.mean(active))


@dataclass(frozen=True)
class OptimizationWeights:
    flight_time: float = 0.25
    entry_angle: float = 0.50
    mechanism_effort: float = 0.15


@dataclass(frozen=True)
class Trajectory:
    time_s: np.ndarray
    x_m: np.ndarray
    z_m: np.ndarray
    vx_m_s: np.ndarray
    vz_m_s: np.ndarray
    spin_rad_s: np.ndarray

    def target_crossing(self, target: Target) -> dict[str, float] | None:
        indices = np.flatnonzero(self.x_m >= target.distance_m)
        if len(indices) == 0 or indices[0] == 0:
            return None
        right = int(indices[0])
        left = right - 1
        dx = self.x_m[right] - self.x_m[left]
        fraction = 0.0 if abs(dx) < 1e-12 else (target.distance_m - self.x_m[left]) / dx

        def lerp(values: np.ndarray) -> float:
            return float(values[left] + fraction * (values[right] - values[left]))

        vx = lerp(self.vx_m_s)
        vz = lerp(self.vz_m_s)
        return {
            "time_s": lerp(self.time_s),
            "height_m": lerp(self.z_m),
            "vx_m_s": vx,
            "vz_m_s": vz,
            "entry_angle_deg": float(np.degrees(np.arctan2(-vz, max(vx, 1e-9)))),
            "height_error_m": lerp(self.z_m) - target.center_height_m,
        }
