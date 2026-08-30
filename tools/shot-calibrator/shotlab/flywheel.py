"""Flywheel sizing model: tube wall thickness against spin-up time and firing droop.

The mechanism is a single steel tube flywheel with a static hood and no back
rollers, driven by two Kraken X60s through a gear pair. Wall thickness is the
only free variable on the rotating mass, so this module maps wall thickness to
rotational inertia, then to the two things inertia trades against:

* spin-up time, which grows with inertia
* speed droop when balls are fired, which shrinks with inertia

Torque is piecewise linear in flywheel speed (a stator-current-limited plateau
below the knee, then the voltage-limited part of the motor curve), so every
spin-up and spin-down time here is integrated in closed form rather than
stepped, which keeps a full wall-thickness sweep exact and cheap.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .models import BallSpec, Environment, ShooterModel, ShotControls, Target
from .physics import _is_scoring_entry, hub_entry_metrics, simulate_shot

INCH_TO_METER = 0.0254
RPM_TO_RAD_S = 2.0 * np.pi / 60.0
RAD_S_TO_RPM = 60.0 / (2.0 * np.pi)

STEEL_DENSITY_KG_M3 = 7850.0
ALUMINUM_DENSITY_KG_M3 = 2700.0

# Stock round-tube walls a team can actually buy, in inches.
STANDARD_WALL_INCHES = (0.035, 0.049, 0.065, 0.083, 0.095, 0.120, 0.156, 0.188, 0.250)

# A full FRC match is 2:30. Battery capacity is the usual 18 Ah at 12 V.
MATCH_SECONDS = 150.0
BATTERY_ENERGY_J = 18.0 * 12.0 * 3600.0

# FRC electrical rules: one 120 A main breaker for the whole robot, and channel
# breakers on the PDP/PDH capped at 40 A. Both are thermal, so a short burst
# above rating rides through; it is sustained draw that trips them.
MAIN_BREAKER_A = 120.0
MAX_CHANNEL_BREAKER_A = 40.0

# Battery health, as teams measure it with a beak. A pack fit for a match sits
# under 15 milliohms; anything over 20 milliohms should be retired rather than
# used even for testing.
BATTERY_MATCH_RESISTANCE_OHM = 0.015
BATTERY_SCRAP_RESISTANCE_OHM = 0.020
BATTERY_RESISTANCE_OHM = BATTERY_MATCH_RESISTANCE_OHM

# Everything between one channel breaker and its motor: branch wiring and the
# breaker itself. A 40 A channel runs 10-12 AWG, which is about 1.0-1.6 mohm per
# foot per conductor, so a few feet out and back lands near 10 milliohms. Unlike
# the battery path this is not shared between motors.
CHANNEL_RESISTANCE_OHM = 0.010

# Ball inertia factors for I = factor * m * r^2.
HOLLOW_SHELL_INERTIA_FACTOR = 2.0 / 3.0
SOLID_SPHERE_INERTIA_FACTOR = 2.0 / 5.0


@dataclass(frozen=True)
class MotorSpec:
    """A single motor's DC curve, in WPILib's DCMotor convention."""

    name: str
    stall_torque_nm: float
    stall_current_a: float
    free_current_a: float
    free_speed_rad_s: float
    nominal_voltage_v: float = 12.0
    # Rotor inertia is not published by CTRE; this is a mass-properties estimate
    # and contributes well under 2 percent once reflected through the gear pair.
    rotor_inertia_kg_m2: float = 6.0e-5

    @property
    def resistance_ohm(self) -> float:
        return self.nominal_voltage_v / self.stall_current_a

    @property
    def kt_nm_per_a(self) -> float:
        return self.stall_torque_nm / self.stall_current_a

    @property
    def kv_rad_s_per_v(self) -> float:
        return self.free_speed_rad_s / (
            self.nominal_voltage_v - self.resistance_ohm * self.free_current_a
        )


KRAKEN_X60 = MotorSpec("Kraken X60", 7.09, 366.0, 2.0, 6000.0 * RPM_TO_RAD_S)
KRAKEN_X60_FOC = MotorSpec("Kraken X60 (FOC)", 9.37, 483.0, 2.0, 5800.0 * RPM_TO_RAD_S)
MOTOR_CHOICES = {motor.name: motor for motor in (KRAKEN_X60, KRAKEN_X60_FOC)}


@dataclass(frozen=True)
class TubeFlywheel:
    """A hollow cylindrical tube spun about its own axis."""

    outer_diameter_m: float
    length_m: float
    wall_thickness_m: float
    density_kg_m3: float = STEEL_DENSITY_KG_M3

    @property
    def outer_radius_m(self) -> float:
        return self.outer_diameter_m / 2.0

    @property
    def inner_radius_m(self) -> float:
        """Zero once the wall reaches the axis, which makes the tube a solid bar."""
        return max(0.0, self.outer_radius_m - self.wall_thickness_m)

    @property
    def mass_kg(self) -> float:
        annulus = np.pi * (self.outer_radius_m**2 - self.inner_radius_m**2)
        return self.density_kg_m3 * annulus * self.length_m

    @property
    def inertia_kg_m2(self) -> float:
        return 0.5 * self.mass_kg * (self.outer_radius_m**2 + self.inner_radius_m**2)

    def with_wall(self, wall_thickness_m: float) -> "TubeFlywheel":
        return replace(self, wall_thickness_m=wall_thickness_m)

    def surface_speed_m_s(self, rpm: float) -> float:
        return rpm * RPM_TO_RAD_S * self.outer_radius_m

    def rpm_for_surface_speed(self, surface_speed_m_s: float) -> float:
        return surface_speed_m_s / self.outer_radius_m * RAD_S_TO_RPM


@dataclass(frozen=True)
class Drivetrain:
    """Motors, gearing and losses between the motors and the flywheel.

    ``motor_teeth`` drives ``flywheel_teeth``, so the flywheel turns slower than
    the motors whenever the flywheel gear is larger. The resulting ratio is
    motor revolutions per flywheel revolution, and it multiplies torque.
    """

    motor: MotorSpec = KRAKEN_X60
    motor_count: int = 2
    motor_teeth: int = 15
    flywheel_teeth: int = 18
    stator_current_limit_a: float = 60.0
    # Open-circuit bus voltage. What the controller actually sees is lower by
    # the sag across ``battery_resistance_ohm``, which this model accounts for.
    bus_voltage_v: float = 12.0
    battery_resistance_ohm: float = BATTERY_RESISTANCE_OHM
    channel_resistance_ohm: float = CHANNEL_RESISTANCE_OHM
    channel_breaker_a: float = MAX_CHANNEL_BREAKER_A
    main_breaker_a: float = MAIN_BREAKER_A
    gear_efficiency: float = 0.97
    # Bearing and windage loss, quoted at the flywheel's no-load top speed and
    # modelled as viscous (linear in speed) between there and rest.
    drag_torque_at_free_speed_nm: float = 0.05
    extra_inertia_kg_m2: float = 0.0

    @property
    def ratio(self) -> float:
        """Motor revolutions per flywheel revolution."""
        return self.flywheel_teeth / self.motor_teeth

    @property
    def free_speed_rpm(self) -> float:
        """Unloaded flywheel speed, ignoring drag."""
        return self.motor.free_speed_rad_s * RAD_S_TO_RPM / self.ratio

    @property
    def reflected_rotor_inertia_kg_m2(self) -> float:
        return self.motor_count * self.motor.rotor_inertia_kg_m2 * self.ratio**2

    @property
    def drag_coefficient_nm_s(self) -> float:
        free_speed_rad_s = self.motor.free_speed_rad_s / self.ratio
        if free_speed_rad_s <= 0.0:
            return 0.0
        return self.drag_torque_at_free_speed_nm / free_speed_rad_s

    @property
    def internal_friction_torque_nm(self) -> float:
        """Flywheel-side torque lost to the motors' own friction.

        A DC curve's free current is the current the motor draws at free speed,
        which is exactly the current spent overcoming internal friction.
        Subtracting it makes net torque cross zero at the quoted free speed.
        """
        return (
            self.motor_count
            * self.motor.kt_nm_per_a
            * self.motor.free_current_a
            * self.ratio
            * self.gear_efficiency
        )

    @property
    def current_limited_torque_nm(self) -> float:
        """Gross flywheel-side torque on the stator-limited plateau, before losses."""
        return (
            self.motor_count
            * self.motor.kt_nm_per_a
            * self.stator_current_limit_a
            * self.ratio
            * self.gear_efficiency
        )

    @property
    def battery_health(self) -> str:
        """Where this pack sits against the usual beak thresholds."""
        if self.battery_resistance_ohm > BATTERY_SCRAP_RESISTANCE_OHM:
            return "retire"
        if self.battery_resistance_ohm > BATTERY_MATCH_RESISTANCE_OHM:
            return "practice only"
        return "match ready"

    @property
    def effective_resistance_ohm(self) -> float:
        """Everything in series with one motor's windings, as that motor sees it.

        Two different paths, and they do not count the same way. Branch wiring
        and the channel breaker carry only this motor's current, so they add
        once. The battery, its cable and the main breaker are shared, so every
        other motor's draw sags the bus this motor is working against — from one
        motor's point of view that path looks ``motor_count`` times larger.

        Both are linear in current, which keeps the torque curve piecewise
        linear in speed and lets spin-up stay closed-form.
        """
        return (
            self.motor.resistance_ohm
            + self.channel_resistance_ohm
            + self.motor_count * self.battery_resistance_ohm
        )

    def _voltage_limited_line(self) -> tuple[float, float]:
        """Gross flywheel torque as ``intercept - slope * omega`` on the motor curve."""
        motor = self.motor
        resistance = self.effective_resistance_ohm
        common = self.motor_count * motor.kt_nm_per_a * self.ratio * self.gear_efficiency
        intercept = common * self.bus_voltage_v / resistance
        slope = common * self.ratio / (motor.kv_rad_s_per_v * resistance)
        return intercept, slope

    def _accelerating_lines(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """Net accelerating torque as ``p - q * omega`` below and above the knee."""
        intercept, slope = self._voltage_limited_line()
        losses = self.internal_friction_torque_nm
        drag = self.drag_coefficient_nm_s
        plateau = (self.current_limited_torque_nm - losses, drag)
        curve = (intercept - losses, slope + drag)
        return plateau, curve

    def _braking_lines(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """Net braking torque as ``p - q * omega`` above and below stator saturation."""
        intercept, slope = self._voltage_limited_line()
        losses = self.internal_friction_torque_nm
        drag = self.drag_coefficient_nm_s
        saturated = (self.current_limited_torque_nm + losses, -drag)
        unsaturated = (intercept + losses, -(slope + drag))
        return saturated, unsaturated

    @property
    def knee_speed_rad_s(self) -> float:
        """Flywheel speed where the motor curve drops below the current limit."""
        intercept, slope = self._voltage_limited_line()
        if slope <= 0.0:
            return np.inf
        return max(0.0, (intercept - self.current_limited_torque_nm) / slope)

    @property
    def brake_saturation_speed_rad_s(self) -> float:
        """Flywheel speed below which reverse drive can no longer hit the stator limit."""
        motor = self.motor
        return max(
            0.0,
            (self.stator_current_limit_a * motor.resistance_ohm - self.bus_voltage_v)
            * motor.kv_rad_s_per_v
            / self.ratio,
        )

    def accelerating_torque_nm(self, omega_rad_s: float | np.ndarray) -> np.ndarray:
        """Net flywheel torque available to accelerate, after gearing and losses."""
        omega = np.asarray(omega_rad_s, dtype=float)
        (plateau_p, plateau_q), (curve_p, curve_q) = self._accelerating_lines()
        return np.minimum(plateau_p - plateau_q * omega, curve_p - curve_q * omega)

    def braking_torque_nm(self, omega_rad_s: float | np.ndarray) -> np.ndarray:
        """Magnitude of retarding torque when the motors drive against rotation.

        Reversing the bridge puts the supply and the back-EMF in series, so the
        commanded current saturates the stator limit at any usable speed. Drag
        and internal friction add to the braking effort rather than opposing it.
        """
        omega = np.asarray(omega_rad_s, dtype=float)
        (sat_p, sat_q), (unsat_p, unsat_q) = self._braking_lines()
        return np.minimum(sat_p - sat_q * omega, unsat_p - unsat_q * omega)

    def idle_draw(self, idle_rpm: float, match_seconds: float = MATCH_SECONDS) -> "IdleDraw":
        """Battery cost of holding the flywheel at a steady idle speed.

        Holding speed needs almost no torque, but it does need applied voltage
        proportional to speed. Supply power is applied voltage times stator
        current, so idle cost climbs roughly with the square of idle speed even
        though the current barely moves. That is why a high idle is expensive.
        """
        motor = self.motor
        omega_flywheel = idle_rpm * RPM_TO_RAD_S
        omega_motor = omega_flywheel * self.ratio
        holding_torque = self.drag_coefficient_nm_s * omega_flywheel

        denominator = self.motor_count * self.ratio * self.gear_efficiency
        torque_each = holding_torque / denominator if denominator > 0.0 else 0.0
        stator_each = torque_each / motor.kt_nm_per_a + motor.free_current_a
        applied_v = min(self.bus_voltage_v, omega_motor / motor.kv_rad_s_per_v + stator_each * motor.resistance_ohm)
        supply_each = stator_each * applied_v / self.bus_voltage_v if self.bus_voltage_v > 0.0 else 0.0
        power = applied_v * stator_each * self.motor_count
        return IdleDraw(
            idle_rpm=idle_rpm,
            holding_torque_nm=holding_torque,
            stator_current_a=stator_each * self.motor_count,
            supply_current_a=supply_each * self.motor_count,
            power_w=power,
            match_energy_j=power * match_seconds,
        )

    def stator_current_a(self, omega_rad_s: float | np.ndarray) -> np.ndarray:
        """Winding current per motor at full accelerating effort, after sag."""
        omega = np.asarray(omega_rad_s, dtype=float)
        back_emf = omega * self.ratio / self.motor.kv_rad_s_per_v
        available = (self.bus_voltage_v - back_emf) / self.effective_resistance_ohm
        return np.clip(available, 0.0, self.stator_current_limit_a)

    def supply_current_a(self, omega_rad_s: float | np.ndarray) -> np.ndarray:
        """Total battery current at full accelerating effort — what breakers see.

        A motor controller is a switching converter, not a resistor: below the
        current-limit knee it holds winding current at the limit while drawing
        only ``duty`` of it from the battery, so supply current is far below
        stator current at low speed. The two converge at the knee, where duty
        reaches one. Peak battery draw therefore lands at the knee, not at stall
        — which is exactly where a spin-up passes through.
        """
        omega = np.asarray(omega_rad_s, dtype=float)
        motor = self.motor
        stator = self.stator_current_a(omega)
        back_emf = omega * self.ratio / motor.kv_rad_s_per_v

        # Voltage the windings need to carry this current at this speed.
        applied = back_emf + stator * motor.resistance_ohm
        # Sag ahead of the controller rises with the supply current it draws, and
        # that current depends on the sag, so solve the pair rather than iterate:
        # sag_path * i^2 - V_open * i + stator * applied = 0, smaller root.
        sag_path = self.motor_count * self.battery_resistance_ohm + self.channel_resistance_ohm
        product = stator * applied
        if sag_path <= 0.0:
            per_motor = np.divide(
                product, self.bus_voltage_v, out=np.zeros_like(product), where=self.bus_voltage_v > 0.0
            )
        else:
            discriminant = self.bus_voltage_v**2 - 4.0 * product * sag_path
            # A negative discriminant means the supply cannot deliver this at all;
            # clamp to the peak-power point rather than returning a complex root.
            root = np.sqrt(np.maximum(0.0, discriminant))
            per_motor = (self.bus_voltage_v - root) / (2.0 * sag_path)
        # Duty cannot exceed one: past that the motor is simply across the bus.
        return self.motor_count * np.minimum(per_motor, stator)

    def loaded_bus_voltage_v(self, omega_rad_s: float | np.ndarray) -> np.ndarray:
        """Bus voltage after sag while accelerating at this speed."""
        omega = np.asarray(omega_rad_s, dtype=float)
        return self.bus_voltage_v - self.supply_current_a(omega) * self.battery_resistance_ohm

    def max_flywheel_speed_rad_s(self) -> float:
        """Speed where net accelerating torque reaches zero, including all losses."""
        (plateau_p, plateau_q), (curve_p, curve_q) = self._accelerating_lines()
        knee = self.knee_speed_rad_s

        if plateau_q > 0.0:
            plateau_terminal = plateau_p / plateau_q
            if plateau_terminal <= knee:
                return max(0.0, plateau_terminal)
        if curve_q <= 0.0:
            return np.inf
        return max(0.0, curve_p / curve_q)


def _linear_segment_time(inertia: float, intercept: float, slope: float, start: float, end: float) -> float:
    """Time to move from ``start`` to ``end`` under torque ``intercept - slope * omega``.

    Exact for the piecewise-linear torque curve: ``J domega / (p - q omega)``
    integrates to a logarithm, or to a plain quotient when the slope vanishes.
    """
    if end <= start:
        return 0.0
    torque_start = intercept - slope * start
    torque_end = intercept - slope * end
    if torque_start <= 0.0 or torque_end <= 0.0:
        return np.inf
    if abs(slope) < 1e-12:
        return inertia * (end - start) / intercept
    return inertia / slope * np.log(torque_start / torque_end)


def spin_up_time_s(inertia_kg_m2: float, drivetrain: Drivetrain, start_rpm: float, end_rpm: float) -> float:
    """Minimum time to accelerate between two speeds at full available torque.

    This is the physical floor a perfect controller could reach, so it is the
    right number for sizing. Returns infinity when the target sits at or above
    the drivetrain's terminal speed.
    """
    if end_rpm <= start_rpm:
        return 0.0
    start = start_rpm * RPM_TO_RAD_S
    end = end_rpm * RPM_TO_RAD_S
    if end >= drivetrain.max_flywheel_speed_rad_s():
        return np.inf

    (plateau_p, plateau_q), (curve_p, curve_q) = drivetrain._accelerating_lines()
    knee = drivetrain.knee_speed_rad_s

    total = 0.0
    plateau_end = min(end, knee)
    if plateau_end > start:
        total += _linear_segment_time(inertia_kg_m2, plateau_p, plateau_q, start, plateau_end)
    curve_start = max(start, knee)
    if end > curve_start:
        total += _linear_segment_time(inertia_kg_m2, curve_p, curve_q, curve_start, end)
    return total


def spin_down_time_s(inertia_kg_m2: float, drivetrain: Drivetrain, start_rpm: float, end_rpm: float) -> float:
    """Minimum time to decelerate between two speeds under powered braking."""
    if end_rpm >= start_rpm:
        return 0.0
    start = start_rpm * RPM_TO_RAD_S
    end = end_rpm * RPM_TO_RAD_S
    (sat_p, sat_q), (unsat_p, unsat_q) = drivetrain._braking_lines()
    saturation_speed = drivetrain.brake_saturation_speed_rad_s

    # Above the saturation speed the stator limit binds; below it the bridge
    # cannot force the full limit and braking torque tapers with speed. Both
    # segments are integrated over increasing speed, which is why the endpoints
    # are passed low-to-high even though the flywheel is slowing down.
    total = 0.0
    boundary = max(end, min(start, saturation_speed))
    if start > boundary:
        total += _linear_segment_time(inertia_kg_m2, sat_p, sat_q, boundary, start)
    if boundary > end:
        total += _linear_segment_time(inertia_kg_m2, unsat_p, unsat_q, end, boundary)
    return total


@dataclass(frozen=True)
class PowerCheck:
    """Battery and breaker draw over a spin-up, against the FRC power rules."""

    peak_channel_a: float
    peak_total_a: float
    peak_speed_rpm: float
    sagged_bus_v: float
    seconds_over_channel: float
    seconds_over_main: float
    channel_breaker_a: float
    main_breaker_a: float
    drivetrain_allowance_a: float

    @property
    def over_channel(self) -> bool:
        return self.peak_channel_a > self.channel_breaker_a

    @property
    def over_main(self) -> bool:
        return self.peak_total_a + self.drivetrain_allowance_a > self.main_breaker_a

    @property
    def headroom_a(self) -> float:
        """Current left for everything else while the flywheel is at peak draw."""
        return self.main_breaker_a - self.peak_total_a


def check_power_budget(
    inertia_kg_m2: float,
    drivetrain: Drivetrain,
    start_rpm: float,
    end_rpm: float,
    drivetrain_allowance_a: float = 0.0,
    samples: int = 400,
) -> PowerCheck:
    """Peak battery draw over a spin-up, and how long it stays above the breakers.

    Both FRC breakers are thermal, so the number that matters is not just the
    peak but how long it is held: a 40 A breaker passes 60 A for tens of
    seconds, and a flywheel spin-up lasts well under one. Time above rating is
    reported so a brief overshoot can be told apart from a real trip risk.
    """
    if end_rpm <= start_rpm:
        zero_speed = start_rpm * RPM_TO_RAD_S
        return PowerCheck(
            peak_channel_a=float(drivetrain.supply_current_a(zero_speed)) / drivetrain.motor_count,
            peak_total_a=float(drivetrain.supply_current_a(zero_speed)),
            peak_speed_rpm=start_rpm,
            sagged_bus_v=float(drivetrain.loaded_bus_voltage_v(zero_speed)),
            seconds_over_channel=0.0,
            seconds_over_main=0.0,
            channel_breaker_a=drivetrain.channel_breaker_a,
            main_breaker_a=drivetrain.main_breaker_a,
            drivetrain_allowance_a=drivetrain_allowance_a,
        )

    omega = np.linspace(start_rpm * RPM_TO_RAD_S, end_rpm * RPM_TO_RAD_S, samples)
    total = drivetrain.supply_current_a(omega)
    channel = total / drivetrain.motor_count
    peak_index = int(np.argmax(total))

    # dt = J domega / torque, integrated only where the draw exceeds a rating.
    torque = drivetrain.accelerating_torque_nm(omega)
    with np.errstate(divide="ignore", invalid="ignore"):
        dt_domega = np.where(torque > 0.0, inertia_kg_m2 / torque, np.inf)

    def seconds_above(mask: np.ndarray) -> float:
        if not mask.any() or not np.all(np.isfinite(dt_domega[mask])):
            return float("inf") if mask.any() else 0.0
        return float(np.trapezoid(np.where(mask, dt_domega, 0.0), omega))

    return PowerCheck(
        peak_channel_a=float(channel[peak_index]),
        peak_total_a=float(total[peak_index]),
        peak_speed_rpm=float(omega[peak_index] * RAD_S_TO_RPM),
        sagged_bus_v=float(drivetrain.loaded_bus_voltage_v(omega[peak_index])),
        seconds_over_channel=seconds_above(channel > drivetrain.channel_breaker_a),
        seconds_over_main=seconds_above(total + drivetrain_allowance_a > drivetrain.main_breaker_a),
        channel_breaker_a=drivetrain.channel_breaker_a,
        main_breaker_a=drivetrain.main_breaker_a,
        drivetrain_allowance_a=drivetrain_allowance_a,
    )


@dataclass(frozen=True)
class IdleDraw:
    idle_rpm: float
    holding_torque_nm: float
    stator_current_a: float
    supply_current_a: float
    power_w: float
    match_energy_j: float

    @property
    def battery_fraction(self) -> float:
        return self.match_energy_j / BATTERY_ENERGY_J


def drag_torque_from_coast_down(
    inertia_kg_m2: float,
    start_rpm: float,
    end_rpm: float,
    coast_seconds: float,
    free_speed_rpm: float,
) -> float:
    """Fit the viscous drag torque from a measured coast-down.

    Spin the flywheel up, disable the motors in coast (not brake) neutral mode,
    and time the fall between two speeds. Purely viscous drag decays speed
    exponentially, so the coefficient follows from the ratio of the two speeds.
    The result is quoted as torque at free speed to match ``Drivetrain``.

    This is the one number in the model worth measuring rather than assuming:
    idle cost is directly proportional to it.
    """
    if coast_seconds <= 0.0 or end_rpm <= 0.0 or start_rpm <= end_rpm:
        return np.nan
    coefficient = inertia_kg_m2 * np.log(start_rpm / end_rpm) / coast_seconds
    return coefficient * free_speed_rpm * RPM_TO_RAD_S


def spin_up_electrical_energy_j(
    inertia_kg_m2: float,
    drivetrain: Drivetrain,
    start_rpm: float,
    end_rpm: float,
    samples: int = 2000,
) -> float:
    """Energy drawn from the battery to accelerate between two speeds.

    Larger than the kinetic energy gained: the stator-limited part of the climb
    burns a lot in winding resistance. Integrated over speed rather than time,
    using ``dt = J domega / torque``.
    """
    if end_rpm <= start_rpm:
        return 0.0
    if end_rpm >= drivetrain.max_flywheel_speed_rad_s() * RAD_S_TO_RPM:
        return np.inf

    motor = drivetrain.motor
    omega = np.linspace(start_rpm * RPM_TO_RAD_S, end_rpm * RPM_TO_RAD_S, samples)
    omega_motor = omega * drivetrain.ratio

    back_emf_v = omega_motor / motor.kv_rad_s_per_v
    demand_a = (drivetrain.bus_voltage_v - back_emf_v) / motor.resistance_ohm
    stator_each = np.minimum(drivetrain.stator_current_limit_a, np.maximum(0.0, demand_a))
    # Below the knee the controller holds back voltage to respect the current
    # limit; above it the bridge is already wide open at the bus voltage.
    applied_v = np.minimum(drivetrain.bus_voltage_v, back_emf_v + stator_each * motor.resistance_ohm)

    power_w = applied_v * stator_each * drivetrain.motor_count
    torque = drivetrain.accelerating_torque_nm(omega)
    if np.any(torque <= 0.0):
        return np.inf
    return float(np.trapezoid(power_w * inertia_kg_m2 / torque, omega))


@dataclass(frozen=True)
class IdleStrategyComparison:
    low_idle_rpm: float
    high_idle_rpm: float
    low_idle_power_w: float
    high_idle_power_w: float
    extra_idle_energy_j: float
    extra_spin_up_energy_per_volley_j: float
    breakeven_volleys: float
    low_idle_spin_up_s: float
    high_idle_spin_up_s: float


def compare_idle_strategies(
    inertia_kg_m2: float,
    drivetrain: Drivetrain,
    shot_rpm: float,
    low_idle_rpm: float,
    high_idle_rpm: float,
    settle_band_rpm: float = 25.0,
    match_seconds: float = MATCH_SECONDS,
) -> IdleStrategyComparison:
    """Weigh holding a high idle against re-climbing from a low idle every volley.

    A high idle burns power continuously. A low idle instead throws away the
    stored kinetic energy after every shot and buys it back, paying winding
    losses each time. Which wins is purely a question of how many volleys the
    match contains, so the useful output is the break-even count.
    """
    target = max(0.0, shot_rpm - settle_band_rpm)
    low_draw = drivetrain.idle_draw(low_idle_rpm, match_seconds)
    high_draw = drivetrain.idle_draw(high_idle_rpm, match_seconds)

    extra_idle = (high_draw.power_w - low_draw.power_w) * match_seconds
    low_climb = spin_up_electrical_energy_j(inertia_kg_m2, drivetrain, low_idle_rpm, target)
    high_climb = spin_up_electrical_energy_j(inertia_kg_m2, drivetrain, high_idle_rpm, target)
    extra_climb = low_climb - high_climb

    breakeven = extra_idle / extra_climb if extra_climb > 0.0 else np.inf
    return IdleStrategyComparison(
        low_idle_rpm=low_idle_rpm,
        high_idle_rpm=high_idle_rpm,
        low_idle_power_w=low_draw.power_w,
        high_idle_power_w=high_draw.power_w,
        extra_idle_energy_j=extra_idle,
        extra_spin_up_energy_per_volley_j=extra_climb,
        breakeven_volleys=breakeven,
        low_idle_spin_up_s=spin_up_time_s(inertia_kg_m2, drivetrain, low_idle_rpm, target),
        high_idle_spin_up_s=spin_up_time_s(inertia_kg_m2, drivetrain, high_idle_rpm, target),
    )


@dataclass(frozen=True)
class ShotEnergy:
    """Energy a volley pulls out of the flywheel.

    A single wheel working against a static hood accelerates the ball to a
    fraction of the wheel's surface speed while spinning it up backwards. With
    no slip at either contact the ball leaves at half surface speed carrying
    full backspin, so ``exit_speed_ratio`` is bounded above by 0.5 in practice.
    """

    ball_mass_kg: float = 0.215
    ball_diameter_m: float = 0.150
    ball_count: int = 4
    inertia_factor: float = HOLLOW_SHELL_INERTIA_FACTOR
    exit_speed_ratio: float = 0.5
    transfer_efficiency: float = 0.6

    def exit_speed_m_s(self, surface_speed_m_s: float) -> float:
        return self.exit_speed_ratio * surface_speed_m_s

    def energy_per_ball_j(self, surface_speed_m_s: float) -> float:
        """Mechanical energy carried away by one ball, translation plus backspin."""
        speed = self.exit_speed_m_s(surface_speed_m_s)
        radius = self.ball_diameter_m / 2.0
        translational = 0.5 * self.ball_mass_kg * speed**2
        spin_rate = speed / radius if radius > 0.0 else 0.0
        ball_inertia = self.inertia_factor * self.ball_mass_kg * radius**2
        rotational = 0.5 * ball_inertia * spin_rate**2
        return translational + rotational

    def energy_removed_j(self, surface_speed_m_s: float) -> float:
        """Energy leaving the flywheel, including everything lost to slip and hysteresis."""
        if self.transfer_efficiency <= 0.0:
            return np.inf
        return self.ball_count * self.energy_per_ball_j(surface_speed_m_s) / self.transfer_efficiency


@dataclass(frozen=True)
class DroopResult:
    rpm_before: float
    rpm_after: float
    drop_rpm: float
    drop_fraction: float
    energy_removed_j: float
    stored_energy_j: float
    exit_speed_before_m_s: float
    exit_speed_after_m_s: float
    mean_exit_speed_m_s: float
    stalled: bool


def firing_droop(
    inertia_kg_m2: float,
    flywheel: TubeFlywheel,
    energy: ShotEnergy,
    shot_rpm: float,
) -> DroopResult:
    """Speed lost when the volley fires, from the flywheel's energy balance.

    Motor torque is ignored across the event on purpose: ball contact lasts a
    few milliseconds, over which the motors contribute a couple of percent of
    the momentum change. Ignoring it is both simpler and conservative.
    """
    omega_before = shot_rpm * RPM_TO_RAD_S
    stored = 0.5 * inertia_kg_m2 * omega_before**2
    removed = energy.energy_removed_j(flywheel.surface_speed_m_s(shot_rpm))

    stalled = removed >= stored
    remaining = max(0.0, stored - removed)
    omega_after = np.sqrt(2.0 * remaining / inertia_kg_m2) if inertia_kg_m2 > 0.0 else 0.0
    rpm_after = omega_after * RAD_S_TO_RPM

    exit_before = energy.exit_speed_m_s(flywheel.surface_speed_m_s(shot_rpm))
    exit_after = energy.exit_speed_m_s(flywheel.surface_speed_m_s(rpm_after))
    return DroopResult(
        rpm_before=shot_rpm,
        rpm_after=rpm_after,
        drop_rpm=shot_rpm - rpm_after,
        drop_fraction=(shot_rpm - rpm_after) / shot_rpm if shot_rpm > 0.0 else 0.0,
        energy_removed_j=removed,
        stored_energy_j=stored,
        exit_speed_before_m_s=exit_before,
        exit_speed_after_m_s=exit_after,
        mean_exit_speed_m_s=0.5 * (exit_before + exit_after),
        stalled=stalled,
    )


def minimum_inertia_for_drop(
    flywheel: TubeFlywheel,
    energy: ShotEnergy,
    shot_rpm: float,
    max_drop_rpm: float,
) -> float:
    """Smallest inertia whose firing droop stays inside ``max_drop_rpm``.

    Solved directly from the energy balance rather than searched: holding the
    drop to a fraction ``f`` of shot speed requires ``2E / (omega^2 (2f - f^2))``.
    """
    if shot_rpm <= 0.0 or max_drop_rpm <= 0.0:
        return np.inf
    fraction = min(1.0, max_drop_rpm / shot_rpm)
    omega = shot_rpm * RPM_TO_RAD_S
    denominator = (2.0 * fraction - fraction**2) * omega**2
    if denominator <= 0.0:
        return np.inf
    removed = energy.energy_removed_j(flywheel.surface_speed_m_s(shot_rpm))
    return 2.0 * removed / denominator


def wall_thickness_for_inertia(flywheel: TubeFlywheel, target_inertia_kg_m2: float) -> float:
    """Invert the tube inertia relation for wall thickness, in metres.

    Inertia rises monotonically with wall thickness, so a bisection on the
    interval from zero wall to a solid bar is both safe and exact enough.
    """
    if not np.isfinite(target_inertia_kg_m2) or target_inertia_kg_m2 <= 0.0:
        return np.nan
    solid_wall = flywheel.outer_radius_m
    if flywheel.with_wall(solid_wall).inertia_kg_m2 < target_inertia_kg_m2:
        return np.nan

    low, high = 0.0, solid_wall
    for _ in range(80):
        mid = 0.5 * (low + high)
        if flywheel.with_wall(mid).inertia_kg_m2 < target_inertia_kg_m2:
            low = mid
        else:
            high = mid
    return 0.5 * (low + high)


def balanced_idle_rpm(
    inertia_kg_m2: float,
    drivetrain: Drivetrain,
    low_shot_rpm: float,
    high_shot_rpm: float,
    settle_band_rpm: float = 25.0,
) -> float:
    """Idle speed that minimises the worst-case move to either end of the shot range.

    Spin-up and powered spin-down are symmetric below the current-limit knee but
    diverge above it, where braking keeps the full stator limit while
    accelerating torque falls off. The balance point therefore sits above the
    midpoint of the shot range, and it is found where the two transition times
    are equal.
    """
    if high_shot_rpm <= low_shot_rpm:
        return low_shot_rpm

    def imbalance(idle_rpm: float) -> float:
        up = spin_up_time_s(inertia_kg_m2, drivetrain, idle_rpm, high_shot_rpm - settle_band_rpm)
        down = spin_down_time_s(inertia_kg_m2, drivetrain, idle_rpm, low_shot_rpm + settle_band_rpm)
        if not np.isfinite(up):
            return 1.0
        return up - down

    low, high = low_shot_rpm, high_shot_rpm
    if imbalance(high) > 0.0:
        # Even idling at the top of the range cannot reach the high shot; the
        # best available choice is the highest idle allowed.
        return high
    for _ in range(60):
        mid = 0.5 * (low + high)
        if imbalance(mid) > 0.0:
            low = mid
        else:
            high = mid
    return 0.5 * (low + high)


@dataclass(frozen=True)
class WallSweepResult:
    wall_thickness_m: np.ndarray
    inertia_kg_m2: np.ndarray
    tube_mass_kg: np.ndarray
    spin_up_s: np.ndarray
    recovery_s: np.ndarray
    drop_rpm: np.ndarray
    mean_exit_speed_m_s: np.ndarray
    feasible_mask: np.ndarray
    minimum_feasible_wall_m: float
    minimum_feasible_inertia_kg_m2: float
    shot_rpm: float
    idle_rpm: float
    max_shot_rpm_reachable: float
    shot_rpm_reachable: bool


def sweep_wall_thickness(
    flywheel: TubeFlywheel,
    drivetrain: Drivetrain,
    energy: ShotEnergy,
    shot_rpm: float,
    idle_rpm: float,
    max_drop_rpm: float,
    settle_band_rpm: float = 25.0,
    wall_range_m: tuple[float, float] = (0.020 * INCH_TO_METER, 0.375 * INCH_TO_METER),
    samples: int = 260,
) -> WallSweepResult:
    """Sweep wall thickness and evaluate spin-up, recovery and droop at each point."""
    walls = np.linspace(wall_range_m[0], wall_range_m[1], samples)
    inertia = np.empty_like(walls)
    mass = np.empty_like(walls)
    spin_up = np.empty_like(walls)
    recovery = np.empty_like(walls)
    drop = np.empty_like(walls)
    mean_exit = np.empty_like(walls)

    rotating_extra = drivetrain.extra_inertia_kg_m2 + drivetrain.reflected_rotor_inertia_kg_m2
    settle_target = max(0.0, shot_rpm - settle_band_rpm)

    for index, wall in enumerate(walls):
        tube = flywheel.with_wall(wall)
        total_inertia = tube.inertia_kg_m2 + rotating_extra
        inertia[index] = total_inertia
        mass[index] = tube.mass_kg

        droop = firing_droop(total_inertia, tube, energy, shot_rpm)
        drop[index] = droop.drop_rpm
        mean_exit[index] = droop.mean_exit_speed_m_s
        spin_up[index] = spin_up_time_s(total_inertia, drivetrain, idle_rpm, settle_target)
        recovery[index] = spin_up_time_s(total_inertia, drivetrain, droop.rpm_after, settle_target)

    feasible = drop <= max_drop_rpm
    required_inertia = minimum_inertia_for_drop(flywheel, energy, shot_rpm, max_drop_rpm)
    required_tube_inertia = max(0.0, required_inertia - rotating_extra)
    minimum_wall = wall_thickness_for_inertia(flywheel, required_tube_inertia)

    max_speed_rpm = drivetrain.max_flywheel_speed_rad_s() * RAD_S_TO_RPM
    return WallSweepResult(
        wall_thickness_m=walls,
        inertia_kg_m2=inertia,
        tube_mass_kg=mass,
        spin_up_s=spin_up,
        recovery_s=recovery,
        drop_rpm=drop,
        mean_exit_speed_m_s=mean_exit,
        feasible_mask=feasible,
        minimum_feasible_wall_m=minimum_wall,
        minimum_feasible_inertia_kg_m2=required_inertia,
        shot_rpm=shot_rpm,
        idle_rpm=idle_rpm,
        max_shot_rpm_reachable=max_speed_rpm,
        shot_rpm_reachable=settle_target < max_speed_rpm,
    )


# --- Shot schedule: what the shot calculator actually demands of the flywheel ---
#
# Wall thickness is tuned against two numbers that only the trajectory model can
# supply: the speed each shot distance needs, and how far that speed may wander
# before the ball stops going in. Both come from running the existing shot
# calculator over the mechanism, not from a tolerance picked by hand.

# Trajectory resolution used for the band search. Coarser than the app's display
# integration because this runs hundreds of times; the band edges move by well
# under a RPM between these settings and the finer default.
_BAND_DT_S = 0.006
_BAND_MAX_TIME_S = 2.5


def tube_shooter_model(
    flywheel: TubeFlywheel,
    energy: "ShotEnergy",
    release_height_m: float,
    robot_velocity_x_m_s: float = 0.0,
    hood_offset_deg: float = 0.0,
) -> ShooterModel:
    """Express the tube-and-static-hood mechanism in ``ShooterModel`` terms.

    ``release_state`` builds exit speed from the mean of the two wheel surface
    speeds, so a lone powered wheel needs ``velocity_transfer = 2 * ratio`` to
    land on the intended fraction of surface speed.

    ``spin_transfer`` is negative on purpose. The dual-wheel sign convention
    treats backspin as the bottom wheel outrunning the top, which assumes the
    idle surface sits opposite ours. Here the ball rolls along a static hood on
    the far side of the tube, so it leaves with backspin of ``v / r`` and the
    sign has to be flipped to say so.
    """
    return ShooterModel(
        mode="fixed_hood",
        top_wheel_diameter_m=flywheel.outer_diameter_m,
        velocity_transfer=2.0 * energy.exit_speed_ratio,
        spin_transfer=-2.0 * energy.exit_speed_ratio,
        release_height_m=release_height_m,
        robot_velocity_x_m_s=robot_velocity_x_m_s,
        hood_offset_deg=hood_offset_deg,
    )


def _scores(rpm: float, hood_deg: float, shooter, ball, environment, target) -> bool:
    controls = ShotControls(float(rpm), 0.0, float(hood_deg))
    trajectory = simulate_shot(
        controls, ball, shooter, environment, target, dt_s=_BAND_DT_S, max_time_s=_BAND_MAX_TIME_S
    )
    return _is_scoring_entry(hub_entry_metrics(trajectory, target, ball), target)


def _plane_crossing_distance(
    rpm: float, hood_deg: float, shooter, ball, environment, target
) -> float:
    """Range at which the falling ball centre crosses the scoring plane.

    Unlike "does it score", this is monotonic in speed, which is what makes the
    band searchable by bisection. A shot that never climbs to the plane counts
    as infinitely short.
    """
    controls = ShotControls(float(rpm), 0.0, float(hood_deg))
    trajectory = simulate_shot(
        controls, ball, shooter, environment, target, dt_s=_BAND_DT_S, max_time_s=_BAND_MAX_TIME_S
    )
    height = target.center_height_m
    heights, ranges = trajectory.z_m, trajectory.x_m
    for index in range(len(heights) - 1):
        upper, lower = float(heights[index]), float(heights[index + 1])
        if upper >= height > lower:
            span = upper - lower
            weight = (upper - height) / span if span else 0.0
            return float(ranges[index] + weight * (ranges[index + 1] - ranges[index]))
    return -np.inf


def _bisect_edge(
    inside: float, outside: float, hood_deg, shooter, ball, environment, target, steps: int
) -> float:
    """Walk the boundary between a scoring speed and a missing one."""
    for _ in range(steps):
        middle = 0.5 * (inside + outside)
        if _scores(middle, hood_deg, shooter, ball, environment, target):
            inside = middle
        else:
            outside = middle
    return inside


def scoring_rpm_band(
    hood_deg: float,
    shooter: ShooterModel,
    ball: BallSpec,
    environment: Environment,
    target: Target,
    rpm_bounds: tuple[float, float],
    refine_steps: int = 12,
) -> tuple[float, float] | None:
    """Widest run of flywheel speeds that still scores at this distance.

    Bisects for the speed that lands on the target, then walks outward to both
    edges of the scoring region. A previous version scanned a coarse speed grid
    looking for any hit, which silently missed whole bands: the grid step was
    around 345 RPM while the bands are only 235-350 RPM wide, so a perfectly
    good band could fall between two samples and the distance would be reported
    as unreachable. Bisecting on a monotonic quantity cannot miss.
    """
    low, high = rpm_bounds
    if _plane_crossing_distance(high, hood_deg, shooter, ball, environment, target) < target.distance_m:
        return None  # even wide open, the shot falls short of this distance

    lower, upper = low, high
    for _ in range(refine_steps):
        middle = 0.5 * (lower + upper)
        if _plane_crossing_distance(middle, hood_deg, shooter, ball, environment, target) < target.distance_m:
            lower = middle
        else:
            upper = middle
    nominal = 0.5 * (lower + upper)

    if not _scores(nominal, hood_deg, shooter, ball, environment, target):
        return None  # lands at the right range but enters too flat or clips the rim

    band_low = _bisect_edge(nominal, low, hood_deg, shooter, ball, environment, target, refine_steps)
    band_high = _bisect_edge(nominal, high, hood_deg, shooter, ball, environment, target, refine_steps)
    return (band_low, band_high) if band_high > band_low else None


@dataclass(frozen=True)
class ShotRequirement:
    distance_m: float
    band_low_rpm: float
    band_high_rpm: float

    @property
    def band_width_rpm(self) -> float:
        return self.band_high_rpm - self.band_low_rpm

    @property
    def band_center_rpm(self) -> float:
        return 0.5 * (self.band_low_rpm + self.band_high_rpm)


@dataclass(frozen=True)
class ShotSchedule:
    hood_deg: float
    requirements: tuple[ShotRequirement, ...]
    distances_m: tuple[float, ...]
    reachable_fraction: float

    @property
    def command_span_rpm(self) -> tuple[float, float]:
        if not self.requirements:
            return (0.0, 0.0)
        centers = [r.band_center_rpm for r in self.requirements]
        return (min(centers), max(centers))


def shot_schedule_candidates(
    shooter: ShooterModel,
    ball: BallSpec,
    environment: Environment,
    target: Target,
    *,
    distances_m: np.ndarray,
    hood_candidates_deg: np.ndarray,
    rpm_bounds: tuple[float, float],
) -> tuple[ShotSchedule, ...]:
    """Map every distance to its scoring speed band, once per candidate hood angle."""
    schedules: list[ShotSchedule] = []
    for hood_deg in hood_candidates_deg:
        requirements: list[ShotRequirement] = []
        for distance in distances_m:
            band = scoring_rpm_band(
                float(hood_deg),
                shooter,
                ball,
                environment,
                replace(target, distance_m=float(distance)),
                rpm_bounds,
            )
            if band is not None:
                requirements.append(ShotRequirement(float(distance), band[0], band[1]))
        schedules.append(
            ShotSchedule(
                hood_deg=float(hood_deg),
                requirements=tuple(requirements),
                distances_m=tuple(float(d) for d in distances_m),
                reachable_fraction=len(requirements) / len(distances_m) if len(distances_m) else 0.0,
            )
        )
    return tuple(schedules)


@dataclass(frozen=True)
class ScheduleSweepResult:
    wall_thickness_m: np.ndarray
    inertia_kg_m2: np.ndarray
    tube_mass_kg: np.ndarray
    in_range_fraction: np.ndarray
    mean_transition_s: np.ndarray
    mean_volley_s: np.ndarray
    hopper_seconds: np.ndarray
    best_idle_rpm: np.ndarray
    schedule: ShotSchedule
    hopper_balls: int
    volleys: int


def in_band_fraction(droop_rpm: float, band_width_rpm: float) -> float:
    """Share of a volley's balls that leave inside the scoring band.

    Balls do not all leave at the commanded speed. The flywheel decays while
    they are in contact, so their exit speeds spread across the droop. Centring
    the command in the band puts that spread either side of ideal, and the
    balls that fall outside the band edges are the ones that miss.

    This is deliberately not all-or-nothing. However far the flywheel sags, the
    balls leaving early are still at a scoring speed, so the fraction falls off
    as ``band / droop`` and never reaches zero.
    """
    if droop_rpm <= 0.0:
        return 1.0
    if band_width_rpm <= 0.0:
        return 0.0
    return float(min(1.0, band_width_rpm / droop_rpm))


def _mean_transition_s(
    inertia_kg_m2: float,
    drivetrain: Drivetrain,
    command_rpm: np.ndarray,
    idle_rpm: float,
    settle_band_rpm: float,
) -> float:
    """Average time to get from idle to each shot's commanded speed.

    Shots below idle are reached by braking down, shots above it by accelerating
    up, so this is the honest cost of being ready rather than spin-up alone.
    """
    total = 0.0
    for target_rpm in command_rpm:
        if target_rpm >= idle_rpm:
            step = spin_up_time_s(inertia_kg_m2, drivetrain, idle_rpm, max(0.0, target_rpm - settle_band_rpm))
        else:
            step = spin_down_time_s(inertia_kg_m2, drivetrain, idle_rpm, target_rpm + settle_band_rpm)
        if not np.isfinite(step):
            return np.inf
        total += step
    return total / len(command_rpm) if len(command_rpm) else np.inf


def best_idle_for_schedule(
    inertia_kg_m2: float,
    drivetrain: Drivetrain,
    command_rpm: np.ndarray,
    settle_band_rpm: float = 25.0,
    samples: int = 60,
) -> tuple[float, float]:
    """Idle speed that minimises the average time to reach a shot.

    Scanned rather than solved: the average of piecewise-logarithmic times is
    not convex in general, and the closed-form times make a scan cheap.
    """
    if not len(command_rpm):
        return 0.0, np.inf
    candidates = np.linspace(0.0, float(np.max(command_rpm)), samples)
    times = [_mean_transition_s(inertia_kg_m2, drivetrain, command_rpm, idle, settle_band_rpm) for idle in candidates]
    best = int(np.argmin(times))
    return float(candidates[best]), float(times[best])


def _volley_plan(
    inertia_kg_m2: float,
    tube: TubeFlywheel,
    energy: "ShotEnergy",
    requirement: ShotRequirement,
) -> tuple[float, float, float]:
    """Command speed, droop and in-band share for one distance.

    Command sits half a droop above the band centre so the excursion straddles
    the ideal speed. That shifts the droop itself, so the command is settled by
    a short fixed-point pass rather than assumed.
    """
    command = requirement.band_center_rpm
    droop = 0.0
    for _ in range(3):
        droop = firing_droop(inertia_kg_m2, tube, energy, command).drop_rpm
        command = requirement.band_center_rpm + droop / 2.0
    return command, droop, in_band_fraction(droop, requirement.band_width_rpm)


def sweep_wall_against_schedule(
    flywheel: TubeFlywheel,
    drivetrain: Drivetrain,
    energy: "ShotEnergy",
    schedule: ShotSchedule,
    *,
    settle_band_rpm: float = 25.0,
    hopper_balls: int = 60,
    wall_range_m: tuple[float, float] = (0.020 * INCH_TO_METER, 0.375 * INCH_TO_METER),
    samples: int = 90,
) -> ScheduleSweepResult:
    """Score every wall thickness on in-band share and time per volley.

    Emptying a full hopper takes many volleys, so the cost that matters is not
    the one-off climb from idle but the recovery between volleys, repeated. Both
    are folded into a single average seconds-per-volley, which collapses to the
    plain spin-up time when the hopper holds a single volley.
    """
    walls = np.linspace(wall_range_m[0], wall_range_m[1], samples)
    rotating_extra = drivetrain.extra_inertia_kg_m2 + drivetrain.reflected_rotor_inertia_kg_m2
    requirements = schedule.requirements
    total_shots = len(schedule.distances_m)
    volleys = max(1, int(np.ceil(hopper_balls / max(1, energy.ball_count))))

    inertia = np.empty_like(walls)
    mass = np.empty_like(walls)
    in_range = np.empty_like(walls)
    mean_time = np.empty_like(walls)
    mean_volley = np.empty_like(walls)
    hopper_time = np.empty_like(walls)
    idles = np.empty_like(walls)

    for index, wall in enumerate(walls):
        tube = flywheel.with_wall(wall)
        total_inertia = tube.inertia_kg_m2 + rotating_extra
        inertia[index] = total_inertia
        mass[index] = tube.mass_kg

        commands: list[float] = []
        recoveries: list[float] = []
        in_band_total = 0.0
        for requirement in requirements:
            command, droop, share = _volley_plan(total_inertia, tube, energy, requirement)
            commands.append(command)
            in_band_total += share
            recoveries.append(
                spin_up_time_s(
                    total_inertia, drivetrain, command - droop, max(0.0, command - settle_band_rpm)
                )
            )

        # Distances the mechanism cannot score at all still count against the
        # hopper: those balls are fired and missed.
        in_range[index] = in_band_total / total_shots if total_shots else 0.0
        idle, mean_s = best_idle_for_schedule(
            total_inertia, drivetrain, np.array(commands), settle_band_rpm
        )
        idles[index] = idle
        mean_time[index] = mean_s

        # A single-volley hopper costs exactly the climb from idle, with no
        # recovery to follow. Guard it explicitly so an unreachable recovery
        # cannot turn into a nan by being multiplied by zero.
        recovery = float(np.mean(recoveries)) if recoveries else np.inf
        hopper_time[index] = mean_s if volleys == 1 else mean_s + (volleys - 1) * recovery
        mean_volley[index] = hopper_time[index] / volleys

    return ScheduleSweepResult(
        wall_thickness_m=walls,
        inertia_kg_m2=inertia,
        tube_mass_kg=mass,
        in_range_fraction=in_range,
        mean_transition_s=mean_time,
        mean_volley_s=mean_volley,
        hopper_seconds=hopper_time,
        best_idle_rpm=idles,
        schedule=schedule,
        hopper_balls=hopper_balls,
        volleys=volleys,
    )


@dataclass(frozen=True)
class WallTuning:
    """The chosen static hood, its sweep, and the wall thickness that wins."""

    schedule: ShotSchedule
    sweep: ScheduleSweepResult
    best_wall_m: float
    best_mean_transition_s: float
    best_mean_volley_s: float
    best_hopper_seconds: float
    best_in_range_fraction: float
    best_idle_rpm: float
    rejected_hoods_deg: tuple[float, ...]

    @property
    def balls_in_band(self) -> float:
        return self.best_in_range_fraction * self.sweep.hopper_balls


def _best_feasible_index(sweep: ScheduleSweepResult, target_fraction: float) -> int | None:
    """Thinnest wall that puts the target share of balls inside the scoring band.

    Deliberately not an argmin over time. Emptying a deep hopper is paced by
    recovery between volleys, and recovery is set by the energy the balls take
    against the power the motors supply, not by inertia. Time per volley is
    therefore near-flat in wall thickness, and what little slope it has runs the
    *same* way as the constraint: a heavier wheel droops less, so it also
    recovers marginally sooner. Minimising time alone would run away to the
    thickest tube on offer for a saving under a millisecond a volley.

    The constraint is what genuinely bounds the problem, so the answer is the
    lightest wheel that satisfies it. Everything thicker is more mass for no
    scoring gain.
    """
    feasible = np.flatnonzero(
        (sweep.in_range_fraction >= target_fraction - 1e-9) & np.isfinite(sweep.mean_volley_s)
    )
    if feasible.size:
        return int(feasible[0])
    usable = np.flatnonzero(np.isfinite(sweep.mean_volley_s))
    if not usable.size:
        return None
    return int(usable[np.argmax(sweep.in_range_fraction[usable])])


def tune_wall_thickness(
    flywheel: TubeFlywheel,
    drivetrain: Drivetrain,
    energy: "ShotEnergy",
    schedules: tuple[ShotSchedule, ...],
    *,
    target_in_range_fraction: float = 1.0,
    settle_band_rpm: float = 25.0,
    hopper_balls: int = 60,
    wall_range_m: tuple[float, float] = (0.020 * INCH_TO_METER, 0.375 * INCH_TO_METER),
    samples: int = 90,
) -> WallTuning:
    """Choose the static hood and wall thickness that minimise average readiness time.

    The hood is a design decision, not a knob, so it is settled here against the
    same objective rather than by a separate rule: sweep wall thickness under
    every candidate hood, keep the walls that hit the required share of shots,
    and take the pairing with the lowest average time to reach shot speed.
    """
    scored: list[tuple[tuple[float, float], ShotSchedule, ScheduleSweepResult, int]] = []
    for schedule in schedules:
        if not schedule.requirements:
            continue
        sweep = sweep_wall_against_schedule(
            flywheel,
            drivetrain,
            energy,
            schedule,
            settle_band_rpm=settle_band_rpm,
            hopper_balls=hopper_balls,
            wall_range_m=wall_range_m,
            samples=samples,
        )
        index = _best_feasible_index(sweep, target_in_range_fraction)
        if index is None:
            continue
        # Rank on hit rate first so a hood that cannot cover the range never wins
        # on speed alone, then on the objective itself.
        key = (-float(sweep.in_range_fraction[index]), float(sweep.mean_volley_s[index]))
        scored.append((key, schedule, sweep, index))

    if not scored:
        raise ValueError("no hood angle scores any shot in the sampled range")

    scored.sort(key=lambda entry: entry[0])
    _, schedule, sweep, index = scored[0]
    rejected = [entry[1].hood_deg for entry in scored[1:]]
    return WallTuning(
        schedule=schedule,
        sweep=sweep,
        best_wall_m=float(sweep.wall_thickness_m[index]),
        best_mean_transition_s=float(sweep.mean_transition_s[index]),
        best_mean_volley_s=float(sweep.mean_volley_s[index]),
        best_hopper_seconds=float(sweep.hopper_seconds[index]),
        best_in_range_fraction=float(sweep.in_range_fraction[index]),
        best_idle_rpm=float(sweep.best_idle_rpm[index]),
        rejected_hoods_deg=tuple(sorted(rejected)),
    )
