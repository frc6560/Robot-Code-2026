from dataclasses import replace

import numpy as np
import pytest

from shotlab.models import BallSpec, Environment, ShotControls, Target
from shotlab.physics import _is_scoring_entry, hub_entry_metrics, release_state, simulate_shot
from shotlab.robot_profile import (
    BALL_DIAMETER_M,
    BALL_MASS_KG,
    HUB_BALL_CENTER_HEIGHT_M,
    RELEASE_HEIGHT_M,
    SHOT_DISTANCE_MAX_M,
    SHOT_DISTANCE_MIN_M,
)

from shotlab.flywheel import (
    HOLLOW_SHELL_INERTIA_FACTOR,
    INCH_TO_METER,
    KRAKEN_X60,
    RAD_S_TO_RPM,
    RPM_TO_RAD_S,
    STEEL_DENSITY_KG_M3,
    Drivetrain,
    ShotEnergy,
    TubeFlywheel,
    balanced_idle_rpm,
    best_idle_for_schedule,
    compare_idle_strategies,
    drag_torque_from_coast_down,
    firing_droop,
    in_band_fraction,
    minimum_inertia_for_drop,
    spin_down_time_s,
    spin_up_electrical_energy_j,
    spin_up_time_s,
    shot_schedule_candidates,
    scoring_rpm_band,
    sweep_wall_against_schedule,
    sweep_wall_thickness,
    tube_shooter_model,
    tune_wall_thickness,
    wall_thickness_for_inertia,
)


def team_tube(wall_in: float = 0.125) -> TubeFlywheel:
    """The mechanism as specified: 4 in OD steel tube, 26 in long."""
    return TubeFlywheel(4.0 * INCH_TO_METER, 26.0 * INCH_TO_METER, wall_in * INCH_TO_METER)


def integrate_speed_change(inertia, drivetrain, start_rpm, end_rpm, step_s=2.0e-6):
    """Reference forward-Euler integration used to check the closed-form times."""
    omega = start_rpm * RPM_TO_RAD_S
    target = end_rpm * RPM_TO_RAD_S
    elapsed = 0.0
    accelerating = end_rpm > start_rpm
    while elapsed < 60.0:
        if accelerating and omega >= target:
            break
        if not accelerating and omega <= target:
            break
        if accelerating:
            omega += float(drivetrain.accelerating_torque_nm(omega)) / inertia * step_s
        else:
            omega -= float(drivetrain.braking_torque_nm(omega)) / inertia * step_s
        elapsed += step_s
    return elapsed


def test_tube_mass_and_inertia_match_hand_calculation():
    tube = team_tube(0.250)
    outer_r, inner_r = 0.0508, 0.0508 - 0.250 * INCH_TO_METER
    expected_mass = STEEL_DENSITY_KG_M3 * np.pi * (outer_r**2 - inner_r**2) * 26.0 * INCH_TO_METER
    np.testing.assert_allclose(tube.mass_kg, expected_mass, rtol=1e-12)
    np.testing.assert_allclose(tube.inertia_kg_m2, 0.5 * expected_mass * (outer_r**2 + inner_r**2), rtol=1e-12)


def test_wall_reaching_the_axis_gives_a_solid_bar():
    tube = team_tube().with_wall(0.0508)
    assert tube.inner_radius_m == 0.0
    np.testing.assert_allclose(tube.inertia_kg_m2, 0.5 * tube.mass_kg * 0.0508**2, rtol=1e-12)


def test_wall_thicker_than_the_radius_does_not_overshoot_the_solid_bar():
    solid = team_tube().with_wall(0.0508)
    overfull = team_tube().with_wall(0.20)
    np.testing.assert_allclose(overfull.inertia_kg_m2, solid.inertia_kg_m2, rtol=1e-12)


def test_inertia_increases_monotonically_with_wall_thickness():
    walls = np.linspace(0.001, 0.05, 60)
    inertia = np.array([team_tube().with_wall(w).inertia_kg_m2 for w in walls])
    assert np.all(np.diff(inertia) > 0.0)


def test_fifteen_to_eighteen_gearing_slows_the_flywheel():
    drivetrain = Drivetrain(motor_teeth=15, flywheel_teeth=18)
    np.testing.assert_allclose(drivetrain.ratio, 1.2)
    np.testing.assert_allclose(drivetrain.free_speed_rpm, 5000.0, rtol=1e-9)


def test_net_torque_crosses_zero_at_the_datasheet_free_speed():
    drivetrain = Drivetrain(drag_torque_at_free_speed_nm=0.0, gear_efficiency=1.0)
    terminal_rpm = drivetrain.max_flywheel_speed_rad_s() * RAD_S_TO_RPM
    np.testing.assert_allclose(terminal_rpm, KRAKEN_X60.free_speed_rad_s * RAD_S_TO_RPM / 1.2, rtol=1e-9)


def test_drag_pulls_the_terminal_speed_below_free_speed():
    with_drag = Drivetrain(drag_torque_at_free_speed_nm=0.05)
    assert with_drag.max_flywheel_speed_rad_s() * RAD_S_TO_RPM < with_drag.free_speed_rpm


def test_current_limit_binds_below_the_knee_only():
    drivetrain = Drivetrain(stator_current_limit_a=60.0)
    knee_rpm = drivetrain.knee_speed_rad_s * RAD_S_TO_RPM
    assert 0.0 < knee_rpm < drivetrain.free_speed_rpm
    plateau = drivetrain.current_limited_torque_nm - drivetrain.internal_friction_torque_nm
    np.testing.assert_allclose(drivetrain.accelerating_torque_nm(0.0), plateau, rtol=1e-12)
    assert drivetrain.accelerating_torque_nm((knee_rpm + 200.0) * RPM_TO_RAD_S) < plateau


@pytest.mark.parametrize(
    ("wall_in", "start_rpm", "end_rpm"),
    [(0.125, 0.0, 3000.0), (0.125, 2500.0, 4000.0), (0.065, 1000.0, 4400.0), (0.250, 3000.0, 4600.0)],
)
def test_closed_form_spin_up_matches_numeric_integration(wall_in, start_rpm, end_rpm):
    drivetrain = Drivetrain()
    inertia = team_tube(wall_in).inertia_kg_m2 + drivetrain.reflected_rotor_inertia_kg_m2
    closed_form = spin_up_time_s(inertia, drivetrain, start_rpm, end_rpm)
    np.testing.assert_allclose(
        closed_form, integrate_speed_change(inertia, drivetrain, start_rpm, end_rpm), rtol=2e-4
    )


@pytest.mark.parametrize(("wall_in", "start_rpm", "end_rpm"), [(0.125, 4500.0, 2500.0), (0.250, 4800.0, 200.0)])
def test_closed_form_spin_down_matches_numeric_integration(wall_in, start_rpm, end_rpm):
    drivetrain = Drivetrain()
    inertia = team_tube(wall_in).inertia_kg_m2 + drivetrain.reflected_rotor_inertia_kg_m2
    closed_form = spin_down_time_s(inertia, drivetrain, start_rpm, end_rpm)
    np.testing.assert_allclose(
        closed_form, integrate_speed_change(inertia, drivetrain, start_rpm, end_rpm), rtol=2e-4
    )


def test_spin_up_to_an_unreachable_target_is_infinite():
    drivetrain = Drivetrain()
    inertia = team_tube().inertia_kg_m2
    unreachable = drivetrain.max_flywheel_speed_rad_s() * RAD_S_TO_RPM + 1.0
    assert not np.isfinite(spin_up_time_s(inertia, drivetrain, 0.0, unreachable))


def test_spin_up_time_grows_with_inertia():
    drivetrain = Drivetrain()
    thin = spin_up_time_s(team_tube(0.049).inertia_kg_m2, drivetrain, 3000.0, 4300.0)
    thick = spin_up_time_s(team_tube(0.250).inertia_kg_m2, drivetrain, 3000.0, 4300.0)
    assert thick > thin


def test_droop_conserves_the_energy_taken_by_the_balls():
    tube = team_tube()
    energy = ShotEnergy()
    result = firing_droop(tube.inertia_kg_m2, tube, energy, 4400.0)
    lost = 0.5 * tube.inertia_kg_m2 * (
        (result.rpm_before * RPM_TO_RAD_S) ** 2 - (result.rpm_after * RPM_TO_RAD_S) ** 2
    )
    np.testing.assert_allclose(lost, result.energy_removed_j, rtol=1e-10)


def test_ball_energy_counts_translation_and_backspin():
    energy = ShotEnergy(ball_count=1, transfer_efficiency=1.0, inertia_factor=HOLLOW_SHELL_INERTIA_FACTOR)
    surface_speed = 22.0
    exit_speed = energy.exit_speed_m_s(surface_speed)
    radius = energy.ball_diameter_m / 2.0
    translational = 0.5 * energy.ball_mass_kg * exit_speed**2
    rotational = 0.5 * (HOLLOW_SHELL_INERTIA_FACTOR * energy.ball_mass_kg * radius**2) * (exit_speed / radius) ** 2
    np.testing.assert_allclose(energy.energy_per_ball_j(surface_speed), translational + rotational, rtol=1e-12)
    # Backspin is not a rounding correction; it is a large share of the budget.
    assert rotational / (translational + rotational) > 0.3


def test_droop_shrinks_as_the_wall_thickens():
    energy = ShotEnergy()
    drops = [firing_droop(team_tube(w).inertia_kg_m2, team_tube(w), energy, 4400.0).drop_rpm for w in (0.049, 0.125, 0.250)]
    assert drops[0] > drops[1] > drops[2]


def test_minimum_inertia_for_drop_lands_exactly_on_the_limit():
    tube = team_tube()
    energy = ShotEnergy()
    required = minimum_inertia_for_drop(tube, energy, 4400.0, 200.0)
    np.testing.assert_allclose(firing_droop(required, tube, energy, 4400.0).drop_rpm, 200.0, rtol=1e-9)


def test_wall_thickness_for_inertia_inverts_the_tube_relation():
    tube = team_tube()
    target = tube.with_wall(0.137 * INCH_TO_METER).inertia_kg_m2
    np.testing.assert_allclose(wall_thickness_for_inertia(tube, target), 0.137 * INCH_TO_METER, rtol=1e-6)


def test_wall_thickness_for_unreachable_inertia_is_nan():
    tube = team_tube()
    solid_inertia = tube.with_wall(tube.outer_radius_m).inertia_kg_m2
    assert np.isnan(wall_thickness_for_inertia(tube, solid_inertia * 2.0))


def test_balanced_idle_equalises_spin_up_and_spin_down():
    drivetrain = Drivetrain()
    inertia = team_tube().inertia_kg_m2 + drivetrain.reflected_rotor_inertia_kg_m2
    idle = balanced_idle_rpm(inertia, drivetrain, 2600.0, 4400.0, settle_band_rpm=25.0)
    assert 2600.0 < idle < 4400.0
    up = spin_up_time_s(inertia, drivetrain, idle, 4375.0)
    down = spin_down_time_s(inertia, drivetrain, idle, 2625.0)
    np.testing.assert_allclose(up, down, rtol=1e-3)


def test_balanced_idle_sits_above_the_midpoint_of_the_shot_band():
    """Braking holds the full stator limit while acceleration fades near free speed,
    so the flywheel should wait nearer the top of the range."""
    drivetrain = Drivetrain()
    inertia = team_tube().inertia_kg_m2 + drivetrain.reflected_rotor_inertia_kg_m2
    assert balanced_idle_rpm(inertia, drivetrain, 2600.0, 4400.0) > 3500.0


def test_sweep_marks_thin_walls_infeasible_and_thick_walls_feasible():
    result = sweep_wall_thickness(
        team_tube(), Drivetrain(), ShotEnergy(), shot_rpm=4400.0, idle_rpm=3500.0, max_drop_rpm=250.0
    )
    assert not result.feasible_mask[0]
    assert result.feasible_mask[-1]
    assert np.all(np.diff(result.spin_up_s) > 0.0)
    assert np.all(np.diff(result.drop_rpm) < 0.0)


def test_sweep_minimum_feasible_wall_is_the_feasibility_boundary():
    result = sweep_wall_thickness(
        team_tube(), Drivetrain(), ShotEnergy(), shot_rpm=4400.0, idle_rpm=3500.0, max_drop_rpm=250.0
    )
    boundary = result.minimum_feasible_wall_m
    walls = result.wall_thickness_m
    assert walls[0] < boundary < walls[-1]
    np.testing.assert_allclose(walls[result.feasible_mask][0], boundary, atol=np.diff(walls)[0])


def test_sweep_flags_a_shot_speed_the_gearing_cannot_reach():
    """4 in tube on a 15:18 reduction tops out near 5000 RPM, so a 14 m/s ball is out of reach."""
    tube = team_tube()
    energy = ShotEnergy()
    unreachable_rpm = tube.rpm_for_surface_speed(14.0 / energy.exit_speed_ratio)
    result = sweep_wall_thickness(
        tube, Drivetrain(), energy, shot_rpm=unreachable_rpm, idle_rpm=3000.0, max_drop_rpm=250.0
    )
    assert not result.shot_rpm_reachable
    assert np.all(~np.isfinite(result.spin_up_s))


def test_recovery_time_is_far_less_sensitive_to_inertia_than_spin_up():
    """Refilling a fixed energy budget at a fixed available power takes a roughly
    fixed time, so extra inertia buys accuracy during the volley rather than a
    faster next volley. Spin-up from idle carries the whole cost instead."""
    result = sweep_wall_thickness(
        team_tube(), Drivetrain(), ShotEnergy(), shot_rpm=4400.0, idle_rpm=3500.0, max_drop_rpm=250.0
    )
    inertia_span = result.inertia_kg_m2.max() / result.inertia_kg_m2.min()
    recovery_span = result.recovery_s.max() / result.recovery_s.min()
    spin_up_span = result.spin_up_s.max() / result.spin_up_s.min()
    assert inertia_span > 10.0
    assert recovery_span < 1.5
    assert spin_up_span > 4.0 * recovery_span


def test_idle_power_climbs_faster_than_idle_speed():
    """Holding speed needs applied voltage proportional to speed, so the cost is
    superlinear even though the current barely moves."""
    drivetrain = Drivetrain()
    low = drivetrain.idle_draw(1000.0)
    high = drivetrain.idle_draw(4000.0)
    assert high.power_w > low.power_w
    assert high.power_w / low.power_w > 4.0 / 1.0
    # Current is nearly flat across that range; voltage is what changes.
    assert high.stator_current_a / low.stator_current_a < 1.5


def test_idle_cost_over_a_match_is_a_small_share_of_the_battery():
    draw = Drivetrain().idle_draw(4300.0)
    assert 0.0 < draw.battery_fraction < 0.05


def test_coast_down_fit_recovers_a_known_drag():
    drivetrain = Drivetrain()
    inertia = team_tube(0.188).inertia_kg_m2
    coefficient = drivetrain.drag_coefficient_nm_s
    start_rpm, end_rpm = 4000.0, 2000.0
    # Viscous drag alone decays speed exponentially.
    coast_seconds = inertia * np.log(start_rpm / end_rpm) / coefficient
    fitted = drag_torque_from_coast_down(
        inertia, start_rpm, end_rpm, coast_seconds, drivetrain.free_speed_rpm
    )
    np.testing.assert_allclose(fitted, drivetrain.drag_torque_at_free_speed_nm, rtol=1e-9)


def test_coast_down_fit_rejects_impossible_measurements():
    assert np.isnan(drag_torque_from_coast_down(0.01, 1000.0, 2000.0, 5.0, 5000.0))
    assert np.isnan(drag_torque_from_coast_down(0.01, 4000.0, 2000.0, 0.0, 5000.0))


def test_spin_up_draws_more_than_the_kinetic_energy_it_stores():
    """The stator-limited part of the climb burns real energy in the windings."""
    drivetrain = Drivetrain()
    inertia = team_tube(0.188).inertia_kg_m2 + drivetrain.reflected_rotor_inertia_kg_m2
    start_rpm, end_rpm = 500.0, 4000.0
    stored = 0.5 * inertia * ((end_rpm * RPM_TO_RAD_S) ** 2 - (start_rpm * RPM_TO_RAD_S) ** 2)
    drawn = spin_up_electrical_energy_j(inertia, drivetrain, start_rpm, end_rpm)
    assert drawn > stored
    assert drawn < 10.0 * stored


def test_idle_strategy_breakeven_is_a_handful_of_volleys():
    """A low idle saves standing power but throws away stored energy every shot,
    so which strategy wins comes down to how many volleys the match holds."""
    drivetrain = Drivetrain()
    inertia = team_tube(0.188).inertia_kg_m2 + drivetrain.reflected_rotor_inertia_kg_m2
    comparison = compare_idle_strategies(inertia, drivetrain, 4237.0, 500.0, 3560.0)
    assert comparison.high_idle_power_w > comparison.low_idle_power_w
    assert comparison.extra_spin_up_energy_per_volley_j > 0.0
    assert 1.0 < comparison.breakeven_volleys < 20.0
    # The real penalty for idling low is time, not energy.
    assert comparison.low_idle_spin_up_s > 3.0 * comparison.high_idle_spin_up_s


# --- Shot-schedule tuning: bands come from the trajectory model, not a guess ---

def shot_context(tube=None):
    tube = tube or team_tube()
    energy = ShotEnergy()
    shooter = tube_shooter_model(tube, energy, RELEASE_HEIGHT_M)
    ball = BallSpec(mass_kg=BALL_MASS_KG, diameter_m=BALL_DIAMETER_M)
    target = Target(distance_m=4.0, center_height_m=HUB_BALL_CENTER_HEIGHT_M)
    return tube, energy, shooter, ball, Environment(), target


def test_tube_shooter_model_reproduces_half_surface_speed_with_full_backspin():
    tube, energy, shooter, ball, _, _ = shot_context()
    controls = ShotControls(4000.0, 0.0, 30.0)
    speed, _, spin = release_state(controls, ball, shooter)
    surface = tube.surface_speed_m_s(4000.0)
    np.testing.assert_allclose(speed, energy.exit_speed_ratio * surface, rtol=1e-9)
    # Positive spin is backspin in this convention, and rolling on the hood gives v / r.
    np.testing.assert_allclose(spin, speed / ball.radius_m, rtol=1e-9)
    assert spin > 0.0


def test_scoring_band_edges_bracket_the_scoring_region():
    _, _, shooter, ball, environment, target = shot_context()
    band = scoring_rpm_band(24.0, shooter, ball, environment, target, (500.0, 4985.0))
    assert band is not None
    low, high = band
    assert high > low
    inside = ShotControls(0.5 * (low + high), 0.0, 24.0)
    trajectory = simulate_shot(inside, ball, shooter, environment, target, dt_s=0.006, max_time_s=2.5)
    assert _is_scoring_entry(hub_entry_metrics(trajectory, target, ball), target)
    for outside_rpm in (low - 60.0, high + 60.0):
        controls = ShotControls(outside_rpm, 0.0, 24.0)
        trajectory = simulate_shot(controls, ball, shooter, environment, target, dt_s=0.006, max_time_s=2.5)
        assert not _is_scoring_entry(hub_entry_metrics(trajectory, target, ball), target)


def test_no_band_exists_for_a_distance_the_mechanism_cannot_reach():
    _, _, shooter, ball, environment, target = shot_context()
    unreachable = replace(target, distance_m=30.0)
    assert scoring_rpm_band(24.0, shooter, ball, environment, unreachable, (500.0, 4985.0)) is None


def test_hit_rate_rises_and_readiness_time_worsens_as_the_wall_thickens():
    tube, energy, shooter, ball, environment, target = shot_context()
    schedules = shot_schedule_candidates(
        shooter, ball, environment, target,
        distances_m=np.linspace(SHOT_DISTANCE_MIN_M, SHOT_DISTANCE_MAX_M, 5),
        hood_candidates_deg=np.array([24.0]),
        rpm_bounds=(500.0, 4985.0),
    )
    sweep = sweep_wall_against_schedule(tube, Drivetrain(), energy, schedules[0], samples=40)
    assert np.all(np.diff(sweep.in_range_fraction) >= 0.0)
    assert sweep.in_range_fraction[0] < sweep.in_range_fraction[-1]
    assert sweep.mean_transition_s[-1] > sweep.mean_transition_s[0]


def test_tuning_picks_the_thinnest_wall_that_hits_every_shot():
    tube, energy, shooter, ball, environment, target = shot_context()
    schedules = shot_schedule_candidates(
        shooter, ball, environment, target,
        distances_m=np.linspace(SHOT_DISTANCE_MIN_M, SHOT_DISTANCE_MAX_M, 5),
        hood_candidates_deg=np.array([21.0, 24.0]),
        rpm_bounds=(500.0, 4985.0),
    )
    tuning = tune_wall_thickness(tube, Drivetrain(), energy, schedules, samples=50)
    np.testing.assert_allclose(tuning.best_in_range_fraction, 1.0)
    sweep = tuning.sweep
    chosen = int(np.argmin(np.abs(sweep.wall_thickness_m - tuning.best_wall_m)))
    # Nothing thinner clears the bar, so this is also the fastest wall that does.
    assert not np.any(sweep.in_range_fraction[:chosen] >= 1.0 - 1e-9)


def test_optimised_idle_beats_parking_at_rest():
    drivetrain = Drivetrain()
    inertia = team_tube(0.095).inertia_kg_m2 + drivetrain.reflected_rotor_inertia_kg_m2
    commands = np.array([2300.0, 2900.0, 3500.0, 4000.0])
    idle, mean_s = best_idle_for_schedule(inertia, drivetrain, commands)
    from shotlab.flywheel import _mean_transition_s

    assert idle > 0.0
    assert mean_s < _mean_transition_s(inertia, drivetrain, commands, 0.0, 25.0)


def test_in_band_share_never_collapses_to_zero():
    """The balls that leave first are still at the commanded speed, which is in
    band by construction, so however far the wheel sags some of them score."""
    band = 300.0
    assert in_band_fraction(0.0, band) == 1.0
    assert in_band_fraction(band, band) == 1.0
    np.testing.assert_allclose(in_band_fraction(2.0 * band, band), 0.5)
    huge = in_band_fraction(50.0 * band, band)
    assert 0.0 < huge < 0.05


def test_in_band_share_falls_off_as_band_over_droop():
    band = 400.0
    for droop in (500.0, 900.0, 1600.0):
        np.testing.assert_allclose(in_band_fraction(droop, band), band / droop)


def test_even_the_thinnest_wall_scores_some_of_the_hopper():
    """Regression: the sweep used to report a flat zero at thin walls because it
    asked whether the whole droop fitted the band instead of what share did."""
    tube, energy, shooter, ball, environment, target = shot_context()
    schedules = shot_schedule_candidates(
        shooter, ball, environment, target,
        distances_m=np.linspace(SHOT_DISTANCE_MIN_M, SHOT_DISTANCE_MAX_M, 4),
        hood_candidates_deg=np.array([24.0]),
        rpm_bounds=(500.0, 4985.0),
    )
    sweep = sweep_wall_against_schedule(
        tube, Drivetrain(), energy, schedules[0], hopper_balls=60, samples=30
    )
    assert sweep.in_range_fraction[0] > 0.0
    assert sweep.in_range_fraction[0] < 0.6


def test_hopper_depth_turns_spin_up_into_recovery():
    """One volley is paced by the climb from idle; sixty balls are paced by the
    recovery between volleys, which is far shorter."""
    tube, energy, shooter, ball, environment, target = shot_context()
    schedules = shot_schedule_candidates(
        shooter, ball, environment, target,
        distances_m=np.linspace(SHOT_DISTANCE_MIN_M, SHOT_DISTANCE_MAX_M, 4),
        hood_candidates_deg=np.array([24.0]),
        rpm_bounds=(500.0, 4985.0),
    )
    single = sweep_wall_against_schedule(tube, Drivetrain(), energy, schedules[0], hopper_balls=4, samples=30)
    deep = sweep_wall_against_schedule(tube, Drivetrain(), energy, schedules[0], hopper_balls=60, samples=30)
    assert single.volleys == 1
    assert deep.volleys == 15
    # With a single volley the per-volley cost is exactly the climb from idle.
    np.testing.assert_allclose(single.mean_volley_s, single.mean_transition_s, rtol=1e-9)
    mid = len(deep.wall_thickness_m) // 2
    assert deep.mean_volley_s[mid] < single.mean_volley_s[mid]
    assert deep.hopper_seconds[mid] > single.hopper_seconds[mid]


def test_deep_hopper_makes_time_per_volley_nearly_flat_in_wall_thickness():
    """Recovery is set by energy against available power, not by inertia, which
    is why the in-band constraint has to pick the wall rather than the clock."""
    tube, energy, shooter, ball, environment, target = shot_context()
    schedules = shot_schedule_candidates(
        shooter, ball, environment, target,
        distances_m=np.linspace(SHOT_DISTANCE_MIN_M, SHOT_DISTANCE_MAX_M, 4),
        hood_candidates_deg=np.array([24.0]),
        rpm_bounds=(500.0, 4985.0),
    )
    sweep = sweep_wall_against_schedule(
        tube, Drivetrain(), energy, schedules[0], hopper_balls=60, samples=40
    )
    scoring = sweep.mean_volley_s[sweep.in_range_fraction >= 0.99]
    assert scoring.size > 5
    assert np.ptp(scoring) / scoring.mean() < 0.05


def test_tuning_returns_the_lightest_wall_meeting_the_target():
    """A heavier wheel recovers marginally sooner, so minimising time alone would
    run away to the thickest tube. The constraint is what bounds the answer."""
    tube, energy, shooter, ball, environment, target = shot_context()
    schedules = shot_schedule_candidates(
        shooter, ball, environment, target,
        distances_m=np.linspace(SHOT_DISTANCE_MIN_M, SHOT_DISTANCE_MAX_M, 4),
        hood_candidates_deg=np.array([24.0]),
        rpm_bounds=(500.0, 4985.0),
    )
    strict = tune_wall_thickness(tube, Drivetrain(), energy, schedules, hopper_balls=60, samples=50)
    loose = tune_wall_thickness(
        tube, Drivetrain(), energy, schedules, hopper_balls=60, samples=50,
        target_in_range_fraction=0.85,
    )
    # A looser in-band target must buy a lighter tube, not an equal or heavier one.
    assert loose.best_wall_m < strict.best_wall_m
    sweep = strict.sweep
    chosen = int(np.argmin(np.abs(sweep.wall_thickness_m - strict.best_wall_m)))
    assert not np.any(sweep.in_range_fraction[:chosen] >= 1.0 - 1e-9)
