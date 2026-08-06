from __future__ import annotations

import base64
import hashlib
import os
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st

from shotlab.calibration import (
    PARAMETER_BOUNDS,
    fit_parameters,
    match_reference_controls,
    model_with_candidate,
    trajectory_rmse,
)
from shotlab.llm import request_parameter_advice
from shotlab.models import BallSpec, Environment, OptimizationWeights, ShotControls, ShooterModel, Target
from shotlab.physics import (
    build_shot_map,
    hub_entry_metrics,
    launch_elevation_deg,
    optimize_shot,
    release_state,
    simulate_shot,
)
from shotlab.robot_profile import (
    BALL_COMPRESSION_M,
    BALL_DIAMETER_M,
    BALL_MASS_KG,
    BOTTOM_WHEEL_DIAMETER_M,
    CONTACT_PATH_LENGTH_M,
    DRIVE_MODULE_LONGITUDINAL_OFFSET_M,
    DRIVE_WHEEL_DIAMETER_M,
    FLYWHEEL_IDLE_RPM,
    FLYWHEEL_MAX_RPM,
    FOLLOWER_TO_LEADER_RPM_RATIO,
    HUB_BODY_DEPTH_M,
    HUB_DEFAULT_MIN_ENTRY_ANGLE_DEG,
    HUB_DEFAULT_RIM_MARGIN_M,
    HUB_FUNNEL_RIM_HEIGHT_M,
    HUB_FUNNEL_RIM_SPAN_M,
    HUB_FUNNEL_THROAT_HEIGHT_M,
    HUB_FUNNEL_THROAT_SPAN_M,
    HUB_OPENING_PLANE_HEIGHT_M,
    HUB_OPENING_SPAN_M,
    HOOD_MAX_DEG,
    HOOD_MIN_DEG,
    PROFILE_ROWS,
    RELEASE_HEIGHT_M,
    ROBOT_LENGTH_M,
    SHOT_DISTANCE_MAX_M,
    SHOT_DISTANCE_MIN_M,
    TOP_WHEEL_DIAMETER_M,
)
from shotlab.tracking import TrackingConfig, track_video, video_reference_frame


ASSET_DIR = Path(__file__).resolve().parent / "assets"
LOGO_PATH = ASSET_DIR / "charging-champions-mark.png"
LOGO_DATA = base64.b64encode(LOGO_PATH.read_bytes()).decode("ascii")
FOOT_TO_METER = 0.3048
DEFAULT_TARGET_DISTANCE_FT = 15.0
VIDEO_CALIBRATION_WIDGET_KEYS = (
    "video_calibration_mode",
    "video_distance_ft",
    "video_release_x_px",
    "video_release_y_px",
    "video_hub_base_x_px",
    "video_hub_base_y_px",
    "video_hub_opening_x_px",
    "video_hub_opening_y_px",
    "video_pixels_per_meter",
)

st.set_page_config(page_title="Charging Champions | Shot Lab", page_icon=str(LOGO_PATH), layout="wide")
st.markdown(
    """
    <style>
    :root {
      --charge-cyan: #00BAFF;
      --charge-blue: #1179EE;
      --charge-teal: #33BECC;
      --ink: #080A0D;
      --muted: #5B6570;
      --line: #DCE2E7;
      --surface: #F5F8FA;
      --warning: #D97706;
      --danger: #C93838;
    }
    html, body, [class*="css"] { font-family: "SF Pro Display", -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    .stApp { background: #FFFFFF; color: var(--ink); }
    [data-testid="stHeader"] { background: rgba(255,255,255,.94); }
    [data-testid="stMainBlockContainer"] { max-width: 1480px; padding-top: 1.25rem; padding-bottom: 3rem; }
    h1, h2, h3, h4 { color: var(--ink); letter-spacing: 0 !important; }
    h1, .brand-name { font-family: "Chalet Comprime Eighty", "Arial Narrow", "SF Pro Display", sans-serif; }
    h2 { font-size: 1.28rem !important; font-weight: 600 !important; }
    h3 { font-size: 1.06rem !important; font-weight: 600 !important; }
    p, label, [data-testid="stCaptionContainer"] { letter-spacing: 0 !important; }
    .brand-header { display: flex; align-items: center; min-height: 104px; border-top: 8px solid var(--charge-teal); border-bottom: 1px solid var(--line); padding: 12px 4px 14px; }
    .brand-mark { width: 47px; height: 78px; object-fit: contain; margin: 0 18px 0 8px; }
    .brand-lockup { min-width: 0; }
    .brand-kicker { color: var(--charge-blue); font-size: .72rem; font-weight: 800; text-transform: uppercase; }
    .brand-name { margin: 0; font-size: 2rem; line-height: 1.02; font-weight: 800; }
    .brand-product { margin-top: 4px; color: var(--muted); font-size: .78rem; font-weight: 700; text-transform: uppercase; }
    .profile-state { margin-left: auto; border-left: 3px solid var(--charge-cyan); padding: 7px 8px 7px 15px; text-align: right; }
    .profile-state span { display: block; color: var(--muted); font-size: .66rem; font-weight: 700; text-transform: uppercase; }
    .profile-state strong { display: block; color: var(--ink); font-size: .86rem; }
    .profile-strip { display: grid; grid-template-columns: repeat(3, 1fr); background: var(--surface); border-bottom: 1px solid var(--line); margin-bottom: 1rem; }
    .profile-strip div { padding: .58rem .9rem; border-right: 1px solid var(--line); }
    .profile-strip div:last-child { border-right: 0; }
    .profile-strip span { display: block; color: var(--muted); font-size: .67rem; font-weight: 700; text-transform: uppercase; }
    .profile-strip strong { font-size: .93rem; }
    .stage-rail { display: grid; grid-template-columns: repeat(3, 1fr); border-top: 1px solid var(--line); border-bottom: 1px solid var(--line); margin: .6rem 0 1rem; }
    .stage-rail div { padding: .66rem .35rem; font-weight: 650; color: var(--muted); }
    .stage-rail span { color: var(--charge-blue); margin-right: .4rem; font-weight: 800; }
    .source-line { margin: -.15rem 0 .65rem; color: var(--muted); font-size: .78rem; }
    .source-chip { display: inline-block; margin-right: .35rem; padding: .16rem .38rem; border: 1px solid var(--charge-teal); color: #087581; font-size: .65rem; font-weight: 800; text-transform: uppercase; }
    .source-chip.code { border-color: var(--charge-blue); color: #0B5EB6; }
    .source-chip.assumption { border-color: var(--warning); color: #9A5500; }
    .compact-note { color: var(--muted); font-size: .84rem; }
    div[data-testid="stMetric"] { border-top: 3px solid var(--charge-cyan); padding-top: .62rem; }
    div[data-testid="stMetricValue"] { font-size: 1.45rem; font-weight: 650; }
    .stButton button, .stDownloadButton button { border-radius: 4px; font-weight: 700; min-height: 2.55rem; }
    .stButton button[kind="primary"] { background: var(--charge-blue); border-color: var(--charge-blue); }
    .stButton button[kind="primary"]:hover { background: #0D68CE; border-color: #0D68CE; }
    [data-testid="stFileUploaderDropzone"], [data-testid="stExpander"] { border-radius: 4px; }
    [data-baseweb="tab-list"] { gap: 1.35rem; border-bottom: 1px solid var(--line); }
    [data-baseweb="tab"] { padding-left: 0; padding-right: 0; }
    [aria-selected="true"] { color: var(--charge-blue) !important; }
    [data-testid="stAlert"] { border-radius: 4px; }
    @media (max-width: 700px) {
      [data-testid="stMainBlockContainer"] { padding: .8rem 1rem 2rem; }
      .brand-header { min-height: 86px; }
      .brand-mark { width: 36px; height: 62px; margin-right: 11px; }
      .brand-name { font-size: 1.5rem; }
      .profile-state { display: none; }
      .profile-strip { grid-template-columns: 1fr; }
      .profile-strip div { border-right: 0; border-bottom: 1px solid var(--line); }
      .profile-strip div:last-child { border-bottom: 0; }
      .stage-rail { grid-template-columns: 1fr; }
      .stage-rail div { padding: .4rem .2rem; }
    }
    </style>
    """,
    unsafe_allow_html=True,
)


EMPIRICAL_WIDGET_KEYS = {
    "velocity_transfer": "model_velocity_transfer",
    "spin_transfer": "model_spin_transfer",
    "hood_offset_deg": "model_hood_offset_deg",
    "drag_scale": "model_drag_scale",
    "lift_slope": "model_lift_slope",
    "spin_decay_per_s": "model_spin_decay_per_s",
}


def initialize_state() -> None:
    had_prior_model_state = "model_velocity_transfer" in st.session_state
    if st.session_state.get("tracker_version") != 3:
        st.session_state.tracker_version = 3
        st.session_state.tracked_shot = None
        st.session_state.fit_result = None
        st.session_state.control_correction = None
        st.session_state.llm_advice = None
        st.session_state.video_signature = None
    if st.session_state.get("optimizer_version") != 4:
        st.session_state.optimizer_version = 4
        st.session_state.optimal_result = None
        st.session_state.optimal_target_distance_m = None
        st.session_state.shot_map = None
        st.session_state.shot_map_context = None
    defaults = {
        "optimal_result": None,
        "shot_map": None,
        "shot_map_context": None,
        "tracked_shot": None,
        "fit_result": None,
        "control_correction": None,
        "llm_advice": None,
        "llm_candidate_rmse": None,
        "video_signature": None,
        "optimal_target_distance_m": None,
        "measure_calibration_open": False,
        "actual_controls_signature": None,
        "model_velocity_transfer": 0.72,
        "model_spin_transfer": 0.70,
        "model_hood_offset_deg": 0.0,
        "model_drag_scale": 1.0,
        "model_lift_slope": 0.75,
        "model_spin_decay_per_s": 0.10,
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value
    if st.session_state.get("hood_angle_convention_version") != 1:
        st.session_state.hood_angle_convention_version = 1
        st.session_state.optimal_result = None
        st.session_state.optimal_target_distance_m = None
        st.session_state.shot_map = None
        st.session_state.shot_map_context = None
        st.session_state.fit_result = None
        st.session_state.control_correction = None
        st.session_state.llm_advice = None
        st.session_state.llm_candidate_rmse = None
        st.session_state.measure_calibration_open = False
        st.session_state.actual_controls_signature = None
        st.session_state.model_velocity_transfer = 0.72
        st.session_state.model_spin_transfer = 0.70
        st.session_state.model_hood_offset_deg = 0.0
        st.session_state.model_drag_scale = 1.0
        st.session_state.model_lift_slope = 0.75
        st.session_state.model_spin_decay_per_s = 0.10
        if had_prior_model_state:
            st.session_state.model_flash = (
                "Hood convention corrected: launch elevation is 90 deg minus the rear-referenced hood command. "
                "Old fitted coefficients and recommendations were reset; the tracked video is still available to refit."
            )


def apply_model_parameters(
    candidate: dict[str, float],
    corrected_result=None,
    target_distance_m: float | None = None,
) -> None:
    for name, value in candidate.items():
        widget_key = EMPIRICAL_WIDGET_KEYS.get(name)
        if widget_key:
            st.session_state[widget_key] = float(value)
    st.session_state.optimal_result = corrected_result
    st.session_state.shot_map = None
    st.session_state.shot_map_context = None
    st.session_state.optimal_target_distance_m = (
        float(target_distance_m) if corrected_result is not None and target_distance_m is not None else None
    )
    st.session_state.fit_result = None
    st.session_state.control_correction = None
    st.session_state.llm_advice = None
    st.session_state.llm_candidate_rmse = None
    st.session_state.model_flash = (
        "Calibration applied. Stage 01 now shows the corrected next-shot recommendation."
        if corrected_result is not None
        else "Candidate parameters applied. Recalculate the recommended shot."
    )


def use_optimal_controls() -> None:
    result = st.session_state.optimal_result
    if result is None:
        return
    st.session_state.actual_top_rpm = float(result.controls.top_rpm)
    st.session_state.actual_bottom_rpm = float(result.controls.bottom_rpm)
    st.session_state.actual_hood_deg = float(result.controls.hood_angle_deg)
    confirm_actual_controls()


def actual_controls_signature() -> tuple[float, float, float] | None:
    values = (
        st.session_state.get("actual_top_rpm"),
        st.session_state.get("actual_bottom_rpm"),
        st.session_state.get("actual_hood_deg"),
    )
    if any(value is None for value in values):
        return None
    return tuple(round(float(value), 6) for value in values)


def confirm_actual_controls() -> None:
    st.session_state.actual_controls_signature = actual_controls_signature()
    st.session_state.measure_calibration_open = False
    st.session_state.fit_result = None
    st.session_state.control_correction = None
    st.session_state.llm_advice = None
    st.session_state.llm_candidate_rmse = None


def invalidate_actual_controls() -> None:
    st.session_state.actual_controls_signature = None
    st.session_state.measure_calibration_open = False
    st.session_state.fit_result = None
    st.session_state.control_correction = None
    st.session_state.llm_advice = None
    st.session_state.llm_candidate_rmse = None


def open_measure_calibration() -> None:
    st.session_state.measure_calibration_open = True


@st.cache_data(show_spinner=False)
def cached_video_reference_frame(video_path: str) -> np.ndarray:
    return video_reference_frame(video_path)


def calibration_reference_figure(
    frame_rgb: np.ndarray,
    *,
    release_x_px: float,
    release_y_px: float,
    hub_base_x_px: float,
    hub_base_y_px: float,
    hub_opening_x_px: float,
    hub_opening_y_px: float,
) -> go.Figure:
    figure = go.Figure(go.Image(z=frame_rgb))
    figure.add_trace(
        go.Scatter(
            x=[release_x_px, hub_base_x_px, hub_opening_x_px],
            y=[release_y_px, hub_base_y_px, hub_opening_y_px],
            mode="markers+text",
            text=["Release X", "HUB base", "Upper funnel rim (72 in)"],
            textposition=["top right", "bottom center", "top center"],
            marker=dict(size=13, color=["#FFBE00", "#1179EE", "#33BECC"], line=dict(color="#FFFFFF", width=2)),
            showlegend=False,
            hovertemplate="%{text}<br>x=%{x:.0f}px<br>y=%{y:.0f}px<extra></extra>",
        )
    )
    figure.add_trace(
        go.Scatter(
            x=[hub_base_x_px, hub_opening_x_px],
            y=[hub_base_y_px, hub_opening_y_px],
            mode="lines",
            line=dict(color="#FFFFFF", width=2, dash="dash"),
            hoverinfo="skip",
            showlegend=False,
        )
    )
    figure.update_layout(
        height=330,
        margin=dict(l=10, r=10, t=10, b=10),
        paper_bgcolor="#FFFFFF",
        plot_bgcolor="#080A0D",
        xaxis=dict(title="Pixel X", showgrid=True, gridcolor="rgba(255,255,255,.22)"),
        yaxis=dict(title="Pixel Y (top = 0)", showgrid=True, gridcolor="rgba(255,255,255,.22)"),
    )
    return figure


def measured_hub_entry(measured, target: Target, ball: BallSpec) -> dict[str, float | bool] | None:
    center_plane_m = HUB_OPENING_PLANE_HEIGHT_M + ball.radius_m
    for index in range(len(measured.z_m) - 1):
        z_left = float(measured.z_m[index])
        z_right = float(measured.z_m[index + 1])
        if not (z_left >= center_plane_m > z_right):
            continue
        fraction = (z_left - center_plane_m) / max(z_left - z_right, 1e-12)
        crossing_x = float(measured.x_m[index] + fraction * (measured.x_m[index + 1] - measured.x_m[index]))
        dt = float(measured.time_s[index + 1] - measured.time_s[index])
        vx = float((measured.x_m[index + 1] - measured.x_m[index]) / max(dt, 1e-12))
        vz = float((measured.z_m[index + 1] - measured.z_m[index]) / max(dt, 1e-12))
        usable_half_span = max(target.opening_span_m / 2.0 - ball.radius_m - target.rim_margin_m, 0.0)
        center_offset = abs(crossing_x - target.distance_m)
        near_rim_x = target.distance_m - target.opening_span_m / 2.0
        near_rim_height = float(np.interp(near_rim_x, measured.x_m, measured.z_m))
        near_rim_clearance = near_rim_height - center_plane_m
        entry_angle = float(np.degrees(np.arctan2(-vz, max(abs(vx), 1e-9))))
        return {
            "inside": center_offset <= usable_half_span and near_rim_clearance >= target.rim_margin_m,
            "crossing_x_m": crossing_x,
            "clearance_m": usable_half_span - center_offset,
            "near_rim_clearance_m": near_rim_clearance,
            "entry_angle_deg": entry_angle,
            "meets_min_entry": entry_angle >= target.min_entry_angle_deg,
        }
    return None


def measured_downward_acceleration(measured) -> float | None:
    if len(measured.time_s) < 5 or float(np.ptp(measured.time_s)) < 0.05:
        return None
    time_s = np.asarray(measured.time_s, dtype=float)
    design = np.column_stack((np.ones_like(time_s), time_s, 0.5 * time_s**2))
    coefficients, *_ = np.linalg.lstsq(design, np.asarray(measured.z_m, dtype=float), rcond=None)
    return float(-coefficients[2])


def render_fitted_equation(
    fit,
    recorded_controls: ShotControls,
    ball: BallSpec,
    environment: Environment,
    correction,
) -> None:
    model = fit.fitted_model
    equation_controls = correction.controls if correction is not None else recorded_controls
    top_surface_m_s = np.pi * model.top_wheel_diameter_m * equation_controls.top_rpm / 60.0
    bottom_surface_m_s = np.pi * model.bottom_wheel_diameter_m * equation_controls.bottom_rpm / 60.0
    speed_m_s, angle_rad, spin_rad_s = release_state(equation_controls, ball, model)
    angle_deg = float(np.degrees(angle_rad))
    initial_vx_m_s = speed_m_s * np.cos(angle_rad) + model.robot_velocity_x_m_s
    initial_vz_m_s = speed_m_s * np.sin(angle_rad)
    aerodynamic_scale = 0.5 * environment.air_density_kg_m3 * ball.area_m2 / ball.mass_kg
    drag_factor = aerodynamic_scale * ball.drag_coefficient * model.drag_scale
    lift_ratio_factor = model.lift_slope * ball.radius_m
    fitted_names = set(fit.fitted_parameters)

    st.markdown("#### Fitted physics equation")
    st.caption(
        "The release state below uses the corrected next-shot controls when available. "
        "Only rows marked Fitted were identified from this recording; the other coefficients were held constant."
    )
    parameter_rows = [
        ("Velocity transfer", model.velocity_transfer, "velocity_transfer"),
        ("Spin transfer", model.spin_transfer, "spin_transfer"),
        ("Launch-angle offset (deg)", model.hood_offset_deg, "hood_offset_deg"),
        ("Drag scale", model.drag_scale, "drag_scale"),
        ("Magnus lift slope", model.lift_slope, "lift_slope"),
        ("Spin decay (1/s)", model.spin_decay_per_s, "spin_decay_per_s"),
    ]
    st.dataframe(
        pd.DataFrame(
            [
                {
                    "Coefficient": label,
                    "Value": value,
                    "Status": "Fitted" if key in fitted_names else "Held constant",
                }
                for label, value, key in parameter_rows
            ]
        ),
        hide_index=True,
        width="stretch",
    )
    st.latex(
        rf"""
        \begin{{aligned}}
        u_t &= \frac{{\pi({model.top_wheel_diameter_m:.5f})({equation_controls.top_rpm:.3f})}}{{60}}
             = {top_surface_m_s:.4f}\ \mathrm{{m/s}} \\
        u_b &= \frac{{\pi({model.bottom_wheel_diameter_m:.5f})({equation_controls.bottom_rpm:.3f})}}{{60}}
             = {bottom_surface_m_s:.4f}\ \mathrm{{m/s}} \\
        v_0 &= {model.velocity_transfer:.6f}\frac{{u_t+u_b}}{{2}}
             = {speed_m_s:.4f}\ \mathrm{{m/s}} \\
        \theta_{{\mathrm{{cmd}}}} &= {equation_controls.hood_angle_deg:.4f}^\circ\quad\text{{(rear reference)}} \\
        \theta_0 &= (90^\circ-\theta_{{\mathrm{{cmd}}}}+{model.hood_offset_deg:.4f}^\circ)
             = {angle_deg:.4f}^\circ \\
        \omega_0 &= {model.spin_transfer:.6f}\frac{{u_b-u_t}}{{2({ball.radius_m:.5f})}}
             = {spin_rad_s:.4f}\ \mathrm{{rad/s}}
        \end{{aligned}}
        """
    )
    st.latex(
        rf"""
        \begin{{aligned}}
        \mathbf{{u}} &= (v_x-{environment.wind_x_m_s:.4f},\ v_z),\qquad s=\lVert\mathbf{{u}}\rVert \\
        C_L &= \min\left({model.max_lift_coefficient:.4f},\ {lift_ratio_factor:.6f}\frac{{|\omega|}}{{s}}\right) \\
        \dot x &= v_x,\qquad \dot z=v_z \\
        \begin{{bmatrix}}\dot v_x\\\dot v_z\end{{bmatrix}}
        &= \begin{{bmatrix}}0\\-{environment.gravity_m_s2:.5f}\end{{bmatrix}}
        -{drag_factor:.8f}s\mathbf{{u}}
        +{aerodynamic_scale:.8f}C_Ls\,\operatorname{{sgn}}(\omega)
        \begin{{bmatrix}}-u_z\\u_x\end{{bmatrix}} \\
        \dot\omega &= -{model.spin_decay_per_s:.6f}\omega \\
        (x,z,v_x,v_z,\omega)_0
        &= (0,{model.release_height_m:.4f},{initial_vx_m_s:.4f},{initial_vz_m_s:.4f},{spin_rad_s:.4f})
        \end{{aligned}}
        """
    )
    equation_text = f"""FITTED SHOT MODEL
Controls: top={equation_controls.top_rpm:.6f} rpm, bottom={equation_controls.bottom_rpm:.6f} rpm, hood_command_from_back={equation_controls.hood_angle_deg:.6f} deg
Angle conversion: theta0=90 - hood_command_from_back + launch_angle_offset = {angle_deg:.6f} deg
Release: v0={speed_m_s:.6f} m/s, theta0={angle_deg:.6f} deg, omega0={spin_rad_s:.6f} rad/s
Initial state: x0=0 m, z0={model.release_height_m:.6f} m, vx0={initial_vx_m_s:.6f} m/s, vz0={initial_vz_m_s:.6f} m/s
Relative velocity: u=(vx-{environment.wind_x_m_s:.6f}, vz), speed=|u|
Lift coefficient: CL=min({model.max_lift_coefficient:.6f}, {lift_ratio_factor:.6f}*|omega|/speed)
Position: dx/dt=vx, dz/dt=vz
Acceleration: d[vx,vz]/dt=[0,-{environment.gravity_m_s2:.6f}] - {drag_factor:.9f}*speed*u + {aerodynamic_scale:.9f}*CL*speed*sign(omega)*[-u_z,u_x]
Spin: domega/dt=-{model.spin_decay_per_s:.6f}*omega
"""
    st.code(equation_text, language="text")


def render_numerical_calibration(
    tracked,
    controls: ShotControls,
    ball: BallSpec,
    shooter: ShooterModel,
    environment: Environment,
    target: Target,
    *,
    widget_prefix: str,
    optimal_trajectory=None,
):
    st.warning(
        "Treat a one-shot fit as a candidate, not a final calibration. Do not fit both spin transfer and Magnus lift slope unless spin is independently measured."
    )
    fit_column, review_column = st.columns([0.45, 0.55], gap="large")
    with fit_column:
        fit_names = st.multiselect(
            "Parameters to fit",
            list(PARAMETER_BOUNDS),
            default=["velocity_transfer", "hood_offset_deg"],
            format_func=lambda name: (
                "Launch Angle Offset" if name == "hood_offset_deg" else name.replace("_", " ").title()
            ),
            key=f"{widget_prefix}_fit_names",
        )
        regularization = st.slider(
            "Stay near current calibration",
            0.0,
            0.5,
            0.08,
            0.01,
            key=f"{widget_prefix}_regularization",
        )
        if st.button(
            "Fit selected parameters",
            type="primary",
            width="stretch",
            key=f"{widget_prefix}_fit_button",
        ):
            try:
                st.session_state.control_correction = None
                with st.spinner("Fitting the measured path with bounded nonlinear least squares..."):
                    st.session_state.fit_result = fit_parameters(
                        tracked,
                        controls,
                        ball,
                        shooter,
                        environment,
                        target,
                        fit_names,
                        regularization=regularization,
                    )
                    fit_candidate = st.session_state.fit_result
                    st.session_state.control_correction = (
                        match_reference_controls(
                            optimal_trajectory,
                            ball,
                            fit_candidate.fitted_model,
                            environment,
                            target,
                        )
                        if optimal_trajectory is not None
                        else None
                    )
                    st.session_state.llm_advice = None
                st.success(
                    "Calibration finished. Orange validates the fitted equation against the recording; "
                    "the corrected next-shot trace changes RPM and hood command to follow the reference optimum."
                )
            except Exception as error:
                st.error(f"Numerical calibration failed: {error}")

    fit = st.session_state.fit_result
    with review_column:
        if fit is None:
            current_prediction = simulate_shot(
                controls,
                ball,
                shooter,
                environment,
                target,
                max_time_s=max(0.5, float(tracked.time_s[-1]) + 0.15),
            )
            st.metric("Current-model RMSE", f"{trajectory_rmse(tracked, current_prediction):.3f} m")
        else:
            r1, r2, r3 = st.columns(3)
            r1.metric("Before RMSE", f"{fit.before_rmse_m:.3f} m")
            r2.metric("After RMSE", f"{fit.after_rmse_m:.3f} m")
            r3.metric("Improvement", f"{fit.improvement_percent:.1f}%")
            if fit.jacobian_condition > 1e5:
                st.warning("The fit is poorly identifiable. Reduce the selected parameters or collect another controlled shot.")

    if fit is not None:
        correction = st.session_state.control_correction
        parameter_rows = []
        for name in fit.fitted_parameters:
            before_value = getattr(fit.original_model, name)
            after_value = getattr(fit.fitted_model, name)
            parameter_rows.append(
                {
                    "Parameter": "launch_angle_offset_deg" if name == "hood_offset_deg" else name,
                    "Current": before_value,
                    "Candidate": after_value,
                    "Change": after_value - before_value,
                    "Allowed range": f"{PARAMETER_BOUNDS[name][0]} to {PARAMETER_BOUNDS[name][1]}",
                }
            )
        st.dataframe(pd.DataFrame(parameter_rows), hide_index=True, width="stretch")
        if optimal_trajectory is None:
            st.info("Calculate a Stage 01 optimum at this same range to generate corrected next-shot controls.")
        elif correction is not None:
            st.markdown("#### Corrected next shot")
            c1, c2, c3, c4 = st.columns(4)
            c1.metric(
                "Command RPM",
                f"{correction.controls.top_rpm:,.0f}",
                f"{correction.controls.top_rpm - controls.top_rpm:+,.0f}",
            )
            c2.metric(
                "Follower RPM",
                "Fixed" if correction.controls.bottom_rpm == 0 else f"{correction.controls.bottom_rpm:,.0f}",
                None if correction.controls.bottom_rpm == 0 else f"{correction.controls.bottom_rpm - controls.bottom_rpm:+,.0f}",
            )
            c3.metric(
                "Hood command (back ref)",
                f"{correction.controls.hood_angle_deg:.1f} deg",
                f"{correction.controls.hood_angle_deg - controls.hood_angle_deg:+.1f} deg",
            )
            c4.metric("Path RMSE vs optimum", f"{correction.objective * 100:.1f} cm")
            if correction.success:
                st.success(
                    "The corrected command scores in simulation and reproduces the reference trajectory within "
                    f"{correction.objective * 100:.1f} cm path RMSE."
                )
            else:
                st.warning(
                    "The fitted model cannot reproduce the reference path within the current RPM and hood-command limits. "
                    "The closest bounded command is shown, but it is not safe to apply as the next recommendation."
                )

        st.button(
            (
                "Apply calibration and corrected recommendation"
                if correction is not None and correction.success
                else "Apply calibration only"
            ),
            on_click=apply_model_parameters,
            args=(
                fit.fitted_model.empirical_parameters(),
                correction if correction is not None and correction.success else None,
                target.distance_m,
            ),
            key=f"{widget_prefix}_apply_button",
        )
        st.plotly_chart(
            trajectory_figure(
                target,
                shooter=shooter,
                controls=controls,
                ball=ball,
                optimal=optimal_trajectory,
                measured=tracked,
                before=fit.before_trajectory,
                after=fit.after_trajectory,
                corrected=correction.trajectory if correction is not None else None,
            ),
            width="stretch",
            config={"displayModeBar": False},
            key=f"{widget_prefix}_fit_plot",
        )
        st.caption(
            "Orange should approach the black recording because it validates the fitted physics at the same controls. "
            "The corrected next-shot trace is the control change intended to approach the teal optimum."
        )
        render_fitted_equation(fit, controls, ball, environment, correction)
    return fit


def _add_circle(
    figure: go.Figure,
    center_x: float,
    center_z: float,
    radius: float,
    *,
    line_color: str,
    fill_color: str,
    line_width: float = 2.0,
    layer: str = "above",
) -> None:
    figure.add_shape(
        type="circle",
        x0=center_x - radius,
        x1=center_x + radius,
        y0=center_z - radius,
        y1=center_z + radius,
        line=dict(color=line_color, width=line_width),
        fillcolor=fill_color,
        layer=layer,
    )


def _add_robot_schematic(
    figure: go.Figure,
    shooter: ShooterModel,
    controls: ShotControls,
    ball: BallSpec,
    *,
    dark: bool = False,
) -> None:
    ink_color = "#E8EDF0" if dark else "#080A0D"
    muted_color = "#B8C3CD" if dark else "#5B6570"
    label_background = "rgba(23,34,53,.84)" if dark else "rgba(255,255,255,.82)"
    release = np.array([0.0, shooter.release_height_m], dtype=float)
    launch_angle_deg = launch_elevation_deg(controls, shooter)
    launch_angle_rad = np.radians(launch_angle_deg)
    direction = np.array([np.cos(launch_angle_rad), np.sin(launch_angle_rad)])
    normal = np.array([-direction[1], direction[0]])

    # The release plane is x=0 because no measured muzzle-to-bumper offset is available.
    chassis_front = 0.0
    chassis_rear = chassis_front - ROBOT_LENGTH_M
    chassis_center = (chassis_rear + chassis_front) / 2.0
    drive_radius = DRIVE_WHEEL_DIAMETER_M / 2.0
    chassis_top = max(0.31, shooter.release_height_m - 0.10)
    figure.add_shape(
        type="rect",
        x0=chassis_rear,
        x1=chassis_front,
        y0=drive_radius,
        y1=chassis_top,
        line=dict(color=ink_color, width=2),
        fillcolor="rgba(17,121,238,.20)" if dark else "rgba(17,121,238,.10)",
        layer="below",
    )
    for wheel_x in (
        chassis_center - DRIVE_MODULE_LONGITUDINAL_OFFSET_M,
        chassis_center + DRIVE_MODULE_LONGITUDINAL_OFFSET_M,
    ):
        _add_circle(
            figure,
            wheel_x,
            drive_radius,
            drive_radius,
            line_color=ink_color,
            fill_color="rgba(8,10,13,.92)",
        )

    hood_end = release
    hood_start = hood_end - direction * CONTACT_PATH_LENGTH_M
    hood_offset = normal * (ball.radius_m - BALL_COMPRESSION_M / 2.0)
    for sign in (-1.0, 1.0):
        line_start = hood_start + sign * hood_offset
        line_end = hood_end + sign * hood_offset
        figure.add_shape(
            type="line",
            x0=line_start[0],
            x1=line_end[0],
            y0=line_start[1],
            y1=line_end[1],
            line=dict(color="#1179EE", width=4),
        )

    wheel_contact = (hood_start + hood_end) / 2.0
    compression_per_side = BALL_COMPRESSION_M / 2.0
    top_radius = shooter.top_wheel_diameter_m / 2.0
    bottom_radius = shooter.bottom_wheel_diameter_m / 2.0
    top_center = wheel_contact + normal * (ball.radius_m + top_radius - compression_per_side)
    bottom_center = wheel_contact - normal * (ball.radius_m + bottom_radius - compression_per_side)
    _add_circle(
        figure,
        top_center[0],
        top_center[1],
        top_radius,
        line_color="#087581",
        fill_color="rgba(51,190,204,.72)",
    )
    _add_circle(
        figure,
        bottom_center[0],
        bottom_center[1],
        bottom_radius,
        line_color="#0B5EB6",
        fill_color="rgba(17,121,238,.64)",
    )
    _add_circle(
        figure,
        release[0],
        release[1],
        ball.radius_m,
        line_color="#A86500",
        fill_color="rgba(255,190,0,.78)",
        line_width=2.5,
    )
    arrow_end = release + direction * 0.30
    figure.add_annotation(
        x=arrow_end[0],
        y=arrow_end[1],
        ax=release[0],
        ay=release[1],
        xref="x",
        yref="y",
        axref="x",
        ayref="y",
        showarrow=True,
        arrowhead=3,
        arrowsize=1.1,
        arrowwidth=2,
        arrowcolor="#1179EE",
        text=f"launch {launch_angle_deg:.1f} deg",
        font=dict(size=10, color="#77C8FF" if dark else "#0B5EB6"),
        bgcolor=label_background,
    )
    figure.add_annotation(
        x=chassis_center,
        y=0.23,
        text=f"FRC 6560 | {ROBOT_LENGTH_M:.3f} m",
        showarrow=False,
        font=dict(size=9, color=ink_color),
    )
    figure.add_annotation(
        x=release[0],
        y=release[1] + ball.radius_m + 0.07,
        text=f"release {release[1]:.3f} m",
        showarrow=False,
        font=dict(size=9, color=muted_color),
        bgcolor=label_background,
    )


def _add_hub_schematic(figure: go.Figure, target: Target, ball: BallSpec, *, dark: bool = False) -> None:
    ink_color = "#E8EDF0" if dark else "#080A0D"
    muted_color = "#B8C3CD" if dark else "#5B6570"
    teal_text = "#73DFE8" if dark else "#087581"
    blue_text = "#77C8FF" if dark else "#0B5EB6"
    label_background = "rgba(23,34,53,.86)" if dark else "rgba(255,255,255,.84)"
    body_front = target.distance_m - HUB_BODY_DEPTH_M / 2.0
    body_back = target.distance_m + HUB_BODY_DEPTH_M / 2.0
    rim_front = target.distance_m - HUB_FUNNEL_RIM_SPAN_M / 2.0
    rim_back = target.distance_m + HUB_FUNNEL_RIM_SPAN_M / 2.0
    throat_front = target.distance_m - HUB_FUNNEL_THROAT_SPAN_M / 2.0
    throat_back = target.distance_m + HUB_FUNNEL_THROAT_SPAN_M / 2.0
    rim_height = HUB_FUNNEL_RIM_HEIGHT_M
    throat_height = HUB_FUNNEL_THROAT_HEIGHT_M
    usable_half_span = max(target.opening_span_m / 2.0 - ball.radius_m - target.rim_margin_m, 0.0)
    safe_front = target.distance_m - usable_half_span
    safe_back = target.distance_m + usable_half_span
    center_clearance_height = rim_height + ball.radius_m

    figure.add_shape(
        type="rect",
        x0=body_front,
        x1=body_back,
        y0=0.0,
        y1=throat_height,
        line=dict(width=0),
        fillcolor="rgba(232,237,240,.07)" if dark else "rgba(8,10,13,.055)",
        layer="below",
    )
    for x0, x1, y0, y1 in (
        (body_front, body_front, 0.0, throat_height),
        (body_back, body_back, 0.0, throat_height),
        (body_front, body_back, 0.0, 0.0),
        (body_front, throat_front, throat_height, throat_height),
        (throat_back, body_back, throat_height, throat_height),
    ):
        figure.add_shape(
            type="line",
            x0=x0,
            x1=x1,
            y0=y0,
            y1=y1,
            line=dict(color="rgba(232,237,240,.58)" if dark else "rgba(8,10,13,.58)", width=2, dash="dot"),
            layer="below",
        )

    # Side-view cross-section of the GE-26329 funnel panels. The open space is
    # intentionally not bridged by a solid line at either the rim or throat.
    for rim_x, throat_x in ((rim_front, throat_front), (rim_back, throat_back)):
        figure.add_shape(
            type="line",
            x0=rim_x,
            x1=throat_x,
            y0=rim_height,
            y1=throat_height,
            line=dict(color="#1179EE", width=8),
        )
    rim_stub_m = 0.065
    throat_stub_m = 0.045
    for edge, direction in ((rim_front, -1.0), (rim_back, 1.0)):
        figure.add_shape(
            type="line",
            x0=edge,
            x1=edge + direction * rim_stub_m,
            y0=rim_height,
            y1=rim_height,
            line=dict(color="#1179EE", width=8),
        )
    for edge, direction in ((throat_front, -1.0), (throat_back, 1.0)):
        figure.add_shape(
            type="line",
            x0=edge,
            x1=edge + direction * throat_stub_m,
            y0=throat_height,
            y1=throat_height,
            line=dict(color="#087581", width=5),
        )

    if usable_half_span > 0.0:
        figure.add_shape(
            type="line",
            x0=safe_front,
            x1=safe_back,
            y0=center_clearance_height,
            y1=center_clearance_height,
            line=dict(color="#33BECC", width=3, dash="dash"),
        )
        for unsafe_left, unsafe_right in ((rim_front, safe_front), (safe_back, rim_back)):
            figure.add_shape(
                type="rect",
                x0=unsafe_left,
                x1=unsafe_right,
                y0=rim_height - 0.015,
                y1=center_clearance_height + target.rim_margin_m,
                line=dict(width=0),
                fillcolor="rgba(201,56,56,.10)",
                layer="below",
            )
    figure.add_shape(
        type="line",
        x0=rim_front,
        x1=rim_front,
        y0=rim_height,
        y1=center_clearance_height + target.rim_margin_m,
        line=dict(color="#D97706", width=2, dash="dash"),
    )

    target_radius = ball.radius_m
    _add_circle(
        figure,
        target.distance_m,
        target.center_height_m,
        target_radius,
        line_color="#A86500",
        fill_color="rgba(255,190,0,.32)",
        line_width=2,
    )
    dimension_x = body_back + 0.10
    figure.add_shape(
        type="line",
        x0=dimension_x,
        x1=dimension_x,
        y0=0.0,
        y1=rim_height,
        line=dict(color=muted_color, width=1.5, dash="dash"),
    )
    for height in (0.0, throat_height, rim_height):
        figure.add_shape(
            type="line",
            x0=dimension_x - 0.035,
            x1=dimension_x + 0.035,
            y0=height,
            y1=height,
            line=dict(color=muted_color, width=1.5),
        )
    figure.add_annotation(
        x=target.distance_m,
        y=rim_height + 0.24,
        text=f"upper funnel entrance 41.7 in | {HUB_FUNNEL_RIM_SPAN_M:.4f} m",
        showarrow=False,
        font=dict(size=10, color=blue_text),
        bgcolor=label_background,
    )
    figure.add_annotation(
        x=target.distance_m,
        y=center_clearance_height + 0.04,
        text=f"usable ball-center window {2.0 * usable_half_span:.3f} m",
        showarrow=False,
        font=dict(size=8, color=teal_text),
        bgcolor=label_background,
    )
    figure.add_annotation(
        x=target.distance_m - 0.05,
        y=throat_height - 0.08,
        text=f"lower HUB throat 23.95 in | {HUB_FUNNEL_THROAT_SPAN_M:.4f} m",
        showarrow=False,
        xanchor="right",
        font=dict(size=8, color=teal_text),
        bgcolor=label_background,
    )
    figure.add_annotation(
        x=target.distance_m,
        y=0.10,
        text=f"47 in footprint | {HUB_BODY_DEPTH_M:.4f} m",
        showarrow=False,
        font=dict(size=9, color=ink_color),
        bgcolor=label_background,
    )
    figure.add_annotation(
        x=dimension_x + 0.04,
        y=(throat_height + rim_height) / 2.0,
        text=f"upper rim 72 in<br>{rim_height:.4f} m",
        showarrow=False,
        xanchor="left",
        font=dict(size=9, color=muted_color),
        bgcolor=label_background,
    )
    figure.add_annotation(
        x=dimension_x + 0.04,
        y=throat_height,
        text=f"throat 56.44 in<br>{throat_height:.4f} m",
        showarrow=False,
        xanchor="left",
        font=dict(size=8, color=teal_text),
        bgcolor=label_background,
    )
    figure.add_annotation(
        x=rim_front,
        y=target.center_height_m,
        text=f"required ball center {target.center_height_m:.4f} m",
        showarrow=True,
        arrowhead=2,
        ax=-72,
        ay=48,
        font=dict(size=8, color="#A86500"),
        bgcolor=label_background,
    )


def trajectory_figure(
    target: Target,
    *,
    shooter: ShooterModel,
    controls: ShotControls,
    ball: BallSpec,
    optimal=None,
    measured=None,
    before=None,
    after=None,
    corrected=None,
    llm_candidate=None,
    shot_map=None,
) -> go.Figure:
    figure = go.Figure()
    map_mode = shot_map is not None and shot_map.robust_result is not None
    if map_mode:
        boundary_styles = {
            "lower": ("Lower scoring boundary", "rgba(255,92,92,.38)"),
            "interior": ("Interior scoring shots", "rgba(255,190,0,.20)"),
            "upper": ("Upper scoring boundary", "rgba(65,214,132,.38)"),
        }
        shown_boundaries: set[str] = set()
        for mapped_shot in shot_map.representative_shots:
            label, color = boundary_styles[mapped_shot.boundary]
            launch_speed, _, _ = release_state(mapped_shot.controls, ball, shooter)
            launch_angle = launch_elevation_deg(mapped_shot.controls, shooter)
            figure.add_trace(
                go.Scatter(
                    x=mapped_shot.trajectory.x_m,
                    y=mapped_shot.trajectory.z_m,
                    mode="lines",
                    name=label,
                    legendgroup=mapped_shot.boundary,
                    showlegend=mapped_shot.boundary not in shown_boundaries,
                    line=dict(color=color, width=1.3),
                    hovertemplate=(
                        f"{label}<br>hood command={mapped_shot.controls.hood_angle_deg:.1f} deg"
                        f"<br>launch={launch_angle:.1f} deg<br>speed={launch_speed:.2f} m/s"
                        f"<br>RPM={mapped_shot.controls.top_rpm:.0f}<extra></extra>"
                    ),
                )
            )
            shown_boundaries.add(mapped_shot.boundary)
    if optimal is not None:
        figure.add_trace(
            go.Scatter(
                x=optimal.x_m,
                y=optimal.z_m,
                mode="lines",
                name="Robust optimum" if map_mode else "Recommended optimum",
                line=dict(color="#3C91FF" if map_mode else "#33BECC", width=5 if map_mode else 4),
            )
        )
    if before is not None:
        figure.add_trace(
            go.Scatter(
                x=before.x_m,
                y=before.z_m,
                mode="lines",
                name="Old model at recorded controls",
                line=dict(color="#1179EE", width=2, dash="dash"),
            )
        )
    if after is not None:
        figure.add_trace(
            go.Scatter(
                x=after.x_m,
                y=after.z_m,
                mode="lines",
                name="Calibrated model at same controls",
                line=dict(color="#D97706", width=3),
            )
        )
    if corrected is not None:
        figure.add_trace(
            go.Scatter(
                x=corrected.x_m,
                y=corrected.z_m,
                mode="lines",
                name="Corrected next shot",
                line=dict(color="#A23B72", width=4, dash="dashdot"),
            )
        )
    if llm_candidate is not None:
        figure.add_trace(
            go.Scatter(x=llm_candidate.x_m, y=llm_candidate.z_m, mode="lines", name="LLM candidate", line=dict(color="#080A0D", width=3, dash="dot"))
        )
    if measured is not None:
        if len(measured.extrapolated_x_m):
            figure.add_trace(
                go.Scatter(
                    x=measured.extrapolated_x_m,
                    y=measured.extrapolated_z_m,
                    mode="lines",
                    name="Release reconstruction",
                    line=dict(color="#89939D", width=2, dash="dot"),
                )
            )
        figure.add_trace(
            go.Scatter(
                x=measured.x_m,
                y=measured.z_m,
                mode="lines+markers",
                name="Observed in video",
                line=dict(color="#080A0D", width=3),
                marker=dict(size=5),
            )
        )

    if optimal is not None:
        entry = hub_entry_metrics(optimal, target, ball)
        if entry is not None:
            crossing_x = float(entry["crossing_x_m"])
            figure.add_trace(
                go.Scatter(
                    x=[crossing_x],
                    y=[target.center_height_m],
                    mode="markers",
                    marker=dict(size=11, symbol="circle-open", color="#087581", line=dict(width=3)),
                    showlegend=False,
                    hovertemplate=(
                        f"Scoring-plane crossing<br>x={crossing_x:.3f} m<br>"
                        f"entry={float(entry['entry_angle_deg']):.1f} deg<br>"
                        f"rim clearance={float(entry['near_rim_clearance_m']) * 100:.1f} cm<extra></extra>"
                    ),
                )
            )
            figure.add_annotation(
                x=crossing_x,
                y=target.center_height_m,
                text=f"{float(entry['entry_angle_deg']):.1f} deg entry",
                showarrow=True,
                arrowhead=2,
                ax=-38,
                ay=-42,
                font=dict(size=9, color="#73DFE8" if map_mode else "#087581"),
                bgcolor="rgba(23,34,53,.86)" if map_mode else "rgba(255,255,255,.86)",
            )

    if map_mode:
        valid_rows = np.isfinite(shot_map.lower_speed_m_s) & np.isfinite(shot_map.upper_speed_m_s)
        order = np.argsort(shot_map.launch_elevations_deg[valid_rows])
        map_angles = shot_map.launch_elevations_deg[valid_rows][order]
        lower_speeds = shot_map.lower_speed_m_s[valid_rows][order]
        upper_speeds = shot_map.upper_speed_m_s[valid_rows][order]
        figure.add_trace(
            go.Scatter(
                x=map_angles,
                y=lower_speeds,
                xaxis="x2",
                yaxis="y2",
                mode="lines+markers",
                name="Minimum scoring speed",
                line=dict(color="#FF5C5C", width=2, shape="spline", smoothing=0.45),
                marker=dict(size=4),
                showlegend=False,
                hovertemplate="launch=%{x:.1f} deg<br>minimum=%{y:.2f} m/s<extra></extra>",
            )
        )
        figure.add_trace(
            go.Scatter(
                x=map_angles,
                y=upper_speeds,
                xaxis="x2",
                yaxis="y2",
                mode="lines+markers",
                name="Maximum scoring speed",
                line=dict(color="#41D684", width=2, shape="spline", smoothing=0.45),
                marker=dict(size=4),
                fill="tonexty",
                fillcolor="rgba(51,190,204,.13)",
                showlegend=False,
                hovertemplate="launch=%{x:.1f} deg<br>maximum=%{y:.2f} m/s<extra></extra>",
            )
        )
        selected_speed, _, _ = release_state(controls, ball, shooter)
        selected_angle = launch_elevation_deg(controls, shooter)
        figure.add_trace(
            go.Scatter(
                x=[selected_angle],
                y=[selected_speed],
                xaxis="x2",
                yaxis="y2",
                mode="markers",
                marker=dict(color="#3C91FF", size=13, symbol="star", line=dict(color="#FFFFFF", width=1.5)),
                name="Selected robust point",
                showlegend=False,
                hovertemplate="selected<br>launch=%{x:.1f} deg<br>speed=%{y:.2f} m/s<extra></extra>",
            )
        )
        figure.add_annotation(
            x=0.895,
            y=1.01,
            xref="paper",
            yref="paper",
            text="<b>VALID SHOT REGION</b>",
            showarrow=False,
            font=dict(size=11, color="#E8EDF0"),
        )
    _add_robot_schematic(figure, shooter, controls, ball, dark=map_mode)
    _add_hub_schematic(figure, target, ball, dark=map_mode)
    figure.add_hline(y=0.0, line_color="#89939D", line_width=1)
    x_max = max(target.distance_m + HUB_BODY_DEPTH_M / 2.0 + 0.35, 2.0)
    y_max = max(HUB_OPENING_PLANE_HEIGHT_M + 0.65, target.center_height_m + 0.35)
    for trajectory in (optimal, measured, before, after, corrected, llm_candidate):
        if trajectory is not None and len(trajectory.x_m):
            x_max = max(x_max, float(np.max(trajectory.x_m)) + 0.20)
            y_max = max(y_max, float(np.max(trajectory.z_m)) + 0.20)
    if map_mode:
        for mapped_shot in shot_map.representative_shots:
            if len(mapped_shot.trajectory.x_m):
                x_max = max(x_max, float(np.max(mapped_shot.trajectory.x_m)) + 0.20)
                y_max = max(y_max, float(np.max(mapped_shot.trajectory.z_m)) + 0.20)
    background = "#172235" if map_mode else "#FFFFFF"
    foreground = "#E8EDF0" if map_mode else "#080A0D"
    grid_color = "rgba(184,195,205,.14)" if map_mode else "#E8EDF0"
    figure.update_layout(
        height=600 if map_mode else 520,
        margin=dict(l=10, r=10, t=45 if map_mode else 25, b=10),
        paper_bgcolor=background,
        plot_bgcolor=background,
        font=dict(family="SF Pro Display, -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif", color=foreground),
        hovermode="closest" if map_mode else "x unified",
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="left",
            x=0,
            font=dict(color=foreground, size=10),
        ),
        xaxis=dict(
            title="Horizontal distance from release (m)",
            gridcolor=grid_color,
            zeroline=False,
            range=[-ROBOT_LENGTH_M - 0.14, x_max],
            domain=[0.0, 0.75] if map_mode else [0.0, 1.0],
        ),
        yaxis=dict(
            title="Height above carpet (m)",
            gridcolor=grid_color,
            zeroline=False,
            range=[-0.03, y_max],
        ),
    )
    if map_mode:
        figure.update_layout(
            xaxis2=dict(
                title=dict(text="Launch angle (deg)", font=dict(size=10)),
                domain=[0.80, 0.99],
                anchor="y2",
                gridcolor=grid_color,
                zeroline=False,
                tickfont=dict(size=9),
            ),
            yaxis2=dict(
                title=dict(text="Launch speed (m/s)", font=dict(size=10)),
                domain=[0.58, 0.96],
                anchor="x2",
                gridcolor=grid_color,
                zeroline=False,
                tickfont=dict(size=9),
            ),
        )
    return figure


def save_uploaded_video(uploaded_file) -> Path:
    data = uploaded_file.getvalue()
    signature = hashlib.sha256(data).hexdigest()
    suffix = Path(uploaded_file.name).suffix.lower() or ".mov"
    cache_dir = Path(tempfile.gettempdir()) / "shot-model-lab"
    cache_dir.mkdir(parents=True, exist_ok=True)
    path = cache_dir / f"{signature[:16]}{suffix}"
    if not path.exists():
        path.write_bytes(data)
    if signature != st.session_state.video_signature:
        st.session_state.video_signature = signature
        st.session_state.tracked_shot = None
        st.session_state.fit_result = None
        st.session_state.control_correction = None
        st.session_state.llm_advice = None
        st.session_state.measure_calibration_open = False
        for key in VIDEO_CALIBRATION_WIDGET_KEYS:
            st.session_state.pop(key, None)
    return path


def open_full_shot_map() -> None:
    st.query_params["view"] = "shot-map"


def close_full_shot_map() -> None:
    st.query_params.clear()


def render_full_shot_map_page() -> None:
    context = st.session_state.shot_map_context
    st.markdown(
        """
        <style>
        [data-testid="stMainBlockContainer"] {
          max-width: none !important;
          padding: .7rem 1.25rem 1.5rem !important;
        }
        .full-map-header {
          display: flex;
          align-items: baseline;
          justify-content: space-between;
          border-bottom: 1px solid var(--line);
          margin-bottom: .55rem;
          padding-bottom: .45rem;
        }
        .full-map-header h1 { margin: 0; font-size: 1.55rem; letter-spacing: 0; }
        .full-map-header span { color: var(--muted); font-size: .82rem; }
        </style>
        """,
        unsafe_allow_html=True,
    )
    action_column, title_column = st.columns([0.16, 0.84], vertical_alignment="center")
    with action_column:
        st.button(
            "Back to calculator",
            on_click=close_full_shot_map,
            icon=":material/arrow_back:",
            width="stretch",
        )
    with title_column:
        st.markdown(
            '<div class="full-map-header"><h1>Robust Shot Map</h1><span>FRC 6560 | Charging Champions</span></div>',
            unsafe_allow_html=True,
        )

    if context is None:
        st.warning("Calculate a robust shot map before opening this view.")
        return

    result = context["optimal_result"]
    shot_map = context["shot_map"]
    controls = result.controls
    shooter = context["shooter"]
    ball = context["ball"]
    target = context["target"]
    metrics = result.metrics
    launch_speed, _, _ = release_state(controls, ball, shooter)

    m1, m2, m3, m4, m5, m6 = st.columns(6)
    m1.metric("Command RPM", f"{controls.top_rpm:,.0f}")
    m2.metric("Hood command", f"{controls.hood_angle_deg:.1f}°")
    m3.metric("Launch elevation", f"{launch_elevation_deg(controls, shooter):.1f}°")
    m4.metric("Launch speed", f"{launch_speed:.2f} m/s")
    m5.metric("Entry angle", f"{float(metrics['entry_angle_deg']):.1f}°")
    m6.metric("Flight time", f"{float(metrics['time_s']):.3f} s")

    figure = trajectory_figure(
        target,
        shooter=shooter,
        controls=controls,
        ball=ball,
        optimal=result.trajectory,
        shot_map=shot_map,
    )
    figure.update_layout(height=760, margin=dict(l=20, r=20, t=48, b=20))
    st.plotly_chart(figure, width="stretch", config={"displayModeBar": True, "scrollZoom": True})

    r1, r2, r3, r4, r5 = st.columns(5)
    r1.metric("Valid samples", f"{shot_map.valid_count} / {shot_map.evaluated_count}")
    r2.metric("RPM scoring window", f"-{shot_map.rpm_tolerance_lower:.0f} / +{shot_map.rpm_tolerance_upper:.0f}")
    r3.metric(
        "Hood scoring window",
        f"-{shot_map.hood_tolerance_lower_deg:.1f}° / +{shot_map.hood_tolerance_upper_deg:.1f}°",
    )
    r4.metric("Near-rim clearance", f"{float(metrics['near_rim_clearance_m']) * 100:.1f} cm")
    r5.metric("Entrance clearance", f"{float(metrics['clearance_m']) * 100:.1f} cm")


initialize_state()

if st.query_params.get("view") == "shot-map":
    render_full_shot_map_page()
    st.stop()

st.markdown(
    f"""
    <header class="brand-header">
      <img class="brand-mark" src="data:image/png;base64,{LOGO_DATA}" alt="Charging Champions mark">
      <div class="brand-lockup">
        <div class="brand-kicker">FRC Team 6560</div>
        <div class="brand-name">Charging Champions</div>
        <div class="brand-product">Shot Model Lab</div>
      </div>
      <div class="profile-state"><span>Robot profile</span><strong>Form + repository loaded</strong></div>
    </header>
    <div class="profile-strip">
      <div><span>Mechanism RPM</span><strong>{FLYWHEEL_IDLE_RPM:.0f}-{FLYWHEEL_MAX_RPM:.0f}</strong></div>
      <div><span>Hood command (back ref)</span><strong>{HOOD_MIN_DEG:.1f}-{HOOD_MAX_DEG:.1f} deg</strong></div>
      <div><span>Calibrated distance</span><strong>{SHOT_DISTANCE_MIN_M:.3f}-{SHOT_DISTANCE_MAX_M:.3f} m</strong></div>
    </div>
    """,
    unsafe_allow_html=True,
)
st.markdown(
    """
    <div class="stage-rail">
      <div><span>01</span> Model</div>
      <div><span>02</span> Measure</div>
      <div><span>03</span> Calibrate</div>
    </div>
    """,
    unsafe_allow_html=True,
)
model_flash = st.session_state.pop("model_flash", None)
if model_flash:
    st.success(model_flash)

stage_model, stage_measure, stage_calibrate = st.tabs(["01  Model", "02  Measure", "03  Calibrate"])

with stage_model:
    input_column, result_column = st.columns([0.38, 0.62], gap="large")
    with input_column:
        st.subheader("Required variables")
        with st.expander("Loaded robot profile", expanded=False):
            profile_frame = pd.DataFrame(PROFILE_ROWS, columns=["Variable", "Value", "Source", "Status"])
            st.dataframe(profile_frame, hide_index=True, width="stretch")
            st.caption("RPM fields use mechanism-side values, matching Shooter.setGoal() and ShotCalculator.java.")

        with st.expander("Ball and environment", expanded=True):
            st.markdown('<div class="source-line"><span class="source-chip">Form default</span>Ball values were not checked as confirmed.</div>', unsafe_allow_html=True)
            c1, c2 = st.columns(2)
            ball_mass = c1.number_input("Ball mass (kg)", 0.01, 2.0, BALL_MASS_KG, 0.005)
            ball_diameter = c2.number_input("Ball diameter (m)", 0.02, 0.50, BALL_DIAMETER_M, 0.005)
            drag_coefficient = c1.number_input("Sphere drag coefficient", 0.0, 2.0, 0.47, 0.01)
            air_density = c2.number_input("Air density (kg/m³)", 0.8, 1.5, 1.225, 0.005)
            wind_x = c1.number_input("Wind toward target (m/s)", -10.0, 10.0, 0.0, 0.1)
            robot_velocity = c2.number_input("Robot velocity toward target (m/s)", -6.0, 6.0, 0.0, 0.1)

        with st.expander("Target and release", expanded=True):
            st.markdown(
                '<div class="source-line"><span class="source-chip">Form</span>Release height '
                '<span class="source-chip code">Manual</span>2026 HUB geometry '
                '<span class="source-chip assumption">Model</span>Entry and rim safety</div>',
                unsafe_allow_html=True,
            )
            c1, c2 = st.columns(2)
            target_distance = c1.number_input(
                "Distance to HUB center (m)",
                0.25,
                15.0,
                DEFAULT_TARGET_DISTANCE_FT * FOOT_TO_METER,
                0.05,
                format="%.3f",
            )
            target_height = HUB_OPENING_PLANE_HEIGHT_M + ball_diameter / 2.0
            c2.metric("Ball-center scoring plane", f"{target_height:.4f} m")
            minimum_entry_angle = c1.number_input(
                "Minimum entry angle (deg)",
                0.0,
                60.0,
                HUB_DEFAULT_MIN_ENTRY_ANGLE_DEG,
                1.0,
                help="Hard constraint at the descending ball-center crossing through the upper funnel entrance.",
            )
            rim_margin_in = c2.number_input(
                "Funnel safety margin (in)",
                0.0,
                6.0,
                HUB_DEFAULT_RIM_MARGIN_M / 0.0254,
                0.25,
                help="Extra clearance beyond the ball radius at the front rim and opening edges.",
            )
            release_height = c1.number_input("Release height (m)", 0.05, 3.0, RELEASE_HEIGHT_M, 0.01)
            st.caption(
                "The upper funnel entrance is 41.7 in wide at 72 in; the lower HUB throat is 23.95 in wide at 56.44 in. "
                "Scoring clearance is checked at the upper rim after subtracting ball radius and the selected safety margin."
            )

        with st.expander("Shooter geometry and limits", expanded=True):
            st.markdown('<div class="source-line"><span class="source-chip">Form</span>Wheel geometry <span class="source-chip code">Code</span>RPM and hood-command limits</div>', unsafe_allow_html=True)
            shooter_mode_label = st.segmented_control(
                "Shooter configuration",
                ["Coupled dual wheel", "Powered wheel + fixed hood"],
                default="Coupled dual wheel",
            )
            c1, c2 = st.columns(2)
            top_wheel_diameter = c1.number_input("Top wheel diameter (m)", 0.02, 0.40, TOP_WHEEL_DIAMETER_M, 0.0005, format="%.4f")
            bottom_wheel_diameter = c2.number_input("Bottom wheel diameter (m)", 0.02, 0.40, BOTTOM_WHEEL_DIAMETER_M, 0.0005, format="%.4f")
            min_rpm = c1.number_input("Minimum mechanism RPM", 0.0, 12000.0, FLYWHEEL_IDLE_RPM, 50.0)
            max_rpm = c2.number_input("Maximum mechanism RPM", 100.0, 15000.0, FLYWHEEL_MAX_RPM, 50.0)
            min_hood = c1.number_input("Minimum hood command from back (deg)", 0.0, 89.0, HOOD_MIN_DEG, 0.1)
            max_hood = c2.number_input("Maximum hood command from back (deg)", 0.0, 89.0, HOOD_MAX_DEG, 0.1)
            st.caption(
                f"Physical launch elevation before fitted offset: {90.0 - max_hood:.1f} deg to "
                f"{90.0 - min_hood:.1f} deg. A smaller hood command produces a steeper launch."
            )
            rpm_ratio = st.number_input(
                "Follower / leader RPM ratio",
                0.0,
                2.0,
                FOLLOWER_TO_LEADER_RPM_RATIO,
                0.01,
                disabled=shooter_mode_label != "Coupled dual wheel",
                help="The current robot code commands an opposed follower at the leader's mechanism RPM.",
            )

        with st.expander("Empirical model parameters", expanded=False):
            c1, c2 = st.columns(2)
            velocity_transfer = c1.number_input("RPM-to-speed transfer", 0.20, 1.20, step=0.01, key="model_velocity_transfer")
            spin_transfer = c2.number_input("RPM-to-spin transfer", 0.05, 1.50, step=0.01, key="model_spin_transfer")
            hood_offset = c1.number_input("Launch-angle offset (deg)", -15.0, 15.0, step=0.1, key="model_hood_offset_deg")
            drag_scale = c2.number_input("Drag scale", 0.20, 3.00, step=0.05, key="model_drag_scale")
            lift_slope = c1.number_input("Magnus lift slope", 0.0, 2.50, step=0.05, key="model_lift_slope")
            spin_decay = c2.number_input("Spin decay (1/s)", 0.0, 2.00, step=0.02, key="model_spin_decay_per_s")

        with st.expander("Robust-map tie-breaks", expanded=False):
            time_weight = st.slider("Short flight time", 0.0, 1.0, 0.25, 0.05)
            entry_weight = st.slider("Steep entry angle", 0.0, 1.0, 0.50, 0.05)
            effort_weight = st.slider("Lower mechanism effort", 0.0, 1.0, 0.15, 0.05)

        optimize_clicked = st.button("Calculate robust shot map", type="primary", width="stretch")

    ball = BallSpec(ball_mass, ball_diameter, drag_coefficient)
    environment = Environment(air_density_kg_m3=air_density, wind_x_m_s=wind_x)
    target = Target(
        distance_m=target_distance,
        center_height_m=target_height,
        opening_height_m=ball_diameter,
        opening_span_m=HUB_OPENING_SPAN_M,
        min_entry_angle_deg=minimum_entry_angle,
        rim_margin_m=rim_margin_in * 0.0254,
    )
    shooter = ShooterModel(
        mode="dual_wheel" if shooter_mode_label == "Coupled dual wheel" else "fixed_hood",
        top_wheel_diameter_m=top_wheel_diameter,
        bottom_wheel_diameter_m=bottom_wheel_diameter,
        min_rpm=min_rpm,
        max_rpm=max_rpm,
        min_hood_deg=min_hood,
        max_hood_deg=max_hood,
        bottom_to_top_rpm_ratio=rpm_ratio if shooter_mode_label == "Coupled dual wheel" else 0.0,
        release_height_m=release_height,
        robot_velocity_x_m_s=robot_velocity,
        velocity_transfer=velocity_transfer,
        spin_transfer=spin_transfer,
        hood_offset_deg=hood_offset,
        drag_scale=drag_scale,
        lift_slope=lift_slope,
        spin_decay_per_s=spin_decay,
    )
    weights = OptimizationWeights(time_weight, entry_weight, effort_weight)
    preview_controls = ShotControls(
        top_rpm=min_rpm,
        bottom_rpm=min_rpm * rpm_ratio if shooter.mode == "dual_wheel" else 0.0,
        hood_angle_deg=(min_hood + max_hood) / 2.0,
    )

    if optimize_clicked:
        if max_rpm <= min_rpm or max_hood <= min_hood:
            st.error("Maximum RPM and hood command must be greater than their minimum values.")
        else:
            with st.spinner("Precomputing the valid RPM and hood-command region..."):
                st.session_state.shot_map = build_shot_map(ball, shooter, environment, target, weights)
                st.session_state.optimal_result = (
                    st.session_state.shot_map.robust_result
                    if st.session_state.shot_map.robust_result is not None
                    else optimize_shot(ball, shooter, environment, target, weights)
                )
                st.session_state.optimal_target_distance_m = target.distance_m
                st.session_state.shot_map_context = {
                    "ball": ball,
                    "shooter": shooter,
                    "target": target,
                    "environment": environment,
                    "optimal_result": st.session_state.optimal_result,
                    "shot_map": st.session_state.shot_map,
                }
                use_optimal_controls()

    with result_column:
        optimal_result = st.session_state.optimal_result
        st.subheader("Robust shot recommendation")
        if optimal_result is None:
            st.info("Enter the mechanism and target variables, then calculate the valid shot map.")
            st.plotly_chart(
                trajectory_figure(target, shooter=shooter, controls=preview_controls, ball=ball),
                width="stretch",
                config={"displayModeBar": False},
            )
        else:
            controls = optimal_result.controls
            metrics = optimal_result.metrics
            shot_map = st.session_state.shot_map
            m1, m2, m3, m4 = st.columns(4)
            m1.metric("Command RPM", f"{controls.top_rpm:,.0f}")
            m2.metric("Follower RPM", "Fixed" if controls.bottom_rpm == 0 else f"{controls.bottom_rpm:,.0f}")
            m3.metric("Hood command (back ref)", f"{controls.hood_angle_deg:.1f}°")
            m4.metric("Entry angle", f"{metrics['entry_angle_deg']:.1f}°")
            st.caption(
                f"Launch elevation: {launch_elevation_deg(controls, shooter):.1f}° above horizontal. "
                "The robot hood command is measured from the back reference."
            )
            if shot_map is not None and shot_map.robust_result is not None:
                st.button(
                    "Open full-page shot map",
                    on_click=open_full_shot_map,
                    icon=":material/open_in_full:",
                    type="primary",
                    width="stretch",
                )
            d1, d2, d3, d4 = st.columns(4)
            d1.metric("Flight time", f"{metrics['time_s']:.3f} s")
            d2.metric("Entrance clearance", f"{float(metrics['clearance_m']) * 100:.1f} cm")
            d3.metric("Near-rim clearance", f"{float(metrics['near_rim_clearance_m']) * 100:.1f} cm")
            d4.metric("Entry speed", f"{np.hypot(metrics['vx_m_s'], metrics['vz_m_s']):.2f} m/s")
            if shot_map is not None and shot_map.robust_result is not None:
                r1, r2, r3 = st.columns(3)
                r1.metric("Valid map samples", f"{shot_map.valid_count} / {shot_map.evaluated_count}")
                r2.metric(
                    "RPM scoring window",
                    f"-{shot_map.rpm_tolerance_lower:.0f} / +{shot_map.rpm_tolerance_upper:.0f}",
                )
                r3.metric(
                    "Hood-command window",
                    f"-{shot_map.hood_tolerance_lower_deg:.1f}° / +{shot_map.hood_tolerance_upper_deg:.1f}°",
                )
            if not optimal_result.success:
                st.warning(
                    "No shot within the configured RPM and hood-command limits clears the scoring funnel and satisfies "
                    f"the {target.min_entry_angle_deg:.1f} deg minimum entry angle."
                )
            else:
                st.caption(
                    f"Validated at the upper funnel entrance: {target.min_entry_angle_deg:.1f} deg minimum entry and "
                    f"{target.rim_margin_m / 0.0254:.2f} in extra rim margin."
                )

with stage_measure:
    st.subheader("Measured shot")
    upload_column, settings_column = st.columns([0.56, 0.44], gap="large")
    with upload_column:
        uploaded_video = st.file_uploader("Shot video", type=["mov", "mp4", "m4v"])
        video_path = save_uploaded_video(uploaded_video) if uploaded_video else None
        reference_frame = None
        if uploaded_video is None and st.session_state.video_signature is not None:
            st.session_state.video_signature = None
            st.session_state.tracked_shot = None
            st.session_state.fit_result = None
            st.session_state.control_correction = None
            st.session_state.llm_advice = None
        if uploaded_video:
            st.video(uploaded_video)
            try:
                reference_frame = cached_video_reference_frame(str(video_path))
            except ValueError as error:
                st.error(str(error))
        st.markdown(
            '<p class="compact-note">A fixed near-side view is supported. HUB-reference calibration corrects image scale, tilt, and mild camera obliqueness before paths are compared in meters.</p>',
            unsafe_allow_html=True,
        )

    with settings_column:
        st.markdown("#### Actual shot controls")
        default_top = st.session_state.optimal_result.controls.top_rpm if st.session_state.optimal_result else 2500.0
        default_bottom = st.session_state.optimal_result.controls.bottom_rpm if st.session_state.optimal_result else 2500.0
        default_hood = st.session_state.optimal_result.controls.hood_angle_deg if st.session_state.optimal_result else 33.0
        for key, value in (
            ("actual_top_rpm", default_top),
            ("actual_bottom_rpm", default_bottom),
            ("actual_hood_deg", default_hood),
        ):
            if key not in st.session_state:
                st.session_state[key] = float(value)
        st.button(
            "Use recommended controls",
            disabled=st.session_state.optimal_result is None,
            on_click=use_optimal_controls,
            width="stretch",
        )
        c1, c2, c3 = st.columns(3)
        actual_top_rpm = c1.number_input(
            "Leader RPM", 0.0, 15000.0, step=25.0, key="actual_top_rpm", on_change=invalidate_actual_controls
        )
        actual_bottom_rpm = c2.number_input(
            "Follower RPM", 0.0, 15000.0, step=25.0, key="actual_bottom_rpm", on_change=invalidate_actual_controls
        )
        actual_hood_deg = c3.number_input(
            "Hood command from back (deg)", 0.0, 90.0, step=0.5, key="actual_hood_deg", on_change=invalidate_actual_controls
        )
        st.button(
            "Confirm recorded controls",
            width="stretch",
            on_click=confirm_actual_controls,
            help="Confirm the RPM and rear-referenced hood command that were actually used in this video.",
        )
        recorded_controls_confirmed = (
            st.session_state.actual_controls_signature is not None
            and st.session_state.actual_controls_signature == actual_controls_signature()
        )
        if recorded_controls_confirmed:
            st.caption("Recorded controls confirmed. The dashed prediction and calibration use these values.")
        else:
            st.warning(
                "Confirm the RPM and rear-referenced hood command used in this recording. Tracking can run now, but model comparison and calibration stay off."
            )

        frame_height, frame_width = reference_frame.shape[:2] if reference_frame is not None else (720, 1280)
        st.markdown("#### Video-to-field calibration")
        calibration_mode_label = st.segmented_control(
            "Coordinate calibration",
            ["HUB references", "Direct pixel scale"],
            default="HUB references",
            key="video_calibration_mode",
            help="HUB references correct a near-side camera using the known range and 72 in upper funnel-rim height.",
        )
        recorded_distance_ft = st.number_input(
            "Recorded release-to-HUB center (ft)",
            1.0,
            50.0,
            target.distance_m / FOOT_TO_METER,
            0.25,
            key="video_distance_ft",
        )
        c1, c2 = st.columns(2)
        origin_x = c1.number_input(
            "Robot release X (px)",
            0.0,
            float(frame_width),
            0.109375 * frame_width,
            2.0,
            key="video_release_x_px",
            help="Place this at the shooter muzzle. The tracker reconstructs the hidden path back to this X coordinate.",
        )
        origin_y = c2.number_input(
            "Release Y hint (px)",
            0.0,
            float(frame_height),
            0.5125 * frame_height,
            2.0,
            key="video_release_y_px",
            help="Used to rank candidate paths; the final release Y is fitted from the selected parabola.",
        )
        direction_label = st.selectbox("Travel direction", ["Left to right", "Right to left"])

        if calibration_mode_label == "HUB references":
            c1, c2 = st.columns(2)
            hub_base_x = c1.number_input(
                "HUB base center X (px)", 0.0, float(frame_width), 0.703125 * frame_width, 2.0, key="video_hub_base_x_px"
            )
            hub_base_y = c2.number_input(
                "HUB base on carpet Y (px)", 0.0, float(frame_height), 0.531944 * frame_height, 2.0, key="video_hub_base_y_px"
            )
            hub_opening_x = c1.number_input(
                "Upper funnel-rim center X (px)", 0.0, float(frame_width), 0.714844 * frame_width, 2.0, key="video_hub_opening_x_px"
            )
            hub_opening_y = c2.number_input(
                "Upper funnel-rim center Y (px)", 0.0, float(frame_height), 0.198611 * frame_height, 2.0, key="video_hub_opening_y_px"
            )
            pixels_per_meter = max(abs(hub_base_x - origin_x) / max(recorded_distance_ft * FOOT_TO_METER, 1e-6), 1.0)
        else:
            pixels_per_meter = st.number_input(
                "Pixels per meter",
                1.0,
                10000.0,
                180.0,
                2.0,
                key="video_pixels_per_meter",
            )
            hub_base_x = hub_opening_x = 0.0
            hub_base_y = hub_opening_y = 0.0

        if reference_frame is not None and calibration_mode_label == "HUB references":
            with st.expander("Reference-point preview", expanded=True):
                st.plotly_chart(
                    calibration_reference_figure(
                        reference_frame,
                        release_x_px=origin_x,
                        release_y_px=origin_y,
                        hub_base_x_px=hub_base_x,
                        hub_base_y_px=hub_base_y,
                        hub_opening_x_px=hub_opening_x,
                        hub_opening_y_px=hub_opening_y,
                    ),
                    width="stretch",
                    config={"displayModeBar": False},
                )
                st.caption(
                    "Hover the image to read pixel coordinates. Put the upper-rim marker on the centerline of the funnel's "
                    "72 in entrance. The lower HUB throat is at 56.44 in and is drawn separately."
                )

        c1, c2 = st.columns(2)
        start_time = c1.number_input("Clip start (s)", 0.0, 120.0, 0.0, 0.1)
        end_time = c2.number_input("Clip end (s)", 0.1, 120.0, 30.0, 0.1)
        st.caption(f"Recorded range: {recorded_distance_ft * FOOT_TO_METER:.3f} m. The first visible ball point may occur after release.")

        with st.expander("OpenCV detector tuning", expanded=False):
            c1, c2 = st.columns(2)
            hue_low = c1.slider("Hue low", 0, 179, 20)
            hue_high = c2.slider("Hue high", 0, 179, 35)
            saturation_low = c1.slider("Minimum saturation", 0, 255, 120)
            value_low = c2.slider("Minimum brightness", 0, 255, 100)
            circularity = c1.slider("Shape sensitivity", 0.10, 1.00, 0.60, 0.05)
            min_area = c2.number_input("Minimum blob area (px²)", 1.0, 10000.0, 100.0, 10.0)
            max_jump = c1.number_input("Maximum frame jump (px)", 10.0, 2000.0, 80.0, 10.0)
            launch_radius = c2.number_input("Hint influence radius (px)", 10.0, 2000.0, 180.0, 10.0)
            track_memory = c1.number_input("Track memory (frames)", 1, 120, 15, 1)
            motion_gate = c2.toggle(
                "Motion-assisted detection",
                value=False,
                help="Leave off for fixed-camera yellow-ball footage. Enable only when stationary yellow objects create false tracks.",
            )

        analyze_clicked = st.button("Analyze video", type="primary", width="stretch", disabled=video_path is None)

    actual_controls = ShotControls(actual_top_rpm, actual_bottom_rpm, actual_hood_deg)
    measurement_target = Target(
        distance_m=recorded_distance_ft * FOOT_TO_METER,
        center_height_m=target.center_height_m,
        opening_height_m=target.opening_height_m,
        opening_span_m=target.opening_span_m,
        min_entry_angle_deg=target.min_entry_angle_deg,
        rim_margin_m=target.rim_margin_m,
    )
    if analyze_clicked and video_path is not None:
        config = TrackingConfig(
            origin_x_px=origin_x,
            origin_y_px=origin_y,
            pixels_per_meter=pixels_per_meter,
            release_height_m=release_height,
            coordinate_mode="hub_affine" if calibration_mode_label == "HUB references" else "scale",
            release_x_px=origin_x,
            target_distance_m=measurement_target.distance_m,
            hub_base_x_px=hub_base_x if calibration_mode_label == "HUB references" else None,
            hub_base_y_px=hub_base_y if calibration_mode_label == "HUB references" else None,
            hub_opening_x_px=hub_opening_x if calibration_mode_label == "HUB references" else None,
            hub_opening_y_px=hub_opening_y if calibration_mode_label == "HUB references" else None,
            hub_opening_height_m=HUB_OPENING_PLANE_HEIGHT_M,
            direction=1 if direction_label == "Left to right" else -1,
            hue_low=hue_low,
            hue_high=hue_high,
            saturation_low=saturation_low,
            value_low=value_low,
            min_area_px=min_area,
            circularity_min=circularity,
            max_match_distance_px=max_jump,
            launch_radius_px=launch_radius,
            max_missing_frames=track_memory,
            motion_gate=motion_gate,
            start_time_s=start_time,
            end_time_s=end_time,
        )
        try:
            with st.spinner("Decoding frames and tracking the ball..."):
                st.session_state.tracked_shot = track_video(video_path, config)
                st.session_state.fit_result = None
                st.session_state.control_correction = None
                st.session_state.llm_advice = None
        except ValueError as error:
            st.error(str(error))

    tracked = st.session_state.tracked_shot
    if tracked is not None:
        current_prediction = None
        if recorded_controls_confirmed:
            current_prediction = simulate_shot(
                actual_controls,
                ball,
                shooter,
                environment,
                measurement_target,
                max_time_s=max(1.5, float(tracked.time_s[-1]) + 0.25),
            )
        optimal_matches_video = (
            st.session_state.optimal_result is not None
            and st.session_state.optimal_target_distance_m is not None
            and abs(st.session_state.optimal_target_distance_m - measurement_target.distance_m) < 0.01
        )
        p1, p2, p3, p4, p5, p6 = st.columns(6)
        p1.metric("Valid trajectories", str(tracked.candidate_tracks))
        p2.metric("Selected points", str(tracked.detected_points))
        p3.metric("Ballistic R²", f"{tracked.fit_r2:.3f}")
        p4.metric("Fit RMSE", f"{tracked.fit_rmse_px:.1f} px")
        p5.metric("Track duration", f"{tracked.time_s[-1]:.3f} s")
        p6.metric("Measured range", f"{np.max(tracked.x_m):.2f} m")

        hub_entry = measured_hub_entry(tracked, measurement_target, ball)
        if hub_entry is None:
            st.warning("The tracked segment does not include a descending crossing of the upper funnel-entrance plane.")
        elif hub_entry["inside"]:
            st.success(
                f"Measured result: inside the safe upper funnel entrance at x={hub_entry['crossing_x_m']:.2f} m "
                f"with {hub_entry['clearance_m'] * 100:.1f} cm horizontal clearance, "
                f"{hub_entry['near_rim_clearance_m'] * 100:.1f} cm near-rim clearance, and a "
                f"{hub_entry['entry_angle_deg']:.1f}° entry angle."
            )
            if not hub_entry["meets_min_entry"]:
                st.warning(
                    f"The measured ball enters the funnel, but its {hub_entry['entry_angle_deg']:.1f}° entry is below "
                    f"the configured {measurement_target.min_entry_angle_deg:.1f}° design minimum."
                )
        else:
            st.error(
                f"Measured result: the ball does not clear the safe scoring envelope. Horizontal clearance is "
                f"{hub_entry['clearance_m'] * 100:.1f} cm and near-rim clearance is "
                f"{hub_entry['near_rim_clearance_m'] * 100:.1f} cm. Check the three video reference points before changing physics."
            )

        downward_acceleration = measured_downward_acceleration(tracked)
        if downward_acceleration is not None:
            measured_apex_m = float(np.max(tracked.z_m))
            gravity_difference = abs(downward_acceleration - 9.80665) / 9.80665
            if 6.0 <= downward_acceleration <= 14.0:
                st.caption(
                    f"Video geometry check: the observed path peaks at {measured_apex_m:.2f} m and its quadratic fit has "
                    f"{downward_acceleration:.2f} m/s² downward acceleration "
                    f"({gravity_difference * 100:.0f}% from gravity; drag and spin can contribute)."
                )
            else:
                st.warning(
                    f"Video geometry check failed: fitted downward acceleration is {downward_acceleration:.2f} m/s². "
                    "Recheck the release, HUB-base, and upper funnel-rim reference points before trusting the measured curve."
                )

        if not optimal_matches_video and st.session_state.optimal_result is not None:
            st.info(
                "The Stage 01 recommendation was calculated for a different range, so it is not overlaid."
            )
        if not recorded_controls_confirmed:
            st.info(
                "The observed video path is still valid. The dashed model prediction is hidden until the recorded RPM and hood command are confirmed."
            )
        image_column, plot_column = st.columns([0.46, 0.54], gap="large")
        image_column.image(
            tracked.annotated_frame_rgb,
            caption=(
                "Teal paths passed the ballistic fit; yellow is the selected shot; the teal dot is its first visible point; "
                f"the white cross is the reconstructed release ({tracked.release_gap_s:.3f} s earlier)."
            ),
        )
        plot_column.plotly_chart(
            trajectory_figure(
                measurement_target,
                shooter=shooter,
                controls=actual_controls,
                ball=ball,
                optimal=st.session_state.optimal_result.trajectory if optimal_matches_video else None,
                measured=tracked,
                before=current_prediction,
            ),
            width="stretch",
            config={"displayModeBar": False},
        )
        plot_column.caption(
            "Observed in video is a measurement, not a recommendation. Recommended optimum is teal; a confirmed recorded-control prediction is dashed blue."
        )
        track_frame = pd.DataFrame(
            {
                "time_s": tracked.time_s,
                "frame": tracked.frame_numbers,
                "x_px": tracked.x_px,
                "y_px": tracked.y_px,
                "x_m": tracked.x_m,
                "z_m": tracked.z_m,
            }
        )
        st.download_button(
            "Download tracked points",
            track_frame.to_csv(index=False).encode("utf-8"),
            file_name="tracked_shot.csv",
            mime="text/csv",
        )
        st.button(
            "Calibrate this shot",
            type="primary",
            width="stretch",
            on_click=open_measure_calibration,
            key="open_measure_calibration",
            disabled=not recorded_controls_confirmed,
            help="Confirm the controls used for this recording before fitting physics parameters.",
        )
        if st.session_state.measure_calibration_open and recorded_controls_confirmed:
            st.divider()
            st.subheader("Numerical calibration")
            render_numerical_calibration(
                tracked,
                actual_controls,
                ball,
                shooter,
                environment,
                measurement_target,
                widget_prefix="measure",
                optimal_trajectory=(
                    st.session_state.optimal_result.trajectory if optimal_matches_video else None
                ),
            )

with stage_calibrate:
    st.subheader("Model calibration")
    tracked = st.session_state.tracked_shot
    optimal_result = st.session_state.optimal_result
    if tracked is None:
        st.info("Analyze a shot video in Stage 02 before fitting parameters.")
    elif not recorded_controls_confirmed:
        st.warning("Confirm the RPM and rear-referenced hood command used in Stage 02 before fitting model parameters.")
    else:
        optimal_trajectory = (
            optimal_result.trajectory
            if optimal_result is not None
            and st.session_state.optimal_target_distance_m is not None
            and abs(st.session_state.optimal_target_distance_m - measurement_target.distance_m) < 0.01
            else None
        )
        fit = render_numerical_calibration(
            tracked,
            actual_controls,
            ball,
            shooter,
            environment,
            measurement_target,
            widget_prefix="calibrate",
            optimal_trajectory=optimal_trajectory,
        )
        if fit is not None:
            st.divider()
            st.markdown("### LLM review")
            st.caption("The API receives numeric controls, residual summaries, bounds, and fitted coefficients. The video is not uploaded.")
            l1, l2 = st.columns([0.55, 0.45])
            api_key = l1.text_input("OpenAI API key", value=os.getenv("OPENAI_API_KEY", ""), type="password")
            model_name = l2.text_input("Model", value=os.getenv("OPENAI_MODEL", "gpt-5.6-luna"))
            if st.button("Request constrained parameter review", disabled=not api_key, width="stretch"):
                try:
                    with st.spinner("Requesting a bounded calibration review..."):
                        advice = request_parameter_advice(
                            api_key=api_key,
                            model_name=model_name,
                            observed=tracked,
                            controls=actual_controls,
                            ball=ball,
                            shooter=shooter,
                            environment=environment,
                            target=measurement_target,
                            fit=fit,
                        )
                        llm_model = model_with_candidate(shooter, advice["recommended_parameters"])
                        llm_trajectory = simulate_shot(
                            actual_controls,
                            ball,
                            llm_model,
                            environment,
                            measurement_target,
                            max_time_s=max(0.5, float(tracked.time_s[-1]) + 0.15),
                        )
                        st.session_state.llm_advice = advice
                        st.session_state.llm_candidate_rmse = trajectory_rmse(tracked, llm_trajectory)
                except Exception as error:
                    st.error(f"LLM review failed: {error}")

            advice = st.session_state.llm_advice
            if advice:
                candidate = model_with_candidate(shooter, advice["recommended_parameters"])
                candidate_trajectory = simulate_shot(
                    actual_controls,
                    ball,
                    candidate,
                    environment,
                    measurement_target,
                    max_time_s=max(0.5, float(tracked.time_s[-1]) + 0.15),
                )
                a1, a2 = st.columns(2)
                a1.metric("LLM confidence", str(advice.get("confidence", "unknown")).title())
                a2.metric("LLM candidate RMSE", f"{st.session_state.llm_candidate_rmse:.3f} m")
                st.json(advice)
                llm_improves = st.session_state.llm_candidate_rmse < fit.before_rmse_m
                st.button(
                    "Apply validated LLM candidate",
                    disabled=not llm_improves,
                    on_click=apply_model_parameters,
                    args=(advice["recommended_parameters"],),
                )
                if not llm_improves:
                    st.error("The LLM candidate does not improve measured-path RMSE and cannot be applied.")
                st.plotly_chart(
                    trajectory_figure(
                        measurement_target,
                        shooter=shooter,
                        controls=actual_controls,
                        ball=ball,
                        optimal=optimal_trajectory,
                        measured=tracked,
                        before=fit.before_trajectory,
                        after=fit.after_trajectory,
                        llm_candidate=candidate_trajectory,
                    ),
                    width="stretch",
                    config={"displayModeBar": False},
                )
