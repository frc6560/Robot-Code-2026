from __future__ import annotations

import json
from typing import Any

from .calibration import FitResult, PARAMETER_BOUNDS, residual_profile
from .models import BallSpec, Environment, ShotControls, ShooterModel, Target
from .tracking import TrackedShot


SYSTEM_INSTRUCTIONS = """
You are reviewing a robot projectile-model calibration. The deterministic optimizer has already fitted a bounded
candidate to one measured side-view trajectory. Do not claim that one shot identifies every coefficient. Recommend
only empirical parameters listed in the supplied bounds. Keep values inside those bounds. Prefer the deterministic
candidate unless the residual pattern supports a smaller conservative change. Return JSON only with this shape:
{
  "recommended_parameters": {"parameter_name": number},
  "confidence": "low|medium|high",
  "reasoning": ["short evidence-based statement"],
  "warnings": ["material limitation"],
  "next_test": "one concrete controlled shot to collect next"
}
The hood control is a robot command measured from the rear/vertical reference. Projectile launch elevation above
horizontal is 90 degrees minus that command plus hood_offset_deg. Never interpret the hood command itself as launch
elevation.
""".strip()


def _extract_json(text: str) -> dict[str, Any]:
    cleaned = text.strip()
    if cleaned.startswith("```"):
        cleaned = cleaned.split("\n", 1)[-1]
        cleaned = cleaned.rsplit("```", 1)[0]
    start = cleaned.find("{")
    end = cleaned.rfind("}")
    if start < 0 or end <= start:
        raise ValueError("The model response did not contain a JSON object.")
    return json.loads(cleaned[start : end + 1])


def request_parameter_advice(
    *,
    api_key: str,
    model_name: str,
    observed: TrackedShot,
    controls: ShotControls,
    ball: BallSpec,
    shooter: ShooterModel,
    environment: Environment,
    target: Target,
    fit: FitResult,
) -> dict[str, Any]:
    from openai import OpenAI

    payload = {
        "coordinate_system": "2D side view; x toward target; z upward; t=0 at first detected point",
        "shot_controls": {
            "top_rpm": controls.top_rpm,
            "bottom_rpm": controls.bottom_rpm,
            "hood_command_from_back_deg": controls.hood_angle_deg,
            "launch_elevation_equation": "90 - hood_command_from_back_deg + hood_offset_deg",
        },
        "ball": {
            "mass_kg": ball.mass_kg,
            "diameter_m": ball.diameter_m,
            "drag_coefficient": ball.drag_coefficient,
        },
        "environment": {
            "air_density_kg_m3": environment.air_density_kg_m3,
            "wind_x_m_s": environment.wind_x_m_s,
        },
        "target": {
            "distance_m": target.distance_m,
            "center_height_m": target.center_height_m,
        },
        "current_parameters": shooter.empirical_parameters(),
        "deterministic_candidate": fit.fitted_model.empirical_parameters(),
        "parameters_fitted": list(fit.fitted_parameters),
        "allowed_bounds": PARAMETER_BOUNDS,
        "before_rmse_m": fit.before_rmse_m,
        "after_rmse_m": fit.after_rmse_m,
        "jacobian_condition": fit.jacobian_condition,
        "residuals_before_fit": residual_profile(observed, fit.before_trajectory),
        "residuals_after_fit": residual_profile(observed, fit.after_trajectory),
        "measurement_summary": {
            "points": observed.detected_points,
            "duration_s": float(observed.time_s[-1]),
            "final_x_m": float(observed.x_m[-1]),
            "final_z_m": float(observed.z_m[-1]),
        },
    }
    client = OpenAI(api_key=api_key)
    response = client.responses.create(
        model=model_name,
        instructions=SYSTEM_INSTRUCTIONS,
        input=json.dumps(payload, separators=(",", ":")),
    )
    advice = _extract_json(response.output_text)
    if not isinstance(advice.get("recommended_parameters"), dict):
        raise ValueError("The model response omitted recommended_parameters.")
    return advice
