from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


HISTORY_VERSION = 1
PARAMETER_NAMES = (
    "velocity_transfer",
    "spin_transfer",
    "hood_offset_deg",
    "drag_scale",
    "lift_slope",
    "max_lift_coefficient",
    "spin_decay_per_s",
)


class CalibrationHistoryError(ValueError):
    pass


def default_history_path() -> Path:
    return Path(__file__).resolve().parents[1] / ".data/calibration_history.json"


def load_calibration_history(path: Path | str | None = None) -> list[dict[str, Any]]:
    history_path = Path(path) if path is not None else default_history_path()
    if not history_path.exists():
        return []
    try:
        payload = json.loads(history_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CalibrationHistoryError(f"Could not read calibration history: {error}") from error
    if not isinstance(payload, dict) or payload.get("version") != HISTORY_VERSION:
        raise CalibrationHistoryError("Calibration history has an unsupported format version.")
    attempts = payload.get("attempts")
    if not isinstance(attempts, list) or not all(isinstance(item, dict) for item in attempts):
        raise CalibrationHistoryError("Calibration history does not contain a valid attempt list.")
    return attempts


def save_calibration_history(
    attempts: list[dict[str, Any]],
    path: Path | str | None = None,
) -> None:
    history_path = Path(path) if path is not None else default_history_path()
    history_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = history_path.with_suffix(history_path.suffix + ".tmp")
    payload = {"version": HISTORY_VERSION, "attempts": attempts}
    try:
        temporary_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
        temporary_path.replace(history_path)
    except OSError as error:
        raise CalibrationHistoryError(f"Could not save calibration history: {error}") from error


def create_calibration_attempt(
    attempts: list[dict[str, Any]],
    *,
    source: str,
    parameters: dict[str, float],
    before_rmse_m: float | None,
    after_rmse_m: float | None,
    target_distance_m: float | None = None,
    top_rpm: float | None = None,
    bottom_rpm: float | None = None,
    hood_command_deg: float | None = None,
    fitted_parameters: list[str] | tuple[str, ...] = (),
    jacobian_condition: float | None = None,
    robot_code_hash: str | None = None,
    video_signature: str | None = None,
    timestamp: str | None = None,
) -> dict[str, Any]:
    missing = [name for name in PARAMETER_NAMES if name not in parameters]
    if missing:
        raise CalibrationHistoryError(
            "Calibration attempt is missing model parameters: " + ", ".join(missing)
        )
    attempt_number = max((int(item.get("attempt", 0)) for item in attempts), default=0) + 1
    improvement_percent = None
    if before_rmse_m is not None and after_rmse_m is not None and before_rmse_m > 1e-12:
        improvement_percent = 100.0 * (before_rmse_m - after_rmse_m) / before_rmse_m
    return {
        "attempt": attempt_number,
        "timestamp": timestamp or datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source": source,
        "target_distance_m": _optional_float(target_distance_m),
        "top_rpm": _optional_float(top_rpm),
        "bottom_rpm": _optional_float(bottom_rpm),
        "hood_command_deg": _optional_float(hood_command_deg),
        "before_rmse_m": _optional_float(before_rmse_m),
        "after_rmse_m": _optional_float(after_rmse_m),
        "improvement_percent": _optional_float(improvement_percent),
        "fitted_parameters": list(fitted_parameters),
        "jacobian_condition": _optional_float(jacobian_condition),
        "robot_code_hash": robot_code_hash,
        "video_signature": video_signature,
        "parameters": {name: float(parameters[name]) for name in PARAMETER_NAMES},
    }


def ensure_robot_code_baseline(
    attempts: list[dict[str, Any]],
    *,
    parameters: dict[str, float],
    robot_code_hash: str,
    timestamp: str | None = None,
) -> tuple[list[dict[str, Any]], bool]:
    already_recorded = any(
        item.get("source") == "Robot code" and item.get("robot_code_hash") == robot_code_hash
        for item in attempts
    )
    if already_recorded:
        return attempts, False
    updated = list(attempts)
    updated.append(
        create_calibration_attempt(
            updated,
            source="Robot code",
            parameters=parameters,
            before_rmse_m=None,
            after_rmse_m=None,
            robot_code_hash=robot_code_hash,
            timestamp=timestamp,
        )
    )
    return updated, True


def history_rows(attempts: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for item in attempts:
        row = {key: value for key, value in item.items() if key != "parameters"}
        row.update(item.get("parameters", {}))
        rows.append(row)
    return rows


def _optional_float(value: float | None) -> float | None:
    return None if value is None else float(value)
