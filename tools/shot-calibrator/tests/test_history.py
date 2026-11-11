import json
from pathlib import Path

from shotlab.history import (
    PARAMETER_NAMES,
    create_calibration_attempt,
    ensure_robot_code_baseline,
    history_rows,
    load_calibration_history,
    save_calibration_history,
)


def parameters(offset: float = 0.0) -> dict[str, float]:
    return {name: float(index) + offset for index, name in enumerate(PARAMETER_NAMES)}


def test_history_round_trip_and_attempt_numbering(tmp_path: Path):
    path = tmp_path / "history.json"
    attempts = [
        create_calibration_attempt(
            [],
            source="Numerical fit",
            parameters=parameters(),
            before_rmse_m=0.20,
            after_rmse_m=0.05,
            target_distance_m=4.572,
            top_rpm=2750.0,
            bottom_rpm=2750.0,
            hood_command_deg=27.0,
            fitted_parameters=["velocity_transfer"],
            timestamp="2026-08-17T12:00:00+00:00",
        )
    ]
    attempts.append(
        create_calibration_attempt(
            attempts,
            source="LLM candidate",
            parameters=parameters(0.1),
            before_rmse_m=0.05,
            after_rmse_m=0.04,
            timestamp="2026-08-17T12:01:00+00:00",
        )
    )

    save_calibration_history(attempts, path)
    loaded = load_calibration_history(path)

    assert [item["attempt"] for item in loaded] == [1, 2]
    assert loaded[0]["improvement_percent"] == 75.0
    assert loaded[1]["parameters"]["velocity_transfer"] == 0.1
    assert history_rows(loaded)[0]["target_distance_m"] == 4.572


def test_robot_code_baseline_is_recorded_once_per_hash():
    first, added = ensure_robot_code_baseline(
        [], parameters=parameters(), robot_code_hash="abc", timestamp="2026-08-17T12:00:00+00:00"
    )
    second, duplicate_added = ensure_robot_code_baseline(
        first, parameters=parameters(), robot_code_hash="abc", timestamp="2026-08-17T12:01:00+00:00"
    )
    third, changed_added = ensure_robot_code_baseline(
        second, parameters=parameters(0.2), robot_code_hash="def", timestamp="2026-08-17T12:02:00+00:00"
    )

    assert added
    assert not duplicate_added
    assert changed_added
    assert len(third) == 2
    assert third[-1]["attempt"] == 2


def test_saved_history_is_versioned_json(tmp_path: Path):
    path = tmp_path / "history.json"
    save_calibration_history([], path)

    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload == {"attempts": [], "version": 1}
