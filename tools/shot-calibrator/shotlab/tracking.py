from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class TrackingConfig:
    origin_x_px: float
    origin_y_px: float
    pixels_per_meter: float
    release_height_m: float
    coordinate_mode: str = "scale"
    release_x_px: float | None = None
    target_distance_m: float | None = None
    hub_base_x_px: float | None = None
    hub_base_y_px: float | None = None
    hub_opening_x_px: float | None = None
    hub_opening_y_px: float | None = None
    hub_opening_height_m: float = 1.8288
    direction: int = 1
    hue_low: int = 20
    hue_high: int = 35
    saturation_low: int = 120
    value_low: int = 100
    min_area_px: float = 100.0
    circularity_min: float = 0.60
    max_match_distance_px: float = 80.0
    launch_radius_px: float = 180.0
    max_missing_frames: int = 15
    min_track_points: int = 6
    min_displacement_px: float = 50.0
    max_flight_time_s: float = 3.0
    ballistic_min_r2: float = 0.90
    motion_gate: bool = False
    start_time_s: float = 0.0
    end_time_s: float = 30.0


@dataclass(frozen=True)
class TrackedShot:
    time_s: np.ndarray
    frame_numbers: np.ndarray
    x_px: np.ndarray
    y_px: np.ndarray
    x_m: np.ndarray
    z_m: np.ndarray
    fps: float
    frame_width: int
    frame_height: int
    annotated_frame_rgb: np.ndarray
    detected_points: int
    decoded_frames: int
    candidate_tracks: int = 1
    rejected_tracks: int = 0
    fit_rmse_px: float = 0.0
    fit_r2: float = 1.0
    calibration_mode: str = "scale"
    release_x_px: float = 0.0
    release_y_px: float = 0.0
    release_gap_s: float = 0.0
    extrapolated_time_s: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    extrapolated_x_m: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    extrapolated_z_m: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))


@dataclass
class _Track:
    points: list[tuple[float, float]]
    frames: list[int]
    missing_count: int = 0


@dataclass(frozen=True)
class _BallisticFit:
    track: _Track
    inlier_mask: np.ndarray
    vx_px_s: float
    vy_px_s: float
    gravity_px_s2: float
    rmse_px: float
    r2: float
    displacement_px: float
    accepted: bool
    reasons: tuple[str, ...] = field(default_factory=tuple)


def _candidate_centers(
    frame: np.ndarray,
    previous_gray: np.ndarray | None,
    config: TrackingConfig,
) -> tuple[list[tuple[float, float]], np.ndarray]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    color_mask = cv2.inRange(
        hsv,
        np.array([config.hue_low, config.saturation_low, config.value_low], dtype=np.uint8),
        np.array([config.hue_high, 255, 255], dtype=np.uint8),
    )
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if config.motion_gate and previous_gray is not None:
        frame_delta = cv2.absdiff(gray, previous_gray)
        _, motion_mask = cv2.threshold(frame_delta, 14, 255, cv2.THRESH_BINARY)
        motion_mask = cv2.dilate(motion_mask, np.ones((9, 9), dtype=np.uint8), iterations=2)
        mask = cv2.bitwise_and(color_mask, motion_mask)
    else:
        mask = color_mask

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), dtype=np.uint8))
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    centers: list[tuple[float, float]] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        perimeter = float(cv2.arcLength(contour, True))
        if area < config.min_area_px or perimeter <= 0.0:
            continue
        circularity = 4.0 * np.pi * area / perimeter**2
        _, _, width, height = cv2.boundingRect(contour)
        aspect_ratio = max(width, height) / max(1, min(width, height))
        shape_ok = circularity >= config.circularity_min
        blurred_ball_ok = config.motion_gate and circularity >= 0.16 and aspect_ratio <= 4.5
        if not (shape_ok or blurred_ball_ok):
            continue
        moments = cv2.moments(contour)
        if moments["m00"]:
            centers.append((moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]))
    return centers, gray


def _predict_position(track: _Track, target_frame: int) -> np.ndarray:
    points = np.asarray(track.points, dtype=float)
    frames = np.asarray(track.frames, dtype=float)
    if len(points) < 2:
        return points[-1]

    sample_count = min(5, len(points))
    recent_points = points[-sample_count:]
    recent_frames = frames[-sample_count:]
    relative_frames = recent_frames - recent_frames[-1]
    elapsed = max(float(target_frame - recent_frames[-1]), 0.0)
    try:
        vx = float(np.polyfit(relative_frames, recent_points[:, 0], 1)[0])
        vy = float(np.polyfit(relative_frames, recent_points[:, 1], 1)[0])
    except (ValueError, np.linalg.LinAlgError):
        frame_gap = max(recent_frames[-1] - recent_frames[-2], 1.0)
        vx, vy = (recent_points[-1] - recent_points[-2]) / frame_gap
    return recent_points[-1] + np.array([vx * elapsed, vy * elapsed + 0.25 * elapsed**2])


def _direction_ok(track: _Track, candidate: tuple[float, float], direction: int) -> bool:
    if len(track.points) < 2:
        return True
    previous = np.asarray(track.points[-1], dtype=float) - np.asarray(track.points[-2], dtype=float)
    proposed = np.asarray(candidate, dtype=float) - np.asarray(track.points[-1], dtype=float)
    if np.linalg.norm(previous) < 1e-6 or np.linalg.norm(proposed) < 1e-6:
        return True
    if direction * proposed[0] < -6.0:
        return False
    cosine = float(np.dot(previous, proposed) / (np.linalg.norm(previous) * np.linalg.norm(proposed)))
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))) < 120.0


def _has_bounced(track: _Track, consistent_frames: int = 3, min_speed_px: float = 2.0) -> bool:
    if len(track.points) < consistent_frames + 2:
        return False
    recent = np.asarray(track.points[-(consistent_frames + 2) :], dtype=float)

    def reversed_after_consistent(deltas: np.ndarray, was_positive: bool) -> bool:
        significant = deltas[np.abs(deltas) >= min_speed_px]
        if len(significant) < consistent_frames:
            return False
        prior, last = significant[:-1], significant[-1]
        if was_positive:
            return int(np.sum(prior > 0)) >= len(prior) - 1 and last < -min_speed_px
        return int(np.sum(prior < 0)) >= len(prior) - 1 and last > min_speed_px

    dx = np.diff(recent[:, 0])
    dy = np.diff(recent[:, 1])
    ground_bounce = reversed_after_consistent(dy, was_positive=True)
    wall_bounce = reversed_after_consistent(dx, was_positive=True) or reversed_after_consistent(dx, was_positive=False)
    return bool(ground_bounce or wall_bounce)


def _mad(values: np.ndarray) -> float:
    median = float(np.median(values))
    return float(1.4826 * np.median(np.abs(values - median)))


def _fit_ballistic_track(track: _Track, fps: float, config: TrackingConfig) -> _BallisticFit:
    points = np.asarray(track.points, dtype=float)
    frames = np.asarray(track.frames, dtype=float)
    empty_mask = np.zeros(len(points), dtype=bool)
    if len(points) < config.min_track_points or len(frames) != len(points):
        return _BallisticFit(track, empty_mask, 0.0, 0.0, 0.0, float("inf"), 0.0, 0.0, False, ("too few points",))

    times = (frames - frames[0]) / fps
    x = config.direction * (points[:, 0] - points[0, 0])
    y = points[:, 1] - points[0, 1]
    mask = np.ones(len(points), dtype=bool)
    vx = vy = gravity = 0.0

    for _ in range(4):
        t = times[mask]
        if len(t) < config.min_track_points:
            break
        try:
            vx = float(np.linalg.lstsq(t[:, None], x[mask], rcond=None)[0][0])
            y_design = np.column_stack((t, 0.5 * t**2))
            vy, gravity = map(float, np.linalg.lstsq(y_design, y[mask], rcond=None)[0])
        except np.linalg.LinAlgError:
            break
        residual = np.hypot(x - vx * times, y - (vy * times + 0.5 * gravity * times**2))
        center = float(np.median(residual[mask]))
        threshold = max(2.5, center + 3.5 * max(_mad(residual[mask]), 0.5))
        next_mask = residual <= threshold
        if int(np.sum(next_mask)) < config.min_track_points or np.array_equal(next_mask, mask):
            break
        mask = next_mask

    if int(np.sum(mask)) < config.min_track_points:
        return _BallisticFit(track, mask, vx, vy, gravity, float("inf"), 0.0, 0.0, False, ("too few inliers",))

    t = times[mask]
    vx = float(np.linalg.lstsq(t[:, None], x[mask], rcond=None)[0][0])
    y_design = np.column_stack((t, 0.5 * t**2))
    vy, gravity = map(float, np.linalg.lstsq(y_design, y[mask], rcond=None)[0])
    x_fit = vx * t
    y_fit = vy * t + 0.5 * gravity * t**2
    errors = np.hypot(x[mask] - x_fit, y[mask] - y_fit)
    rmse = float(np.sqrt(np.mean(errors**2)))
    variance = float(np.sum((x[mask] - np.mean(x[mask])) ** 2 + (y[mask] - np.mean(y[mask])) ** 2))
    r2 = 1.0 - float(np.sum(errors**2)) / variance if variance > 1e-9 else 0.0
    displacement = float(np.linalg.norm(points[-1] - points[0]))
    reasons: list[str] = []
    if displacement < config.min_displacement_px * (30.0 / fps):
        reasons.append("too little displacement")
    if vx <= 0.0:
        reasons.append("no forward motion")
    if gravity <= 0.0:
        reasons.append("non-ballistic acceleration")
    if r2 < config.ballistic_min_r2:
        reasons.append("weak parabolic fit")
    if rmse > max(8.0, 0.08 * max(displacement, 1.0)):
        reasons.append("large fit error")
    if int(np.sum(mask)) < max(config.min_track_points, int(0.6 * len(points))):
        reasons.append("too many outliers")
    return _BallisticFit(track, mask, vx, vy, gravity, rmse, r2, displacement, not reasons, tuple(reasons))


def _selected_fit(fits: list[_BallisticFit], config: TrackingConfig) -> _BallisticFit:
    accepted = [fit for fit in fits if fit.accepted]
    if not accepted:
        best = max(fits, key=lambda fit: (fit.r2, fit.displacement_px), default=None)
        detail = ""
        if best is not None:
            detail = f" Best candidate: R²={best.r2:.2f}, RMSE={best.rmse_px:.1f}px ({', '.join(best.reasons)})."
        raise ValueError(
            "No physically consistent ball trajectory was found. Use the tighter yellow/shape defaults, verify travel direction, "
            f"or trim the clip around a shot.{detail}"
        )

    launch_hint = np.array([config.origin_x_px, config.origin_y_px], dtype=float)

    def score(fit: _BallisticFit) -> float:
        start_distance = float(np.linalg.norm(np.asarray(fit.track.points[0]) - launch_hint))
        hint_penalty = min(start_distance / max(config.launch_radius_px, 1.0), 4.0)
        return 100.0 * fit.r2 + 0.8 * int(np.sum(fit.inlier_mask)) + fit.displacement_px / 20.0 - fit.rmse_px - hint_penalty

    return max(accepted, key=score)


def _release_solution(
    points: np.ndarray,
    frames: np.ndarray,
    fps: float,
    config: TrackingConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    relative_times = (frames - frames[0]) / fps
    if config.release_x_px is None:
        return points[0].copy(), relative_times, np.empty((0, 2), dtype=float), 0.0

    directional_x = config.direction * (points[:, 0] - points[0, 0])
    vx_px_s = float(np.linalg.lstsq(relative_times[:, None], directional_x, rcond=None)[0][0])
    y_design = np.column_stack((relative_times, 0.5 * relative_times**2))
    vy_px_s, gravity_px_s2 = map(
        float,
        np.linalg.lstsq(y_design, points[:, 1] - points[0, 1], rcond=None)[0],
    )
    if vx_px_s <= 1e-6:
        raise ValueError("The selected path has no usable forward velocity for release reconstruction.")

    release_relative_s = config.direction * (float(config.release_x_px) - points[0, 0]) / vx_px_s
    latest_allowed_s = max(2.0 / fps, 0.05)
    if release_relative_s > latest_allowed_s:
        raise ValueError(
            f"Release X={config.release_x_px:.0f}px is after the first detected ball point at "
            f"X={points[0, 0]:.0f}px. Place the release marker at the robot muzzle."
        )
    if release_relative_s < -0.75:
        raise ValueError(
            "The release marker requires more than 0.75 s of backward extrapolation. "
            "Move Release X closer to the robot muzzle or trim the clip around one shot."
        )

    release_y_px = (
        points[0, 1]
        + vy_px_s * release_relative_s
        + 0.5 * gravity_px_s2 * release_relative_s**2
    )
    release_point = np.array([float(config.release_x_px), release_y_px], dtype=float)
    observed_times = relative_times - release_relative_s
    if release_relative_s >= -1e-9:
        return release_point, observed_times, np.empty((0, 2), dtype=float), 0.0

    sample_count = max(3, int(np.ceil(-release_relative_s * fps)) + 2)
    extrapolated_times = np.linspace(release_relative_s, 0.0, sample_count)
    extrapolated_points = np.column_stack(
        (
            points[0, 0] + config.direction * vx_px_s * extrapolated_times,
            points[0, 1]
            + vy_px_s * extrapolated_times
            + 0.5 * gravity_px_s2 * extrapolated_times**2,
        )
    )
    return release_point, observed_times, extrapolated_points, -release_relative_s


def _image_to_world_matrix(config: TrackingConfig, release_point: np.ndarray) -> np.ndarray:
    if config.coordinate_mode == "hub_affine":
        reference_values = (
            config.target_distance_m,
            config.hub_base_x_px,
            config.hub_base_y_px,
            config.hub_opening_x_px,
            config.hub_opening_y_px,
        )
        if any(value is None for value in reference_values):
            raise ValueError("HUB-reference calibration requires the HUB base and upper funnel-rim pixel coordinates.")
        if float(config.target_distance_m) <= 0.0 or config.hub_opening_height_m <= 0.0:
            raise ValueError("HUB-reference distances and heights must be positive.")

        source = np.float32(
            [
                release_point,
                [config.hub_base_x_px, config.hub_base_y_px],
                [config.hub_opening_x_px, config.hub_opening_y_px],
            ]
        )
        first_edge = source[1] - source[0]
        second_edge = source[2] - source[0]
        twice_area = abs(float(first_edge[0] * second_edge[1] - first_edge[1] * second_edge[0]))
        if twice_area < 100.0:
            raise ValueError("The release, HUB base, and upper funnel-rim reference points are nearly collinear.")
        destination = np.float32(
            [
                [0.0, config.release_height_m],
                [config.target_distance_m, 0.0],
                [config.target_distance_m, config.hub_opening_height_m],
            ]
        )
        return cv2.getAffineTransform(source, destination)

    if config.pixels_per_meter <= 0.0:
        raise ValueError("Pixels per meter must be positive.")
    scale = 1.0 / config.pixels_per_meter
    return np.array(
        [
            [config.direction * scale, 0.0, -config.direction * release_point[0] * scale],
            [0.0, -scale, config.release_height_m + release_point[1] * scale],
        ],
        dtype=float,
    )


def _transform_points(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return np.empty((0, 2), dtype=float)
    homogeneous = np.column_stack((points, np.ones(len(points), dtype=float)))
    return homogeneous @ matrix.T


def video_reference_frame(video_path: str | Path, time_s: float = 0.0) -> np.ndarray:
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise ValueError("OpenCV could not open the uploaded video for calibration.")
    capture.set(cv2.CAP_PROP_POS_MSEC, max(0.0, time_s) * 1000.0)
    ok, frame = capture.read()
    capture.release()
    if not ok:
        raise ValueError("OpenCV could not decode a calibration frame from the uploaded video.")
    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


def track_video(video_path: str | Path, config: TrackingConfig) -> TrackedShot:
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise ValueError("OpenCV could not open the uploaded video. Convert it to H.264 MOV/MP4 and retry.")

    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if not np.isfinite(fps) or fps <= 0.0:
        fps = 30.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    start_frame = max(0, int(round(config.start_time_s * fps)))
    end_frame = int(round(config.end_time_s * fps)) if config.end_time_s > config.start_time_s else 2**31 - 1
    capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    active_tracks: list[_Track] = []
    completed_tracks: list[_Track] = []
    previous_gray = None
    decoded = 0
    frame_number = start_frame
    representative_frame = None
    max_track_frames = max(1, int(round(config.max_flight_time_s * fps)))

    def finalize(track: _Track, *, remove_last: bool = False) -> None:
        if remove_last and len(track.points) > 1:
            track.points.pop()
            track.frames.pop()
        if len(track.points) >= config.min_track_points:
            completed_tracks.append(track)

    while frame_number <= end_frame:
        ok, frame = capture.read()
        if not ok:
            break
        decoded += 1
        representative_frame = frame.copy()
        centers, current_gray = _candidate_centers(frame, previous_gray, config)
        previous_gray = current_gray

        potential_matches: list[tuple[float, int, int]] = []
        for center_index, center in enumerate(centers):
            for track_index, track in enumerate(active_tracks):
                predicted = _predict_position(track, frame_number)
                distance = float(np.linalg.norm(np.asarray(center) - predicted))
                gap = max(1, frame_number - track.frames[-1])
                threshold = config.max_match_distance_px * min(3.0, np.sqrt(gap))
                if distance <= threshold and _direction_ok(track, center, config.direction):
                    potential_matches.append((distance, center_index, track_index))
        potential_matches.sort(key=lambda match: match[0])

        matched_centers: set[int] = set()
        matched_tracks: set[int] = set()
        next_active: list[_Track] = []
        for _, center_index, track_index in potential_matches:
            if center_index in matched_centers or track_index in matched_tracks:
                continue
            track = active_tracks[track_index]
            track.points.append(centers[center_index])
            track.frames.append(frame_number)
            track.missing_count = 0
            next_active.append(track)
            matched_centers.add(center_index)
            matched_tracks.add(track_index)

        for center_index, center in enumerate(centers):
            if center_index not in matched_centers:
                next_active.append(_Track(points=[center], frames=[frame_number]))

        for track_index, track in enumerate(active_tracks):
            if track_index in matched_tracks:
                continue
            track.missing_count += 1
            if track.missing_count >= config.max_missing_frames:
                finalize(track)
            else:
                next_active.append(track)

        still_active: list[_Track] = []
        for track in next_active:
            age = frame_number - track.frames[0]
            if _has_bounced(track):
                finalize(track, remove_last=True)
            elif age >= max_track_frames:
                finalize(track)
            else:
                still_active.append(track)
        active_tracks = still_active
        frame_number += 1

    capture.release()
    for track in active_tracks:
        finalize(track)

    fits = [_fit_ballistic_track(track, fps, config) for track in completed_tracks]
    selected = _selected_fit(fits, config)
    accepted_fits = [fit for fit in fits if fit.accepted]
    points = np.asarray(selected.track.points, dtype=float)[selected.inlier_mask]
    frames = np.asarray(selected.track.frames, dtype=int)[selected.inlier_mask]
    order = np.argsort(frames)
    points = points[order]
    frames = frames[order]
    release_point, times, extrapolated_points, release_gap_s = _release_solution(points, frames, fps, config)
    world_matrix = _image_to_world_matrix(config, release_point)
    world_points = _transform_points(points, world_matrix)
    extrapolated_world = _transform_points(extrapolated_points, world_matrix)
    x_m = world_points[:, 0]
    z_m = world_points[:, 1]

    annotated = representative_frame if representative_frame is not None else np.zeros((height, width, 3), dtype=np.uint8)
    for fit in accepted_fits:
        candidate_points = np.round(np.asarray(fit.track.points, dtype=float)[fit.inlier_mask]).astype(np.int32)
        if len(candidate_points) >= 2:
            cv2.polylines(annotated, [candidate_points.reshape((-1, 1, 2))], False, (204, 190, 51), 2, cv2.LINE_AA)
    selected_polyline = np.round(points).astype(np.int32).reshape((-1, 1, 2))
    cv2.polylines(annotated, [selected_polyline], False, (0, 186, 255), 5, cv2.LINE_AA)
    if len(extrapolated_points) >= 2:
        extrapolated_polyline = np.round(extrapolated_points).astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(annotated, [extrapolated_polyline], False, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.circle(annotated, tuple(np.round(points[0]).astype(int)), 11, (51, 190, 204), -1)
    cv2.circle(annotated, tuple(np.round(points[-1]).astype(int)), 11, (17, 121, 238), -1)
    cv2.drawMarker(annotated, tuple(np.round(release_point).astype(int)), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)

    return TrackedShot(
        time_s=times,
        frame_numbers=frames,
        x_px=points[:, 0],
        y_px=points[:, 1],
        x_m=x_m,
        z_m=z_m,
        fps=fps,
        frame_width=width,
        frame_height=height,
        annotated_frame_rgb=cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB),
        detected_points=len(points),
        decoded_frames=decoded,
        candidate_tracks=len(accepted_fits),
        rejected_tracks=len(fits) - len(accepted_fits),
        fit_rmse_px=selected.rmse_px,
        fit_r2=selected.r2,
        calibration_mode=config.coordinate_mode,
        release_x_px=float(release_point[0]),
        release_y_px=float(release_point[1]),
        release_gap_s=release_gap_s,
        extrapolated_time_s=(
            np.linspace(0.0, release_gap_s, len(extrapolated_world))
            if len(extrapolated_world)
            else np.array([], dtype=float)
        ),
        extrapolated_x_m=extrapolated_world[:, 0],
        extrapolated_z_m=extrapolated_world[:, 1],
    )
