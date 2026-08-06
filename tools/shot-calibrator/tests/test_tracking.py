import cv2
import numpy as np

from shotlab.tracking import TrackingConfig, track_video


def test_tracker_follows_synthetic_yellow_ball(tmp_path):
    path = tmp_path / "synthetic-shot.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 60.0, (320, 240))
    assert writer.isOpened()
    for frame_index in range(28):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        x = 48 + 8 * frame_index
        y = int(180 - 7.5 * frame_index + 0.25 * frame_index**2)
        cv2.circle(frame, (x, y), 9, (0, 255, 255), -1)
        writer.write(frame)
    writer.release()

    tracked = track_video(
        path,
        TrackingConfig(
            origin_x_px=48,
            origin_y_px=180,
            pixels_per_meter=100.0,
            release_height_m=0.8,
            min_area_px=80.0,
            max_match_distance_px=35.0,
            launch_radius_px=30.0,
            motion_gate=False,
            end_time_s=1.0,
        ),
    )
    assert tracked.detected_points >= 25
    assert tracked.x_m[-1] > 2.0
    assert np.max(tracked.z_m) > 1.3


def test_tracker_rejects_yellow_distractors_and_bad_release_hint(tmp_path):
    path = tmp_path / "synthetic-shot-with-distractors.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 60.0, (400, 300))
    assert writer.isOpened()
    for frame_index in range(60):
        frame = np.zeros((300, 400, 3), dtype=np.uint8)
        ball_x = 35 + 5 * frame_index
        ball_y = int(240 - 5.0 * frame_index + 0.08 * frame_index**2)
        cv2.circle(frame, (ball_x, ball_y), 9, (0, 255, 255), -1)

        cv2.circle(frame, (200, 55), 13, (0, 255, 255), -1)
        zigzag_x = 40 + 4 * frame_index
        zigzag_y = 45 + (frame_index % 10) * 8
        cv2.circle(frame, (zigzag_x, zigzag_y), 8, (0, 255, 255), -1)
        writer.write(frame)
    writer.release()

    tracked = track_video(
        path,
        TrackingConfig(
            origin_x_px=360,
            origin_y_px=285,
            pixels_per_meter=100.0,
            release_height_m=0.8,
            min_area_px=80.0,
            max_match_distance_px=35.0,
            launch_radius_px=30.0,
            motion_gate=False,
            end_time_s=1.2,
        ),
    )

    assert tracked.fit_r2 > 0.97
    assert tracked.candidate_tracks >= 1
    assert tracked.rejected_tracks >= 1
    assert np.linalg.norm(np.array([tracked.x_px[0] - 35, tracked.y_px[0] - 240])) < 5
    assert np.max(np.linalg.norm(np.diff(np.column_stack((tracked.x_px, tracked.y_px)), axis=0), axis=1)) < 15


def test_hub_reference_calibration_recovers_world_path_and_hidden_release(tmp_path):
    path = tmp_path / "affine-shot.avi"
    fps = 60.0
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), fps, (360, 260))
    assert writer.isOpened()

    expected_world = []
    for frame_index in range(42):
        frame = np.zeros((260, 360, 3), dtype=np.uint8)
        time_s = frame_index / fps
        x_m = 5.0 * time_s
        z_m = 0.8 + 4.5 * time_s - 4.9 * time_s**2
        x_px = 60.0 + 75.0 * x_m
        y_px = 220.0 + 5.0 * x_m - 80.0 * z_m
        if frame_index >= 6:
            cv2.circle(frame, (round(x_px), round(y_px)), 8, (0, 255, 255), -1)
            expected_world.append((x_m, z_m))
        writer.write(frame)
    writer.release()

    tracked = track_video(
        path,
        TrackingConfig(
            origin_x_px=60.0,
            origin_y_px=156.0,
            pixels_per_meter=100.0,
            release_height_m=0.8,
            coordinate_mode="hub_affine",
            release_x_px=60.0,
            target_distance_m=3.0,
            hub_base_x_px=285.0,
            hub_base_y_px=235.0,
            hub_opening_x_px=285.0,
            hub_opening_y_px=91.0,
            hub_opening_height_m=1.8,
            min_area_px=60.0,
            max_match_distance_px=35.0,
            launch_radius_px=120.0,
            end_time_s=1.0,
        ),
    )

    expected = np.asarray(expected_world[: tracked.detected_points])
    assert tracked.calibration_mode == "hub_affine"
    assert tracked.release_gap_s > 0.08
    assert tracked.time_s[0] > 0.08
    np.testing.assert_allclose(tracked.extrapolated_x_m[0], 0.0, atol=0.02)
    np.testing.assert_allclose(tracked.extrapolated_z_m[0], 0.8, atol=0.02)
    np.testing.assert_allclose(tracked.x_m, expected[:, 0], atol=0.04)
    np.testing.assert_allclose(tracked.z_m, expected[:, 1], atol=0.04)
