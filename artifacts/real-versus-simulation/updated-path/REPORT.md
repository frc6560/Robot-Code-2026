# Recorded robot history versus simulated autonomous

Direct comparison of the 169 recorded x/y positions in `LogsFrThePath.csv` against the 440 active-auto saved simulation poses from the latest updated-path simulation. No GUI preview is used as either curve. The simulation reference includes its first post-completion pose, but excludes three seconds of coast monitoring.

| Approximate order-aligned difference | Result |
| --- | ---: |
| RMS | 10.52 cm |
| Mean | 6.52 cm |
| 95th percentile | 28.47 cm |
| Maximum sampled difference | 38.79 cm |

Blue: simulation. Orange: recorded robot estimated positions. Red: configured waypoints. The bottom graph plots difference against distance traveled along the reconstructed recorded robot path, not time.

Same geometry method as preceding plots: both curves resampled every 2 cm traveled distance, monotone XY dynamic-time-warping correspondence, continuous projection onto locally matched simulation segments. No fitted translation, rotation, scale, or endpoint snapping. The numbers are directed recorded-path-to-simulation differences, not a symmetric trajectory distance.

The largest order-aligned difference is near recorded position (7.768, 5.980) m at traveled distance 15.14 m. The matched simulation reference point is approximately (7.398, 5.863) m. Nearby outbound/return/lap portions overlap here. The peak depends on the approximate path-order correspondence; it is not evidence of a simultaneous 38.79 cm physical separation. Unrestricted nearest-point distance anywhere on the simulation curve gives RMS 5.19 cm, P95 11.84 cm, maximum 22.86 cm, but can match the wrong leg/lap and is only an optimistic lower bound.

Important limits: the recorded history has no individual timestamps or headings, so timing and heading differences cannot be established. Recorded positions are the robot's estimate, not independently measured physical position. Its latest-run identity and historical config/PID gains cannot be authenticated from this snapshot. Current simulation uses translation P/I/D 5/0/0.5, rotation 3/0/0.3, cross-track 5/0/0, and the updated path. It defaults to YAGSL's 2025 MapleSim arena rather than a validated 2026 field. Sparse robot-history interpolation can miss intervening motion. The graph does not attribute differences to carpet, localization, or PID.

Artifacts: `path-comparison.png`, `path-comparison.svg`, `analysis-summary.json`, `deviation-by-distance.csv`, and `reconstructed-positions.csv`. The legacy CSV column names `matched_gui_*` identify matched simulation-reference coordinates in this direct-comparison mode, not GUI coordinates; likewise `gui_segment_index` identifies a simulation-reference segment.

Provenance: recorded CSV SHA256 `61ce592a9c09640f57f0349d1a7f44f01f653c554d291e92b96bf94e05d7463b`; simulation trace SHA256 `f93c7b174b3831669735c47d568d143a2db753ae796835fd09498339956da98e`; updated path SHA256 `6db4c6319e7eba25192ea2c288d4d59eb48523cfad93657232e24d13d9090df0`. Neither source trace nor robot gains were changed for this graph.
