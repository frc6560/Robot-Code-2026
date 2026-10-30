#!/usr/bin/env python3
"""Convert manual slow-motion measurements into robot launch calibration. Standard library only."""
import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path

HEADER = ["rpm", "tunnel_rpm", "horizontal_m", "frames", "capture_fps", "launch_angle_deg", "spin_turns", "spin_frames"]
OUTPUT = ["rpm", "exit_speed_mps", "backspin_rps", "tunnel_rpm"]


def number(row, key):
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"{key} must be finite")
    return value


def build(source, destination):
    groups = defaultdict(list)
    tunnels = set()
    spin_presence = set()
    with Path(source).open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != HEADER:
            raise ValueError("Input header must be: " + ",".join(HEADER))
        for line, row in enumerate(reader, 2):
            try:
                rpm, tunnel = number(row, "rpm"), number(row, "tunnel_rpm")
                distance, frames, fps = (number(row, k) for k in ("horizontal_m", "frames", "capture_fps"))
                angle = number(row, "launch_angle_deg")
                if not (1000 <= rpm <= 3800 and 0 < tunnel <= 3000 and distance > 0
                        and frames > 0 and fps > 0 and 0 <= angle < 85):
                    raise ValueError("RPM, distance, timing or angle outside supported range")
                # Horizontal velocity is vx; total launch speed = vx / cos(actual launch angle).
                speed = distance * fps / frames / math.cos(math.radians(angle))
                if not 0 < speed <= 30:
                    raise ValueError("Implausible exit speed; check units and capture frame rate")
                has_spin = bool(row["spin_turns"].strip() or row["spin_frames"].strip())
                spin = None
                if has_spin:
                    turns, spin_frames = number(row, "spin_turns"), number(row, "spin_frames")
                    if spin_frames <= 0:
                        raise ValueError("spin_frames must be positive")
                    spin = turns * fps / spin_frames
                    if abs(spin) > 200:
                        raise ValueError("Implausible spin")
                spin_presence.add(has_spin)
                tunnels.add(tunnel)
                groups[rpm].append((speed, spin))
            except (ValueError, KeyError, TypeError) as error:
                raise ValueError(f"Row {line}: {error}") from error
    if len(tunnels) != 1 or len(groups) < 3 or len(spin_presence) != 1:
        raise ValueError("Need >=3 RPM levels, one fixed tunnel RPM, and spin recorded for all rows or none")
    rows = []
    previous_speed = 0
    report = []
    for rpm, shots in sorted(groups.items()):
        if len(shots) < 3:
            raise ValueError(f"Need >=3 repeat shots at {rpm:g} RPM")
        speeds = [s[0] for s in shots]
        speed = statistics.mean(speeds)
        scatter = statistics.stdev(speeds) / speed
        if speed <= previous_speed:
            raise ValueError("Mean exit speed must rise with measured flywheel RPM; recheck measurements")
        if scatter > 0.05:
            raise ValueError(f"{rpm:g} RPM: speed scatter exceeds 5%; fix repeatability/measurement before fitting")
        previous_speed = speed
        spin = statistics.mean(s[1] for s in shots) if True in spin_presence else ""
        rows.append([f"{rpm:.6f}", f"{speed:.6f}", f"{spin:.6f}" if spin != "" else "", f"{next(iter(tunnels)):.6f}"])
        report.append(f"{rpm:g} RPM: {speed:.3f} m/s, {len(shots)} shots, scatter {100*scatter:.1f}%")
    # Validate all rows before touching an existing calibration.
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    with temporary.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(OUTPUT)
        writer.writerows(rows)
    temporary.replace(destination)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("measurements", type=Path)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[2] / "src/main/deploy/fuel-launch.csv")
    args = parser.parse_args()
    try:
        for line in build(args.measurements, args.output):
            print(line)
        print(f"Wrote {args.output}. Redeploy/restart to load; validate actual basket shots next.")
    except (ValueError, OSError) as error:
        parser.exit(1, f"Calibration rejected: {error}\n")


if __name__ == "__main__":
    main()
