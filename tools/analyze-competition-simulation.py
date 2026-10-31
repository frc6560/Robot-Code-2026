#!/usr/bin/env python3
"""Summarize full-rate physics traces and draw saved, shareable path overlays."""
import argparse
import csv
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

p = argparse.ArgumentParser()
p.add_argument("output", type=Path)
args = p.parse_args()
root = Path(__file__).resolve().parent.parent
L, W = 16.54, 8.07
hub_side = 47 * .0254
fig, axes = plt.subplots(2, 3, figsize=(17, 9), constrained_layout=True)
results = []

def conservative_hub_clearance(sample):
    x, y, angle = [float(sample[k]) for k in ("x", "y", "heading")]
    c, s = math.cos(angle), math.sin(angle)
    nearest = float("inf")
    for hx, hy in ((4.62, 4.03), (11.92, 4.03)):
        separation = max(abs((hx-x)*ux+(hy-y)*uy)
            - .406*(abs(ux*c+uy*s)+abs(-ux*s+uy*c))
            - hub_side/2*(abs(ux)+abs(uy))
            for ux, uy in ((1, 0), (0, 1), (c, s), (-s, c)))
        nearest = min(nearest, separation)
    return nearest

for row, alliance in enumerate(("blue", "red")):
    for col, (position, source) in enumerate((("left", "non-human"), ("center", "hub"), ("right", "human"))):
        name = f"{alliance}-{position}"
        folder = args.output / name
        with (folder / "trace.csv").open() as f:
            samples = list(csv.DictReader(f))
        raw = dict(item.split("=", 1) for item in (folder / "summary.txt").read_text().split())
        summary = {k: (v if k == "selection" else float(v)) for k, v in raw.items()}
        closest = min(samples, key=lambda r: float(r["wall_clearance_0812"]))
        summary["closest_wall_time_seconds"] = float(closest["time"])
        summary["closest_wall_robot_center"] = [float(closest[k]) for k in ("x", "y")]
        summary["closest_wall_heading_degrees"] = math.degrees(float(closest["heading"]))
        # No deployed-intake geometry is available. Record how little forward protrusion
        # beyond this conservative bumper would consume the left auto's wall clearance.
        if position == "left":
            summary["front_extension_margin_to_wall_meters"] = float(closest["wall_clearance_0812"])

        summary["min_hub_clearance_0812_meters"] = min(map(conservative_hub_clearance, samples))
        summary["final_robot_center"] = [float(samples[-1][k]) for k in ("x", "y")]
        results.append(summary)
        ax = axes[row][col]
        ax.add_patch(Rectangle((0, 0), L, W, facecolor="#f3f5f7", edgecolor="#263238", lw=2))
        for x, y in ((4.62, 4.03), (11.92, 4.03)):
            ax.add_patch(Rectangle((x-hub_side/2, y-hub_side/2), hub_side, hub_side,
                                   facecolor="#88939d", edgecolor="#495057"))
            ax.text(x, y, "HUB", ha="center", va="center", fontsize=7, color="white")
        path = json.loads((root / f"src/main/deploy/autos/paths/{source}.json").read_text())
        points = []
        for e in path["path_elements"]:
            target = e.get("translation_target") if e["type"] == "waypoint" else e if e["type"] == "translation" else None
            if target is not None:
                x, y = target["x_meters"], target["y_meters"]
                points.append((L-x, W-y) if alliance == "red" else (x, y))
        ax.plot(*zip(*points), "--o", color="#a8adb5", markersize=3, lw=1, label="B-Line targets")
        color = "#1976d2" if alliance == "blue" else "#c62828"
        ax.plot([float(s["x"]) for s in samples], [float(s["y"]) for s in samples], color=color, lw=1.7, label="Physics trace")
        ax.scatter(float(samples[0]["x"]), float(samples[0]["y"]), marker="s", color="#26834a", s=30, zorder=5)
        # Show the conservative .812 m bumper envelope at the minimum wall-clearance point.
        x, y, angle = [float(closest[k]) for k in ("x", "y", "heading")]
        c, s = math.cos(angle), math.sin(angle)
        corners = [(x+c*dx-s*dy, y+s*dx+c*dy) for dx, dy in ((-.406,-.406),(.406,-.406),(.406,.406),(-.406,.406))]
        ax.add_patch(Polygon(corners, fill=False, edgecolor="#ff8f00", lw=1.5))
        status = "CONTACT" if summary["wallContactSamples"] or summary["hubContactSamples"] else "No wall/hub contact"
        ax.set_title(f"{alliance.title()} {position.title()} — {status}\n"
                     f"Wall margin (.812 m body): {summary['minWall0812Meters']:.2f} m; "
                     f"shoot starts: {summary['firstShotSeconds']:.2f} s", fontsize=10)
        ax.set_xlim((-.2, 8.4) if alliance == "blue" else (L-8.4, L+.2))
        ax.set_ylim(-.2, W+.2); ax.set_aspect("equal")
        ax.set_xlabel("Field x (m)"); ax.set_ylabel("Field y (m)")
        ax.grid(alpha=.15); ax.legend(loc="upper center", fontsize=7)
fig.suptitle("2026 competition autos — 15-second robot-code/MapleSim runs\n"
             "Walls and hub footprints only; ramps, trench height, tower, intake envelope and other robots are not modeled", fontsize=13)
fig.savefig(args.output / "six-auto-traces.png", dpi=180)
(args.output / "results.json").write_text(json.dumps(results, indent=2)+"\n")
for r in results:
    print(r)
