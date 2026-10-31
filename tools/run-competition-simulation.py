#!/usr/bin/env python3
"""Run real robot-code/MapleSim autos sequentially (one shared NT/CTRE process at a time)."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess

CHOICES = [f"{alliance}-{position}" for alliance in ("blue", "red") for position in ("left", "center", "right")]
p = argparse.ArgumentParser()
p.add_argument("--output", required=True, type=Path)
p.add_argument("--choices", nargs="+", choices=CHOICES, default=CHOICES)
args = p.parse_args()
root = Path(__file__).resolve().parent.parent
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=True)
inputs = ["src/main/deploy/autos/config.json", "src/main/java/frc/robot/Constants.java",
          "src/main/java/frc/robot/autonomous/AutoCommands.java",
          "src/main/java/frc/robot/autonomous/EventPathAuto.java",
          "src/main/java/frc/robot/simulation/CompetitionAutoDiagnostic.java"]
inputs += [str(f.relative_to(root)) for f in (root / "src/main/deploy/swerve/falcon").rglob("*.json")]
inputs += [f"src/main/deploy/autos/paths/{name}.json" for name in ("hub", "human", "non-human")]
manifest = {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in inputs}
(args.output / "input-sha256.json").write_text(json.dumps(manifest, indent=2) + "\n")
for choice in args.choices:
    run_dir = args.output / choice
    run_dir.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "BLINE_HEADLESS_DIAGNOSTIC": "1", "BLINE_HEADLESS_AUTO": choice,
           "BLINE_COMPARE_LOG_DIR": str(run_dir)}
    print(f"Running {choice} for 15 seconds of autonomous...", flush=True)
    with (run_dir / "console.txt").open("w") as console:
        process = subprocess.Popen(["./gradlew", "simulateJava", "-Pheadless", "--console=plain"],
                                   cwd=root, env=env, stdout=console, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            result = process.wait(timeout=120)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)
            raise RuntimeError(f"Simulation timed out: {choice}")
    summary = run_dir / "summary.txt"
    if result or not summary.exists():
        raise RuntimeError(f"Simulation failed: {choice}; inspect {run_dir / 'console.txt'}")
    print(summary.read_text().strip(), flush=True)
