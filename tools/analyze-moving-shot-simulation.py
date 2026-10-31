#!/usr/bin/env python3
"""Plot/report the production-controller moving-shot diagnostic (requires matplotlib).

uv run --with matplotlib tools/analyze-moving-shot-simulation.py
"""
import argparse
import csv
import hashlib
import json
import re
from pathlib import Path
import shutil
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, default=Path('build/reports/moving-shot-simulation'))
parser.add_argument('--output', type=Path, default=Path('artifacts/moving-shot-simulation'))
parser.add_argument('--comparison', type=Path, help='Previous simulation report directory for the stationary PID comparison')
args = parser.parse_args()
source, output = args.source.resolve(), args.output.resolve()
output.mkdir(parents=True, exist_ok=True)


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def shooter_gains(directory):
    text = (directory / 'configuration.txt').read_text()
    match = re.search(r'Shooter PID: ([\d.]+), ([\d.]+), ([\d.]+)', text)
    if not match:
        raise ValueError(f'Missing shooter gains in {directory}')
    return tuple(float(v) for v in match.groups())


summary = rows(source / 'summary.csv')
current_p, current_i, current_d = shooter_gains(source)
stress = any(r['scenario'] == 'blue-orbit' for r in summary)
motion_end = 8.5 if stress else 4.5
comparison = args.comparison.resolve() if args.comparison else None
if comparison:
    previous_p, previous_i, previous_d = shooter_gains(comparison)
    comparison_trace = rows(comparison / 'stationary-trace.csv')
    comparison_label = f'Previous D = {previous_d:g}'
else:
    comparison_trace = rows(source / 'stationary-p-only-reference-trace.csv')
    comparison_label = 'Control D = 0'

for item in source.glob('*'):
    if item.is_file():
        shutil.copy2(item, output / item.name)

traces = {r['scenario']: rows(source / (r['scenario'] + '-trace.csv')) for r in summary}
shots = {r['scenario']: rows(source / (r['scenario'] + '-shots.csv')) for r in summary}
labels = {
    'stationary': 'Stationary', 'approach': 'Approach, 0.5 m/s', 'retreat': 'Retreat, 0.5 m/s',
    'sideways-1mps': 'Sideways, 1 m/s', 'sideways-2mps': 'Sideways, 2 m/s',
    'diagonal': 'Diagonal, 0.4 / 0.8 m/s', 'reversal': 'Reverse sideways direction',
    'move-stop': 'Sideways then stop', 'red-sideways': 'Red alliance, sideways 1 m/s',
    'stationary-p-only-reference': 'Control: stationary with D disabled',
    'blue-orbit': 'Blue alliance, curved arc', 'red-orbit': 'Red alliance, curved arc',
    'blue-figure-eight': 'Blue alliance, figure eight', 'red-figure-eight': 'Red alliance, figure eight',
    'near-weave': 'Near weaving, start 2.7 m', 'far-weave': 'Far weaving, start 4.0 m',
}
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
                     'figure.facecolor': '#fafbfc', 'axes.facecolor': '#fafbfc'})
fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex='col')
tracking_names = ['blue-orbit', 'blue-figure-eight'] if stress else ['sideways-2mps', 'reversal']
for col, name in enumerate(tracking_names):
    trace = traces[name]
    t = [float(r['time']) for r in trace]
    feeding = [r['feeding'] == 'true' for r in trace]
    axes[0, col].plot(t, [float(r['goal_rpm']) for r in trace], color='#2563eb', label='Requested RPM')
    axes[0, col].plot(t, [float(r['measured_rpm']) for r in trace], color='#f59e0b', label='Actual RPM')
    axes[0, col].set_title(labels[name])
    axes[0, col].set_ylabel('Flywheel RPM')
    axes[0, col].legend(loc='lower right', fontsize=8)
    axes[1, col].plot(t, [float(r['heading_error_deg']) for r in trace], color='#7c3aed')
    axes[1, col].axhspan(-7.5, 7.5, color='#7c3aed', alpha=.07)
    axes[1, col].axhline(0, color='#64748b', lw=.6)
    axes[1, col].set_ylabel('Aim error (degrees)')
    axes[2, col].step(t, [int(v) for v in feeding], where='post', color='#059669', label='Feeding')
    axes[2, col].plot(t, [(float(r['vx'])**2 + float(r['vy'])**2)**.5 for r in trace],
                      color='#334155', label='Actual speed (m/s)')
    axes[2, col].set_ylabel('Feed state / speed')
    axes[2, col].set_xlabel('Simulation time (seconds)')
    axes[2, col].legend(loc='upper left', fontsize=8)
    for row in range(3):
        axes[row, col].axvspan(1.5, motion_end, color='#94a3b8', alpha=.09)
        axes[row, col].grid(alpha=.18)
fig.suptitle('Production PID controllers in closed-loop simulation\n' +
             (source / 'configuration.txt').read_text().splitlines()[0].split(';')[0] + '  |  ' +
             (source / 'configuration.txt').read_text().splitlines()[2].split(';')[0], fontsize=14)
fig.tight_layout(rect=(0, 0, 1, .93))
fig.savefig(output / 'tracking.png', dpi=170)
plt.close(fig)

fig, ax = plt.subplots(figsize=(11, 5.5))
half_width = 41.7 * .0254 / 2 - .15 / 2 - .0254
for i, row in enumerate(summary):
    name = row['scenario']
    data = shots[name]
    # Deterministic horizontal spread keeps coincident samples visible.
    x = [i + ((j % 7) - 3) * .025 for j in range(len(data))]
    colors = ['#dc2626' if r['scored'] == 'false' else
              '#2563eb' if float(r['speed']) > .15 else '#94a3b8' for r in data]
    ax.scatter(x, [100 * float(r['miss_m']) for r in data], s=25, c=colors, alpha=.7)
ax.axhline(half_width * 100, color='#dc2626', linestyle='--', label='Usable opening: 42.9 cm from center')
ax.set_xticks(range(len(summary)), [labels[r['scenario']] for r in summary], rotation=25, ha='right')
if not any(shots.values()):
    ax.text(.5, .5, 'No releases with the current P/D motor-loop model', ha='center',
            va='center', transform=ax.transAxes, fontsize=13)
ax.set_ylabel('Distance from hub center at descending rim crossing (cm)')
ax.set_title('Independent 3D flight check of sampled releases\nShared provisional aerodynamic constants; inherited velocity added at release')
from matplotlib.lines import Line2D
ax.legend(handles=[
    Line2D([], [], color='#dc2626', linestyle='--', label='Usable opening: 42.9 cm from center'),
    Line2D([], [], color='#2563eb', marker='o', linestyle='', label='Moving release'),
    Line2D([], [], color='#94a3b8', marker='o', linestyle='', label='Slow / stopped release'),
    Line2D([], [], color='#dc2626', marker='o', linestyle='', label='Predicted miss'),
], loc='upper left', fontsize=8)
ax.grid(axis='y', alpha=.2)
fig.tight_layout()
fig.savefig(output / 'landing-errors.png', dpi=170)
plt.close(fig)

# Stationary comparison against the previous saved run, or the D=0 control.
fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
for trace, label, color in [(traces['stationary'], f'Current D = {current_d:g}', '#2563eb'),
                                (comparison_trace, comparison_label, '#dc2626')]:
    t = [float(r['time']) for r in trace]
    axes[0].plot(t, [float(r['measured_rpm']) for r in trace], color=color, label=label)
    axes[1].plot(t, [float(r['measured_rpm']) - float(r['goal_rpm']) for r in trace], color=color)
trace = traces['stationary']
axes[0].plot([float(r['time']) for r in trace], [float(r['goal_rpm']) for r in trace],
             color='#334155', linestyle='--', label='Requested RPM')
axes[0].set_ylabel('Flywheel RPM')
axes[0].legend(loc='lower right')
axes[1].axhspan(-60, 60, color='#059669', alpha=.12, label='Readiness band: ±60 RPM')
axes[1].set_ylim(-600, 150)
axes[1].set_ylabel('RPM error (actual − requested)')
axes[1].set_xlabel('Simulation time (seconds)')
axes[1].legend(loc='lower right')
for ax in axes:
    ax.grid(alpha=.18)
fig.suptitle(f'Shooter response in a 1 ms motor-loop model\nCurrent P = {current_p:g}, D = {current_d:g} compared with {comparison_label.lower()}', fontsize=13)
fig.tight_layout(rect=(0, 0, 1, .92))
fig.savefig(output / 'flywheel-pid-comparison.png', dpi=170)
plt.close(fig)

stationary_tail = [r for r in traces['stationary'] if float(r['time']) >= 5]
control_tail = [r for r in comparison_trace if float(r['time']) >= 5]
steady_note = (f"In the final second of the stationary test, the requested speed averaged "
               f"{statistics.mean(float(r['goal_rpm']) for r in stationary_tail):.0f} RPM. "
               f"The current P/D model averaged {statistics.mean(float(r['measured_rpm']) for r in stationary_tail):.0f} RPM; "
               f"the {comparison_label.lower()} run averaged {statistics.mean(float(r['measured_rpm']) for r in control_tail):.0f} RPM. "
               "The feed gate requires measured speed within 60 RPM of the goal. The simulation does not reproduce Talon firmware filtering or CAN latency.")

if stress:
    suite_description = ('Each case lasts 10 seconds, with translation from 1.5 to 8.5 seconds and a '
        '0.3-second initial ramp. Curved arcs reverse smoothly around the hub; figure-eight paths vary '
        'both X and Y velocity continuously. Near and far weaving cases start 2.7 m and 4.0 m from '
        'the hub; all other cases start 3.5 m away with a 35-degree heading error. '
        'Both alliances are exercised. The extra stationary control disables D only in the simulator. '
        'All production gains are read from the current robot configuration.')
else:
    suite_description = ('The initial pose is 3.5 m from the hub with a 35-degree heading error. '
        'Each case lasts 6 seconds: hold for 1.5 seconds, ramp translation over 0.3 seconds, stop at '
        '4.5 seconds. Reversal/early-stop cases change at 3 seconds. Labeled speeds are commands. '
        'The extra stationary control disables D only in the simulator. No gains are tuned by this harness.')

if stress:
    fig, axes = plt.subplots(2, 3, figsize=(12, 7))
    for ax, name in zip(axes.flat, ['blue-orbit', 'red-orbit', 'blue-figure-eight',
                                   'red-figure-eight', 'near-weave', 'far-weave']):
        trace = traces[name]
        start_x, start_y = float(trace[0]['x']), float(trace[0]['y'])
        ax.plot([float(r['x']) - start_x for r in trace],
                [float(r['y']) - start_y for r in trace], color='#64748b', lw=1)
        for feeding, color, label in [(False, '#f59e0b', 'Paused'), (True, '#059669', 'Feeding')]:
            points = [r for r in trace[::3] if (r['feeding'] == 'true') == feeding and
                      1.5 <= float(r['time']) <= motion_end]
            ax.scatter([float(r['x']) - start_x for r in points],
                       [float(r['y']) - start_y for r in points], s=10, color=color, label=label)
        ax.set_title(labels[name], fontsize=10)
        ax.set_xlabel('X displacement (m)')
        ax.set_ylabel('Y displacement (m)')
        ax.axis('equal')
        ax.grid(alpha=.18)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle('Fresh closed-loop motion cases: changing X/Y velocity and aim direction', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .94))
    fig.savefig(output / 'field-paths.png', dpi=170)
    plt.close(fig)

report = [
    '# Shoot-on-the-move closed-loop simulation', '',
    'These are simulated outcomes, not measured robot hit rates. The production command and controller code ran at 20 ms against YAGSL/MapleSim drivetrain physics and WPILib FlywheelSim. No alignment or flywheel speed was mocked. Indexer readiness was idealized.', '',
    '## Controller values and setup', '',
    '```text', (source / 'configuration.txt').read_text().strip(), '```', '',
    suite_description, '',
    'YAGSL uses simplified module motor loops rather than reproducing Talon firmware. The production hub-aim PID, angular profile, tracking feedforward, shooter P/D gains, voltage feedforward, current-limited flywheel plant with a 1 ms unfiltered software motor-loop stand-in, acceleration limiter, and feed dwell all run directly. The simulated field is flat with no obstacles. Battery is fixed at 12 V; ball impacts, inventory, flywheel droop from shots, vision noise, and release-delay uncertainty are absent.', '',
    '## Results', '', steady_note, '', '![Effect of D in the motor-loop model](flywheel-pid-comparison.png)', '',
    '| Scenario (commanded speed) | Actual moving speed, median | First feed | Feed open while moving | All sampled scores / releases | Moving scores / releases | Largest miss from center |',
    '|---|---:|---:|---:|---:|---:|---:|',
]
for row in summary:
    name = row['scenario']
    moving = [r for r in traces[name] if 2 <= float(r['time']) < motion_end and
              (float(r['vx'])**2 + float(r['vy'])**2)**.5 > .15]
    speed = statistics.median([(float(r['vx'])**2 + float(r['vy'])**2)**.5 for r in moving]) if moving else None
    feed = float(row['moving_feed_fraction'])
    first_feed = f"{float(row['first_feed_s']):.2f} s" if float(row['first_feed_s']) >= 0 else 'Never'
    largest = f"{float(row['max_miss_m'])*100:.1f} cm" if int(row['shots']) else 'No releases'
    report.append(f"| {labels[name]} | {speed:.2f} m/s | {first_feed} | {feed:.0%} | "
                  f"{row['scored']} / {row['shots']} | {row['moving_scored']} / {row['moving_shots']} | {largest} |" if moving else
                  f"| {labels[name]} | 0 m/s | {first_feed} | — | {row['scored']} / {row['shots']} | — | {largest} |")
total_releases = sum(int(r['shots']) for r in summary if not r['scenario'].endswith('-reference'))
total_scores = sum(int(r['scored']) for r in summary if not r['scenario'].endswith('-reference'))
moving_shots = sum(int(r['moving_shots']) for r in summary if not r['scenario'].endswith('-reference'))
moving_scores = sum(int(r['moving_scored']) for r in summary if not r['scenario'].endswith('-reference'))
if total_releases == 0:
    current_finding = ('The current motor-loop model did not release any balls. The measured-speed '
        'readiness condition never passed, including while stationary. This is a simulation result; '
        'the motor model does not reproduce Talon firmware filtering.')
else:
    current_finding = (f'The configured P={current_p:g}, I={current_i:g}, D={current_d:g} run produced '
        f'{moving_scores}/{moving_shots} passing moving samples and {total_scores}/{total_releases} '
        'passing samples overall, excluding the duplicate stationary control. '
        'Releases after stopping can miss because the preserved stationary 7.5-degree heading tolerance '
        'allows too much angular error at longer ranges. Feed continuity still drops during reversals '
        'and faster sideways motion. These results are simulated checks, not a physical success rate.')

report += [
    '', f'Feed-open fraction uses only the 2–{motion_end:g} second interval and samples with actual speed above 0.15 m/s. It includes pauses for aiming, dwell, RPM, or an unreachable trajectory; it is not a ball throughput or success percentage. If present, first feed can occur during the initial stationary spin-up.', '',
    'Releases are sampled at most once every 0.2 seconds while the feed gate is open. Every scenario samples releases before, during, and after translation. Moving releases are classified by actual speed above 0.15 m/s; all other sampled releases are stationary. The independent RK4 model integrates 3D gravity, quadratic drag and backspin lift after adding ground velocity at release. It shares the provisional aero/launch constants, so it can detect algorithm inconsistencies but cannot validate those constants against real balls. A separate stationary cross-check verifies the independent integrator against the existing model. The usable center-distance limit is 42.9 cm, with near-rim clearance and entry-angle checks also required.', '',
    '![Physical paths and feed state](field-paths.png)' if stress else '', '',
    '![PID tracking and feed state](tracking.png)', '', '![Sampled landing error](landing-errors.png)', '',
    '## Findings', '',
    ('This second motion suite uses the production P=2, I=0, D=0 flywheel gains and the production hub-aim controller. An initial run found four marginal moving misses with the translated stationary trajectory check. The release gate now integrates 3D drag/lift with inherited velocity, measured heading and RPM. Cache-based aim/RPM selection and all stationary behavior are unchanged. The initial traces are saved in `before-3d-release-check/`.' if stress else 'The moving-only scoring gate uses measured heading and RPM after an initial diagnostic exposed a stopping error. Shooter D was set to zero as requested while retaining P=2. Earlier data is retained in the archived run folders.'), '',
    current_finding, '',
    '## Timing', '',
    'CSV traces contain wall time of command execution on this desktop, including trajectory selection, readiness, aiming, telemetry and JVM scheduling/JIT effects. These are not roboRIO timings or CPU-utilization percentages. Warmed later scenarios are more representative than the first stationary scenario; do not compare those directly as an overhead ratio.', '',
    '## Reproduce', '',
    '```sh', './gradlew movingShotStressSimulation' if stress else './gradlew movingShotSimulation',
    ('uv run --with matplotlib tools/analyze-moving-shot-simulation.py --source build/reports/moving-shot-stress-simulation --output artifacts/moving-shot-stress-simulation --comparison artifacts/moving-shot-simulation' if stress else 'uv run --with matplotlib tools/analyze-moving-shot-simulation.py --comparison artifacts/moving-shot-simulation/d-0p2-motor-loop'), '```', '',
]
(output / 'report.md').write_text('\n'.join(report))
inputs = [Path('src/test/java/frc/robot/simulation/MovingShotClosedLoopSimulationTest.java'),
          Path('src/main/java/frc/robot/Constants.java'), Path('src/main/java/frc/robot/commands/scoring/FuelShootCommand.java'),
          Path('src/main/java/frc/robot/commands/swervedrive/drivebase/AimAtHub.java')]
inputs += list(Path('src/main/java/frc/robot/utility/shooting').glob('*.java'))
inputs += list(Path('src/main/deploy/swerve/falcon').rglob('*.json'))
(output / 'input-sha256.json').write_text(json.dumps({str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}, indent=2)+'\n')
print(output / 'report.md')
