# BLine editor and robot settings

Open `src/main/deploy/autos` as the desktop BLine Web project. This makes the
editor, optimizer, exported JSON paths, and Java BLine defaults share `config.json`.
Save before building/deploying. Java reads defaults at startup; redeploy and restart
after changing them. The separate `Documents/Bline-paths` project is a copy, not a
live connection to this repository.

Current defaults are 4.5 m/s, 12 m/s², 716.1972439 deg/s (12.5 rad/s), and
1375.0987083 deg/s² (24 rad/s²). The drivetrain's 19.5 ft/s = 5.9436 m/s wheel
speed is a separate hardware speed bound, not the selected autonomous speed.
The top-level angular constants and `DrivebaseConstants.HUB_AIM_MAX_*` read the
same JSON angular defaults (converted to radians). Hub aim retains its own PID
gains and profiled controller; path-specific constraints do not change hub aim.
Change the shared angular limits in this JSON, rather than adding another literal
to Constants.java. The Documents/Bline-paths config has also been corrected, but
it remains an independent project: use the repository autos folder for ongoing work.
Reopen projects after external configuration edits, and refresh stale optimizer
results with Auto all. Existing input signatures deliberately retain the old
settings so the editor can identify which generated caps need refreshing.

Explicit path/segment constraints override defaults. Java-authored paths in
`BLinePaths.java` also intentionally set their own translation constraints.
An embedded `default_global_constraints` block can replace loaded defaults in
BLine-Lib v0.9.1; avoid adding it to exported paths. This library stores globals
statically, so a later-loaded path with embedded globals can affect other paths.

After changing defaults, review and rerun the GUI's auto-velocity optimizer.
It writes actual per-segment limits to path JSON; old optimized values do not
magically become the new defaults. Keep its 0.9 velocity / 0.8 acceleration safety
factors unless robot testing justifies changing them. Preserve manually tuned
constraints when reviewing generated values.

## What “realistic” means here

BLine Web is an idealized kinematic preview, not a TalonFX/drivetrain physics
simulation. It cannot use PID gains, 40 A limits, steering response, battery sag,
wheel slip, or robot-code event timing. In particular, our `Wait` event pauses
robot output for five seconds; a GUI marker does not reproduce that callback.
Matching configuration fixes input disagreement but does not guarantee matching
lap times or trajectories. See https://bline-docs.pages.dev/gui/simulation/.

The 24 rad/s² and 12 m/s² limits are tuning assumptions, not measured physical
maxima. A 40 A limit alone cannot establish angular acceleration: supply versus
stator limiting, yaw inertia, gearing, wheel size, traction and voltage matter.
The module locations (±10.875 inches) give a radius of 0.39064 m. With 5.9436 m/s
wheel speed, the ideal stationary spin ceiling is about 15.21 rad/s; the configured
12.5 rad/s is below that. Simultaneous translation reduces available spin speed.

The footprint here uses the existing 0.812 m square code dimensions; measure the
actual bumper envelope. Mass is also unverified: Constants.java says 62.59 kg
(TODO), while YAGSL physicalproperties.json says 136 lb (61.69 kg). Do not treat
either as a measurement or silently average them.

Validate actual robot behavior with:

```
BLINE_HEADLESS_DIAGNOSTIC=1 ./gradlew simulateJava -Pheadless
```

Then compare real WPILOG requested/applied/measured chassis velocities and heading
errors. Measure straight and spin acceleration on carpet, with normal battery and
mechanism load, and tune `config.json` from those measurements. A successful desktop
run checks robot command execution; it does not certify real traction or timing.
