from __future__ import annotations

INCH_TO_METER = 0.0254

TEAM_NUMBER = 6560
TEAM_NAME = "Charging Champions"

BALL_MASS_KG = 0.215
BALL_DIAMETER_M = 0.150

# Current shooter: one powered 4 in steel-tube flywheel (6 lb) and a fixed hood. The app's model
# has a "top" and "bottom" wheel; the flywheel fills the top slot and nothing powers the bottom.
SHOOTER_CONFIGURATION = "Single powered flywheel + fixed hood"
FLYWHEEL_DIAMETER_M = 4.0 * INCH_TO_METER
FLYWHEEL_MASS_KG = 6.0 * 0.45359237
FLYWHEEL_MOTOR_TEETH = 12
FLYWHEEL_GEAR_TEETH = 18
LAUNCH_ELEVATION_DEG = 48.36
TOP_WHEEL_DIAMETER_M = FLYWHEEL_DIAMETER_M
BOTTOM_WHEEL_DIAMETER_M = FLYWHEEL_DIAMETER_M  # unused in fixed-hood mode
WHEEL_MATERIAL = "Steel tube"
RELEASE_HEIGHT_M = 27.875 * INCH_TO_METER
# Carried over from the previous two-wheel shooter for the release-geometry drawing; not measured
# on the current mechanism.
CONTACT_PATH_LENGTH_M = 7.0 * INCH_TO_METER
BALL_COMPRESSION_M = 0.9 * INCH_TO_METER

# Robot drawing dimensions from code and deploy configuration.
ROBOT_LENGTH_M = 0.812
DRIVE_WHEEL_DIAMETER_M = 3.91 * INCH_TO_METER
DRIVE_MODULE_LONGITUDINAL_OFFSET_M = 10.875 * INCH_TO_METER

# 2026 REBUILT HUB dimensions from Game Manual section 5.4, Figure 5-7.
HUB_BODY_DEPTH_M = 47.0 * INCH_TO_METER
HUB_FUNNEL_RIM_SPAN_M = 41.7 * INCH_TO_METER
HUB_FUNNEL_RIM_HEIGHT_M = 72.0 * INCH_TO_METER

# GE-26329 funnel profile from FIRST's TE-26300 STEP model. The build instructions
# specify that this polycarbonate funnel panel is the same panel used on the field.
HUB_FUNNEL_THROAT_SPAN_M = 0.608441194809816
HUB_FUNNEL_THROAT_HEIGHT_M = 1.4336

# Compatibility names used by saved app state and the physics model.
HUB_OPENING_SPAN_M = HUB_FUNNEL_RIM_SPAN_M
HUB_OPENING_PLANE_HEIGHT_M = HUB_FUNNEL_RIM_HEIGHT_M
HUB_BALL_CENTER_HEIGHT_M = HUB_FUNNEL_RIM_HEIGHT_M + BALL_DIAMETER_M / 2.0
HUB_DEFAULT_MIN_ENTRY_ANGLE_DEG = 20.0
HUB_DEFAULT_RIM_MARGIN_M = 1.0 * INCH_TO_METER

# Robot-code limits and conventions.
FLYWHEEL_IDLE_RPM = 500.0
FLYWHEEL_MAX_RPM = 3800.0
FLYWHEEL_GEAR_RATIO = FLYWHEEL_MOTOR_TEETH / FLYWHEEL_GEAR_TEETH  # flywheel speed / motor speed
FOLLOWER_TO_LEADER_RPM_RATIO = 1.0
# The fixed hood as a rear-referenced command (90 - launch), +/-0.5 deg of measurement error so the
# app's shot map has a hood axis to work with.
HOOD_MIN_DEG = 90.0 - LAUNCH_ELEVATION_DEG - 0.5
HOOD_MAX_DEG = 90.0 - LAUNCH_ELEVATION_DEG + 0.5
SHOT_DISTANCE_MIN_M = 2.9
SHOT_DISTANCE_MAX_M = 6.050

# Legacy target fields retained for saved calibration compatibility.
ASSUMED_TARGET_CENTER_HEIGHT_M = HUB_BALL_CENTER_HEIGHT_M
ASSUMED_TARGET_OPENING_HEIGHT_M = BALL_DIAMETER_M

PROFILE_ROWS = (
    ("Ball mass", f"{BALL_MASS_KG:.3f} kg", "Form default", "Unconfirmed"),
    ("Ball diameter", f"{BALL_DIAMETER_M:.3f} m", "Form default", "Unconfirmed"),
    ("Shooter type", SHOOTER_CONFIGURATION, "Team", "Specified"),
    ("Flywheel", "4 in steel tube, 6 lb", "Team", "Measured"),
    ("Flywheel gearing", "12T motor drives 18T flywheel (0.667 flywheel / motor)", "Constants.java", "Code"),
    ("Hood", "Fixed, 48.36 deg from horizontal to the top of the hood", "Team", "Measured"),
    ("Release height", "27.875 in / 0.708 m", "Team", "Measured"),
    ("Robot footprint length", "0.812 m", "Constants.java", "Code"),
    ("Drive wheel", "3.91 in / 0.0993 m", "Swerve config", "Code"),
    ("Drive module X offset", "10.875 in / 0.2762 m", "Swerve config", "Code"),
    ("Flywheel command", "Leader + opposed follower", "ShooterIOTalonFX.java", "Current code"),
    ("Flywheel range", "500-3800 RPM (4000 free speed)", "Constants.java", "Code"),
    ("Shot distance range", "2.9-6.05 m (no scoring band inside 2.87 m)", "Constants.java", "Code"),
    ("HUB body footprint", "47 in / 1.1938 m", "2026 Manual 5.4", "Manual"),
    ("Funnel upper opening", "41.7 in / 1.0592 m hexagon", "2026 Manual 5.4", "Manual"),
    ("Funnel upper rim", "72 in / 1.8288 m", "2026 Manual 5.4", "Manual"),
    ("Funnel lower throat", "23.95 in at 56.44 in", "FIRST TE-26300 STEP", "CAD"),
    ("Ball-center entry plane", "1.9038 m", "Upper rim + ball radius", "Derived"),
    ("Minimum entry angle", "20.0 deg", "Shot constraint", "Tunable"),
    ("Rim safety margin", "1.0 in / 0.0254 m", "Shot constraint", "Tunable"),
)
