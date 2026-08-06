from __future__ import annotations

INCH_TO_METER = 0.0254

TEAM_NUMBER = 6560
TEAM_NAME = "Charging Champions"

# Values transcribed from the supplied robot information form.
BALL_MASS_KG = 0.215
BALL_DIAMETER_M = 0.150
FORM_SHOOTER_CONFIGURATION = "Two independently powered wheels"
TOP_WHEEL_DIAMETER_M = 2.5 * INCH_TO_METER
BOTTOM_WHEEL_DIAMETER_M = 4.0 * INCH_TO_METER
WHEEL_MATERIAL = "Neoprene rubber"
FORM_TOP_MOTOR_TO_WHEEL_RATIO = 1.0
FORM_BOTTOM_MOTOR_TO_WHEEL_RATIO = 30.0 / 24.0
RELEASE_HEIGHT_M = 21.0 * INCH_TO_METER
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
FLYWHEEL_MAX_RPM = 5000.0
FLYWHEEL_GEAR_RATIO = 1.25
FOLLOWER_TO_LEADER_RPM_RATIO = 1.0
HOOD_MIN_DEG = 25.1
HOOD_MAX_DEG = 45.0
SHOT_DISTANCE_MIN_M = 1.628
SHOT_DISTANCE_MAX_M = 6.050

SHOT_LUT = (
    (1.628, 1500.0, 25.9, 0.801),
    (1.942, 1500.0, 25.9, 0.931),
    (2.573, 1700.0, 25.9, 1.071),
    (3.219, 2000.0, 27.0, 1.125),
    (3.742, 2100.0, 31.0, 1.127),
    (4.357, 2200.0, 33.0, 1.161),
    (4.943, 2400.0, 36.0, 1.222),
    (5.590, 2700.0, 38.0, 1.250),
    (6.050, 2800.0, 40.0, 1.250),
)

# Legacy target fields retained for saved calibration compatibility.
ASSUMED_TARGET_CENTER_HEIGHT_M = HUB_BALL_CENTER_HEIGHT_M
ASSUMED_TARGET_OPENING_HEIGHT_M = BALL_DIAMETER_M

PROFILE_ROWS = (
    ("Ball mass", f"{BALL_MASS_KG:.3f} kg", "Form default", "Unconfirmed"),
    ("Ball diameter", f"{BALL_DIAMETER_M:.3f} m", "Form default", "Unconfirmed"),
    ("Top wheel", "2.5 in / 0.0635 m", "Robot form", "Measured"),
    ("Bottom wheel", "4.0 in / 0.1016 m", "Robot form", "Measured"),
    ("Wheel tread", WHEEL_MATERIAL, "Robot form", "Specified"),
    ("Top gearing", "1:1 motor / wheel", "Robot form", "Specified"),
    ("Bottom gearing", "30:24 (1.25x wheel speed)", "Robot form", "Specified"),
    ("Ball contact path", "7 in / 0.1778 m", "Robot form", "Measured"),
    ("Nominal compression", "0.9 in / 0.0229 m", "Robot form", "Measured"),
    ("Release height", "21 in / 0.5334 m", "Robot form", "Measured"),
    ("Robot footprint length", "0.812 m", "Constants.java", "Code"),
    ("Drive wheel", "3.91 in / 0.0993 m", "Swerve config", "Code"),
    ("Drive module X offset", "10.875 in / 0.2762 m", "Swerve config", "Code"),
    ("Form shooter type", FORM_SHOOTER_CONFIGURATION, "Robot form", "Conflicts with code"),
    ("Flywheel command", "Leader + opposed follower", "ShooterIOTalonFX.java", "Current code"),
    ("Flywheel gearing", "1.25 mechanism / motor", "Constants.java", "Code"),
    ("Flywheel range", "500-5000 RPM", "Constants.java", "Code"),
    ("Hood command range", "25.1-45.0 deg from back (64.9-45.0 deg launch)", "Constants.java", "Code + convention"),
    ("Shot distance range", "1.628-6.050 m", "ShotCalculator.java", "Code"),
    ("HUB body footprint", "47 in / 1.1938 m", "2026 Manual 5.4", "Manual"),
    ("Funnel upper opening", "41.7 in / 1.0592 m hexagon", "2026 Manual 5.4", "Manual"),
    ("Funnel upper rim", "72 in / 1.8288 m", "2026 Manual 5.4", "Manual"),
    ("Funnel lower throat", "23.95 in at 56.44 in", "FIRST TE-26300 STEP", "CAD"),
    ("Ball-center entry plane", "1.9038 m", "Upper rim + ball radius", "Derived"),
    ("Minimum entry angle", "20.0 deg", "Shot constraint", "Tunable"),
    ("Rim safety margin", "1.0 in / 0.0254 m", "Shot constraint", "Tunable"),
)
