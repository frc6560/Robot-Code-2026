// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.BLine.JsonUtils;
import frc.robot.lib.BLine.Path;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 */
public final class Constants {
  public enum Mode { REAL, SIM, REPLAY }

  public static final Mode currentMode = Mode.REAL; // Change to REPLAY for log replay

  public static final double MAX_SPEED = Units.feetToMeters(19.5);
  // Shared with the BLine editor/optimizer via deploy/autos/config.json.
  // These are selected operating limits, not measured physical maxima.
  public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC =
      Units.degreesToRadians(BLineConstants.GLOBAL_MAX_ANGULAR_VELOCITY_DEG_PER_SEC);
  public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ =
      Units.degreesToRadians(BLineConstants.GLOBAL_MAX_ANGULAR_ACCELERATION_DEG_PER_SEC2);

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double kS = 0.186; 
    public static final double kV = 2.004;
    public static final double kA = 0.173;

    /**
     * Proportional gain for the drivetrain's heading-only alignment controller, in
     * (radians/second) per radian of heading error. This is used by rotateToAngle and
     * alignToTrench, not by BLine. Increasing it makes alignment turn harder and finish faster but
     * can cause overshoot or oscillation; decreasing it makes turning gentler but can leave the
     * robot slow to converge.
     */
    public static final double ALIGN_ROTATION_KP = 3.8;

    /**
     * Integral gain for heading-only alignment, in (radians/second) per radian-second of
     * accumulated error. Increasing it can remove a persistent heading offset caused by friction,
     * but it also creates windup and overshoot; keep it at zero unless a repeatable steady-state
     * error remains after tuning P and D.
     */
    public static final double ALIGN_ROTATION_KI = 0.0;

    /**
     * Derivative gain for heading-only alignment, in (radians/second) per
     * (radian/second) of error change. Increasing it adds damping and can reduce overshoot, but too
     * much amplifies gyro noise and makes output jitter; decreasing it produces a more responsive
     * but less damped turn.
     */
    public static final double ALIGN_ROTATION_KD = 0.2;

    /** Heading controller gains used while the driver holds the aim-at-hub button. */
    public static final double HUB_AIM_KP = 1.2;
    public static final double HUB_AIM_KI = 0.0;
    public static final double HUB_AIM_KD = 0.2;

    /**
     * Shared maximum velocity for the shooting auto-turn profile. Path-specific overrides do not
     * affect hub aim; the shared project default sets its operating ceiling.
     */
    public static final double HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC = MAX_ANGULAR_VELOCITY_RAD_PER_SEC;

    /**
     * Shared maximum acceleration for the shooting auto-turn profile.
     */
    public static final double HUB_AIM_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ =
        MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ;

    /** Stops small heading corrections once the chassis is pointed at the hub. */
    public static final double HUB_AIM_TOLERANCE_RADIANS = Units.degreesToRadians(7.5);

    /** Required angular speed before hub alignment reports that it has settled. */
    public static final double HUB_AIM_VELOCITY_TOLERANCE_RAD_PER_SEC =
        Units.degreesToRadians(5.0);

    // Pure pursuit tuning (meters, meters per second)

  }

  /**
   * BLine path-following tuning.
   *
   * <p>Tune in this order: translation, rotation, then cross-track. Keep translation I at zero;
   * accumulated distance error makes that controller unstable near the end of a path. All default
   * path limits below are loaded from deploy/autos/config.json, shared with BLine Web. Explicit
   * path/segment constraints still override these defaults. Hub aim has separate tuning above.
   */
  public static final class BLineConstants {
    private BLineConstants() {}

    private static final Path.DefaultGlobalConstraints CONFIG =
        JsonUtils.loadGlobalConstraints(JsonUtils.PROJECT_ROOT);

    /**
     * Proportional gain for translation, in (meters/second) per meter of remaining path distance.
     * BLine multiplies remaining distance by this gain before applying velocity constraints.
     * Increasing it commands maximum speed for longer and brakes later, which shortens travel time
     * but increases overshoot/endpoint oscillation; decreasing it starts decelerating earlier and
     * is smoother, but can make the robot crawl toward the final target.
     */
    public static final double TRANSLATION_KP = 5;

    /**
     * Integral translation gain, in (meters/second) per meter-second of accumulated distance error.
     * Increasing it can overcome a persistent friction-induced shortfall, but remaining-distance
     * error is large for most of a path, so the integral winds up and commonly makes the robot blast
     * through or oscillate at the endpoint. BLine's recommended value is zero; tune P and possibly D
     * before considering any nonzero value.
     */
    public static final double TRANSLATION_KI = 0.0;

    /**
     * Derivative translation gain, in (meters/second) per (meter/second) of distance-error change.
     * Increasing it adds endpoint damping and may reduce overshoot, but excessive D reacts to noisy
     * odometry/velocity estimates and produces jerky speed commands; decreasing it makes translation
     * more responsive but leaves P alone to control braking.
     */
    public static final double TRANSLATION_KD = 0.5;

    /**
     * Proportional gain for holonomic rotation, in (radians/second) per radian of heading error.
     * BLine wraps heading error across -pi to +pi. Increasing it makes the robot acquire the path
     * heading faster but can overshoot or shake near the target; decreasing it produces slower,
     * smoother rotation and may leave heading behind during fast translation.
     */
    public static final double ROTATION_KP = 3;

    /**
     * Integral holonomic-rotation gain, in (radians/second) per radian-second of accumulated heading
     * error. Increasing it can correct a consistent rotational bias, but it introduces windup and
     * endpoint overshoot; leave it at zero unless the robot settles at a repeatable nonzero error.
     */
    public static final double ROTATION_KI = 0;

    /**
     * Derivative holonomic-rotation gain, in (radians/second) per (radian/second) of heading-error
     * change. Increasing it damps rotation and can suppress overshoot, while too much causes noisy or
     * hesitant turning; decreasing it makes rotation sharper but less damped.
     */
    public static final double ROTATION_KD = 0.3;

    /**
     * Proportional gain for cross-track correction, in (meters/second) per meter of perpendicular
     * displacement from the active line segment. Increasing it pulls the robot back to the drawn
     * line more aggressively, but too much causes fishtailing and unstable corners; decreasing it
     * allows wider deviation and smoother turns. Tune only after translation and rotation work.
     */
    public static final double CROSS_TRACK_KP = 5;

    /**
     * Integral cross-track gain, in (meters/second) per meter-second of accumulated lateral error.
     * Increasing it can remove a persistent sideways bias, but can continue steering after the bias
     * disappears and destabilize segment handoffs; zero is the safe default.
     */
    public static final double CROSS_TRACK_KI = 0.0;

    /**
     * Derivative cross-track gain, in (meters/second) per (meter/second) of lateral-error change.
     * Increasing it damps lateral correction and may reduce fishtailing, but noisy pose estimates can
     * make it twitch; decreasing it makes correction more immediate and potentially more oscillatory.
     */
    public static final double CROSS_TRACK_KD = 0.0;

    /**
     * Default maximum translational speed in meters per second. Increasing it permits faster paths
     * but lengthens stopping distance, increases wheel slip, and may require larger handoff radii;
     * decreasing it makes paths slower, easier to track, and safer during initial tuning. A path-level
     * constraint may override this value.
     */
    public static final double GLOBAL_MAX_VELOCITY_MPS = CONFIG.getMaxVelocityMetersPerSec();

    /**
     * Default maximum translational acceleration in meters per second squared. Increasing it makes
     * launches and braking more aggressive but raises current draw, slip, and mechanism disturbance;
     * decreasing it produces gentler motion but increases path time and stopping distance in time.
     * JSON paths read the matching value from deploy/autos/config.json.
     */
    public static final double GLOBAL_MAX_ACCELERATION_MPS2 = CONFIG.getMaxAccelerationMetersPerSec2();

    /**
     * Default maximum angular velocity in degrees per second. Increasing it allows faster heading
     * changes but can saturate modules and harm translation tracking; decreasing it makes rotation
     * slower and may prevent the requested heading from completing before a segment ends.
     */
    public static final double GLOBAL_MAX_ANGULAR_VELOCITY_DEG_PER_SEC =
        CONFIG.getMaxVelocityDegPerSec();

    /**
     * Default maximum angular acceleration in degrees per second squared. Increasing it makes angular
     * velocity change more abruptly, improving response while increasing slip/current and mechanical
     * shock; decreasing it smooths rotation but can make heading lag along short segments.
     */
    public static final double GLOBAL_MAX_ANGULAR_ACCELERATION_DEG_PER_SEC2 =
        CONFIG.getMaxAccelerationDegPerSec2();

    /**
     * Maximum final-position error in meters for path completion. Increasing it lets commands finish
     * sooner and avoids endless endpoint hunting, at the cost of positional accuracy; decreasing it
     * demands greater precision but can prevent completion when odometry noise or wheel slip exceeds
     * the tolerance. Both translation and rotation tolerances must be satisfied.
     */
    public static final double END_TRANSLATION_TOLERANCE_METERS = CONFIG.getEndTranslationToleranceMeters();

    /**
     * Maximum final-heading error in degrees for path completion. Increasing it finishes sooner with
     * looser orientation; decreasing it improves final heading accuracy but can cause prolonged
     * rotation or jitter near the endpoint. Both endpoint tolerances must be satisfied.
     */
    public static final double END_ROTATION_TOLERANCE_DEGREES = CONFIG.getEndRotationToleranceDeg();

    /**
     * Default intermediate handoff radius in meters. With radius-based handoffs, BLine advances to
     * the next target once the robot enters this circle. Increasing it cuts corners earlier and makes
     * paths smoother/faster but follows waypoints less precisely; decreasing it hugs waypoints more
     * closely but can make the robot overshoot, reverse, and oscillate if the radius is smaller than
     * its stopping distance. This does not control final-point tolerance.
     */
    public static final double INTERMEDIATE_HANDOFF_RADIUS_METERS = CONFIG.getIntermediateHandoffRadiusMeters();

    /**
     * Selects the translation-target handoff algorithm; this is dimensionless. False uses distance
     * to the target and INTERMEDIATE_HANDOFF_RADIUS_METERS. True switches when the robot's projected
     * progress passes the segment endpoint, which can be more robust at high speed but is less
     * forgiving of a badly positioned first segment. Change this only after radius-based paths are
     * working, then retest every multi-segment route.
     */
    public static final boolean USE_T_RATIO_BASED_TRANSLATION_HANDOFFS = false;
  }

  public static final class FieldConstants{
    public static final double FIELD_LENGTH = 16.54;
    public static final double FIELD_WIDTH = 8.07;

    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.62, 4.03);
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.92, 4.03);

    public static final double BLUE_ZONE_X = 4.70;
    public static final double RED_ZONE_X = 11.80;

    // A list of auto start poses.
    public static final Pose2d BLUE_RIGHT_START = new Pose2d(3.70, 0.75, Rotation2d.fromDegrees(0.0));
    public static final Pose2d BLUE_LEFT_START = new Pose2d(3.70, 7.25, Rotation2d.fromDegrees(0.0));
    public static final Pose2d RED_RIGHT_START = new Pose2d(12.85, 7.25, Rotation2d.fromDegrees(180.0));
    public static final Pose2d RED_LEFT_START = new Pose2d(12.85, 0.75, Rotation2d.fromDegrees(180.0));

    public static final Pose2d BLUE_TESTING_START = new Pose2d(3.70, 4.00, Rotation2d.fromDegrees(0.0));
    public static final Pose2d RED_TESTING_START = new Pose2d(12.85, 4.00, Rotation2d.fromDegrees(180.0));

  }

  public static class OperatorConstants
  {
    // Driver Station USB slots. Keep the button board separate from the driver Xbox.
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final int OPERATOR_CONTROLLER_PORT = 0;
    public static final int BUTTON_BOARD_PORT = OPERATOR_CONTROLLER_PORT;

    // Joystick Deadband
    public static final double DEADBAND = 0.1;

  }

  public static class LimelightConstants
  {
    public static final String FRONT_CAMERA = "limelight-front";
    public static final String LEFT_CAMERA = "limelight-left";
    public static final String RIGHT_CAMERA = "limelight-right";
    public static final String CONFIGURATION_ID = "three-ll4-named-2026-10-30";
    public static final String[] LIMELIGHT_NAMES = {
      LEFT_CAMERA, FRONT_CAMERA, RIGHT_CAMERA
    };

    /**
     * Mounting values passed directly to Limelight: meters forward/right/up, then
     * roll/pitch/yaw (stored in radians). These are Limelight robot-space values,
     * not a WPILib robot-to-camera transform with positive Y to the left.
     */
    public static Pose3d getLimelightPose(String name){
      Pose3d limelightPose;
      switch(name){
        case LEFT_CAMERA: // Physical left camera, facing left.
          limelightPose = new Pose3d(
            -0.21325, // backward from drivetrain center
            -0.32168, // left of drivetrain center
            0.42430, // above floor
            new Rotation3d(0, Units.degreesToRadians(15.0), Units.degreesToRadians(90.0))
          );
          break;
        case FRONT_CAMERA: // Physical front camera, facing forward.
          limelightPose = new Pose3d(
            -0.00918, // backward from drivetrain center
            0.0, // Assumed centered horizontally; confirm with CAD.
            0.73553, // above floor
            new Rotation3d(0, Units.degreesToRadians(22.0), 0)
          );
          break;
        case RIGHT_CAMERA: // Physical right camera, facing right.
          limelightPose = new Pose3d(
            -0.21325, // backward from drivetrain center
            0.32168, // right of drivetrain center
            0.42430, // above floor
            new Rotation3d(0, Units.degreesToRadians(15.0), Units.degreesToRadians(-90.0))
          );
          break;
        default:
          throw new IllegalArgumentException("Invalid limelight name. You might want to double check your configs: " + name);
      }
      return limelightPose;
    }
    public static final double kStdvXYBase = 0.05;
    /** Lower bound on the per-measurement XY std dev (meters) before it's scaled by
     *  {@link #kStdvXYBase}. Prevents close-range high-tag-count solutions from claiming
     *  unphysically tight covariances. */
    public static final double kStdvXYFloor = 0.30;
    public static final double kStdvThetaBase = 9999.0; // disables rotation updates. trust the gyro!
    public static final double JUMP_TOLERANCE = 2.0;
  }

  public static final class ShooterConstants{
    /** CAN IDs */
    public static final int RIGHT_FLYWHEEL_ID = 15;
    public static final int LEFT_FLYWHEEL_ID = 24;
    public static final String CAN_BUS = "Canivore";

    /** PID Gains */
    public static final double kP = 0.6;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.117; 
    public static final double kS = 0.15;

    /** Flywheel Geometry */
    /** Kraken free speed (6000 RPM) through the 12:18 reduction is 4000; leave headroom to hold speed. */
    public static final double MAX_RPM = 3800;

    /** 12T motor gear drives the 18T flywheel gear, so flywheel speed = motor speed * 12/18. */
    public static final double FLYWHEEL_GEAR_RATIO = 12.0 / 18.0;
    public static final double FLYWHEEL_IDLE_RPM = 1000;
    /** Temporary no-vision shot speed commanded while the driver holds right bumper. */
    public static final double FIXED_SHOT_RPM = 3000.0;
    public static final double FLYWHEEL_RPM_TOLERANCE = 200.0; 

    /** Current Limits */
    public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT = 40.0; // Amps

    /** Motor Inversion Settings */
    public static final boolean RIGHT_FLYWHEEL_INVERTED = true;

    public static final int FLYWHEEL_STATOR_CURRENT_LIMIT = 40;

    /**
     * Spin-up controller, ported from 6328's 2026 flywheel. Everything below is mechanism side
     * (flywheel shaft, rad/s) unless noted. kP/kD above stay on the Talon in V per motor rot/s;
     * kS/kV/kA here are applied as a Java feedforward.
     */
    public static final int FLYWHEEL_MOTOR_COUNT = 2;
    /** Mechanism speed = motor speed * FLYWHEEL_GEAR_RATIO, so the motor-to-mechanism reduction is the inverse. */
    public static final DCMotor FLYWHEEL_GEARBOX =
        DCMotor.getKrakenX60(FLYWHEEL_MOTOR_COUNT).withReduction(1.0 / FLYWHEEL_GEAR_RATIO);
    /**
     * 6 lb steel tube, 4 in OD, 0.065 in wall: I = m(ro^2 + ri^2)/2. Gears and rotors add a little;
     * refine from a coast-down if spin-up doesn't match.
     */
    public static final double FLYWHEEL_MOI = 0.0068; // kg*m^2

    public static final double FF_kS = kS; // V
    public static final double FF_kV = kV / (2.0 * Math.PI * FLYWHEEL_GEAR_RATIO); // V per rad/s
    /** Model-derived: V = I*R with I = J*alpha/Kt. Re-derive if MOI changes. */
    public static final double FF_kA =
        FLYWHEEL_GEARBOX.rOhms * FLYWHEEL_MOI / FLYWHEEL_GEARBOX.KtNMPerAmp; // V per rad/s^2

    /** Power budget the spin-up rate limiter plans against (total across both motors). */
    public static final double FLYWHEEL_SUPPLY_BUDGET_TELEOP = FLYWHEEL_SUPPLY_CURRENT_LIMIT * FLYWHEEL_MOTOR_COUNT; // A
    public static final double FLYWHEEL_SUPPLY_BUDGET_AUTO = 60.0; // A
    public static final double FLYWHEEL_EFFICIENCY = 0.8;
    public static final double FLYWHEEL_MAX_ACCELERATION = 350.0; // rad/s^2
    public static final double FLYWHEEL_ACCEL_FILTER_TIME_CONSTANT = 0.05; // s

    /** Bang-bang (feedforward x constant while below setpoint) is used for hub shots past this range. */
    public static final double BANGBANG_CONSTANT = 2.0;
    /** Added to the PID setpoint only (not the feedforward) to offset steady-state droop. */
    public static final double PID_SETPOINT_OFFSET = 0.0; // rad/s
    /** atTarget() compares the rate-limited setpoint against the goal, not the encoder. */
    public static final double AT_GOAL_EPSILON = 2.0; // rad/s

    /** Feedforward characterization ramp and the tolerance the fit is checked against. */
    public static final double FF_CHARACTERIZATION_START_DELAY = 0.5; // s
    public static final double FF_CHARACTERIZATION_RAMP_RATE = 0.2; // V/s
    public static final double FF_kS_TOLERANCE = 0.15; // V
    public static final double FF_kV_TOLERANCE = 0.002; // V per rad/s
  }

  /** Geometry used by the active fixed-hood trajectory model. */
  public static final class ShotModelConstants {
    private ShotModelConstants() {}
    public static final double BALL_MASS_KG = 0.215;
    public static final double BALL_DIAMETER_METERS = 0.150;
    public static final double FLYWHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final double LAUNCH_ELEVATION_DEGREES = 48.36;
    public static final double RELEASE_HEIGHT_METERS = Units.inchesToMeters(27.875);
    public static final double HUB_OPENING_SPAN_METERS = Units.inchesToMeters(41.7);
    public static final double HUB_RIM_HEIGHT_METERS = Units.inchesToMeters(72.0);
    public static final double HUB_RIM_MARGIN_METERS = Units.inchesToMeters(1.0);
  }

  public static final class IndexerConstants {
    public static final int FLOOR_MOTOR_ID = 25;
    public static final int FLOOR_SECONDARY_MOTOR_ID = 0;
    // Viewed from each shaft end: ID 25 clockwise, ID 0 counterclockwise.
    public static final boolean FLOOR_MOTOR_INVERTED = true;
    public static final boolean FLOOR_SECONDARY_MOTOR_INVERTED = false;
    public static final int TUNNEL_MOTOR_ID = 16;
    public static final String CAN_BUS = "Canivore";
    public static final double FLOOR_RUNNING_RPM = 800.0;
    public static final double FLOOR_STATOR_CURRENT_LIMIT = 40.0;
    public static final double FLOOR_SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double TUNNEL_RUNNING_RPM = 1500.0;
    /** Motor rotations per mechanism rotation; retained from the indexer hardware configuration. */
    public static final double FLOOR_GEAR_RATIO = 2.5;
    public static final double TUNNEL_GEAR_RATIO = 5.0 / 3.0;
    /** Initial velocity gains, in volts per mechanism RPS and volts per motor RPS respectively. */
    public static final double VELOCITY_KP = 0.25;
    public static final double MOTOR_KV = 0.12;
  }

  public static final class IntakeConstants {
    private IntakeConstants() {}

    // CAN IDs. The rack is a Kraken X44; the separate rollers are Kraken X60s.
    public static final int DEPLOY_MOTOR_ID = 20;
    public static final int LEFT_ROLLER_MOTOR_ID = 36;
    public static final int RIGHT_ROLLER_MOTOR_ID = 18;
    public static final String CAN_BUS = "Canivore";

    public static final boolean DEPLOY_MOTOR_INVERTED = false;
    // Viewed from each motor's shaft end: left CCW, right CW for positive RPM.
    public static final boolean LEFT_ROLLER_MOTOR_INVERTED = false;
    public static final boolean RIGHT_ROLLER_MOTOR_INVERTED = true;

    // Current Limits
    public static final double DEPLOY_SUPPLY_CURRENT_LIMIT = 30.0;
    public static final double DEPLOY_STATOR_CURRENT_LIMIT = 40.0;
    public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double ROLLER_STATOR_CURRENT_LIMIT = 40.0;

    // Mechanism RPM for both separate rollers; inversion supplies opposite directions.
    public static final double ROLLER_RPM = 2500.0;
    public static final double ROLLER_GEARING = 1.5;

    public static final double ROLLER_kP = 0.5;
    public static final double ROLLER_kV = 0.12;

    // Rack positions are X44 motor rotations.
    public static final double RETRACTED_POSITION_ROTATIONS = 0.0;
    // Operator-requested quick-test override for both target and forward soft limit.
    // 9.5 motor rotations exceeds the 7.81 rotations calculated from measured travel.
    public static final double EXTENDED_POSITION_ROTATIONS = 9.5;
    public static final double DEPLOY_POSITION_TOLERANCE_ROTATIONS = 0.25;
    public static final double ROLLER_MIN_EXTENSION_FRACTION = 0.40;
    public static final double DEPLOY_SPIKE_AMPS = 35.0;
    public static final double DEPLOY_SPIKE_SECONDS = 0.06;
    public static final double DEPLOY_SPIKE_RESET_AMPS = 15.0;
    public static final double DEPLOY_SPIKE_RESET_SECONDS = 0.10;
    public static final double ROLLER_EXTENSION_SETTLE_SECONDS = 0.15;
    public static final double ROLLER_MAX_DEPLOY_VELOCITY_RPS = 0.1;

    // Motion Magic values are motor rotations/sec and motor rotations/sec^2.
    public static final double DEPLOY_CRUISE_VELOCITY_RPS = 5.5;
    public static final double DEPLOY_ACCELERATION_RPS_SQ = 50.0;
    public static final double DEPLOY_kP = 1.0;
    public static final double DEPLOY_kV = 0.24;

  }

}
