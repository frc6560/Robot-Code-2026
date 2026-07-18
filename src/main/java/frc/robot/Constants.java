// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 */
public final class Constants {
  public enum Mode { REAL, SIM, REPLAY }

  public static final Mode currentMode = Mode.REAL; // Change to REPLAY for log replay

  public static final double ROBOT_MASS = 62.59; // TODO: replace with true robot mass
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // Robot dimensions (in meters, from PathPlanner settings)
  public static final double robotLength = 0.812;
  public static final double robotWidth = 0.812;

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double kS = 0.186; 
    public static final double kV = 2.004;
    public static final double kA = 0.173;

    public static final double kP_translation = 3.0; // originally 3.0
    public static final double kP_rotation = 3.8; // originally 3.8

    public static final double kI_translation = 0.0;
    public static final double kI_rotation = 0.0;

    public static final double kD_translation = 0.2; // originally 0.2
    public static final double kD_rotation = 0.2; 

    // Pure pursuit tuning (meters, meters per second)
    public static final double kPurePursuitMinLookahead = 0.3;
    public static final double kPurePursuitMaxLookahead = 1.5;
    public static final double kPurePursuitLookaheadSpeedFactor = 0.15;
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

    public static final double PASS_DEADZONE_MIN_Y = 3.657;
    public static final double PASS_DEADZONE_MAX_Y = 4.343;
    public static final double HUB_TOLERANCE = 0.65;

    // A list of pass poses
    public static final Translation2d BLUE_BOTTOM_PASS_POS = new Translation2d(2.24, 2.36);
    public static final Translation2d BLUE_TOP_PASS_POS = new Translation2d(2.24, 5.64);
    public static final Translation2d RED_BOTTOM_PASS_POS = new Translation2d(14.29, 2.36);
    public static final Translation2d RED_TOP_PASS_POS = new Translation2d(14.29, 5.64);
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class LimelightConstants
  {
    public static final String[] LIMELIGHT_NAMES = {
      "limelight-br", "limelight-cl","limelight-cr","limelight-back"
    }; // one limelight for now

    public static final String GAMEPIECE_LIMELIGHT_NAME = "limelight-cl";

    
    public static Pose3d getLimelightPose(String name){
      Pose3d limelightPose;
      switch(name){
        case "limelight-br":
          limelightPose = new Pose3d(
            -0.299, // front/back
            0.2667, // left/right
            0.540, // height
            new Rotation3d(0, Units.degreesToRadians(30.0), 0)
          );
          break;
        case "limelight-cl":
          limelightPose = new Pose3d(
            -0.257, // front/back. this is temporary.
            -0.325, // left/right 
            0.336, // height
            new Rotation3d(0, Units.degreesToRadians(25.0), Units.degreesToRadians(90.0))
          );
          break;
        case "limelight-cr":
          limelightPose = new Pose3d(
            -0.257, // front/back
            0.325, // left/right
            0.336, // height
            new Rotation3d(0, Units.degreesToRadians(25.0), Units.degreesToRadians(-90.0))
          );
          break;
        case "limelight-back":
          limelightPose = new Pose3d(
            -0.293, // front/back
            -0.296, // left/right
            0.499, // height
            new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180.0))
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

  public static final class VisionConstants {
    // If false, distance is not estimated/used by the game-piece vision pipeline.
    public static final boolean ENABLE_DISTANCE_ESTIMATION = false;
    public static final boolean USE_DISTANCE_IN_TARGET_SCORE = false;

    // Single-camera perspective distance model: d = (realHeight * focalLengthPx) / pixelHeight
    public static final double CAMERA_FOCAL_LENGTH_PX = 700.0;
    public static final double GAME_PIECE_REAL_HEIGHT_M = 0.165;
    public static final double DISTANCE_CALIBRATION_SCALE = 1.0;
    public static final double DISTANCE_CALIBRATION_OFFSET_M = 0.0;

    public static final double MIN_BBOX_HEIGHT_PX = 6.0;
    public static final double MAX_USABLE_DISTANCE_M = 6.0;
    public static final double DISTANCE_FILTER_ALPHA = 0.35;
    public static final double DISTANCE_PIXEL_NOISE_PX = 2.0;

    public static final double TARGET_WEIGHT_CENTER = 1.0;
    public static final double TARGET_WEIGHT_DISTANCE = 0.35;

    // Intake reference and left-right camera offset compensation for heading-only mode.
    public static final double INTAKE_CENTER_LEFT_OFFSET_M = 0.0;
    public static final double HEADING_REFERENCE_DISTANCE_M = 1.2;
    public static final double MANUAL_TX_OFFSET_DEG = 0.0;
    public static final double HEADING_ERROR_DEADBAND_DEG = 0.8;
    public static final double MAX_HEADING_ERROR_DEG = 60.0;

    // Shared-control heading assist (rotation only, additive to driver omega).
    public static final boolean ENABLE_HEADING_ASSIST = true;
    public static final boolean REQUIRE_ASSIST_HOLD = true;
    public static final double HEADING_ASSIST_KP = 2.4;
    public static final double HEADING_ASSIST_SIGN = 1.0;
    public static final double HEADING_ASSIST_MAX_OMEGA_RAD_PER_SEC = Units.degreesToRadians(120.0);
    public static final double HEADING_ASSIST_MAX_FINAL_OMEGA_RAD_PER_SEC = Units.degreesToRadians(240.0);

    // Dynamic alpha scaling based on heading error magnitude.
    public static final double ASSIST_ALPHA_MAX = 0.85;
    public static final double ASSIST_ALPHA_START_ERROR_DEG = 1.5;
    public static final double ASSIST_ALPHA_FULL_ERROR_DEG = 12.0;

    // Driver override: if driver is rotating strongly, assist fades out.
    public static final double DRIVER_OVERRIDE_START_OMEGA_RAD_PER_SEC = Units.degreesToRadians(35.0);
    public static final double DRIVER_OVERRIDE_FULL_OMEGA_RAD_PER_SEC = Units.degreesToRadians(95.0);
    public static final double DRIVER_OVERRIDE_DISABLE_SCALE = 0.05;

    // Assist state management.
    public static final double ASSIST_TARGET_MAX_AGE_SEC = 0.18;
    public static final double ASSIST_DISABLE_COOLDOWN_SEC = 0.20;
    public static final double ASSIST_MIN_ACTIVE_ALPHA = 0.03;

    public static final double LOCK_MAX_ANGULAR_DELTA_DEG = 6.0;
    public static final double LOCK_HOLD_SECONDS = 0.30;

    public static final double MIN_USABLE_DISTANCE_M = 0.05;
  }

  public static final class HoodConstants{
    /** CAN IDs */
    public static final int HOOD_MOTOR_ID = 21;

    /** Feedforward Gains (for TalonFX Slot0) */
    public static final double kS = 0.14;   // Static friction voltage
    public static final double kV = 0.11;  // Velocity feedforward
    public static final double kA = 0.0;   // Acceleration feedforward
    public static final double kG = 0.168;   // Gravity feedforward (arm/wrist style - uses cosine)

    /** PID Gains */
    public static final double kP = 5.0;   // Proportional gain
    public static final double kI = 0.0;   // Integral gain
    public static final double kD = 0.0;   // Derivative gain

    /** Motion Profile Constraints */
    public static final double kMaxV = 360.0;  // Max velocity (degrees/second)
    public static final double kMaxA = 720.0;  // Max acceleration (degrees/second²)

    /** Hood Geometry */
    public static final double HOOD_GEAR_RATIO = 40.0; 
    public static final double ABSOLUTE_HOOD_ENCODER_GEAR_RATIO = 90.0 / 11.0;
  
    /** Motor Inversion */
    public static final boolean HOOD_MOTOR_INVERTED = true; // TODO: Test and adjust

    /** Current Limits */
    public static final double HOOD_CURRENT_LIMIT = 40.0; // Amps

    /** Hood Angle Limits */
    public static final double HOOD_MIN_ANGLE = 25.1;   // degrees
    public static final double HOOD_MAX_ANGLE = 45.0;  // degrees

    /** Angle at which hood is horizontal (for gravity feedforward calculation) */
    public static final double HOOD_HORIZONTAL_OFFSET = 25.0;  // degrees - adjust based on mechanism geometry

    public static final int HOOD_ABSOLUTE_ENCODER_ID = 22;

    public static final double HOOD_ABSOLUTE_ENCODER_OFFSET = -0.472; // tune frequently.
  }

  public static final class ShooterConstants{
    /** CAN IDs */
    public static final int LEFT_FLYWHEEL_ID = 19;
    public static final int RIGHT_FLYWHEEL_ID = 20;

    /** PID Gains */
    public static final double kP = 0.6;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.117; 
    public static final double kS = 0.15;

    /** Flywheel Geometry */
    public static final double MAX_RPM = 5000; 
    public static final double FLYWHEEL_ACCELERATION = 2000.0;
    public static final double PASS_RPM = 1600.0; 
    public static final double FLYWHEEL_GEAR_RATIO = 1.25; 
    public static final double FLYWHEEL_IDLE_RPM = 500; //kraken x60 
    public static final double BANGBANG_TOLERANCE = 200.0;
    public static final double FLYWHEEL_RPM_TOLERANCE = 200.0; 

    /** Current Limits */
    public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT = 50.0; // Amps

    /** Motor Inversion Settings */
    public static final boolean LEFT_FLYWHEEL_INVERTED = false; 
    public static final boolean RIGHT_FLYWHEEL_OPPOSED = true; 

    public static final int FLYWHEEL_STATOR_CURRENT_LIMIT = 50;
  }

  public static final class TurretConstants{ 
    public static final Transform3d ROBOT_RELATIVE_TURRET = new Transform3d(-0.048, 0.1143, 0, new Rotation3d()); // x --> front/back, y --> left/right

    /** CAN IDs */
    public static final int MOTOR_ID = 17;      
    public static final int ENCODER_ID = 18;   

    /** Characterization Gains */
    public static final double kS = 0.1;
    public static final double kV = 0.2;
    public static final double kA = 0.05;

    public static final double kG = 0.6;

    /** PID Gains */
    public static final double kP = 8.0; // old: 5.0
    public static final double kI = 0.05;
    public static final double kD = 0.3; // old: 0.2

    /** Motion Constraints. IN DEGREES PER SECOND */
    public static final double kMaxV = 450;
    public static final double kMaxA = 900;

    public static final boolean MOTOR_INVERTED = false; 

    /** Turret Geometry */
    public static final double MOTOR_GEAR_RATIO = 254.0 / 28.0; // turret spins once for every ~9 motor rotations
    public static final double ENCODER_GEAR_RATIO = 127.0 / 168.0; // turret spins once for every ~0.75 encoder rotations. fix.

    // Absolute encoder setup
    public static final double ABSOLUTE_ENCODER_OFFSET = 0.377; 
    public static final boolean ABSOLUTE_ENCODER_REVERSED = false; 

    // soft limits
    public static final double LOWER_SOFT_LIMIT = -100.0;
    public static final double UPPER_SOFT_LIMIT = 270.0;

    // Wire protection thresholds - prefer unwinding toward center when outside these bounds
    public static final double WIRE_PROTECTION_LOWER = -100.0;
    public static final double WIRE_PROTECTION_UPPER = 270.0;
  }

  public static final class IntakeConstants {
    // CAN IDs
    public static final int LEFT_MOTOR_ID = 24;
    public static final int RIGHT_MOTOR_ID = 25;
    public static final String CAN_BUS = "rio";


    // Motor Inversions (opposed motors)
    public static final boolean LEFT_MOTOR_INVERTED = false;
    public static final boolean RIGHT_MOTOR_INVERTED = true;

    // Current Limits
    public static final double SUPPLY_CURRENT_LIMIT = 30;
    public static final double STATOR_CURRENT_LIMIT = 40;

    // Roller Speed
    public static final double ROLLER_RPM = 2100.0;
    public static final double ROLLER_GEARING = 1.5;

    // Velocity PID Gains
    public static final double kP = 0.5;
    public static final double kV = 0.12;
  }

  public static final class FeederConstants {
    // CAN IDs
    public static final int PAN_MOTOR_ID = 14;
    public static final int PUSHER_MOTOR_ID = 23;
    public static final int FLOOR_ID = 15; // placeholder. ask derek for actual ids.
    public static final String CAN_BUS = "rio";

    // Gear Ratios
    public static final double PAN_GEAR_RATIO = 324 / 2688;
    public static final double PUSHER_GEAR_RATIO = 1.0 / 2.5;
    public static final double WALL_GEAR_RATIO = 1.0 / 2.5; 
    public static final double FLOOR_GEAR_RATIO = 1.0 / 2.5; 

    // PID Gains
    public static final double PAN_kP = 0.25;
    public static final double PUSHER_kP = 0.15;
    public static final double WALL_kP = 0.15; // tune  
    public static final double FLOOR_kP = 0.15; // tune
    public static final double kV = 0.12;

    // Current Limits
    public static final int SUPPLY_CURRENT_LIMIT = 40;
    public static final int STATOR_CURRENT_LIMIT = 40;

    // Motor Inversions
    public static final boolean PAN_MOTOR_INVERTED = false;
    public static final boolean PUSHER_MOTOR_INVERTED = true;
    public static final boolean WALL_MOTOR_INVERTED = false; // tune
    public static final boolean FLOOR_MOTOR_INVERTED = true; // tune

    // RPM Settings
    public static final double PAN_RUNNING_RPM = 7000.0;
    public static final double PUSHER_RUNNING_RPM = 7000.0;
    public static final double IDLE_RPM = 0.0;

    public static final double WALL_RPM = 1000.0;
    public static final double FLOOR_RPM = 3000.0;

    // Tolerances
    public static final double PAN_SPEED_TOLERANCE_RPM = Double.POSITIVE_INFINITY;
  }
}