// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 */
public final class Constants {
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // TODO: replace with true robot mass
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double kS = 0.142; // TODO: tune with sysid at workshop
    public static final double kV = 2.474;
    public static final double kA = 0.230;

    public static final double kStdvX = 0.08; // TODO: tune once i get my hands on LL
    public static final double kStdvY = 0.08; 
    public static final double kStdvTheta = 3;

    public static final double kP_translation = 4.0;
  // Increased slightly per tuning request
  public static final double kP_rotation = 8.0;

    public static final double kI_translation = 0.0;
    public static final double kI_rotation = 0.0;

    public static final double kD_translation = 0.0;
    // Tuned derivative for rotation to reduce oscillation
    public static final double kD_rotation = 0.7;
  }

  public static final class FieldConstants{
    // public static final Pose2d START = new Pose2d(3.152, 4.018, Rotation2d.fromDegrees(-90));
    public static final Pose2d TARGET_POSE = new Pose2d(3.650, 4.010, Rotation2d.fromDegrees(0));
    public static final Pose2d HUB_BLUE_POSITION = new Pose2d(4.63,8.07/2, new Rotation2d(0));
    public static final Pose2d HUB_RED_POSITION = new Pose2d(16.54-4.63,8.07/2, new Rotation2d(0));
    public static final Pose2d PASS_BLUE_POSITION = new Pose2d(-1.5,8.07/2, new Rotation2d(0));
    public static final Pose2d PASS_RED_POSITION = new Pose2d(16.54+1.5,8.07/2, new Rotation2d(0));
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
      "limelight"
    }; // one limelight for now

    
    public static Pose3d getLimelightPose(String name){
      Pose3d limelightPose;
      switch(name){
        case "limelight":
          limelightPose = new Pose3d(
            0.394,
            -0.0248,
            0.192,
            new Rotation3d(0, 40, 0)
          );
          break;
        default:
          limelightPose = new Pose3d();
      }
      return limelightPose;
    }
    public static final double kStdvXYBase = 0.3; // No idea how to tune these base values.
    public static final double kStdvThetaBase = 1.0; // See above
    public static final double JUMP_TOLERANCE = 0.5; // meters. again, needs tuning.
  }

  public static final class FlywheelConstants {
    public static final int RIGHT_FLYWHEEL_ID = 21; // TODO: set correct ID
    public static final int LEFT_FLYWHEEL_ID = 20; // TODO: set correct ID

    
    public static final double kV = 0.12; // TODO: tune with SysId
    public static final double kP = 1.3; // TODO: tune
    public static final double kI = 0.01; // TODO: tune
    public static final double kD = 0.2; // TODO: tune

    public static final double FLYWHEEL_GEAR_RATIO = 2.0; // TODO: this is a reduction, 2 rotations of motor is 1 rotation flywheel
    public static final double FLYWHEEL_IDLE_RPM = 600.0; //kraken x60 
    public static final double FLYWHEEL_RPM_TOLERANCE = 100.0;
    public static final double FLYWHEEL_MAX_RPM = 3200.0;

    public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT = 40.0;

    public static final Slot0Configs FLYWHEEL_PID_CONFIG = null;

  }

  public static final class TurretConstants {
    /** CAN IDs */
  public static final int MOTOR_ID = 30;      // Default placeholder ID (change to your wiring)
  public static final int ENCODER_ID = 31;    // Default placeholder ID (change to your wiring)

    /** Characterization Gains */
    public static final double kS = 0.1;
    public static final double kV = 0.2;
    public static final double kA = 0.01;
    public static final double kG = 0.0;

    /** PID Gains */
    public static final double kP = 5;
    public static final double kI = 0.01;
    public static final double kD = 0.2;

    /** Motion Constraints */
    public static final double kMaxV = 35; // m/s
    public static final double kMaxA = 45; // m/s^2

    /** Turret Geometry */
    public static final double MOTOR_GEAR_RATIO = 1; // Motor reduction ratio
    public static final double ENCODER_GEAR_RATIO = 1; // Encoder reduction ratio
    

  // Absolute encoder setup
  public static final int ABS_ENCODER_DIO_PORT = 0;   // change to your wiring
  public static final double ABS_ENCODER_OFFSET_DEG = 0.0; // tune so stow = 0°
  public static final boolean ABS_ENCODER_REVERSED = false; 
  }

  public static final class HoodConstants {
    /** CAN IDs */
    public static final int MOTOR_ID = 32;      // Default placeholder ID (change to your wiring)
    public static final int ENCODER_ID = 33;    // Default placeholder ID (change to your wiring)

    /** Characterization Gains */
    public static final double kS = 0.1;
    public static final double kV = 0.2;
    public static final double kA = 0.01;
    public static final double kG = 0.0;

    /** PID Gains */
    public static final double kP = 0.7;
    public static final double kI = 0.0;
    public static final double kD = 0.01;

    /** Motion Constraints */
    public static final double kMaxV = 3; // m/s
    public static final double kMaxA = 2; // m/s^2

    /** Turret Geometry */
    public static final double MOTOR_GEAR_RATIO = 1; // Motor reduction ratio
    public static final double ENCODER_GEAR_RATIO = 1; // Encoder reduction ratio
    

  // Absolute encoder setup
  public static final int ABS_ENCODER_DIO_PORT = 0;   // change to your wiring
  public static final double ABS_ENCODER_OFFSET_DEG = 0.0; // tune so stow = 0°
  public static final boolean ABS_ENCODER_REVERSED = false; 
  }

  public static final class FeederConstants {
    public static final int MOTOR_ID = 22; // TODO: set correct ID
  }

  public static final class GroundIntakeConstants {
    // CAN IDs
    public static final int EXTENSION_MOTOR_ID = 20; // TODO: Set correct CAN ID
    public static final int ROLLER_MOTOR_ID = 21;    // TODO: Set correct CAN ID
    
    public static final int RETRACT_LIMIT_SWITCH_ID = 5; // TODO: Set correct
    
    // Current Limits (in Amps)
    public static final double EXTENSION_NORMAL_CURRENT_LIMIT = 40.0;
    public static final double EXTENSION_SPRINGY_CURRENT_LIMIT = 10.0; // Reduced for springy mode
    public static final double ROLLER_CURRENT_LIMIT = 30.0;
    
    // Motor Speeds (as percentage: -1.0 to 1.0)
    public static final double EXTENSION_OUT_SPEED = 0.6;
    public static final double EXTENSION_IN_SPEED = -0.6;
    public static final double ROLLER_INTAKE_SPEED = 0.8;
    public static final double ROLLER_OUTTAKE_SPEED = -0.5;
    
    // Motor Inversions
    public static final boolean EXTENSION_MOTOR_INVERTED = false; // TODO: Test and adjust
    public static final boolean ROLLER_MOTOR_INVERTED = false;    // TODO: Test and adjust
    
    // Extension Thresholds
    public static final double EXTENSION_THRESHOLD = 0.5; // Motor output threshold to consider "fully extended"
  }
  
}
