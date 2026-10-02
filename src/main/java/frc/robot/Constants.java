// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
}
