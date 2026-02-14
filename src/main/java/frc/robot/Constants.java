// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

  // Robot dimensions (in meters, from PathPlanner settings)
  public static final double robotLength = 0.812;
  public static final double robotWidth = 0.812;

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double kS = 0.0994; 
    public static final double kV = 2.4482;
    public static final double kA = 0.1997;

    public static final double kP_translation = 2.0;
    public static final double kP_rotation = 4.0;

    public static final double kI_translation = 0.0;
    public static final double kI_rotation = 0.0;

    public static final double kD_translation = 0.2;
    public static final double kD_rotation = 0.2; 

    // Pure pursuit tuning (meters, meters per second)
    public static final double kPurePursuitMinLookahead = 0.3;
    public static final double kPurePursuitMaxLookahead = 1.5;
    public static final double kPurePursuitLookaheadSpeedFactor = 0.15;
  }

  public static final class FieldConstants{

    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.68, 4.03);
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.85, 4.03);

    public static final double BLUE_ZONE_X = 4.0;
    public static final double RED_ZONE_X = 12.46;
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
      "limelight-br",
    }; // one limelight for now

    
    public static Pose3d getLimelightPose(String name){
      Pose3d limelightPose;
      switch(name){
        case "limelight-br":
          limelightPose = new Pose3d(
            -0.299, // front/back
            0.267, // left/right
            0.540, // height
            new Rotation3d(0, Units.degreesToRadians(30.0), 0)
          );
          break;
        default:
          throw new IllegalArgumentException("Invalid limelight name. You might want to double check your configs: " + name);
      }
      return limelightPose;
    }
    public static final double kStdvXYBase = 0.15; 
    public static final double kStdvThetaBase = 1.0; 
    public static final double JUMP_TOLERANCE = 0.6;
  }

  public static final class HoodConstants{
    /** CAN IDs */
    public static final int HOOD_MOTOR_ID = 21;

    /** Feedforward Gains (for TalonFX Slot0) */
    public static final double kS = 0.5;   // Static friction voltage
    public static final double kV = 0.12;  // Velocity feedforward
    public static final double kA = 0.0;   // Acceleration feedforward

    /** PID Gains */
    public static final double kP = 0.01;   // Proportional gain
    public static final double kI = 0.0;   // Integral gain
    public static final double kD = 0.0;   // Derivative gain

    /** Motion Profile Constraints */
    public static final double kMaxV = 360.0;  // Max velocity (degrees/second)
    public static final double kMaxA = 720.0;  // Max acceleration (degrees/second²)

    /** Hood Geometry */
    public static final double HOOD_GEAR_RATIO = 40.0; // 40:1 total gear reduction
    public static final double ABSOLUTE_HOOD_ENCODER_GEAR_RATIO = 44.0 / 18.0; // 2.44:1

    /** Motor Inversion */
    public static final boolean HOOD_MOTOR_INVERTED = true; // TODO: Test and adjust

    /** Current Limits */
    public static final double HOOD_CURRENT_LIMIT = 60.0; // Amps

    /** Hood Angle Limits */
    public static final double HOOD_MIN_ANGLE = 0.0;   // degrees
    public static final double HOOD_MAX_ANGLE = 37.0;  // degrees

    public static final int HOOD_ABSOLUTE_ENCODER_ID = 22;

    public static final double HOOD_ABSOLUTE_ENCODER_OFFSET = -0.42;
  }

  public static final class ShooterConstants{
    /** CAN IDs */
    public static final int LEFT_FLYWHEEL_ID = 19;
    public static final int RIGHT_FLYWHEEL_ID = 20;

    /** PID Gains */
    public static final double kP = 0.5;
    public static final double kI = 0.00;
    public static final double kD = 0.00;
    public static final double kV = 0.1183; 
     public static final double kS = 0.00;

    /** Flywheel Geometry */
       public static final double MAX_RPM = 5000; 
    public static final double TARGET_RPM_HARDSET = 2000.0; 
    public static final double FLYWHEEL_GEAR_RATIO = 1.25; 
    public static final double FLYWHEEL_IDLE_RPM = 60.0; //kraken x60 
    public static final double FLYWHEEL_RPM_TOLERANCE = 100.0;

    /** Current Limits */
    public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT = 60.0; // Amps

    /** Motor Inversion Settings */
    public static final boolean LEFT_FLYWHEEL_INVERTED = false; 
    public static final boolean RIGHT_FLYWHEEL_OPPOSED = true; 

    public static final int FLYWHEEL_STATOR_CURRENT_LIMIT = 80;
  }

  public static final class FeederConstants{
    public static final double FEEDER_RPM = -1000;
  }

  public static final class TurretConstants{
    public static final Transform3d ROBOT_RELATIVE_TURRET = new Transform3d(0, 0, 0, new Rotation3d()); // update when turret constants come out

    /** CAN IDs */
    public static final int MOTOR_ID = 17;      // TODO: set correct ID
    public static final int ENCODER_ID = 18;    // TODO: set correct ID

    /** Characterization Gains */
    public static final double kS = 0.1;
    public static final double kV = 0.2;
    public static final double kA = 0.01;
    public static final double kG = 0.0;

    /** PID Gains */
    public static final double kP = 5;
    public static final double kI = 0.01;
    public static final double kD = 0.2;

    /** Motion Constraints. IN DEGREES PER SECOND */
    public static final double kMaxV = 35; // m/s
    public static final double kMaxA = 45; // m/s^2

    public static final boolean MOTOR_INVERTED = true; // TODO: test and set

    /** Turret Geometry */
    public static final double MOTOR_GEAR_RATIO = 10.0; // Motor reduction ratio
    public static final double ENCODER_GEAR_RATIO =  150.0 / 259.0 ; // Encoder reduction ratio
    

    // Absolute encoder setup
    public static final double ABSOLUTE_ENCODER_OFFSET = 0.199; 
    public static final boolean ABSOLUTE_ENCODER_REVERSED = true; 

    // soft limits
    public static final double LOWER_SOFT_LIMIT = -135;
    public static final double UPPER_SOFT_LIMIT = 135;
  }

  public static final class IntakeConstants{
    public static final boolean EXTENSION_ENABLED = false; // temp: set true once extension + limit switch are wired
    public static final int EXTEND_MOTOR_ID = 15; 
    public static final int SPIN_MOTOR_ID = 16; 
    public static final String CAN_BUS = "Canivore";

    public static final int RETRACT_LIMIT_SWITCH_ID = 67; // TODO: set correct DIO port
    public static final boolean RETRACT_LIMIT_SWITCH_INVERTED = false; // toDo: confirm if this needs to be true or false based on wiring and testing; eg, does it start as true or false when stowed.

    public static final boolean EXTEND_MOTOR_INVERTED = false; //TODO
    public static final boolean SPIN_MOTOR_INVERTED = false; //TODO

    public static final double EXTEND_SPEED = 0.65; //tune
  public static final double RETRACT_SPEED = -0.5; //tune
    public static final double SPIN_SPEED = 0.7; //tune
    public static final double SPRINGY_EXTEND_SPEED = 0.12; //tune
    public static final double SPRINGY_SPIN_SPEED = 0.7; //tune

  public static final double MAX_EXTENSION_ROTATIONS = 60.0; // TODO: tune
  public static final double SPRINGY_TRIGGER_ROTATIONS = 55.0; // TODO: tune

    public static final double EXTEND_SUPPLY_CURRENT_LIMIT = 35;
    public static final double EXTEND_STATOR_CURRENT_LIMIT = 60;

    public static final double EXTEND_SPRINGY_SUPPLY_CURRENT_LIMIT = 12;
    public static final double EXTEND_SPRINGY_STATOR_CURRENT_LIMIT = 20;

    public static final double SPIN_SUPPLY_CURRENT_LIMIT = 30;
    public static final double SPIN_STATOR_CURRENT_LIMIT = 50;
  }
}
