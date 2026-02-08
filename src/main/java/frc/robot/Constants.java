// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalSource;
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
    public static final double kP_rotation = 4.0;

    public static final double kI_translation = 0.0;
    public static final double kI_rotation = 0.0;

    public static final double kD_translation = 0.0;
    public static final double kD_rotation = 0.0;
  }

  public static final class FieldConstants{
    // public static final Pose2d START = new Pose2d(3.152, 4.018, Rotation2d.fromDegrees(-90));
    public static final Pose2d TARGET_POSE = new Pose2d(3.650, 4.010, Rotation2d.fromDegrees(0));
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
            new Rotation3d(0, Units.degreesToRadians(40), 0)
          );
          break;
        default:
          limelightPose = new Pose3d();
      }
      return limelightPose;
    }
    public static final double kStdvXYBase = 0.15;
    public static final double kStdvThetaBase = 1.0;
    public static final double JUMP_TOLERANCE = 0.6; // meters
  }

  public static final class FlywheelConstants {
    /** CAN IDs */
    public static final int LEFT_FLYWHEEL_ID = 19;
    public static final int RIGHT_FLYWHEEL_ID = 20;

    /** PID Gains */
    public static final double kP = 0.2;
    public static final double kI = 0.005;
    public static final double kD = 0.0008;
    public static final double kV = 0.12; // TODO: tune
     public static final double kS = 0.25;

    /** Flywheel Geometry */
       public static final double MAX_RPM = 5000; // Example value for Falcon 500
    public static final double TARGET_RPM_HARDSET = 2000.0; 
    public static final double FLYWHEEL_GEAR_RATIO = 1.25; // TODO: set correct gear ratio
    public static final double FLYWHEEL_IDLE_RPM = 50.0; //kraken x60 
    public static final double FLYWHEEL_RPM_TOLERANCE = 100.0;

    /** Current Limits */
    public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT = 60.0; // Amps

    /** Motor Inversion Settings */
    public static final boolean LEFT_FLYWHEEL_INVERTED = false; // TODO: Test and adjust
    public static final boolean RIGHT_FLYWHEEL_OPPOSED = true; // Opposite of leader

    /** Hub Positions (used by ShotCalculator) */
    public static final Translation2d HUB_BLUE_POSITION = new Translation2d(4.03, 8.07/2);
    public static final Translation2d HUB_RED_POSITION = new Translation2d(16.54-4.03, 8.07/2);

    /** Distance to RPM Lookup Table */
    public static final double[][] DISTANCE_RPM_TABLE = {
      {0.0, 1000.0},
      {1.0, 1200.0},
      {2.0, 1400.0},
      {3.0, 1600.0},
      {4.0, 1800.0},
      {5.0, 2000.0}
      // TODO: Fill in with actual measured values
    };
   
    public static final int FLYWHEEL_STATOR_CURRENT_LIMIT = 80;
  }

  public static final class HoodConstants {
    /** CAN IDs */
    public static final int HOOD_MOTOR_ID = 21;

    /** Feedforward Gains (for TalonFX Slot0) */
    public static final double kS = 0.2;   // Static friction voltage
    public static final double kV = 0.12;  // Velocity feedforward
    public static final double kA = 0.0;   // Acceleration feedforward

    /** PID Gains */
    public static final double kP = 0.5;   // Proportional gain
    public static final double kI = 0.0;   // Integral gain
    public static final double kD = 0.0;   // Derivative gain

    /** Motion Profile Constraints */
    public static final double kMaxV = 360.0;  // Max velocity (degrees/second)
    public static final double kMaxA = 720.0;  // Max acceleration (degrees/second²)

    /** Hood Geometry */
    public static final double HOOD_GEAR_RATIO = 40.0; // 40:1 total gear reduction
    public static final double ABSOLUTE_HOOD_ENCODER_GEAR_RATIO = 44.0 / 18.0; // 2.44:1

    /** Motor Inversion */
    public static final boolean HOOD_MOTOR_INVERTED = false; // TODO: Test and adjust

    /** Current Limits */
    public static final double HOOD_CURRENT_LIMIT = 30.0; // Amps

    /** Hood Angle Limits */
    public static final double HOOD_MIN_ANGLE = 0.0;   // degrees
    public static final double HOOD_MAX_ANGLE = 37.0;  // degrees

    /** Distance to Angle Lookup Table */
    public static final double[][] DISTANCE_ANGLE_TABLE = {
      {0.0, 10.0},
      {1.0, 15.0},
      {2.0, 20.0},
      {3.0, 25.0},
      {4.0, 30.0},
      {5.0, 35.0}
      // TODO: Fill in with actual measured values
    };

    public static final int HOOD_ABSOLUTE_ENCODER_ID = 22;

    public static final double HOOD_ABSOLUTE_ENCODER_OFFSET = 0;
  }

  public static final class ElevatorConstants {
    /** CAN IDs */
    public static int ElevLeftCanID = 15;
    public static int ElevRightCanID = 16;
    
    /** Limit Switches */
    public static final int TopLimitSwitchID = 3;
    public static final int BotLimitSwitchID = 4;

    /** Feedforward Gains */
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0.4;

    /** PID Gains */
    public static final double kP = 3.5;
    public static final double kI = 0.1;
    public static final double kD = 0.1;

    /** Motion Constraints */
    public static final double kMaxV = 30;
    public static final double kMaxA = 23;

    /** Elevator Geometry */
    public static final double NumInPerRot = 13.4962820398;
    public static final double WristHeightOffGround = 17;

    /** Helper Method */
    public static double HeightToRotations(double TargetHeight) {
      return ((TargetHeight - WristHeightOffGround) / NumInPerRot);
    }

    /** Elevator States */
    public static enum ElevState {
      L2BALL(HeightToRotations(32 + 8.125)),
      L3BALL(HeightToRotations(47.625 + 8.125)),
      SHOOTBALL(HeightToRotations(76 + 8.125)),
      STOW(HeightToRotations(18)),
      GROUNDBALL(HeightToRotations(20));

      public final double elevatorSetPoint;
      
      private ElevState(double elevatorSetpoint) {
        this.elevatorSetPoint = elevatorSetpoint;
      }

      public double getValue() {
        return elevatorSetPoint;
      }
    }

    /** Position Constants */
    public static final double L2 = 5;
    public static final double L3 = 10;
    public static final double L4 = 15;
    public static final double REMOVEBALLL2 = 0.2;
    public static final double REMOVEBALLL3 = 14;
    public static final double SHOOTBALL = 20.22;
    public static final double L2BALL = 1.8;
    public static final double L3BALL = 6.7;
    public static final double STOW = 0.2;
    public static final double GROUNDBALL = 0.2;
  }

  public static final class ArmConstants {
    /** CAN IDs */
    public static final int MOTOR_ID = 40;
    public static final int ENCODER_ID = 0;

    /** Feedforward Gains */
    public static final double kS = 0.1;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.0;

    /** PID Gains */
    public static final double kP = 4.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    /** Motion Constraints */
    public static final double kMaxV = 128000; // degrees/s
    public static final double kMaxA = 128000; // degrees/s²

    /** Arm Geometry */
    public static final double MOTOR_GEAR_RATIO = 108.0;
    public static final double ENCODER_GEAR_RATIO = 81.0;
    public static final double ARM_LENGTH_METERS = 0.5; // TODO: measure
    public static final double ARM_MASS_KG = 5.0;       // TODO: measure
    public static final double MAX_ANGLE_DEG = 0.0;
    public static final double MIN_ANGLE_DEG = -70.0;

    /** Gravity Constant */
    public static final double GRAVITY = 9.81; // m/s²

    /** Arm Setpoints (Degrees) */
    public static final double STOW_POSITION_DEG = -8.0;
    public static final double PICKUP_POSITION_DEG = -95.0;
    public static final double REEF_POSITION_DEG_low = -19.0;
    public static final double REEF_POSITION_DEG_high = -19.0;
    public static final double BARGE = -8.0;
    public static final double PROCESSOR_DEG = 0.0;

    /** Absolute Encoder Setup */
    public static final int ABS_ENCODER_DIO_PORT = 0;
    public static final double ABS_ENCODER_OFFSET_DEG = 0.0;
    public static final boolean ABS_ENCODER_REVERSED = false;

    /** Arm Setpoints (Radians) */
    public static final double STOW_POSITION_RAD = Math.toRadians(STOW_POSITION_DEG);
    public static final double PICKUP_POSITION_RAD = Math.toRadians(PICKUP_POSITION_DEG);
    public static final double REEF_POSITION_RAD_high = Math.toRadians(REEF_POSITION_DEG_high);
    public static final double REEF_POSITION_RAD_low = Math.toRadians(REEF_POSITION_DEG_low);

    /** Arm State Enum */
    public enum ArmState {
      STOW(STOW_POSITION_DEG),
      PICKUP(PICKUP_POSITION_DEG),
      REEF_high(REEF_POSITION_DEG_high),
      REEF_low(REEF_POSITION_DEG_low);

      public final double angleDeg;

      ArmState(double angleDeg) {
        this.angleDeg = angleDeg;
      }
    }
  }
  public static final class IntakeConstants {
    public static final boolean EXTENSION_ENABLED = false; // temp: set true once extension + limit switch are wired
    public static final int EXTEND_MOTOR_ID = 15; // TODO: set correct ID //completed
    public static final int SPIN_MOTOR_ID = 16; // TODO: set correct ID /completed
    public static final String CAN_BUS = "Canivore";

    public static final int RETRACT_LIMIT_SWITCH_ID = 5; // TODO: set correct DIO port
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