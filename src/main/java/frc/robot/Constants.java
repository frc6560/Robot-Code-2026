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
            0.354,
            -0.0248,
            0.192,
            new Rotation3d(0, Units.degreesToRadians(17.23), 0)
          );
          break;
        default:
          throw new IllegalArgumentException("Invalid limelight name. You might want to double check your configs: " + name);
      }
      return limelightPose;
    }
    public static final double kStdvXYBase = 0.15; // No idea how to tune these base values.
    public static final double kStdvThetaBase = 1.0; // See above
    public static final double JUMP_TOLERANCE = 0.6; // meters. again, needs tuning.
  }

  public static final class ElevatorConstants {
    
    //unknown
    public static int ElevLeftCanID = 15;
    public static int ElevRightCanID = 16;
    
    public static final int TopLimitSwitchID = 3;
    public static final int BotLimitSwitchID = 4;
    

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double kP = 3.5;
    public static final double kI = 0.1;
    public static final double kD = 0.1;

    public static final double kMaxV = 30;
    public static final double kMaxA = 23;

    public static final double kG = 0.4;
    
      
        
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

        public static double HeightToRotations(double TargetHeight) {
          return ((TargetHeight-WristHeightOffGround)/NumInPerRot);
        } 

        //placeholder
        public static final double NumInPerRot = 13.4962820398;
        public static final double WristHeightOffGround = 17;
        //need to be tested


        public static final double L2 = 5;///*5*/ HeightToRotations(32);
        public static final double L3 = 10;///*10*/HeightToRotations(47.625);
        public static final double L4 = 15;///*15*/ HeightToRotations(72);
        public static final double REMOVEBALLL2 = 0.2;///*4*/ HeightToRotations(32 + 8.125);
        public static final double REMOVEBALLL3 = 14;///*8*/ HeightToRotations(47.625 + 8.125);
        public static final double SHOOTBALL = 20.22;///*18*/ HeightToRotations(76 + 8.125);
        public static final double L2BALL = 1.8;//HeightToRotations(32 + 8.125);
        public static final double L3BALL = 6.7;//HeightToRotations(47.625 + 8.125); 677777777777 haahahahahahahahahahhahahahahaha
        public static final double STOW = 0.2;//HeightToRotations(18);
        public static final double GROUNDBALL = 0.2;//HeightToRotations(20);
    
  }

  public static final class ShooterConstants{
    public static final double FLYWHEEL_RPM = -1750;
  }

  public static final class FeederConstants{
    public static final double FEEDER_RPM = -1000;
  }

  public static final class TurretConstants{
    public static final Transform3d ROBOT_RELATIVE_TURRET = new Transform3d(-0.275, 0, 0, new Rotation3d()); // update when turret constants come out
    // MAKE SURE THIS IS UP LEFT. NOT UP RIGHT.
  }

  public static final class ArmConstants
  {
    /** CAN IDs */
    public static final int MOTOR_ID = 40;      // TODO: set correct ID
    public static final int ENCODER_ID = 0;    // TODO: set correct ID

    /** Characterization Gains */
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.0;

    /** PID Gains */
    public static final double kP = 4;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    /** Motion Constraints */
    public static final double kMaxV = 128000; // m/s
    public static final double kMaxA = 128000; // m/s^2

    /** Arm PID Gains */
    public static final double ARM_KP = 3.5;
    public static final double ARM_KI = 0.1;
    public static final double ARM_KD = 0.1;
    public static final double ARM_KS = 0.1;
    public static final double ARM_KG = 0.0;
    public static final double ARM_KV = 0.0;
    public static final double ARM_KA = 0.0;

    /** Arm Geometry */
    public static final double MOTOR_GEAR_RATIO = 108.0; // Motor reduction ratio
    public static final double ENCODER_GEAR_RATIO = 81.0; // Encoder reduction ratio
    public static final double ARM_LENGTH_METERS = 0.5; // TODO: measure (m)
    public static final double ARM_MASS_KG = 5.0;       // TODO: measure (kg)
    public static final double MAX_ANGLE_DEG = 0.0;
    public static final double MIN_ANGLE_DEG = -70.0;
    

    /** Gravity constant */
    public static final double GRAVITY = 9.81; // m/s^2

    /** Arm Setpoints (Degrees) */
    public static final double STOW_POSITION_DEG = -8.0;
    public static final double PICKUP_POSITION_DEG = -95;
    public static final double REEF_POSITION_DEG_low = -19; //11.0+90.0;
    public static final double REEF_POSITION_DEG_high = -19; //11.0+90.0;
    public static final double BARGE = -8.0; //31.0+90.0;
    public static final double PROCESSOR_DEG = 0.0;
    

    // Absolute encoder setup
  public static final int ABS_ENCODER_DIO_PORT = 0;   // change to your wiring
  public static final double ABS_ENCODER_OFFSET_DEG = 0.0; // tune so stow = 0°
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

}
