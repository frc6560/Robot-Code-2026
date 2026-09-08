package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** All measurements are SI units unless their names explicitly state otherwise. */
public final class Constants {
  private Constants() {}

  public static final class Drive {
    private Drive() {}

    // The swerve motors and CANcoders are connected through the roboRIO CAN bus.
    public static final String CAN_BUS = "rio";
    // The Pigeon2 is connected to the roboRIO CAN network, matching the Beeline configuration.
    public static final String GYRO_CAN_BUS = "rio";
    public static final int GYRO_CAN_ID = 13;
    public static final double LOOP_PERIOD_SECONDS = 0.020;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.91);
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
    public static final double DRIVE_GEAR_RATIO = 5.357;
    public static final double STEER_GEAR_RATIO = 21.4285714286;
    public static final double MODULE_DISTANCE_METERS = Units.inchesToMeters(10.875);
    public static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(19.5);
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 8.0;
    public static final double MAX_LINEAR_ACCELERATION_MPS_SQUARED = 6.0;
    public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED = 12.0;
    public static final double DRIVE_CURRENT_LIMIT_AMPS = 40.0;
    public static final double STEER_CURRENT_LIMIT_AMPS = 40.0;
    public static final double STEER_KP_VOLTS_PER_ROTATION = 12.0;
    public static final double STEER_KD_VOLTS_PER_ROTATION_PER_SECOND = 0.0;
    public static final double STEER_KS_VOLTS = 0.10;
    public static final double MAX_STEER_VOLTAGE = 4.0;
    public static final double STEER_CLOSED_LOOP_RAMP_SECONDS = 0.10;
    public static final double OPEN_LOOP_RAMP_SECONDS = 0.25;
    public static final double IDLE_LINEAR_SPEED_EPSILON_MPS = 0.01;
    public static final double IDLE_ANGULAR_SPEED_EPSILON_RAD_PER_SEC = 0.01;
    public static final double CONTROLLER_TRANSLATION_DEADBAND = 0.08;
    public static final double CONTROLLER_ROTATION_DEADBAND = 0.08;

    // kV and kA must be replaced with drivetrain SysId values before competition use.
    // These conservative startup values keep the 12 V LQR loop bounded for first characterization.
    public static final double DRIVE_KV_VOLTS_PER_MPS = 2.004;
    public static final double DRIVE_KA_VOLTS_PER_MPS_SQUARED = 0.173;

    public static final ModuleConfig FRONT_LEFT =
        new ModuleConfig("FrontLeft", 4, 6, 5, 9.967,
            new Translation2d(MODULE_DISTANCE_METERS, MODULE_DISTANCE_METERS));
    public static final ModuleConfig FRONT_RIGHT =
        new ModuleConfig("FrontRight", 1, 3, 2, 112.500,
            new Translation2d(MODULE_DISTANCE_METERS, -MODULE_DISTANCE_METERS));
    public static final ModuleConfig BACK_LEFT =
        new ModuleConfig("BackLeft", 7, 9, 8, 254.813,
            new Translation2d(-MODULE_DISTANCE_METERS, MODULE_DISTANCE_METERS));
    public static final ModuleConfig BACK_RIGHT =
        new ModuleConfig("BackRight", 10, 12, 11, 220.730,
            new Translation2d(-MODULE_DISTANCE_METERS, -MODULE_DISTANCE_METERS));
    public static final ModuleConfig[] MODULES = {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
  }

  public record ModuleConfig(
      String name, int driveMotorCanId, int steerMotorCanId, int encoderCanId,
      double encoderOffsetDegrees, Translation2d location) {}
}
