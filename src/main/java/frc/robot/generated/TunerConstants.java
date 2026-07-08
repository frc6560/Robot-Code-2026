package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

public class TunerConstants {
  // Steer motor gains (firmware units: volts per rotation of mechanism)
  // Starting with AKit template defaults — needs on-robot tuning
  private static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(0.5)
          .withKS(0.1)
          .withKV(1.91)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  // Drive motor gains (firmware units: volts per rotation/sec)
  // kS and kV from characterization — needs re-validation on robot
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0.186).withKV(0.124);

  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  private static final DriveMotorArrangement kDriveMotorType =
      DriveMotorArrangement.TalonFX_Integrated;
  private static final SteerMotorArrangement kSteerMotorType =
      SteerMotorArrangement.TalonFX_Integrated;

  // NOTE: no longer used by ModuleIOTalonFX. The steer PID now runs on the motor's internal
  // rotor sensor (seeded from the CANcoder at startup) because RemoteCANcoder feedback
  // (~100Hz CAN frames) caused steer chatter with these gains. See ModuleIOTalonFX.
  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

  private static final Current kSlipCurrent = Amps.of(120.0);

  // Match the drive feel of the YAGSL setup on glendale-working-branch:
  // - 40A SUPPLY limit (YAGSL's "currentLimit.drive: 40" is a supply limit, not stator —
  //   the previous 40A stator limit here clamped torque much harder than glendale did)
  // - 0.25s voltage ramps (YAGSL "rampRate.drive: 0.25"), which smooth out joystick steps
  private static final TalonFXConfiguration driveInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLimitEnable(true))
          .withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.25))
          .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.25));
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(40))
                  .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
  private static final Pigeon2Configuration pigeonConfigs = null;

  public static final CANBus kCANBus = new CANBus("Canivore", "./logs/example.hoot");

  // Theoretical free speed at 12V — measured ~4.42 m/s
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.42);

  // Coupling ratio: how much the azimuth rotation back-drives the drive encoder
  // For L2 swerve modules this is typically ~3.5-4.0
  private static final double kCoupleRatio = 3.818;

  private static final double kDriveGearRatio = 5.357;
  private static final double kSteerGearRatio = 21.4285714286;
  private static final Distance kWheelRadius = Inches.of(1.955);

  // YAGSL config on glendale-working-branch ran ALL drive motors uninverted, so the right side
  // must not be inverted either (the CTRE template default of inverting the right side was wrong
  // for this robot and made the right wheels spin backwards)
  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = false;

  private static final int kPigeonId = 13;

  // Simulation constants
  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
  private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withCouplingGearRatio(kCoupleRatio)
              .withWheelRadius(kWheelRadius)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
              .withSlipCurrent(kSlipCurrent)
              .withSpeedAt12Volts(kSpeedAt12Volts)
              .withDriveMotorType(kDriveMotorType)
              .withSteerMotorType(kSteerMotorType)
              .withFeedbackSource(kSteerFeedbackType)
              .withDriveMotorInitialConfigs(driveInitialConfigs)
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withEncoderInitialConfigs(encoderInitialConfigs)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 4;
  private static final int kFrontLeftSteerMotorId = 6;
  private static final int kFrontLeftEncoderId = 5;
  private static final Angle kFrontLeftEncoderOffset = Degrees.of(-0.967);
  // Steer motors MUST be inverted: log akit_26-07-08_01-18-37.wpilog shows that with
  // uninverted steer motors, applied volts and CANcoder velocity anti-correlate on all four
  // modules (positive feedback → uncontrolled spinning). YAGSL's "inverted.angle: false"
  // does not translate to Phoenix inversion semantics.
  private static final boolean kFrontLeftSteerMotorInverted = true;
  private static final boolean kFrontLeftEncoderInverted = false;

  private static final Distance kFrontLeftXPos = Inches.of(10.875);
  private static final Distance kFrontLeftYPos = Inches.of(10.875);

  // Front Right
  private static final int kFrontRightDriveMotorId = 1;
  private static final int kFrontRightSteerMotorId = 3;
  private static final int kFrontRightEncoderId = 2;
  // Measured 2026-07-07: CANcoder read 1.970 rad (112.87°) with wheel straight and offset zeroed
  private static final Angle kFrontRightEncoderOffset = Degrees.of(-112.872);
  private static final boolean kFrontRightSteerMotorInverted = true;
  private static final boolean kFrontRightEncoderInverted = false;

  private static final Distance kFrontRightXPos = Inches.of(10.875);
  private static final Distance kFrontRightYPos = Inches.of(-10.875);

  // Back Left
  private static final int kBackLeftDriveMotorId = 7;
  private static final int kBackLeftSteerMotorId = 9;
  private static final int kBackLeftEncoderId = 8;
  // Measured 2026-07-07: CANcoder read -1.913 rad (-109.61°) with wheel straight and offset zeroed
  private static final Angle kBackLeftEncoderOffset = Degrees.of(109.606);
  private static final boolean kBackLeftSteerMotorInverted = true;
  private static final boolean kBackLeftEncoderInverted = false;

  private static final Distance kBackLeftXPos = Inches.of(-10.875);
  private static final Distance kBackLeftYPos = Inches.of(10.875);

  // Back Right
  private static final int kBackRightDriveMotorId = 10;
  private static final int kBackRightSteerMotorId = 12;
  private static final int kBackRightEncoderId = 11;
  // Measured 2026-07-07: CANcoder read -2.724 rad (-156.07°) with wheel straight and offset zeroed
  private static final Angle kBackRightEncoderOffset = Degrees.of(156.073);
  private static final boolean kBackRightSteerMotorInverted = true;
  private static final boolean kBackRightEncoderInverted = false;

  private static final Distance kBackRightXPos = Inches.of(-10.875);
  private static final Distance kBackRightYPos = Inches.of(-10.875);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              kFrontLeftXPos,
              kFrontLeftYPos,
              kInvertLeftSide,
              kFrontLeftSteerMotorInverted,
              kFrontLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              kFrontRightXPos,
              kFrontRightYPos,
              kInvertRightSide,
              kFrontRightSteerMotorInverted,
              kFrontRightEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              kBackLeftXPos,
              kBackLeftYPos,
              kInvertLeftSide,
              kBackLeftSteerMotorInverted,
              kBackLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              kBackRightXPos,
              kBackRightYPos,
              kInvertRightSide,
              kBackRightSteerMotorInverted,
              kBackRightEncoderInverted);

  public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    }

    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new, TalonFX::new, CANcoder::new,
          drivetrainConstants, odometryUpdateFrequency, modules);
    }

    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new, TalonFX::new, CANcoder::new,
          drivetrainConstants, odometryUpdateFrequency,
          odometryStandardDeviation, visionStandardDeviation, modules);
    }
  }
}
