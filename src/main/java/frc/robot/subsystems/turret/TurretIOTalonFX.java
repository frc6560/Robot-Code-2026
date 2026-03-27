package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TurretConstants;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX turretMotor;
    private final CANcoder absoluteEncoder;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorCurrent;
    private final StatusSignal<Temperature> motorTemp;
    private final StatusSignal<Angle> absolutePosition;

    public TurretIOTalonFX() {
        turretMotor = new TalonFX(TurretConstants.MOTOR_ID, "rio");
        absoluteEncoder = new CANcoder(TurretConstants.ENCODER_ID, "rio");

        configureAbsoluteEncoder();
        configureMotor();
        seedMotorEncoder();

        motorPosition = turretMotor.getPosition();
        motorVelocity = turretMotor.getVelocity();
        motorVoltage = turretMotor.getMotorVoltage();
        motorCurrent = turretMotor.getSupplyCurrent();
        motorTemp = turretMotor.getDeviceTemp();
        absolutePosition = absoluteEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            motorPosition, motorVelocity, motorVoltage, motorCurrent, motorTemp, absolutePosition
        );

        turretMotor.optimizeBusUtilization();
        absoluteEncoder.optimizeBusUtilization();
    }

    private void configureAbsoluteEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = TurretConstants.ABSOLUTE_ENCODER_OFFSET;
        config.MagnetSensor.SensorDirection = TurretConstants.ABSOLUTE_ENCODER_REVERSED
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
        absoluteEncoder.getConfigurator().apply(config);
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = TurretConstants.kS;
        slot0.kV = TurretConstants.kV;
        slot0.kA = TurretConstants.kA;
        slot0.kP = TurretConstants.kP;
        slot0.kI = TurretConstants.kI;
        slot0.kD = TurretConstants.kD;

        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = TurretConstants.kMaxV * TurretConstants.MOTOR_GEAR_RATIO / 360.0;
        motionMagicConfigs.MotionMagicAcceleration = TurretConstants.kMaxA * TurretConstants.MOTOR_GEAR_RATIO / 360.0;
        motionMagicConfigs.MotionMagicJerk = 0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = TurretConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;

        // Soft limits (convert from degrees to motor rotations)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            TurretConstants.UPPER_SOFT_LIMIT * TurretConstants.MOTOR_GEAR_RATIO / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            TurretConstants.LOWER_SOFT_LIMIT * TurretConstants.MOTOR_GEAR_RATIO / 360.0;

        turretMotor.getConfigurator().apply(config);
    }

    @Override
    public void seedMotorEncoder() {
        Timer.delay(0.25);
        double cancoderRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        double turretRotations = cancoderRotations / TurretConstants.ENCODER_GEAR_RATIO;
        double motorRotations = turretRotations * TurretConstants.MOTOR_GEAR_RATIO;
        turretMotor.setPosition(motorRotations);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            motorPosition, motorVelocity, motorVoltage, motorCurrent, motorTemp, absolutePosition
        );

        inputs.motorPositionRotations = motorPosition.getValueAsDouble();
        inputs.motorVelocityRPS = motorVelocity.getValueAsDouble();
        inputs.motorAppliedVolts = motorVoltage.getValueAsDouble();
        inputs.motorCurrentAmps = motorCurrent.getValueAsDouble();
        inputs.motorTempCelsius = motorTemp.getValueAsDouble();

        inputs.absoluteEncoderPositionRotations = absolutePosition.getValueAsDouble();
        inputs.turretAngleDegrees = inputs.motorPositionRotations * 360.0 / TurretConstants.MOTOR_GEAR_RATIO;
        inputs.turretVelocityDegreesPerSec = inputs.motorVelocityRPS * 360.0 / TurretConstants.MOTOR_GEAR_RATIO;
    }

    @Override
    public void setTargetAngle(double angleDegrees) {
        double targetMotorRotations = angleDegrees * TurretConstants.MOTOR_GEAR_RATIO / 360.0;
        turretMotor.setControl(motionMagicRequest.withPosition(targetMotorRotations));
    }

    @Override
    public void setTargetAngleWithVelocity(double angleDegrees, double velocityDegreesPerSec) {
        double targetMotorRotations = angleDegrees * TurretConstants.MOTOR_GEAR_RATIO / 360.0;
        double targetMotorVelocityRPS = velocityDegreesPerSec * TurretConstants.MOTOR_GEAR_RATIO / 360.0;

        // Constant force spring compensation - push away from center at extreme angles
        double springCompensationVolts = 0.0;
        if (angleDegrees > 180.0) {
            springCompensationVolts = TurretConstants.kG;
        } else if (angleDegrees < -60.0) {
            springCompensationVolts = -TurretConstants.kG;
        }

        turretMotor.setControl(positionRequest
            .withPosition(targetMotorRotations)
            .withVelocity(targetMotorVelocityRPS)
            .withFeedForward(springCompensationVolts));
    }

    @Override
    public void stop() {
        turretMotor.set(0);
    }

    @Override
    public void setVoltage(double volts) {
        turretMotor.setControl(voltageRequest.withOutput(volts));
    }
}
