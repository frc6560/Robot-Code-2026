package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.Constants.HoodConstants;

public class HoodIOTalonFX implements HoodIO {
    private final TalonFX hoodMotor;
    private final CANcoder absoluteEncoder;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorCurrent;
    private final StatusSignal<Temperature> motorTemp;
    private final StatusSignal<Angle> absolutePosition;

    public HoodIOTalonFX() {
        hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID, "rio");
        absoluteEncoder = new CANcoder(HoodConstants.HOOD_ABSOLUTE_ENCODER_ID, "rio");

        configureAbsoluteEncoder();
        configureMotor();
        seedMotorEncoder();

        motorPosition = hoodMotor.getPosition();
        motorVelocity = hoodMotor.getVelocity();
        motorVoltage = hoodMotor.getMotorVoltage();
        motorCurrent = hoodMotor.getSupplyCurrent();
        motorTemp = hoodMotor.getDeviceTemp();
        absolutePosition = absoluteEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            motorPosition, motorVelocity, motorVoltage, motorCurrent, motorTemp, absolutePosition
        );

        hoodMotor.optimizeBusUtilization();
        absoluteEncoder.optimizeBusUtilization();
    }

    private void configureAbsoluteEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = HoodConstants.HOOD_ABSOLUTE_ENCODER_OFFSET;
        absoluteEncoder.getConfigurator().apply(config);
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = HoodConstants.kS;
        slot0.kV = HoodConstants.kV;
        slot0.kA = HoodConstants.kA;
        slot0.kP = HoodConstants.kP;
        slot0.kI = HoodConstants.kI;
        slot0.kD = HoodConstants.kD;

        config.MotorOutput.Inverted = HoodConstants.HOOD_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = HoodConstants.kMaxV * HoodConstants.HOOD_GEAR_RATIO / 360.0;
        motionMagicConfigs.MotionMagicAcceleration = HoodConstants.kMaxA * HoodConstants.HOOD_GEAR_RATIO / 360.0;
        motionMagicConfigs.MotionMagicJerk = 0;

        hoodMotor.getConfigurator().apply(config);
    }

    private void seedMotorEncoder() {
        Timer.delay(0.25);

        double raw = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        double cancoderRotations = raw - Math.floor(raw);
        double hoodRotations = cancoderRotations / HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO;
        double motorRotations = hoodRotations * HoodConstants.HOOD_GEAR_RATIO;
        hoodMotor.setPosition(motorRotations);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            motorPosition, motorVelocity, motorVoltage, motorCurrent, motorTemp, absolutePosition
        );

        inputs.motorPositionRotations = motorPosition.getValueAsDouble();
        inputs.motorVelocityRPS = motorVelocity.getValueAsDouble();
        inputs.motorAppliedVolts = motorVoltage.getValueAsDouble();
        inputs.motorCurrentAmps = motorCurrent.getValueAsDouble();
        inputs.motorTempCelsius = motorTemp.getValueAsDouble();

        double rawAbsolute = absolutePosition.getValueAsDouble();
        inputs.absoluteEncoderPositionRotations = rawAbsolute - Math.floor(rawAbsolute);
        inputs.hoodAngleDegrees = inputs.motorPositionRotations / HoodConstants.HOOD_GEAR_RATIO * 360.0;
    }

    @Override
    public void setTargetAngle(double angleDegrees) {
        double targetMotorRotations = angleDegrees * HoodConstants.HOOD_GEAR_RATIO / 360.0;
        hoodMotor.setControl(motionMagicRequest.withPosition(targetMotorRotations));
    }

    @Override
    public void stop() {
        hoodMotor.stopMotor();
    }
}
