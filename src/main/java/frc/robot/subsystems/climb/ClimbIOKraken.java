package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOKraken implements ClimbIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final DigitalInput leftLimitSwitch;
    private final DigitalInput rightLimitSwitch;

    private final MotionMagicVoltage leftMotionMagicReq = new MotionMagicVoltage(0).withSlot(0);
    private final MotionMagicVoltage rightMotionMagicReq = new MotionMagicVoltage(0).withSlot(0);

    // Signals
    private final StatusSignal<Angle> leftPos, rightPos;
    private final StatusSignal<AngularVelocity> leftVel, rightVel;
    private final StatusSignal<Voltage> leftVolts, rightVolts;
    private final StatusSignal<Current> leftCurrent, rightCurrent;
    private final StatusSignal<Temperature> leftTemp, rightTemp;

    public ClimbIOKraken() {
        leftMotor = new TalonFX(ClimbConstants.LEFT_MOTOR_ID, ClimbConstants.CAN_BUS);
        rightMotor = new TalonFX(ClimbConstants.RIGHT_MOTOR_ID, ClimbConstants.CAN_BUS);
        leftLimitSwitch = new DigitalInput(ClimbConstants.LEFT_LIMIT_SWITCH_ID);
        rightLimitSwitch = new DigitalInput(ClimbConstants.RIGHT_LIMIT_SWITCH_ID);

        configureMotor(leftMotor, ClimbConstants.LEFT_MOTOR_INVERTED);
        configureMotor(rightMotor, ClimbConstants.RIGHT_MOTOR_INVERTED);

        leftPos = leftMotor.getPosition(); rightPos = rightMotor.getPosition();
        leftVel = leftMotor.getVelocity(); rightVel = rightMotor.getVelocity();
        leftVolts = leftMotor.getMotorVoltage(); rightVolts = rightMotor.getMotorVoltage();
        leftCurrent = leftMotor.getSupplyCurrent(); rightCurrent = rightMotor.getSupplyCurrent();
        leftTemp = leftMotor.getDeviceTemp(); rightTemp = rightMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
            leftPos, rightPos, leftVel, rightVel, leftVolts, rightVolts, leftCurrent, rightCurrent, leftTemp, rightTemp
        );

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    private void configureMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

       
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0; // Max amps drawn from battery (Prevents brownouts)
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0; // Max amps sent to motor (Prevents melting / stripping gears)
        
        
        config.Slot0.kP = ClimbConstants.kP;
        config.Slot0.kI = ClimbConstants.kI;
        config.Slot0.kD = ClimbConstants.kD;
        config.Slot0.kV = ClimbConstants.kV;
        config.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.CRUISE_VELOCITY_RPS;
        config.MotionMagic.MotionMagicAcceleration = ClimbConstants.ACCELERATION_RPS2;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        BaseStatusSignal.refreshAll(leftPos, rightPos, leftVel, rightVel, leftVolts, rightVolts, leftCurrent, rightCurrent, leftTemp, rightTemp);

        inputs.leftPositionRotations = leftPos.getValueAsDouble();
        inputs.rightPositionRotations = rightPos.getValueAsDouble();
        inputs.leftVelocityRPS = leftVel.getValueAsDouble();
        inputs.rightVelocityRPS = rightVel.getValueAsDouble();
        
        inputs.appliedVolts[0] = leftVolts.getValueAsDouble(); inputs.appliedVolts[1] = rightVolts.getValueAsDouble();
        inputs.currentAmps[0] = leftCurrent.getValueAsDouble(); inputs.currentAmps[1] = rightCurrent.getValueAsDouble();
        inputs.tempCelsius[0] = leftTemp.getValueAsDouble(); inputs.tempCelsius[1] = rightTemp.getValueAsDouble();

        inputs.leftLimitSwitch = ClimbConstants.LIMIT_SWITCH_INVERTED ? !leftLimitSwitch.get() : leftLimitSwitch.get();
        inputs.rightLimitSwitch = ClimbConstants.LIMIT_SWITCH_INVERTED ? !rightLimitSwitch.get() : rightLimitSwitch.get();
    }

    @Override
    public void setLeftTarget(double target) { leftMotor.setControl(leftMotionMagicReq.withPosition(target)); }
    @Override
    public void setRightTarget(double target) { rightMotor.setControl(rightMotionMagicReq.withPosition(target)); }
    @Override
    public void setLeftPercent(double pct) { leftMotor.set(pct); }
    @Override
    public void setRightPercent(double pct) { rightMotor.set(pct); }
    @Override
    public void resetLeftPosition() { leftMotor.setPosition(0.0); }
    @Override
    public void resetRightPosition() { rightMotor.setPosition(0.0); }
}