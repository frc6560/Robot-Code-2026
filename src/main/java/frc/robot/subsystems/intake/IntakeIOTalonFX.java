package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX extendMotor;
    private final TalonFX spinMotor;
    private final DigitalInput retractLimitSwitch;

    private final StatusSignal<Angle> extendPosition;
    private final StatusSignal<AngularVelocity> extendVelocity;
    private final StatusSignal<Voltage> extendVoltage;
    private final StatusSignal<Current> extendCurrent;
    private final StatusSignal<Temperature> extendTemp;

    private final StatusSignal<AngularVelocity> spinVelocity;
    private final StatusSignal<Voltage> spinVoltage;
    private final StatusSignal<Current> spinCurrent;
    private final StatusSignal<Temperature> spinTemp;

    public IntakeIOTalonFX() {
        extendMotor = new TalonFX(IntakeConstants.EXTEND_MOTOR_ID, IntakeConstants.CAN_BUS);
        spinMotor = new TalonFX(IntakeConstants.SPIN_MOTOR_ID, IntakeConstants.CAN_BUS);
        retractLimitSwitch = new DigitalInput(IntakeConstants.RETRACT_LIMIT_SWITCH_ID);

        configureExtendMotor();
        configureSpinMotor();

        extendPosition = extendMotor.getPosition();
        extendVelocity = extendMotor.getVelocity();
        extendVoltage = extendMotor.getMotorVoltage();
        extendCurrent = extendMotor.getSupplyCurrent();
        extendTemp = extendMotor.getDeviceTemp();

        spinVelocity = spinMotor.getVelocity();
        spinVoltage = spinMotor.getMotorVoltage();
        spinCurrent = spinMotor.getSupplyCurrent();
        spinTemp = spinMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            extendPosition, extendVelocity, extendVoltage, extendCurrent, extendTemp,
            spinVelocity, spinVoltage, spinCurrent, spinTemp
        );

        extendMotor.optimizeBusUtilization();
        spinMotor.optimizeBusUtilization();
    }

    private void configureExtendMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IntakeConstants.EXTEND_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.EXTEND_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.EXTEND_STATOR_CURRENT_LIMIT;

        extendMotor.getConfigurator().apply(config);
    }

    private void configureSpinMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = IntakeConstants.SPIN_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SPIN_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.SPIN_STATOR_CURRENT_LIMIT;

        spinMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            extendPosition, extendVelocity, extendVoltage, extendCurrent, extendTemp,
            spinVelocity, spinVoltage, spinCurrent, spinTemp
        );

        inputs.extendPositionRotations = extendPosition.getValueAsDouble();
        inputs.extendVelocityRPS = extendVelocity.getValueAsDouble();
        inputs.extendAppliedVolts = extendVoltage.getValueAsDouble();
        inputs.extendCurrentAmps = extendCurrent.getValueAsDouble();
        inputs.extendTempCelsius = extendTemp.getValueAsDouble();

        inputs.spinVelocityRPS = spinVelocity.getValueAsDouble();
        inputs.spinAppliedVolts = spinVoltage.getValueAsDouble();
        inputs.spinCurrentAmps = spinCurrent.getValueAsDouble();
        inputs.spinTempCelsius = spinTemp.getValueAsDouble();

        boolean rawSwitch = retractLimitSwitch.get();
        inputs.retractLimitSwitch = IntakeConstants.RETRACT_LIMIT_SWITCH_INVERTED ? !rawSwitch : rawSwitch;
    }

    @Override
    public void setExtendPercent(double percent) {
        extendMotor.set(percent);
    }

    @Override
    public void setSpinPercent(double percent) {
        spinMotor.set(percent);
    }

    @Override
    public void resetExtendPosition() {
        extendMotor.setPosition(0.0);
    }

    @Override
    public void setSpringyCurrentLimits(boolean springy) {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        if (springy) {
            limits.SupplyCurrentLimitEnable = true;
            limits.SupplyCurrentLimit = IntakeConstants.EXTEND_SPRINGY_SUPPLY_CURRENT_LIMIT;
            limits.StatorCurrentLimitEnable = true;
            limits.StatorCurrentLimit = IntakeConstants.EXTEND_SPRINGY_STATOR_CURRENT_LIMIT;
        } else {
            limits.SupplyCurrentLimitEnable = true;
            limits.SupplyCurrentLimit = IntakeConstants.EXTEND_SUPPLY_CURRENT_LIMIT;
            limits.StatorCurrentLimitEnable = true;
            limits.StatorCurrentLimit = IntakeConstants.EXTEND_STATOR_CURRENT_LIMIT;
        }
        extendMotor.getConfigurator().apply(limits);
    }
}
