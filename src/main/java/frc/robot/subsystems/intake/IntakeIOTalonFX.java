package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Voltage> leftVoltage;
    private final StatusSignal<Current> leftCurrent;
    private final StatusSignal<Temperature> leftTemp;

    private final StatusSignal<AngularVelocity> rightVelocity;
    private final StatusSignal<Voltage> rightVoltage;
    private final StatusSignal<Current> rightCurrent;
    private final StatusSignal<Temperature> rightTemp;

    public IntakeIOTalonFX() {
        leftMotor = new TalonFX(IntakeConstants.LEFT_MOTOR_ID, IntakeConstants.CAN_BUS);
        rightMotor = new TalonFX(IntakeConstants.RIGHT_MOTOR_ID, IntakeConstants.CAN_BUS);

        configureMotor(leftMotor, IntakeConstants.LEFT_MOTOR_INVERTED);
        configureMotor(rightMotor, IntakeConstants.RIGHT_MOTOR_INVERTED);

        leftVelocity = leftMotor.getVelocity();
        leftVoltage = leftMotor.getMotorVoltage();
        leftCurrent = leftMotor.getSupplyCurrent();
        leftTemp = leftMotor.getDeviceTemp();

        rightVelocity = rightMotor.getVelocity();
        rightVoltage = rightMotor.getMotorVoltage();
        rightCurrent = rightMotor.getSupplyCurrent();
        rightTemp = rightMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            leftVelocity, leftVoltage, leftCurrent, leftTemp,
            rightVelocity, rightVoltage, rightCurrent, rightTemp
        );

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    private void configureMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;

        config.Slot0.kP = IntakeConstants.kP;
        config.Slot0.kV = IntakeConstants.kV;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leftVelocity, leftVoltage, leftCurrent, leftTemp,
            rightVelocity, rightVoltage, rightCurrent, rightTemp
        );

        inputs.leftVelocityRPS = leftVelocity.getValueAsDouble();
        inputs.leftAppliedVolts = leftVoltage.getValueAsDouble();
        inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTemp.getValueAsDouble();

        inputs.rightVelocityRPS = rightVelocity.getValueAsDouble();
        inputs.rightAppliedVolts = rightVoltage.getValueAsDouble();
        inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTemp.getValueAsDouble();
        inputs.currentAmps = Math.max(inputs.leftCurrentAmps, inputs.rightCurrentAmps);
    }

    @Override
    public void setRollerRPM(double rpm) {
        double rps = rpm / 60.0 * IntakeConstants.ROLLER_GEARING;
        leftMotor.setControl(velocityRequest.withVelocity(rps));
        rightMotor.setControl(velocityRequest.withVelocity(rps));
    }

    @Override
    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}
