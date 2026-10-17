package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

/** Single reciprocating motor; sensing stays available while the output interlock is disabled. */
public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX motor = new TalonFX(IntakeConstants.MOTOR_ID, IntakeConstants.CAN_BUS);
    private final VoltageOut request = new VoltageOut(0);
    private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
    private final StatusSignal<Temperature> temperature = motor.getDeviceTemp();
    private final StatusCode configurationStatus;

    public IntakeIOTalonFX() {
        motor.stopMotor();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IntakeConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT_AMPS;
        StatusCode result = motor.getConfigurator().apply(config, .1);
        for (int attempt = 1; attempt < 3 && !result.isOK(); attempt++) {
            result = motor.getConfigurator().apply(config, .1);
        }
        configurationStatus = result;
        BaseStatusSignal.setUpdateFrequencyForAll(50, statorCurrent, supplyCurrent, voltage, temperature);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(statorCurrent, supplyCurrent, voltage, temperature);
        inputs.motorConnected = motor.isConnected();
        inputs.motorCurrentValid = configurationStatus.isOK() && statorCurrent.getStatus().isOK()
            && statorCurrent.getTimestamp().getLatency() <= IntakeConstants.MAX_CURRENT_SAMPLE_AGE_SECONDS;
        inputs.motorStatorCurrentAmps = Math.abs(statorCurrent.getValueAsDouble());
        inputs.motorSupplyCurrentAmps = Math.abs(supplyCurrent.getValueAsDouble());
        inputs.motorAppliedVolts = voltage.getValueAsDouble();
        inputs.motorTempCelsius = temperature.getValueAsDouble();
        inputs.motorConfigurationStatus = configurationStatus.toString();
    }

    @Override
    public void setMotorVoltage(double volts) {
        // A second output interlock protects the broken intake, even if called outside Intake.
        if (!IntakeConstants.ENABLED || !Double.isFinite(volts) || volts == 0) motor.stopMotor();
        else motor.setControl(request.withOutput(volts));
    }

    @Override public void stop() { motor.stopMotor(); }
    @Override public void setCoastMode(boolean coast) {
        motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }
}
