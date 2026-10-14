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

/** Single-motor intake on the new robot; the legacy rack commands have no hardware output. */
public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.CAN_BUS);
    private final VelocityVoltage rollerRequest = new VelocityVoltage(0).withSlot(0);

    private final StatusSignal<AngularVelocity> rollerVelocity = rollerMotor.getVelocity();
    private final StatusSignal<Voltage> rollerVoltage = rollerMotor.getMotorVoltage();
    private final StatusSignal<Current> rollerCurrent = rollerMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> rollerTemp = rollerMotor.getDeviceTemp();

    public IntakeIOTalonFX() {
        configureRollerMotor();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            rollerVelocity, rollerVoltage, rollerCurrent, rollerTemp);
        rollerMotor.optimizeBusUtilization();
    }

    private void configureRollerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = IntakeConstants.ROLLER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
        config.Slot0.kP = IntakeConstants.ROLLER_kP;
        config.Slot0.kV = IntakeConstants.ROLLER_kV;
        rollerMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(rollerVelocity, rollerVoltage, rollerCurrent, rollerTemp);
        inputs.rollerVelocityRPS = rollerVelocity.getValueAsDouble();
        inputs.rollerAppliedVolts = rollerVoltage.getValueAsDouble();
        inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerTemp.getValueAsDouble();
    }

    @Override
    public void setRollerRPM(double rpm) {
        rollerMotor.setControl(rollerRequest.withVelocity(rpm / 60.0 * IntakeConstants.ROLLER_GEARING));
    }

    @Override
    public void stop() {
        rollerMotor.stopMotor();
    }

    @Override
    public void setCoastMode(boolean coast) {
        rollerMotor.setNeutralMode(NeutralModeValue.Coast);
    }
}
