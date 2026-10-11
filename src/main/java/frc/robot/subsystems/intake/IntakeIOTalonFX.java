package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

/** Hardware implementation for one X44 rack motor and one X60 roller motor. */
public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID, IntakeConstants.CAN_BUS);
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.CAN_BUS);
    private final MotionMagicVoltage deployRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VelocityVoltage rollerRequest = new VelocityVoltage(0).withSlot(0);

    private final StatusSignal<Angle> deployPosition = deployMotor.getPosition();
    private final StatusSignal<AngularVelocity> deployVelocity = deployMotor.getVelocity();
    private final StatusSignal<Voltage> deployVoltage = deployMotor.getMotorVoltage();
    private final StatusSignal<Current> deployCurrent = deployMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> deployTemp = deployMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> rollerVelocity = rollerMotor.getVelocity();
    private final StatusSignal<Voltage> rollerVoltage = rollerMotor.getMotorVoltage();
    private final StatusSignal<Current> rollerCurrent = rollerMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> rollerTemp = rollerMotor.getDeviceTemp();

    public IntakeIOTalonFX() {
        configureDeployMotor();
        configureRollerMotor();
        // Without an absolute sensor or limit switch, the intake must boot mechanically retracted.
        deployMotor.setPosition(IntakeConstants.RETRACTED_POSITION_ROTATIONS);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            deployPosition, deployVelocity, deployVoltage, deployCurrent, deployTemp,
            rollerVelocity, rollerVoltage, rollerCurrent, rollerTemp);
        deployMotor.optimizeBusUtilization();
        rollerMotor.optimizeBusUtilization();
    }

    private void configureDeployMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IntakeConstants.DEPLOY_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
        config.Slot0.kP = IntakeConstants.DEPLOY_kP;
        config.Slot0.kV = IntakeConstants.DEPLOY_kV;
        config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.DEPLOY_CRUISE_VELOCITY_RPS;
        config.MotionMagic.MotionMagicAcceleration = IntakeConstants.DEPLOY_ACCELERATION_RPS_SQ;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        deployMotor.getConfigurator().apply(config);
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
        BaseStatusSignal.refreshAll(deployPosition, deployVelocity, deployVoltage, deployCurrent,
            deployTemp, rollerVelocity, rollerVoltage, rollerCurrent, rollerTemp);
        inputs.deployPositionRotations = deployPosition.getValueAsDouble();
        inputs.deployVelocityRPS = deployVelocity.getValueAsDouble();
        inputs.deployAppliedVolts = deployVoltage.getValueAsDouble();
        inputs.deployCurrentAmps = deployCurrent.getValueAsDouble();
        inputs.deployTempCelsius = deployTemp.getValueAsDouble();
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
    public void setDeployPosition(double motorRotations) {
        deployMotor.setControl(deployRequest.withPosition(motorRotations));
    }

    @Override
    public void stop() {
        deployMotor.stopMotor();
        rollerMotor.stopMotor();
    }

    @Override
    public void setCoastMode(boolean coast) {
        deployMotor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        rollerMotor.setNeutralMode(NeutralModeValue.Coast);
    }
}
