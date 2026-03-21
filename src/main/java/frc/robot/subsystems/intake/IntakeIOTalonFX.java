package frc.robot.subsystems.intake;

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
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX extendIntakeMotor;
    private final TalonFX spinMotor;

    private final StatusSignal<Angle> extendIntakePosition;
    private final StatusSignal<AngularVelocity> extendIntakeVelocity;
    private final StatusSignal<Voltage> extendIntakeVoltage;
    private final StatusSignal<Current> extendIntakeCurrent;
    private final StatusSignal<Temperature> extendIntakeTemp;

    private final StatusSignal<AngularVelocity> spinVelocity;
    private final StatusSignal<Voltage> spinVoltage;
    private final StatusSignal<Current> spinCurrent;
    private final StatusSignal<Temperature> spinTemp;

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

    public IntakeIOTalonFX() {
        extendIntakeMotor = new TalonFX(IntakeConstants.EXTEND_INTAKE_MOTOR_ID, IntakeConstants.CAN_BUS);
        spinMotor = new TalonFX(IntakeConstants.SPIN_MOTOR_ID, IntakeConstants.CAN_BUS);

        configureExtendIntakeMotor();
        configureSpinMotor();

        extendIntakePosition = extendIntakeMotor.getPosition();
        extendIntakeVelocity = extendIntakeMotor.getVelocity();
        extendIntakeVoltage = extendIntakeMotor.getMotorVoltage();
        extendIntakeCurrent = extendIntakeMotor.getSupplyCurrent();
        extendIntakeTemp = extendIntakeMotor.getDeviceTemp();

        spinVelocity = spinMotor.getVelocity();
        spinVoltage = spinMotor.getMotorVoltage();
        spinCurrent = spinMotor.getSupplyCurrent();
        spinTemp = spinMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            extendIntakePosition, extendIntakeVelocity, extendIntakeVoltage, extendIntakeCurrent, extendIntakeTemp,
            spinVelocity, spinVoltage, spinCurrent, spinTemp
        );

        extendIntakeMotor.optimizeBusUtilization();
        spinMotor.optimizeBusUtilization();
    }

    private void configureExtendIntakeMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IntakeConstants.EXTEND_INTAKE_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.EXTEND_INTAKE_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.EXTEND_INTAKE_STATOR_CURRENT_LIMIT;

        config.Slot0.kS = IntakeConstants.EXTEND_INTAKE_kS;
        config.Slot0.kV = IntakeConstants.EXTEND_INTAKE_kV;
        config.Slot0.kA = IntakeConstants.EXTEND_INTAKE_kA;
        config.Slot0.kP = IntakeConstants.EXTEND_INTAKE_kP;
        config.Slot0.kI = IntakeConstants.EXTEND_INTAKE_kI;
        config.Slot0.kD = IntakeConstants.EXTEND_INTAKE_kD;

        // Motion Magic is only used for the fixed latch-release quarter turn.
        config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.EXTEND_INTAKE_MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = IntakeConstants.EXTEND_INTAKE_MAX_ACCELERATION;

        extendIntakeMotor.getConfigurator().apply(config);
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
            extendIntakePosition, extendIntakeVelocity, extendIntakeVoltage, extendIntakeCurrent, extendIntakeTemp,
            spinVelocity, spinVoltage, spinCurrent, spinTemp
        );

        inputs.extendIntakePositionRotations = extendIntakePosition.getValueAsDouble();
        inputs.extendIntakeVelocityRPS = extendIntakeVelocity.getValueAsDouble();
        inputs.extendIntakeAppliedVolts = extendIntakeVoltage.getValueAsDouble();
        inputs.extendIntakeCurrentAmps = extendIntakeCurrent.getValueAsDouble();
        inputs.extendIntakeTempCelsius = extendIntakeTemp.getValueAsDouble();

        inputs.spinVelocityRPS = spinVelocity.getValueAsDouble();
        inputs.spinAppliedVolts = spinVoltage.getValueAsDouble();
        inputs.spinCurrentAmps = spinCurrent.getValueAsDouble();
        inputs.spinTempCelsius = spinTemp.getValueAsDouble();
    }

    @Override
    public void setExtendIntakePercent(double percent) {
        extendIntakeMotor.set(percent);
    }

    @Override
    public void setExtendIntakePosition(double rotations) {
        extendIntakeMotor.setControl(positionControl.withPosition(rotations));
    }

    @Override
    public void stopExtendIntake() {
        extendIntakeMotor.set(0.0);
    }

    @Override
    public void setSpinPercent(double percent) {
        spinMotor.set(percent);
    }

    @Override
    public void resetExtendIntakePosition() {
        // Treat the latched start position as zero so the release target stays mechanical-relative.
        extendIntakeMotor.setPosition(0.0);
    }
}
