package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class intake extends SubsystemBase {

	private final TalonFX extendMotor = new TalonFX(IntakeConstants.EXTEND_MOTOR_ID, IntakeConstants.CAN_BUS);
	private final TalonFX spinMotor = new TalonFX(IntakeConstants.SPIN_MOTOR_ID, IntakeConstants.CAN_BUS);
	private final DigitalInput retractLimitSwitch =
			new DigitalInput(IntakeConstants.RETRACT_LIMIT_SWITCH_ID);
	private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

	private double lastExtendCommand = 0.0;
	private Mode mode = Mode.IDLE;

	public enum Mode {
		IDLE,
		EXTENSION,
		SPRINGY
	}

		public intake() {
		configureMotor(extendMotor, NeutralModeValue.Brake, IntakeConstants.EXTEND_MOTOR_INVERTED);
		configureMotor(spinMotor, NeutralModeValue.Coast, IntakeConstants.SPIN_MOTOR_INVERTED);
		applyExtendCurrentLimits(false);
		applySpinCurrentLimits();
			intakeTab.addNumber("Extension Rotations", this::getExtensionRotations);
			intakeTab.addNumber("Spin RPS", () -> spinMotor.getVelocity().getValueAsDouble());
			intakeTab.addBoolean("Magnetic Switch", () -> retractLimitSwitch.get());
	}

	private void configureMotor(TalonFX motor, NeutralModeValue neutralMode, boolean inverted) {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.NeutralMode = neutralMode;
		config.MotorOutput.Inverted =
				inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		motor.getConfigurator().apply(config);
	}

	private void applyExtendCurrentLimits(boolean springy) {
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

	private void applySpinCurrentLimits() {
		CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
		limits.SupplyCurrentLimitEnable = true;
		limits.SupplyCurrentLimit = IntakeConstants.SPIN_SUPPLY_CURRENT_LIMIT;
		limits.StatorCurrentLimitEnable = true;
		limits.StatorCurrentLimit = IntakeConstants.SPIN_STATOR_CURRENT_LIMIT;
		spinMotor.getConfigurator().apply(limits);
	}

	public void setMode(Mode mode) {
		if (this.mode == mode) {
			return;
		}

		this.mode = mode;
		applyExtendCurrentLimits(mode == Mode.SPRINGY);
	}

	public void setExtensionMode() {
		setMode(Mode.EXTENSION);
	}

	public void setSpringyMode() {
		setMode(Mode.SPRINGY);
	}

	public void setIdleMode() {
		setMode(Mode.IDLE);
	}

	public void setExtendPercent(double percent) {
		if (!IntakeConstants.EXTENSION_ENABLED) {
			lastExtendCommand = 0.0;
			extendMotor.set(0.0);
			return;
		}

		if (percent < 0 && isRetracted()) {
			lastExtendCommand = 0.0;
			extendMotor.set(0.0);
			return;
		}

		lastExtendCommand = percent;
		extendMotor.set(percent);
	}

	public void stopExtend() {
		setExtendPercent(0.0);
	}

	public void setSpinPercent(double percent) {
		spinMotor.set(percent);
	}

	public void stopSpin() {
		spinMotor.set(0.0);
	}

	public void stopAll() {
		stopExtend();
		stopSpin();
	}

	public boolean isRetracted() {
		if (!IntakeConstants.EXTENSION_ENABLED) {
			return true;
		}

		boolean raw = retractLimitSwitch.get();
		return IntakeConstants.RETRACT_LIMIT_SWITCH_INVERTED ? !raw : raw;
	}

	@Override
	public void periodic() {
		if (!IntakeConstants.EXTENSION_ENABLED) {
			if (mode == Mode.EXTENSION || mode == Mode.SPRINGY) {
				stopExtend();
				setSpinPercent(IntakeConstants.SPIN_SPEED);
			} else {
				stopExtend();
				stopSpin();
			}
			return;
		}

		if (isRetracted()) {
			extendMotor.setPosition(0.0);
		}

		if (getExtensionRotations() >= IntakeConstants.MAX_EXTENSION_ROTATIONS) {
			stopExtend();
		}

		if (lastExtendCommand < 0 && isRetracted()) {
			lastExtendCommand = 0.0;
			extendMotor.set(0.0);
		}

		if (mode == Mode.EXTENSION
				&& getExtensionRotations() >= IntakeConstants.SPRINGY_TRIGGER_ROTATIONS) {
			setSpringyMode();
		}

		switch (mode) {
			case EXTENSION:
				if (getExtensionRotations() < IntakeConstants.MAX_EXTENSION_ROTATIONS) {
					setExtendPercent(IntakeConstants.EXTEND_SPEED);
				} else {
					stopExtend();
				}
				setSpinPercent(IntakeConstants.SPIN_SPEED);
				break;
			case SPRINGY:
				stopExtend();
				setSpinPercent(IntakeConstants.SPRINGY_SPIN_SPEED);
				break;
			case IDLE:
			default:
				if (!isRetracted()) {
					setExtendPercent(IntakeConstants.RETRACT_SPEED);
				} else {
					stopExtend();
				}
				stopSpin();
				break;
		}
	}

	public double getExtensionRotations() {
		return extendMotor.getPosition().getValueAsDouble();
	}
}
