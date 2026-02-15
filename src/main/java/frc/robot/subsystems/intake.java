package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
	private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
	private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

	private double targetExtensionInches = 0.0;
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
			intakeTab.addNumber("Extension Position (in)", this::getExtensionInches);
				intakeTab.addNumber("Extension Target (in)", () -> targetExtensionInches);
			intakeTab.addNumber("Spin RPS", () -> spinMotor.getVelocity().getValueAsDouble());
			intakeTab.addBoolean("Magnetic Switch", () -> retractLimitSwitch.get());
	}

	private void configureMotor(TalonFX motor, NeutralModeValue neutralMode, boolean inverted) {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.NeutralMode = neutralMode;
		config.MotorOutput.Inverted =
				inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		if (motor == extendMotor) {
			Slot0Configs slot0 = config.Slot0;
			slot0.kS = IntakeConstants.EXTEND_kS;
			slot0.kV = IntakeConstants.EXTEND_kV;
			slot0.kA = IntakeConstants.EXTEND_kA;
			slot0.kP = IntakeConstants.EXTEND_kP;
			slot0.kI = IntakeConstants.EXTEND_kI;
			slot0.kD = IntakeConstants.EXTEND_kD;

			MotionMagicConfigs mm = config.MotionMagic;
			mm.MotionMagicCruiseVelocity = IntakeConstants.EXTEND_MAX_VELOCITY;
			mm.MotionMagicAcceleration = IntakeConstants.EXTEND_MAX_ACCELERATION;
			mm.MotionMagicJerk = 0;

			double maxExtensionRotations = inchesToMotorRotations(IntakeConstants.MAX_EXTENSION_INCHES);
			config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
			config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxExtensionRotations;
			config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
			config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
		}
		motor.getConfigurator().apply(config);
	}

	private double inchesToMotorRotations(double inches) {
		double pinionRotations = inches / IntakeConstants.INCHES_PER_PINION_ROTATION;
		return pinionRotations * IntakeConstants.EXTEND_GEAR_RATIO;
	}

	private double motorRotationsToInches(double motorRotations) {
		double pinionRotations = motorRotations / IntakeConstants.EXTEND_GEAR_RATIO;
		return pinionRotations * IntakeConstants.INCHES_PER_PINION_ROTATION;
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
			extendMotor.set(0.0);
			return;
		}

		if (percent < 0 && isRetracted()) {
			extendMotor.set(0.0);
			return;
		}

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

		if (mode == Mode.EXTENSION
				&& getExtensionRotations() >= IntakeConstants.SPRINGY_TRIGGER_ROTATIONS) {
			setSpringyMode();
		}

		switch (mode) {
			case EXTENSION:
				targetExtensionInches = IntakeConstants.MAX_EXTENSION_INCHES;
				if (getExtensionInches() >= IntakeConstants.MAX_EXTENSION_INCHES - 2.0) {
					setSpinPercent(IntakeConstants.SPIN_SPEED);
				} else {
					stopSpin();
				}
				break;
			case SPRINGY:
				targetExtensionInches = IntakeConstants.MAX_EXTENSION_INCHES;
				if (getExtensionInches() >= IntakeConstants.MAX_EXTENSION_INCHES - 2.0) {
					setSpinPercent(IntakeConstants.SPRINGY_SPIN_SPEED);
				} else {
					stopSpin();
				}
				break;
			case IDLE:
			default:
				targetExtensionInches = 0.0;
				stopSpin();
				break;
		}

		double targetMotorRotations = inchesToMotorRotations(targetExtensionInches);
		extendMotor.setControl(motionMagicRequest.withPosition(targetMotorRotations));
	}

	public double getExtensionRotations() {
		return extendMotor.getPosition().getValueAsDouble();
	}

	public double getExtensionInches() {
		return motorRotationsToInches(getExtensionRotations());
	}
}
