package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utility.StartupCurrentLimit;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.IntakeConstants;

/** Motion Magic rack positioning and two independently velocity-controlled rollers. */
public class IntakeIOTalonFX implements IntakeIO {
    private final CANBus canBus = new CANBus(IntakeConstants.CAN_BUS);
    private final CurrentLimitsConfigs runtimeCurrentLimits = deployCurrentLimits(IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT);
    private final TalonFX motor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID,
        canBus);
    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
    private final TalonFX leftRoller = new TalonFX(IntakeConstants.LEFT_ROLLER_MOTOR_ID,
        canBus);
    private final TalonFX rightRoller = new TalonFX(IntakeConstants.RIGHT_ROLLER_MOTOR_ID,
        canBus);
    private final VelocityVoltage leftRollerRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage rightRollerRequest = new VelocityVoltage(0).withSlot(0);
    private final StatusCode leftRollerConfigurationStatus;
    private final StatusCode rightRollerConfigurationStatus;
    private final StatusCode configurationStatus;
    private boolean homed;
    private boolean stopped = true;
    private boolean extensionLimitReached;
    private boolean rackFeedbackFresh;
    private double requestedRackPosition = Double.NaN;
    private final StartupCurrentLimit deployLimit;
    private boolean forceRetracting;
    private boolean forceExtending;
    private final VoltageOut forceRetractRequest = forceRetractControl();
    private final VoltageOut forceExtendRequest = forceExtendControl();

    static VoltageOut forceRetractControl() {
        return new VoltageOut(IntakeConstants.DEPLOY_FORCE_RETRACT_VOLTS).withIgnoreSoftwareLimits(true);
    }

    static VoltageOut forceExtendControl() {
        return new VoltageOut(IntakeConstants.DEPLOY_FORCE_EXTEND_VOLTS).withIgnoreSoftwareLimits(true);
    }

    private final BaseStatusSignal[] refreshGroup0;
    private final BaseStatusSignal[] refreshGroup1;
    private final BaseStatusSignal[] refreshGroup2;
    private final BaseStatusSignal[] refreshGroup3;
    public IntakeIOTalonFX() {

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
        StatusCode status = motor.getConfigurator().apply(config, 0.1);
        for (int attempt = 1; attempt < 3 && !status.isOK(); attempt++) {
            status = motor.getConfigurator().apply(config, 0.1);
        }
        configurationStatus = status;
        deployLimit = new StartupCurrentLimit(IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT,
            IntakeConstants.DEPLOY_STARTUP_STATOR_CURRENT_LIMIT, IntakeConstants.DEPLOY_STARTUP_SECONDS,
            Timer::getFPGATimestamp,
            amps -> {
                updateDeployCurrentLimits(runtimeCurrentLimits, amps);
                return motor.getConfigurator().apply(runtimeCurrentLimits,
                    IntakeConstants.DEPLOY_CURRENT_CONFIG_TIMEOUT_SECONDS).isOK();
            }, () -> {
                motor.stopMotor();
            });
        // Hardware-test assumption: rack is physically fully retracted at code startup.
        StatusCode zero = status.isOK()
            ? motor.setPosition(IntakeConstants.RETRACTED_POSITION_ROTATIONS) : status;
        homed = status.isOK() && zero.isOK();
        motor.hasResetOccurred();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, motor.getPosition(), motor.getVelocity(),
            motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getStatorCurrent());
        motor.optimizeBusUtilization();
        motor.getClosedLoopReferenceSlope().setUpdateFrequency(2.0);
        leftRollerConfigurationStatus = configureRoller(leftRoller,
            IntakeConstants.LEFT_ROLLER_MOTOR_INVERTED);
        rightRollerConfigurationStatus = configureRoller(rightRoller,
            IntakeConstants.RIGHT_ROLLER_MOTOR_INVERTED);
        refreshGroup0 = new BaseStatusSignal[] {motor.getPosition(), motor.getVelocity()};
        refreshGroup1 = new BaseStatusSignal[] {motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getStatorCurrent()};
        refreshGroup2 = new BaseStatusSignal[] {
            leftRoller.getVelocity(), rightRoller.getVelocity()};
        refreshGroup3 = new BaseStatusSignal[] {leftRoller.getMotorVoltage(), leftRoller.getSupplyCurrent(),
            rightRoller.getMotorVoltage(), rightRoller.getSupplyCurrent()};
    }

    static CurrentLimitsConfigs deployCurrentLimits(double statorAmps) {
        return updateDeployCurrentLimits(new CurrentLimitsConfigs(), statorAmps);
    }

    static CurrentLimitsConfigs updateDeployCurrentLimits(CurrentLimitsConfigs limits, double statorAmps) {
        limits.StatorCurrentLimitEnable = true;
        limits.StatorCurrentLimit = statorAmps;
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT;
        return limits;
    }

    private boolean rackNeedsMotion() {
        double position = motor.getPosition().getValueAsDouble();
        return Double.isFinite(position) && Double.isFinite(requestedRackPosition)
            && Math.abs(position - requestedRackPosition) > IntakeConstants.DEPLOY_POSITION_TOLERANCE_ROTATIONS;
    }

    static TalonFXConfiguration rollerConfiguration(boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
        config.Feedback.SensorToMechanismRatio = IntakeConstants.ROLLER_GEARING;
        config.Slot0.kP = IntakeConstants.ROLLER_kP;
        config.Slot0.kV = IntakeConstants.ROLLER_kV * IntakeConstants.ROLLER_GEARING;
        return config;
    }

    private static StatusCode configureRoller(TalonFX roller, boolean inverted) {
        TalonFXConfiguration config = rollerConfiguration(inverted);
        StatusCode status = roller.getConfigurator().apply(config, 0.1);
        for (int attempt = 1; attempt < 3 && !status.isOK(); attempt++) {
            status = roller.getConfigurator().apply(config, 0.1);
        }
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, roller.getPosition(), roller.getVelocity(),
            roller.getMotorVoltage(), roller.getSupplyCurrent());
        roller.optimizeBusUtilization();
        return status;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Only motion feedback determines connection readiness; stale temperature
        // or current telemetry must not suppress an otherwise valid position command.
        boolean fresh = BaseStatusSignal.refreshAll(refreshGroup0).isOK();
        BaseStatusSignal.refreshAll(refreshGroup1);
        if (motor.hasResetOccurred()) {
            homed = false;
        }
        rackFeedbackFresh = fresh && motor.isConnected();
        deployLimit.update(DriverStation.isEnabled() && !stopped && homed && rackFeedbackFresh,
            false, forceRetracting || forceExtending || rackNeedsMotion());
        inputs.deployStatorLimitAmps = deployLimit.appliedAmps();
        inputs.deployCurrentLimitOK = configurationStatus.isOK() && deployLimit.confirmed();
        inputs.deployConnected = rackFeedbackFresh;
        inputs.configured = configurationStatus.isOK();
        inputs.deployHomed = homed;
        inputs.deployPositionRotations = motor.getPosition().getValueAsDouble();
        inputs.deployTargetRPS = stopped || forceRetracting || forceExtending
            ? Double.NaN : motor.getClosedLoopReferenceSlope().getValueAsDouble();
        inputs.leftRollerPositionRotations = leftRoller.getPosition().getValueAsDouble();
        inputs.rightRollerPositionRotations = rightRoller.getPosition().getValueAsDouble();
        inputs.deployVelocityRPS = motor.getVelocity().getValueAsDouble();
        inputs.deployAppliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.deployCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.deployStatorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.deployCurrentFresh = motor.getStatorCurrent().getStatus().isOK();
        boolean rollersFresh = BaseStatusSignal.refreshAll(refreshGroup2).isOK();
        BaseStatusSignal.refreshAll(refreshGroup3);
        inputs.rollerConnected = rollersFresh && leftRoller.isConnected() && rightRoller.isConnected();
        inputs.rollerConfigured = leftRollerConfigurationStatus.isOK()
            && rightRollerConfigurationStatus.isOK();
        inputs.rollerVelocityRPS = leftRoller.getVelocity().getValueAsDouble();
        inputs.rightRollerVelocityRPS = rightRoller.getVelocity().getValueAsDouble();
        inputs.rollerAppliedVolts = leftRoller.getMotorVoltage().getValueAsDouble();
        inputs.rightRollerAppliedVolts = rightRoller.getMotorVoltage().getValueAsDouble();
        inputs.leftRollerSupplyAmps = leftRoller.getSupplyCurrent().getValueAsDouble();
        inputs.rightRollerSupplyAmps = rightRoller.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setRollerRPM(double rpm) {
        if (!Double.isFinite(rpm) || rpm == 0.0) {
            leftRoller.stopMotor();
            rightRoller.stopMotor();
            return;
        }
        leftRoller.setControl(leftRollerRequest.withVelocity(rpm / 60.0));
        rightRoller.setControl(rightRollerRequest.withVelocity(rpm / 60.0));
    }

    @Override
    public void setDeployPosition(double rotations) {
        if (homed && configurationStatus.isOK() && Double.isFinite(rotations)) {
            double commandedTarget = MathUtil.clamp(rotations,
                IntakeConstants.RETRACTED_POSITION_ROTATIONS, IntakeConstants.EXTENDED_POSITION_ROTATIONS);
            boolean newMove = forceRetracting || forceExtending || !Double.isFinite(requestedRackPosition)
                || requestedRackPosition != commandedTarget;
            forceRetracting = forceExtending = false;
            requestedRackPosition = commandedTarget;
            if (!deployLimit.update(DriverStation.isEnabled() && rackFeedbackFresh, newMove, rackNeedsMotion())) {
                return;
            }
            StatusCode control = motor.setControl(request.withPosition(commandedTarget));
            if (!control.isOK()) { stop(); return; }
            stopped = false;
        } else {
            stop();
        }
    }

    @Override public void setExtensionLimitReached(boolean reached) {
        extensionLimitReached = reached;
        if (reached) requestedRackPosition = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
    }

    @Override public void forceRetract() {
        forceMove(false);
    }

    @Override public void forceExtend() {
        forceMove(true);
    }

    private void forceMove(boolean extending) {
        if (!DriverStation.isEnabled() || !homed || !rackFeedbackFresh || !configurationStatus.isOK()) {
            stop();
            return;
        }
        boolean newMove = extending ? !forceExtending : !forceRetracting;
        forceExtending = extending;
        forceRetracting = !extending;
        requestedRackPosition = extending ? IntakeConstants.EXTENDED_POSITION_ROTATIONS
            : IntakeConstants.RETRACTED_POSITION_ROTATIONS;
        if (!deployLimit.update(true, newMove, true)) {
            return;
        }
        StatusCode control = motor.setControl(extending ? forceExtendRequest : forceRetractRequest);
        if (!control.isOK()) { stop(); return; }
        stopped = false;
    }

    @Override public void finishForceRetract() {
        stopDeploy();
        StatusCode zero = motor.setPosition(IntakeConstants.RETRACTED_POSITION_ROTATIONS);
        homed = configurationStatus.isOK() && zero.isOK();
    }

    @Override public void stopDeploy() {
        motor.stopMotor();
        forceRetracting = forceExtending = false;
        deployLimit.update(false, false, false);
        stopped = true;
    }

    @Override
    public void stop() {
        forceRetracting = forceExtending = false;
        requestedRackPosition = Double.NaN;
        deployLimit.update(false, false, false);
        motor.stopMotor();
        setRollerRPM(0.0);
        stopped = true;
    }

    @Override
    public void setCoastMode(boolean coast) {
        motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }
}
