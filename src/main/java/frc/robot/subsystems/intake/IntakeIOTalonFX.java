package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.StartupCurrentConstants;
import frc.robot.utility.StartupCurrentLimit;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.IntakeConstants;

/** Motion Magic rack positioning and two independently velocity-controlled rollers. */
public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX motor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID,
        new CANBus(IntakeConstants.CAN_BUS));
    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
    private final TalonFX leftRoller = new TalonFX(IntakeConstants.LEFT_ROLLER_MOTOR_ID,
        new CANBus(IntakeConstants.CAN_BUS));
    private final TalonFX rightRoller = new TalonFX(IntakeConstants.RIGHT_ROLLER_MOTOR_ID,
        new CANBus(IntakeConstants.CAN_BUS));
    private final VelocityVoltage leftRollerRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage rightRollerRequest = new VelocityVoltage(0).withSlot(0);
    private final StatusCode leftRollerConfigurationStatus;
    private final StatusCode rightRollerConfigurationStatus;
    private String rollerControlStatus = "No request";
    private final StatusCode configurationStatus;
    private boolean homed;
    private String zeroStatus;
    private String controlStatus = "No request";
    private double lastPrintedTarget = Double.NaN;
    private String lastPrintedControlStatus = "";
    private boolean stopped = true;
    private boolean rackFeedbackFresh;
    private double requestedRackPosition = Double.NaN;
    private final StartupCurrentLimit deployLimit;

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
            StartupCurrentConstants.PEAK_AMPS, StartupCurrentConstants.DURATION_SECONDS,
            Timer::getFPGATimestamp,
            amps -> motor.getConfigurator().apply(deployCurrentLimits(amps),
                StartupCurrentConstants.CONFIG_TIMEOUT_SECONDS).isOK(), () -> {
                motor.stopMotor();
                leftRoller.stopMotor();
                rightRoller.stopMotor();
            });
        // Hardware-test assumption: rack is physically fully retracted at code startup.
        StatusCode zero = status.isOK()
            ? motor.setPosition(IntakeConstants.RETRACTED_POSITION_ROTATIONS) : status;
        homed = status.isOK() && zero.isOK();
        zeroStatus = zero.toString();
        System.out.printf("[Intake motor] STARTUP bus=%s id=%d configuration=%s zero=%s homed=%s%n",
            IntakeConstants.CAN_BUS, IntakeConstants.DEPLOY_MOTOR_ID,
            configurationStatus, zeroStatus, homed);
        motor.hasResetOccurred();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, motor.getPosition(), motor.getVelocity(),
            motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getStatorCurrent(), motor.getDeviceTemp());
        motor.optimizeBusUtilization();
        leftRollerConfigurationStatus = configureRoller(leftRoller,
            IntakeConstants.LEFT_ROLLER_MOTOR_INVERTED);
        rightRollerConfigurationStatus = configureRoller(rightRoller,
            IntakeConstants.RIGHT_ROLLER_MOTOR_INVERTED);
    }

    static CurrentLimitsConfigs deployCurrentLimits(double statorAmps) {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
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
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, roller.getVelocity(),
            roller.getMotorVoltage(), roller.getSupplyCurrent(), roller.getStatorCurrent(), roller.getDeviceTemp());
        roller.optimizeBusUtilization();
        return status;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Only motion feedback determines connection readiness; stale temperature
        // or current telemetry must not suppress an otherwise valid position command.
        boolean fresh = BaseStatusSignal.refreshAll(motor.getPosition(), motor.getVelocity()).isOK();
        BaseStatusSignal.refreshAll(motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getStatorCurrent(), motor.getDeviceTemp());
        if (motor.hasResetOccurred()) {
            homed = false;
            zeroStatus = "Rack controller reset: physically retract and restart code";
        }
        rackFeedbackFresh = fresh && motor.isConnected();
        deployLimit.update(DriverStation.isEnabled() && !stopped && homed && rackFeedbackFresh,
            false, rackNeedsMotion());
        inputs.deployStatorLimitAmps = deployLimit.appliedAmps();
        inputs.deployCurrentLimitOK = deployLimit.confirmed();
        inputs.deployConnected = rackFeedbackFresh;
        inputs.configured = configurationStatus.isOK();
        inputs.deployHomed = homed;
        inputs.deployConfigurationStatus = configurationStatus.toString();
        inputs.zeroStatus = zeroStatus;
        inputs.deployControlStatus = controlStatus;
        inputs.deployPositionRotations = motor.getPosition().getValueAsDouble();
        inputs.deployVelocityRPS = motor.getVelocity().getValueAsDouble();
        inputs.deployAppliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.deployCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.deployStatorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.deployTempCelsius = motor.getDeviceTemp().getValueAsDouble();
        boolean rollersFresh = BaseStatusSignal.refreshAll(
            leftRoller.getVelocity(), rightRoller.getVelocity()).isOK();
        BaseStatusSignal.refreshAll(leftRoller.getMotorVoltage(), leftRoller.getSupplyCurrent(), leftRoller.getStatorCurrent(),
            leftRoller.getDeviceTemp(), rightRoller.getMotorVoltage(), rightRoller.getSupplyCurrent(), rightRoller.getStatorCurrent(),
            rightRoller.getDeviceTemp());
        inputs.rollerConnected = rollersFresh && leftRoller.isConnected() && rightRoller.isConnected();
        inputs.rollerConfigured = leftRollerConfigurationStatus.isOK()
            && rightRollerConfigurationStatus.isOK();
        inputs.rollerConfigurationStatus = "Left=" + leftRollerConfigurationStatus
            + " Right=" + rightRollerConfigurationStatus;
        inputs.rollerControlStatus = rollerControlStatus;
        inputs.rollerVelocityRPS = leftRoller.getVelocity().getValueAsDouble();
        inputs.rightRollerVelocityRPS = rightRoller.getVelocity().getValueAsDouble();
        inputs.rollerAppliedVolts = leftRoller.getMotorVoltage().getValueAsDouble();
        inputs.rightRollerAppliedVolts = rightRoller.getMotorVoltage().getValueAsDouble();
        inputs.leftRollerSupplyAmps = leftRoller.getSupplyCurrent().getValueAsDouble();
        inputs.rightRollerSupplyAmps = rightRoller.getSupplyCurrent().getValueAsDouble();
        inputs.leftRollerStatorAmps = leftRoller.getStatorCurrent().getValueAsDouble();
        inputs.rightRollerStatorAmps = rightRoller.getStatorCurrent().getValueAsDouble();
        inputs.rollerCurrentAmps = inputs.leftRollerSupplyAmps + inputs.rightRollerSupplyAmps;
        inputs.rollerTempCelsius = Math.max(leftRoller.getDeviceTemp().getValueAsDouble(),
            rightRoller.getDeviceTemp().getValueAsDouble());
    }

    static boolean rackAllowsRollers(boolean homed, boolean fresh, boolean stopped,
            double requestedPosition, double measuredPosition, double measuredVelocity) {
        return homed && fresh && !stopped
            && requestedPosition == IntakeConstants.EXTENDED_POSITION_ROTATIONS
            && Double.isFinite(measuredPosition) && Double.isFinite(measuredVelocity)
            && Math.abs(measuredPosition - IntakeConstants.EXTENDED_POSITION_ROTATIONS)
                <= IntakeConstants.DEPLOY_POSITION_TOLERANCE_ROTATIONS
            && Math.abs(measuredVelocity) <= IntakeConstants.ROLLER_MAX_DEPLOY_VELOCITY_RPS;
    }

    @Override
    public void setRollerRPM(double rpm) {
        // A second interlock at the hardware boundary rejects roller commands even if a caller errs.
        boolean allowed = configurationStatus.isOK() && deployLimit.confirmed()
            && deployLimit.appliedAmps() == IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT
            && rackAllowsRollers(homed, rackFeedbackFresh, stopped, requestedRackPosition,
                motor.getPosition().getValueAsDouble(), motor.getVelocity().getValueAsDouble());
        if (!Double.isFinite(rpm) || rpm == 0.0 || !allowed || !leftRollerConfigurationStatus.isOK()
                || !rightRollerConfigurationStatus.isOK()) {
            leftRoller.stopMotor();
            rightRoller.stopMotor();
            rollerControlStatus = rpm == 0 ? "Stopped"
                : "Blocked: rack not confirmed extended / startup push active / motor configuration";
            return;
        }
        StatusCode left = leftRoller.setControl(leftRollerRequest.withVelocity(rpm / 60.0));
        StatusCode right = rightRoller.setControl(rightRollerRequest.withVelocity(rpm / 60.0));
        rollerControlStatus = "Left=" + left + " Right=" + right;
        if (!left.isOK() || !right.isOK()) {
            leftRoller.stopMotor();
            rightRoller.stopMotor();
        }
    }

    @Override
    public void setDeployPosition(double rotations) {
        if (homed && configurationStatus.isOK() && Double.isFinite(rotations)) {
            double commandedTarget = MathUtil.clamp(rotations,
                IntakeConstants.RETRACTED_POSITION_ROTATIONS, IntakeConstants.EXTENDED_POSITION_ROTATIONS);
            boolean newMove = stopped || requestedRackPosition != commandedTarget;
            requestedRackPosition = commandedTarget;
            if (!deployLimit.update(DriverStation.isEnabled() && rackFeedbackFresh, newMove, rackNeedsMotion())) {
                controlStatus = "Current-limit update failed; motor stopped";
                return;
            }
            if (commandedTarget != IntakeConstants.EXTENDED_POSITION_ROTATIONS) setRollerRPM(0);
            StatusCode control = motor.setControl(request.withPosition(commandedTarget));
            controlStatus = control.toString();
            if (!control.isOK()) { stop(); return; }
            if (stopped || commandedTarget != lastPrintedTarget
                    || !controlStatus.equals(lastPrintedControlStatus)) {
                System.out.printf("[Intake motor] SENT MotionMagic position=%.3f rot bus=%s id=%d setControl=%s%n",
                    commandedTarget, IntakeConstants.CAN_BUS, IntakeConstants.DEPLOY_MOTOR_ID, controlStatus);
                lastPrintedTarget = commandedTarget;
                lastPrintedControlStatus = controlStatus;
            }
            stopped = false;
        } else {
            stop();
        }
    }

    @Override
    public void stop() {
        requestedRackPosition = Double.NaN;
        deployLimit.update(false, false, false);
        motor.stopMotor();
        setRollerRPM(0.0);
        if (!stopped) {
            System.out.printf("[Intake motor] STOP sent bus=%s id=%d%n",
                IntakeConstants.CAN_BUS, IntakeConstants.DEPLOY_MOTOR_ID);
        }
        stopped = true;
    }

    @Override
    public void setCoastMode(boolean coast) {
        motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }
}
