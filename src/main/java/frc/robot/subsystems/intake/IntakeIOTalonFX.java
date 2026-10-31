package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    private boolean stopped = true;
    private boolean extensionLimitReached;
    private boolean rackFeedbackFresh;
    private double requestedRackPosition = Double.NaN;

    public IntakeIOTalonFX() {
        frc.robot.utility.CurrentUsageMonitor.register("Intake extension", motor);
        frc.robot.utility.CurrentUsageMonitor.register("Left intake roller", leftRoller);
        frc.robot.utility.CurrentUsageMonitor.register("Right intake roller", rightRoller);

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
        // Hardware-test assumption: rack is physically fully retracted at code startup.
        StatusCode zero = status.isOK()
            ? motor.setPosition(IntakeConstants.RETRACTED_POSITION_ROTATIONS) : status;
        homed = status.isOK() && zero.isOK();
        zeroStatus = zero.toString();
        motor.hasResetOccurred();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, motor.getPosition(), motor.getVelocity(),
            motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getStatorCurrent(), motor.getDeviceTemp());
        motor.optimizeBusUtilization();
        leftRollerConfigurationStatus = configureRoller(leftRoller,
            IntakeConstants.LEFT_ROLLER_MOTOR_INVERTED);
        rightRollerConfigurationStatus = configureRoller(rightRoller,
            IntakeConstants.RIGHT_ROLLER_MOTOR_INVERTED);
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
        inputs.deployStatorLimitAmps = IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
        inputs.deployCurrentLimitOK = configurationStatus.isOK();
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
        inputs.deployCurrentFresh = motor.getStatorCurrent().getStatus().isOK();
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
        return rackAllowsRollers(homed, fresh, stopped, requestedPosition, measuredPosition,
            measuredVelocity, false);
    }

    static boolean rackAllowsRollers(boolean homed, boolean fresh, boolean stopped,
            double requestedPosition, double measuredPosition, double measuredVelocity,
            boolean extensionLimitReached) {
        return homed && fresh && (!stopped || extensionLimitReached)
            && requestedPosition == IntakeConstants.EXTENDED_POSITION_ROTATIONS
            && (extensionLimitReached || Double.isFinite(measuredPosition)
                && measuredPosition >= IntakeConstants.RETRACTED_POSITION_ROTATIONS
                    + IntakeConstants.ROLLER_MIN_EXTENSION_FRACTION
                        * (IntakeConstants.EXTENDED_POSITION_ROTATIONS - IntakeConstants.RETRACTED_POSITION_ROTATIONS));
    }

    @Override
    public void setRollerRPM(double rpm) {
        // A second interlock at the hardware boundary rejects roller commands even if a caller errs.
        boolean allowed = configurationStatus.isOK()
            && rackAllowsRollers(homed, rackFeedbackFresh, stopped, requestedRackPosition,
                motor.getPosition().getValueAsDouble(), motor.getVelocity().getValueAsDouble(), extensionLimitReached);
        if (!Double.isFinite(rpm) || rpm == 0.0 || !allowed || !leftRollerConfigurationStatus.isOK()
                || !rightRollerConfigurationStatus.isOK()) {
            leftRoller.stopMotor();
            rightRoller.stopMotor();
            rollerControlStatus = rpm == 0 ? "Stopped"
                : "Blocked: rack not confirmed extended / motor configuration";
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
            requestedRackPosition = commandedTarget;
            if (commandedTarget != IntakeConstants.EXTENDED_POSITION_ROTATIONS) setRollerRPM(0);
            StatusCode control = motor.setControl(request.withPosition(commandedTarget));
            controlStatus = control.toString();
            if (!control.isOK()) { stop(); return; }
            stopped = false;
        } else {
            stop();
        }
    }

    @Override public void setExtensionLimitReached(boolean reached) { extensionLimitReached = reached; }

    @Override public void stopDeploy() {
        motor.stopMotor();
        stopped = true;
        controlStatus = "Stopped at second extension current spike";
    }

    @Override
    public void stop() {
        requestedRackPosition = Double.NaN;
        motor.stopMotor();
        setRollerRPM(0.0);
        stopped = true;
    }

    @Override
    public void setCoastMode(boolean coast) {
        motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }
}
