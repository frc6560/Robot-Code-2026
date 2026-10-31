package frc.robot.subsystems.intake;

import frc.robot.utility.telemetry.RateLimitedTelemetry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** Requested intake motion with a 40% roller gate and a latched second-current-spike stop. */
public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    private boolean pitCoastMode;
    private double target = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
    private double requestedDeployTarget = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
    private boolean rackRequested;
    private double nextTelemetry;
    private boolean rollerRequested;
    private double rollerTargetRPM;
    private int deploySpikeCount;
    private boolean spikeArmed = true;
    private boolean extensionLimitReached;
    private double highCurrentSince = Double.NaN, lowCurrentSince = Double.NaN;

    public enum State {
        RETRACTED_STOPPED, EXTENDED_STOPPED, EXTENDED_SPINNING, EXTENDED_OUTTAKING
    }

    public Intake(IntakeIO io) { this.io = io; }

    /** Select a rack endpoint; periodic retains the request until the next selection. */
    public void requestRackRetracted() {
        stopRollers();
        resetExtensionLimit();
        requestedDeployTarget = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
        rackRequested = true;
    }

    public void requestRackExtended() {
        stopRollers();
        resetExtensionLimit();
        requestedDeployTarget = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        rackRequested = true;
    }

    public void setRollersRequested(boolean requested) {
        rollerRequested = requested;
        if (!requested) stopRollers();
    }

    private void stopRollers() {
        rollerRequested = false;
        rollerTargetRPM = 0.0;
        io.setRollerRPM(0.0);
    }

    private void resetExtensionLimit() {
        deploySpikeCount = 0;
        spikeArmed = true;
        extensionLimitReached = false;
        highCurrentSince = lowCurrentSince = Double.NaN;
        io.setExtensionLimitReached(false);
    }

    /** End of auto/pit mode: stop both mechanisms and clear persistent requests. */
    public void stop() {
        rackRequested = false;
        stopRollers();
        resetExtensionLimit();
        io.stop();
    }

    public boolean isExtensionLimitReached() { return extensionLimitReached; }
    public int getDeploySpikeCount() { return deploySpikeCount; }

    private void updateExtensionLimit(double now) {
        if (!rackRequested || requestedDeployTarget != IntakeConstants.EXTENDED_POSITION_ROTATIONS
                || !isReady() || !inputs.deployCurrentFresh
                || !Double.isFinite(inputs.deployStatorCurrentAmps)) {
            highCurrentSince = lowCurrentSince = Double.NaN;
            return;
        }
        double amps = Math.abs(inputs.deployStatorCurrentAmps);
        if (spikeArmed) {
            if (amps < IntakeConstants.DEPLOY_SPIKE_AMPS) highCurrentSince = Double.NaN;
            else {
                if (!Double.isFinite(highCurrentSince)) highCurrentSince = now;
                if (now - highCurrentSince >= IntakeConstants.DEPLOY_SPIKE_SECONDS - 1e-9) {
                    deploySpikeCount++;
                    spikeArmed = false;
                    highCurrentSince = Double.NaN;
                    if (deploySpikeCount >= 2) extensionLimitReached = true;
                }
            }
        } else {
            if (amps >= IntakeConstants.DEPLOY_SPIKE_RESET_AMPS) lowCurrentSince = Double.NaN;
            else {
                if (!Double.isFinite(lowCurrentSince)) lowCurrentSince = now;
                if (now - lowCurrentSince >= IntakeConstants.DEPLOY_SPIKE_RESET_SECONDS - 1e-9) {
                    spikeArmed = true;
                    lowCurrentSince = Double.NaN;
                }
            }
        }
    }

    // Legacy intake/shot callers do not override the driver's rack selection.
    public boolean isActive() { return rollerTargetRPM != 0.0; }

    public State getState() {
        return target > IntakeConstants.RETRACTED_POSITION_ROTATIONS
            ? isActive() ? State.EXTENDED_SPINNING : State.EXTENDED_STOPPED
            : State.RETRACTED_STOPPED;
    }

    public boolean isReady() {
        return inputs.configured && inputs.deployConnected && inputs.deployHomed;
    }

    public boolean atDeployTarget() {
        return isReady() && Math.abs(inputs.deployPositionRotations - target)
            <= IntakeConstants.DEPLOY_POSITION_TOLERANCE_ROTATIONS;
    }

    public void setPitCoastMode(boolean enabled) {
        pitCoastMode = enabled;
        stop();
        io.setCoastMode(enabled);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        RateLimitedTelemetry.processInputs("Intake", inputs);
        boolean enabled = DriverStation.isEnabled();
        if (!enabled) {
            rackRequested = false;
            rollerRequested = false;
            resetExtensionLimit();
        }
        double extensionNow = Timer.getFPGATimestamp();
        if (enabled && !pitCoastMode && !extensionLimitReached) updateExtensionLimit(extensionNow);
        io.setExtensionLimitReached(extensionLimitReached);
        target = enabled
            ? requestedDeployTarget
            : IntakeConstants.RETRACTED_POSITION_ROTATIONS;
        if (enabled && !pitCoastMode && rackRequested && isReady()) {
            if (extensionLimitReached && requestedDeployTarget == IntakeConstants.EXTENDED_POSITION_ROTATIONS)
                io.stopDeploy();
            else io.setDeployPosition(target);
        } else {
            io.stop();
        }
        boolean extending = rackRequested && isReady()
            && requestedDeployTarget == IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        boolean fullyExtended = extending && (extensionLimitReached
            || Math.abs(inputs.deployPositionRotations - IntakeConstants.EXTENDED_POSITION_ROTATIONS)
                <= IntakeConstants.DEPLOY_POSITION_TOLERANCE_ROTATIONS);
        boolean enoughExtension = extending && (extensionLimitReached
            || Double.isFinite(inputs.deployPositionRotations)
                && inputs.deployPositionRotations >= IntakeConstants.RETRACTED_POSITION_ROTATIONS
                    + IntakeConstants.ROLLER_MIN_EXTENSION_FRACTION
                        * (IntakeConstants.EXTENDED_POSITION_ROTATIONS - IntakeConstants.RETRACTED_POSITION_ROTATIONS));
        boolean continuousLimit = inputs.deployCurrentLimitOK
            && inputs.deployStatorLimitAmps <= IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
        boolean rollerPermitted = enabled && !pitCoastMode && rollerRequested && enoughExtension
            && continuousLimit && inputs.rollerConnected && inputs.rollerConfigured;
        rollerTargetRPM = rollerPermitted ? IntakeConstants.ROLLER_RPM : 0.0;
        io.setRollerRPM(rollerTargetRPM);
        String rollerBlockedReason = !enabled ? "Robot is disabled"
            : pitCoastMode ? "Pit coast mode is on"
            : !rollerRequested ? "Right trigger not held"
            : !enoughExtension ? "Waiting for 40% rack extension"
            : !inputs.rollerConnected ? "Check Canivore roller IDs " + IntakeConstants.LEFT_ROLLER_MOTOR_ID
                + " and " + IntakeConstants.RIGHT_ROLLER_MOTOR_ID + ", power and wiring"
            : !inputs.rollerConfigured ? "Roller configuration failed; correct and restart code"
            : !inputs.deployCurrentLimitOK ? "Rack current-limit update not confirmed"
            : !continuousLimit ? "Rack continuous current limit not confirmed"
            : "None";
        // Outputs/interlocks above remain at control-loop frequency.
        if (edu.wpi.first.wpilibj.RobotBase.isReal() && extensionNow < nextTelemetry) return;
        nextTelemetry = extensionNow + 0.5;
        RateLimitedTelemetry.putBoolean("Intake/RollerTriggerHeld", rollerRequested);
        RateLimitedTelemetry.recordOutput("Intake/RollerTriggerHeld", rollerRequested);
        RateLimitedTelemetry.recordOutput("Intake/RollerCommandedRPM", rollerTargetRPM);
        RateLimitedTelemetry.recordOutput("Intake/RollerBlockedReason", rollerBlockedReason);
        RateLimitedTelemetry.putNumber("Intake/RollerRequestedRPM", rollerTargetRPM);
        RateLimitedTelemetry.putBoolean("Intake/FullyExtended", fullyExtended);
        RateLimitedTelemetry.putBoolean("Intake/RollerPermitted", rollerPermitted);
        RateLimitedTelemetry.putBoolean("Intake/DeployStallFault", extensionLimitReached);
        RateLimitedTelemetry.putNumber("Intake/DeploySpikeCount", deploySpikeCount);
        RateLimitedTelemetry.putBoolean("Intake/ExtensionLimitReached", extensionLimitReached);
        RateLimitedTelemetry.putNumber("Intake/DeployStatorLimitAmps", inputs.deployStatorLimitAmps);
        RateLimitedTelemetry.putBoolean("Intake/DeployCurrentLimitOK", inputs.deployCurrentLimitOK);
        RateLimitedTelemetry.putNumber("Intake/DeployStatorAmps", inputs.deployStatorCurrentAmps);
        RateLimitedTelemetry.putNumber("Intake/DeploySupplyAmps", inputs.deployCurrentAmps);
        RateLimitedTelemetry.putNumber("Intake/LeftRollerSupplyAmps", inputs.leftRollerSupplyAmps);
        RateLimitedTelemetry.recordOutput("Intake/LeftRollerSupplyAmps", inputs.leftRollerSupplyAmps);
        RateLimitedTelemetry.putNumber("Intake/LeftRollerStatorAmps", inputs.leftRollerStatorAmps);
        RateLimitedTelemetry.recordOutput("Intake/LeftRollerStatorAmps", inputs.leftRollerStatorAmps);
        RateLimitedTelemetry.putNumber("Intake/RightRollerSupplyAmps", inputs.rightRollerSupplyAmps);
        RateLimitedTelemetry.recordOutput("Intake/RightRollerSupplyAmps", inputs.rightRollerSupplyAmps);
        RateLimitedTelemetry.putNumber("Intake/RightRollerStatorAmps", inputs.rightRollerStatorAmps);
        RateLimitedTelemetry.recordOutput("Intake/RightRollerStatorAmps", inputs.rightRollerStatorAmps);
        RateLimitedTelemetry.putNumber("Intake/LeftRollerVolts", inputs.rollerAppliedVolts);
        RateLimitedTelemetry.putNumber("Intake/RightRollerVolts", inputs.rightRollerAppliedVolts);
        RateLimitedTelemetry.recordOutput("Intake/DeployStallFault", extensionLimitReached);
        RateLimitedTelemetry.putNumber("Intake/RollerCommandedRPM", rollerTargetRPM);
        RateLimitedTelemetry.putString("Intake/RollerBlockedReason", rollerBlockedReason);
        RateLimitedTelemetry.putBoolean("Intake/RollerConnected", inputs.rollerConnected);
        RateLimitedTelemetry.putString("Intake/RollerConfigurationStatus", inputs.rollerConfigurationStatus);
        RateLimitedTelemetry.putString("Intake/RollerControlStatus", inputs.rollerControlStatus);
        RateLimitedTelemetry.putNumber("Intake/LeftRollerRPM", inputs.rollerVelocityRPS * 60.0);
        RateLimitedTelemetry.putNumber("Intake/RightRollerRPM", inputs.rightRollerVelocityRPS * 60.0);
        RateLimitedTelemetry.putBoolean("Intake/Ready", isReady());
        RateLimitedTelemetry.putBoolean("Intake/DeployConnected", inputs.deployConnected);
        RateLimitedTelemetry.putString("Intake/DeployConfigurationStatus", inputs.deployConfigurationStatus);
        RateLimitedTelemetry.putString("Intake/ZeroStatus", inputs.zeroStatus);
        String blockedReason = !enabled ? "Robot is disabled"
            : pitCoastMode ? "Pit coast mode is on"
            : !inputs.deployConnected ? "No rack feedback: check " + IntakeConstants.CAN_BUS
                + " CAN ID " + IntakeConstants.DEPLOY_MOTOR_ID + ", power and wiring"
            : !inputs.configured ? "Motor configuration failed: see configuration status; restart after fixing"
            : !inputs.deployHomed ? "Startup zero failed or controller reset: retract and restart code"
            : extensionLimitReached ? "Extension stopped at second current spike"
            : !rackRequested ? "Waiting for LT/RT rack request"
            : "None";
        RateLimitedTelemetry.putString("Intake/BlockedReason", blockedReason);
        RateLimitedTelemetry.putBoolean("Intake/RackRequested", rackRequested);
        RateLimitedTelemetry.putString("Intake/DeployControlStatus", inputs.deployControlStatus);
        RateLimitedTelemetry.putNumber("Intake/DeployCANID", IntakeConstants.DEPLOY_MOTOR_ID);
        RateLimitedTelemetry.putString("Intake/DeployCANBus", IntakeConstants.CAN_BUS);
        RateLimitedTelemetry.putNumber("Intake/DeployAppliedVolts", inputs.deployAppliedVolts);
        RateLimitedTelemetry.putNumber("Intake/DeployCurrentAmps", inputs.deployCurrentAmps);
        RateLimitedTelemetry.recordOutput("Intake/BlockedReason", blockedReason);
        RateLimitedTelemetry.putNumber("Intake/DeployPositionRotations", inputs.deployPositionRotations);
        RateLimitedTelemetry.putNumber("Intake/DeployTargetRotations", target);
        RateLimitedTelemetry.recordOutput("Intake/DeployTargetRotations", target);
        RateLimitedTelemetry.recordOutput("Intake/State", getState().toString());
    }
}
