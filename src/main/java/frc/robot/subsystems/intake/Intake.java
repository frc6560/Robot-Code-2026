package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** Rack requests with sustained-current physical endpoint stops and requested rollers. */
public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    private boolean pitCoastMode;
    private double target = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
    private double requestedDeployTarget = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
    private boolean rackRequested;
    private double nextTelemetry;
    private boolean rollerRequested;
    private java.util.function.BooleanSupplier teleopRollerRequest;
    private java.util.function.BooleanSupplier teleopOuttakeRequest = () -> false;
    private double rollerTargetRPM;
    private int deploySpikeCount;
    private boolean extensionLimitReached;
    private boolean forceRetractRequested, retractionLimitReached;
    private boolean forceExtendRequested;
    private double highCurrentSince = Double.NaN;

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

    /** Teleop rollers follow the physical trigger independently of rack commands. */
    public void setTeleopRollerRequest(java.util.function.BooleanSupplier request) {
        teleopRollerRequest = request;
    }

    public void setTeleopOuttakeRequest(java.util.function.BooleanSupplier request) {
        teleopOuttakeRequest = request;
    }

    public void setRollersRequested(boolean requested) {
        rollerRequested = requested;
        if (!requested) stopRollers();
    }

    public void requestForceRetracted() {
        requestRackRetracted();
        forceRetractRequested = true;
    }

    public boolean isRetractionLimitReached() { return retractionLimitReached; }

    public void requestForceExtended() {
        requestRackExtended();
        forceExtendRequested = true;
    }

    /** Driver confirms the physical endpoint without changing the encoder reference. */
    public void confirmFullyExtended() {
        if (!DriverStation.isTeleopEnabled() || pitCoastMode || !isReady()) return;
        requestedDeployTarget = target = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        rackRequested = true;
        forceExtendRequested = forceRetractRequested = retractionLimitReached = false;
        extensionLimitReached = true;
        highCurrentSince = Double.NaN;
        io.setExtensionLimitReached(true);
        io.stopDeploy();
    }

    private void stopRollers() {
        rollerRequested = false;
        rollerTargetRPM = 0.0;
        io.setRollerRPM(0.0);
    }

    private void resetExtensionLimit() {
        deploySpikeCount = 0;
        extensionLimitReached = false;
        forceExtendRequested = forceRetractRequested = retractionLimitReached = false;
        highCurrentSince = Double.NaN;
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
        if (!rackRequested || (!forceRetractRequested
                && requestedDeployTarget != IntakeConstants.EXTENDED_POSITION_ROTATIONS)
                || !isReady() || !inputs.deployCurrentFresh
                || !Double.isFinite(inputs.deployStatorCurrentAmps)) {
            highCurrentSince = Double.NaN;
            return;
        }
        double amps = Math.abs(inputs.deployStatorCurrentAmps);
        if (amps <= IntakeConstants.DEPLOY_SPIKE_AMPS) highCurrentSince = Double.NaN;
        else {
            if (!Double.isFinite(highCurrentSince)) highCurrentSince = now;
            if (now - highCurrentSince >= IntakeConstants.DEPLOY_SPIKE_SECONDS - 1e-9) {
                deploySpikeCount++;
                highCurrentSince = Double.NaN;
                if (forceRetractRequested) {
                    retractionLimitReached = true;
                    io.finishForceRetract();
                } else extensionLimitReached = true;
            }
        }
    }

    // Legacy intake/shot callers do not override the driver's rack selection.
    public boolean isActive() { return rollerTargetRPM != 0.0; }

    public State getState() {
        if (rollerTargetRPM < 0) return State.EXTENDED_OUTTAKING;
        return target > IntakeConstants.RETRACTED_POSITION_ROTATIONS
            ? isActive() ? State.EXTENDED_SPINNING : State.EXTENDED_STOPPED
            : State.RETRACTED_STOPPED;
    }

    public boolean isReady() {
        return inputs.configured && inputs.deployConnected && inputs.deployHomed;
    }

    public boolean atDeployTarget() {
        if (forceRetractRequested) return isReady() && retractionLimitReached;
        if (forceExtendRequested) return isReady() && extensionLimitReached;
        return isReady() && (target == IntakeConstants.EXTENDED_POSITION_ROTATIONS
            ? extensionLimitReached : retractionLimitReached);
    }

    public void setPitCoastMode(boolean enabled) {
        pitCoastMode = enabled;
        stop();
        io.setCoastMode(enabled);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        boolean enabled = DriverStation.isEnabled();
        if (!enabled) {
            rackRequested = false;
            rollerRequested = false;
            resetExtensionLimit();
        }
        double extensionNow = Timer.getFPGATimestamp();
        if (enabled && !pitCoastMode && !extensionLimitReached && !retractionLimitReached)
            updateExtensionLimit(extensionNow);
        io.setExtensionLimitReached(extensionLimitReached);
        target = enabled
            ? requestedDeployTarget
            : IntakeConstants.RETRACTED_POSITION_ROTATIONS;
        if (enabled && !pitCoastMode && rackRequested && isReady()) {
            if (retractionLimitReached || extensionLimitReached)
                io.stopDeploy();
            else if (forceRetractRequested) io.forceRetract();
            else if (forceExtendRequested) io.forceExtend();
            else io.setDeployPosition(target);
        } else {
            io.stopDeploy();
        }
        if (DriverStation.isTeleopEnabled() && teleopRollerRequest != null)
            rollerRequested = teleopRollerRequest.getAsBoolean();
        boolean outtaking = DriverStation.isTeleopEnabled() && teleopOuttakeRequest.getAsBoolean();
        boolean rollerPermitted = enabled && !pitCoastMode && (rollerRequested || outtaking);
        rollerTargetRPM = rollerPermitted ? (outtaking ? -IntakeConstants.ROLLER_RPM : IntakeConstants.ROLLER_RPM) : 0.0;
        io.setRollerRPM(rollerTargetRPM);
        if (edu.wpi.first.wpilibj.RobotBase.isReal() && extensionNow < nextTelemetry) return;
        nextTelemetry = extensionNow + 0.5;
        SmartDashboard.putNumber("Intake/DeployPositionRotations", inputs.deployPositionRotations);
        SmartDashboard.putNumber("Intake/DeployRPM", inputs.deployVelocityRPS * 60.0);
        SmartDashboard.putNumber("Intake/DeployTargetRPM", inputs.deployTargetRPS * 60.0);
        SmartDashboard.putNumber("Intake/DeployTargetRotations", target);
        SmartDashboard.putNumber("Intake/DeployAppliedVolts", inputs.deployAppliedVolts);
        SmartDashboard.putNumber("Intake/DeployCurrentAmps", inputs.deployCurrentAmps);
        SmartDashboard.putNumber("Intake/DeployStatorAmps", inputs.deployStatorCurrentAmps);
        SmartDashboard.putNumber("Intake/LeftRollerPositionRotations", inputs.leftRollerPositionRotations);
        SmartDashboard.putNumber("Intake/LeftRollerMotorRPM", inputs.rollerVelocityRPS * 60.0 * IntakeConstants.ROLLER_GEARING);
        SmartDashboard.putNumber("Intake/LeftRollerMotorTargetRPM", rollerTargetRPM * IntakeConstants.ROLLER_GEARING);
        SmartDashboard.putNumber("Intake/LeftRollerVolts", inputs.rollerAppliedVolts);
        SmartDashboard.putNumber("Intake/LeftRollerSupplyAmps", inputs.leftRollerSupplyAmps);
        SmartDashboard.putNumber("Intake/RightRollerPositionRotations", inputs.rightRollerPositionRotations);
        SmartDashboard.putNumber("Intake/RightRollerMotorRPM", inputs.rightRollerVelocityRPS * 60.0 * IntakeConstants.ROLLER_GEARING);
        SmartDashboard.putNumber("Intake/RightRollerMotorTargetRPM", rollerTargetRPM * IntakeConstants.ROLLER_GEARING);
        SmartDashboard.putNumber("Intake/RightRollerVolts", inputs.rightRollerAppliedVolts);
        SmartDashboard.putNumber("Intake/RightRollerSupplyAmps", inputs.rightRollerSupplyAmps);
    }
}
