package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** Hold rack position; rollers require an explicit held-trigger request and full extension. */
public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    private boolean pitCoastMode;
    private double target = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
    private double requestedDeployTarget = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
    private boolean rackRequested;
    private boolean rollerRequested;
    private double rollerTargetRPM;
    private double extendedSince = Double.NaN;
    private String lastConsoleState = "";
    private double nextConsolePrint;

    public enum State {
        RETRACTED_STOPPED, EXTENDED_STOPPED, EXTENDED_SPINNING, EXTENDED_OUTTAKING
    }

    public Intake(IntakeIO io) { this.io = io; }

    /** Select a rack endpoint; periodic retains the request until the next selection. */
    public void requestRackRetracted() {
        stopRollers();
        requestedDeployTarget = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
        rackRequested = true;
    }

    public void requestRackExtended() {
        stopRollers();
        requestedDeployTarget = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        rackRequested = true;
    }

    public void setRollersRequested(boolean requested) {
        rollerRequested = requested;
        if (!requested) stopRollers();
    }

    private void stopRollers() {
        rollerRequested = false;
        extendedSince = Double.NaN;
        rollerTargetRPM = 0.0;
        io.setRollerRPM(0.0);
    }

    // Legacy intake/shot callers do not override the driver's rack selection.
    public void activate() {}
    public void deactivate() {}
    public void activateOuttake() {}
    public void setExtendedSpinning() {}
    public void setExtendedStopped() {}
    public void setRetracted() {}
    public void startShootingOscillation() {}
    public void stopShootingOscillation() {}
    public boolean isActive() { return rollerTargetRPM != 0.0; }
    public boolean isShootingOscillationActive() { return false; }

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
        rackRequested = false;
        stopRollers();
        io.stop();
        io.setCoastMode(enabled);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        boolean enabled = DriverStation.isEnabled();
        if (!enabled) {
            rackRequested = false;
            rollerRequested = false;
        }
        target = enabled
            ? requestedDeployTarget
            : IntakeConstants.RETRACTED_POSITION_ROTATIONS;
        if (enabled && !pitCoastMode && rackRequested && isReady()) {
            io.setDeployPosition(target);
        } else {
            io.stop();
        }
        // Require the extended selection AND actual homed position; an extension request alone is insufficient.
        boolean fullyExtended = rackRequested && isReady()
            && requestedDeployTarget == IntakeConstants.EXTENDED_POSITION_ROTATIONS
            && Math.abs(inputs.deployPositionRotations - IntakeConstants.EXTENDED_POSITION_ROTATIONS)
                <= IntakeConstants.DEPLOY_POSITION_TOLERANCE_ROTATIONS;
        boolean startupPushFinished = inputs.deployCurrentLimitOK
            && inputs.deployStatorLimitAmps == IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
        boolean extensionStable = enabled && !pitCoastMode && rollerRequested && fullyExtended && startupPushFinished
            && Double.isFinite(inputs.deployVelocityRPS)
            && Math.abs(inputs.deployVelocityRPS) <= IntakeConstants.ROLLER_MAX_DEPLOY_VELOCITY_RPS
            && inputs.rollerConnected && inputs.rollerConfigured;
        double extensionNow = Timer.getFPGATimestamp();
        if (!extensionStable) extendedSince = Double.NaN;
        else if (!Double.isFinite(extendedSince)) extendedSince = extensionNow;
        boolean rollerPermitted = extensionStable
            && extensionNow - extendedSince >= IntakeConstants.ROLLER_EXTENSION_SETTLE_SECONDS;
        rollerTargetRPM = rollerPermitted ? IntakeConstants.ROLLER_RPM : 0.0;
        io.setRollerRPM(rollerTargetRPM);
        String rollerBlockedReason = !enabled ? "Robot is disabled"
            : pitCoastMode ? "Pit coast mode is on"
            : !rollerRequested ? "Right trigger not held"
            : !fullyExtended ? "Waiting for fully extended rack"
            : !inputs.rollerConnected ? "Check Canivore roller IDs 17 and 18, power and wiring"
            : !inputs.rollerConfigured ? "Roller configuration failed; correct and restart code"
            : !inputs.deployCurrentLimitOK ? "Rack current-limit update not confirmed"
            : !startupPushFinished ? "Waiting for rack startup push to end"
            : !rollerPermitted ? "Waiting for rack to settle at full extension"
            : "None";
        SmartDashboard.putBoolean("Intake/RollerTriggerHeld", rollerRequested);
        Logger.recordOutput("Intake/RollerTriggerHeld", rollerRequested);
        Logger.recordOutput("Intake/RollerCommandedRPM", rollerTargetRPM);
        Logger.recordOutput("Intake/RollerBlockedReason", rollerBlockedReason);
        SmartDashboard.putNumber("Intake/RollerRequestedRPM", rollerTargetRPM);
        SmartDashboard.putBoolean("Intake/FullyExtended", fullyExtended);
        SmartDashboard.putBoolean("Intake/RollerPermitted", rollerPermitted);
        SmartDashboard.putBoolean("Intake/StartupPushFinished", startupPushFinished);
        SmartDashboard.putBoolean("Intake/DeployStallFault", false);
        SmartDashboard.putNumber("Intake/DeployStatorLimitAmps", inputs.deployStatorLimitAmps);
        SmartDashboard.putBoolean("Intake/DeployCurrentLimitOK", inputs.deployCurrentLimitOK);
        SmartDashboard.putNumber("Intake/DeployStatorAmps", inputs.deployStatorCurrentAmps);
        SmartDashboard.putNumber("Intake/DeploySupplyAmps", inputs.deployCurrentAmps);
        SmartDashboard.putNumber("Intake/LeftRollerSupplyAmps", inputs.leftRollerSupplyAmps);
        Logger.recordOutput("Intake/LeftRollerSupplyAmps", inputs.leftRollerSupplyAmps);
        SmartDashboard.putNumber("Intake/LeftRollerStatorAmps", inputs.leftRollerStatorAmps);
        Logger.recordOutput("Intake/LeftRollerStatorAmps", inputs.leftRollerStatorAmps);
        SmartDashboard.putNumber("Intake/RightRollerSupplyAmps", inputs.rightRollerSupplyAmps);
        Logger.recordOutput("Intake/RightRollerSupplyAmps", inputs.rightRollerSupplyAmps);
        SmartDashboard.putNumber("Intake/RightRollerStatorAmps", inputs.rightRollerStatorAmps);
        Logger.recordOutput("Intake/RightRollerStatorAmps", inputs.rightRollerStatorAmps);
        SmartDashboard.putNumber("Intake/LeftRollerVolts", inputs.rollerAppliedVolts);
        SmartDashboard.putNumber("Intake/RightRollerVolts", inputs.rightRollerAppliedVolts);
        Logger.recordOutput("Intake/DeployStallFault", false);
        SmartDashboard.putNumber("Intake/RollerCommandedRPM", rollerTargetRPM);
        SmartDashboard.putString("Intake/RollerBlockedReason", rollerBlockedReason);
        SmartDashboard.putBoolean("Intake/RollerConnected", inputs.rollerConnected);
        SmartDashboard.putString("Intake/RollerConfigurationStatus", inputs.rollerConfigurationStatus);
        SmartDashboard.putString("Intake/RollerControlStatus", inputs.rollerControlStatus);
        SmartDashboard.putNumber("Intake/LeftRollerRPM", inputs.rollerVelocityRPS * 60.0);
        SmartDashboard.putNumber("Intake/RightRollerRPM", inputs.rightRollerVelocityRPS * 60.0);
        SmartDashboard.putBoolean("Intake/Ready", isReady());
        SmartDashboard.putBoolean("Intake/DeployConnected", inputs.deployConnected);
        SmartDashboard.putString("Intake/DeployConfigurationStatus", inputs.deployConfigurationStatus);
        SmartDashboard.putString("Intake/ZeroStatus", inputs.zeroStatus);
        String blockedReason = !enabled ? "Robot is disabled"
            : pitCoastMode ? "Pit coast mode is on"
            : !inputs.deployConnected ? "No rack feedback: check " + IntakeConstants.CAN_BUS
                + " CAN ID " + IntakeConstants.DEPLOY_MOTOR_ID + ", power and wiring"
            : !inputs.configured ? "Motor configuration failed: see configuration status; restart after fixing"
            : !inputs.deployHomed ? "Startup zero failed or controller reset: retract and restart code"
            : !rackRequested ? "Waiting for LT/RT rack request"
            : "None";
        String consoleState = blockedReason.equals("None")
            ? (target == IntakeConstants.RETRACTED_POSITION_ROTATIONS ? "RETRACT / HOLD" : "EXTEND / HOLD")
            : "STOP: " + blockedReason;
        double now = Timer.getFPGATimestamp();
        // Print transitions immediately, plus one heartbeat/second while enabled.
        if (!consoleState.equals(lastConsoleState) || (enabled && now >= nextConsolePrint)) {
            System.out.printf(
                "[Intake] %s | bus=%s id=%d target=%.3f rot position=%.3f rot "
                    + "volts=%.2f amps=%.2f configured=%s zero=%s lastControl=%s%n",
                consoleState, IntakeConstants.CAN_BUS, IntakeConstants.DEPLOY_MOTOR_ID,
                target, inputs.deployPositionRotations, inputs.deployAppliedVolts,
                inputs.deployCurrentAmps, inputs.deployConfigurationStatus,
                inputs.zeroStatus, inputs.deployControlStatus);
            lastConsoleState = consoleState;
            nextConsolePrint = now + 1.0;
        }
        SmartDashboard.putString("Intake/BlockedReason", blockedReason);
        SmartDashboard.putBoolean("Intake/RackRequested", rackRequested);
        SmartDashboard.putString("Intake/DeployControlStatus", inputs.deployControlStatus);
        SmartDashboard.putNumber("Intake/DeployCANID", IntakeConstants.DEPLOY_MOTOR_ID);
        SmartDashboard.putString("Intake/DeployCANBus", IntakeConstants.CAN_BUS);
        SmartDashboard.putNumber("Intake/DeployAppliedVolts", inputs.deployAppliedVolts);
        SmartDashboard.putNumber("Intake/DeployCurrentAmps", inputs.deployCurrentAmps);
        Logger.recordOutput("Intake/BlockedReason", blockedReason);
        SmartDashboard.putNumber("Intake/DeployPositionRotations", inputs.deployPositionRotations);
        SmartDashboard.putNumber("Intake/DeployTargetRotations", target);
        Logger.recordOutput("Intake/DeployTargetRotations", target);
        Logger.recordOutput("Intake/State", getState().toString());
    }
}
