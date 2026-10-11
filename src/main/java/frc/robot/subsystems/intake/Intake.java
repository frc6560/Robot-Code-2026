package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public enum State {
        RETRACTED_STOPPED,
        EXTENDED_STOPPED,
        EXTENDED_SPINNING
    }

    private State state = State.RETRACTED_STOPPED;
    private boolean shootingOscillation = false;
    private double oscillationStartSeconds = 0.0;
    private boolean pitCoastMode = false;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void activate() {
        setExtendedSpinning();
    }

    public void deactivate() {
        setRetracted();
    }

    public void setExtendedSpinning() {
        setOperatingState(State.EXTENDED_SPINNING);
    }

    public void setExtendedStopped() {
        setOperatingState(State.EXTENDED_STOPPED);
    }

    public void setRetracted() {
        setOperatingState(State.RETRACTED_STOPPED);
    }

    private void setOperatingState(State newState) {
        state = newState;
    }

    /** Temporarily oscillates the rack for shooting, then restores the previous intake state. */
    public void startShootingOscillation() {
        if (!shootingOscillation) {
            oscillationStartSeconds = Timer.getFPGATimestamp();
            shootingOscillation = true;
        }
    }

    public void stopShootingOscillation() {
        if (shootingOscillation) {
            shootingOscillation = false;
        }
    }

    public State getState() {
        return state;
    }

    public boolean isActive() {
        return state == State.EXTENDED_SPINNING;
    }

    public boolean isShootingOscillationActive() {
        return shootingOscillation;
    }

    public boolean atDeployTarget() {
        return Math.abs(inputs.deployPositionRotations - getDeployTargetRotations())
            <= IntakeConstants.DEPLOY_POSITION_TOLERANCE_ROTATIONS;
    }

    public void setPitCoastMode(boolean enabled) {
        pitCoastMode = enabled;
        shootingOscillation = false;
        state = State.RETRACTED_STOPPED;
        io.stop();
        io.setCoastMode(enabled);
    }

    private double getDeployTargetRotations() {
        if (shootingOscillation) {
            long halfPeriods = (long) ((Timer.getFPGATimestamp() - oscillationStartSeconds)
                / IntakeConstants.SHOOT_OSCILLATION_HALF_PERIOD_SECONDS);
            return halfPeriods % 2 == 0
                ? IntakeConstants.SHOOT_OSCILLATION_FORWARD_POSITION_ROTATIONS
                : IntakeConstants.SHOOT_OSCILLATION_REAR_POSITION_ROTATIONS;
        }

        return state == State.RETRACTED_STOPPED
            ? IntakeConstants.RETRACTED_POSITION_ROTATIONS
            : IntakeConstants.EXTENDED_POSITION_ROTATIONS;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        Logger.recordOutput("Intake/PitCoastMode", pitCoastMode);
        if (pitCoastMode) {
            io.stop();
            return;
        }

        double deployTargetRotations = getDeployTargetRotations();
        io.setDeployPosition(deployTargetRotations);
        io.setRollerRPM(!shootingOscillation && state == State.EXTENDED_SPINNING
            ? IntakeConstants.ROLLER_RPM
            : 0.0);

        Logger.recordOutput("Intake/State", state.toString());
        Logger.recordOutput("Intake/ShootingOscillation", shootingOscillation);
        Logger.recordOutput("Intake/DeployTargetRotations", deployTargetRotations);
        Logger.recordOutput("Intake/AtDeployTarget", atDeployTarget());
    }
}
