package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private ClimbState currentState = ClimbState.RETRACTED;

    public enum ClimbState {
        RETRACTED,
        EXTENDED,
        PULL_UP
    }

    public Climb(ClimbIO io) {
        this.io = io;
    }

    public void setState(ClimbState state) {
        this.currentState = state;
    }

    public ClimbState getState() {
        return currentState;
    }

    public void stop() {
        io.setPercent(0.0);
    }

    public double getPosition() {
        return inputs.leftPositionRotations;
    }

    public boolean isRetracted() {
        return inputs.retractLimitSwitch;
    }

    /**
     * Returns a command that slowly retracts the climb until the limit switch
     * triggers, then resets the encoder to zero. Times out after RESET_TIMEOUT_SECONDS
     * and sets zero at that point as a fallback if the limit switch doesn't work.
     */
    public Command resetPositionCommand() {
        return Commands.run(() -> {
            io.setPercent(ClimbConstants.RESET_RETRACT_PERCENT);
        }, this)
        .until(() -> inputs.retractLimitSwitch)
        .withTimeout(ClimbConstants.RESET_TIMEOUT_SECONDS)
        .finallyDo((interrupted) -> {
            io.setPercent(0.0);
            io.zeroPosition();
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        if (!ClimbConstants.CLIMB_ENABLED) {
            stop();
            Logger.recordOutput("Climb/Error", "Climb disabled in constants");
            return;
        }

        double targetPosition = 0.0;
        switch (currentState) {
            case EXTENDED:
                targetPosition = ClimbConstants.EXTENDED_ROTATIONS;
                io.setTarget(targetPosition);
                break;
            case PULL_UP:
                targetPosition = ClimbConstants.PULL_UP_ROTATIONS;
                io.setTarget(targetPosition);
                break;
            case RETRACTED:
            default:
                targetPosition = ClimbConstants.RETRACTED_ROTATIONS;
                io.setTarget(targetPosition);
                break;
        }

        // Calculate errors and status
        double currentPosition = getPosition();
        double positionError = Math.abs(targetPosition - currentPosition);
        boolean atTarget = positionError < 0.5; // Within 0.5 rotations

        // Log state and targets
        Logger.recordOutput("Climb/State", currentState.toString());
        Logger.recordOutput("Climb/Position", currentPosition);
        Logger.recordOutput("Climb/TargetPosition", targetPosition);
        Logger.recordOutput("Climb/PositionError", positionError);
        Logger.recordOutput("Climb/AtTarget", atTarget);

        

        
    }
}
