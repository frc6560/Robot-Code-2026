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
    private boolean manualControl = false;  // Flag to pause periodic control

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
     * triggers, then resets the encoder to zero. Pauses the periodic control
     * to allow direct percent control.
     */
    public Command resetPositionCommand() {
        return Commands.runOnce(() -> {
            manualControl = true;  // Pause periodic control
            System.out.println("Climb reset started - using voltage control");
            System.out.println("  Initial limit switch state: " + inputs.retractLimitSwitch);
            System.out.println("  Initial position: " + inputs.leftPositionRotations);
        }, this)
        .andThen(Commands.run(() -> {
            io.setVoltage(ClimbConstants.HOMING_VOLTS);  // Use voltage instead of percent
            Logger.recordOutput("Climb/ResetActive", true);
            Logger.recordOutput("Climb/ResetVoltage", ClimbConstants.HOMING_VOLTS);
            Logger.recordOutput("Climb/ResetLimitSwitch", inputs.retractLimitSwitch);
        }, this)
        .until(() -> {
            boolean limitHit = inputs.retractLimitSwitch;
            if (limitHit) {
                System.out.println("Limit switch triggered! Ending reset.");
            }
            return limitHit;
        })
        .withTimeout(ClimbConstants.RESET_TIMEOUT_SECONDS))
        .finallyDo((interrupted) -> {
            io.setVoltage(0.0);
            io.zeroPosition();
            manualControl = false;  // Resume periodic control
            currentState = ClimbState.RETRACTED;
            Logger.recordOutput("Climb/ResetActive", false);
            System.out.println("Climb reset finished - Interrupted: " + interrupted + " | Limit switch: " + inputs.retractLimitSwitch);
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

        // Skip automatic control if manual control is active (e.g., during reset)
        if (manualControl) {
            Logger.recordOutput("Climb/ManualControlActive", true);
            return;
        }
        Logger.recordOutput("Climb/ManualControlActive", false);

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
        Logger.recordOutput("Climb/LimitSwitch", inputs.retractLimitSwitch);

        

        
    }
}
