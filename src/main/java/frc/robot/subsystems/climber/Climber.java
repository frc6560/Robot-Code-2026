package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private boolean manualControl = false; 
    private ClimbState currentState = ClimbState.RETRACTED;

    public enum ClimbState {
        RETRACTED,
        EXTENDED,
        PULL_UP
    }

    public Climber(ClimberIO io) {
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
        return Commands.runOnce(() -> {
            manualControl = true;  // Pause periodic control
            io.setSoftLimits(false);
        }, this)
        .andThen(Commands.run(() -> {
            io.setVoltage(ClimbConstants.HOMING_VOLTS);
        }, this)
        .until(() -> inputs.retractLimitSwitch)
        .withTimeout(ClimbConstants.RESET_TIMEOUT_SECONDS)
        .andThen(Commands.runOnce(() -> {
            io.setVoltage(0.0);
            io.zeroPosition();
            System.out.println("Limit switch hit - zeroed encoder, backing off...");
        }))
        .andThen(Commands.waitSeconds(0.1))  // Brief pause
        .andThen(Commands.runOnce(() -> {
            manualControl = false;  // Resume periodic control for position control
            currentState = ClimbState.RETRACTED;  // This will command it to RETRACTED_ROTATIONS (0.05)
        }))
        .andThen(Commands.waitSeconds(0.3))  // Wait for it to back off
        .finallyDo((interrupted) -> {
            io.setSoftLimits(true);  // Re-enable soft limits
        }));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        if (!ClimbConstants.CLIMB_ENABLED) {
            stop();
            return;
        }

        if (manualControl) {
            return;
        }

        switch (currentState) {
            case EXTENDED:
                io.setTarget(ClimbConstants.EXTENDED_ROTATIONS);
                break;
            case PULL_UP:
                io.setTarget(ClimbConstants.PULL_UP_ROTATIONS);
                break;
            case RETRACTED:
            default:
                io.setTarget(ClimbConstants.RETRACTED_ROTATIONS);
                break;
        }

        Logger.recordOutput("Climb/State", currentState.toString());
        Logger.recordOutput("Climb/Position", getPosition());
    }
}