package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        if (!ClimbConstants.CLIMB_ENABLED) {
            stop();
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