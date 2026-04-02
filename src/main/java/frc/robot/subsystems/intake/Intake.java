package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public enum State {
        IDLE,
        ACTIVE,
        OUTTAKE
    }

    private State state = State.IDLE;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void activate() {
        state = State.ACTIVE;
    }

    public void deactivate() {
        state = State.IDLE;
    }

    public void activateOuttake() {
        state = State.OUTTAKE;
    }

    public State getState() {
        return state;
    }

    public boolean isActive() {
        return state == State.ACTIVE;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        switch (state) {
            case ACTIVE:
                io.setRollerRPM(IntakeConstants.ROLLER_RPM);
                break;
            case OUTTAKE:
                io.setRollerRPM(-IntakeConstants.ROLLER_RPM);
                break;
            case IDLE:
            default:
                io.stop();
                break;
        }

        Logger.recordOutput("Intake/State", state.toString());
    }
}
