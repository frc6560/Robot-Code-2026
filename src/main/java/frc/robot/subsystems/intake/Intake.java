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
        OUTTAKE,
        CYCLING
    }

    private static final int CYCLE_TICKS_PER_DIRECTION = 500;
    private static final int CYCLE_TOTAL_TICKS = CYCLE_TICKS_PER_DIRECTION * 2;

    private State state = State.IDLE;
    private int cycleTicks = 0;

    public Intake(IntakeIO io) {
        Logger.recordOutput(getName(), "Intake class initialized");
        this.io = io;
        state=State.ACTIVE;
    }

    public void activate() {
        state = State.ACTIVE;
        cycleTicks = 0;
    }

    public void deactivate() {
        state = State.IDLE;
        cycleTicks = 0;
    }

    public void activateOuttake() {
        state = State.OUTTAKE;
        cycleTicks = 0;
    }

    public void activateCycle() {
        state = State.CYCLING;
        cycleTicks = 0;
    }

    public State getState() {
        return state;
    }

    public boolean isActive() {
        return state == State.ACTIVE;
    }

    @Override
    public void periodic() {
    //     Logger.recordOutput("LOG", "ITS RUNNING");
    //     io.updateInputs(inputs);
    //     Logger.processInputs("Intake", inputs);

    //     switch (state) {
    //         case ACTIVE:
    //             io.setRollerRPM(IntakeConstants.ROLLER_RPM);
    //             break;
    //         case OUTTAKE:
    //             io.setRollerRPM(-IntakeConstants.ROLLER_RPM);
    //             break;
    //         case CYCLING:
    //             if (cycleTicks < CYCLE_TICKS_PER_DIRECTION) {
    //                 io.setRollerRPM(IntakeConstants.ROLLER_RPM);
    //             } else {
    //                 io.setRollerRPM(-IntakeConstants.ROLLER_RPM);
    //             }
    //             cycleTicks = (cycleTicks + 1) % CYCLE_TOTAL_TICKS;
    //             break;
    //         case IDLE:
    //         default:
    //             io.stop();
    //             break;
    //     }

    //     Logger.recordOutput("Intake/State", state.toString());
    //     Logger.recordOutput("Intake/CycleTicks", cycleTicks);
    //     Logger.recordOutput("Intake/CyclingIn", state == State.CYCLING && cycleTicks < CYCLE_TICKS_PER_DIRECTION);
    }
}
