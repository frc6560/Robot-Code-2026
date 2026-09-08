package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    public final double CURRENT_THRESHOLD = 20.0;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final Debouncer spikeDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

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

    public boolean isIntakingPiece() {
        return spikeDebouncer.calculate(inputs.currentAmps >= CURRENT_THRESHOLD);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake/CurrentAmps", inputs.currentAmps);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("IntakeCurrent_Direct", inputs.currentAmps);

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
