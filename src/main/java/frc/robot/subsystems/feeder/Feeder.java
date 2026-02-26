package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    private static final double PAN_RUNNING_RPM = 3000.0;
    private static final double PUSHER_RUNNING_RPM = 3500.0;
    private static final double IDLE_RPM = 0.0;

    private static final double PAN_GEAR_RATIO = 324/2688;
    private static final double PUSHER_GEAR_RATIO = 1.0 / 2.5;

    private static final double PAN_SPEED_TOLERANCE_RPM = 20.0;

    public enum RevolverState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        STAGED,
        OVERRIDE
    }

    private RevolverState state = RevolverState.IDLE;

    public Feeder(FeederIO io) {
        this.io = io;
    }

    public boolean hasBall() {
        return true; // Always assume ball is present (no beam break sensor)
    }

    public double getPanActualRPM() {
        return inputs.panVelocityRPS * 60.0 * PAN_GEAR_RATIO;
    }

    public double getPusherActualRPM() {
        return inputs.pusherVelocityRPS * 60.0 * PUSHER_GEAR_RATIO;
    }

    public boolean panAtSpeed() {
        return Math.abs(getPanActualRPM() - PAN_RUNNING_RPM) < PAN_SPEED_TOLERANCE_RPM;
    }

    public void requestFeed() {
        if (state == RevolverState.IDLE) {
            state = RevolverState.SPINNING_UP;
        }
    }

    public void requestStop() {
        state = RevolverState.IDLE;
    }

    public void requestOverride() {
        state = RevolverState.OVERRIDE;
    }

    public RevolverState getState() {
        return state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        switch (state) {
            case IDLE:
                io.setPanRPM(IDLE_RPM);
                io.setPusherRPM(IDLE_RPM);
                break;

            case SPINNING_UP:
                io.setPanRPM(PAN_RUNNING_RPM);
                io.setPusherRPM(IDLE_RPM);
                if (panAtSpeed()) {
                    state = RevolverState.FEEDING;
                }
                break;

            case FEEDING:
                io.setPanRPM(PAN_RUNNING_RPM);
                io.setPusherRPM(PUSHER_RUNNING_RPM);
                break;

            case STAGED:
                io.setPanRPM(PAN_RUNNING_RPM);
                io.setPusherRPM(IDLE_RPM);
                break;

            case OVERRIDE:
                io.setPanRPM(PAN_RUNNING_RPM);
                io.setPusherRPM(PUSHER_RUNNING_RPM);
                break;
        }

        Logger.recordOutput("Feeder/State", state.toString());
        Logger.recordOutput("Feeder/PanActualRPM", getPanActualRPM());
        Logger.recordOutput("Feeder/PusherActualRPM", getPusherActualRPM());
        Logger.recordOutput("Feeder/PanAtSpeed", panAtSpeed());
    }
}
