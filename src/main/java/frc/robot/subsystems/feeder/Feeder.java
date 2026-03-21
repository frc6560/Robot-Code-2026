package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {
    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    private static final double PAN_RUNNING_RPM = -5.0;
    private static final double PUSHER_RUNNING_RPM = 2500.0;
    private static final double IDLE_RPM = 0.0;

    private static final double PAN_SPEED_TOLERANCE_RPM = 20.0;

    private static final double JAM_CURRENT_THRESHOLD = 45.0;
    private static final double JAM_DETECT_TIME = 0.25;
    private static final double DEJAM_REVERSE_TIME = 0.3;
    private static final double DEJAM_PAUSE_TIME = 0.15;

    public enum RevolverState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        STAGED,
        OVERRIDE,
        DEJAM_REVERSING,
        DEJAM_PAUSE
    }

    private RevolverState state = RevolverState.IDLE;

    private double jamStartTime = 0;
    private double dejamStartTime = 0;

    public Feeder(FeederIO io) {
        this.io = io;
    }

    public boolean hasBall() {
        return true;
    }

    public double getPanActualRPM() {
        return inputs.panVelocityRPS * 60.0 * FeederConstants.PAN_GEAR_RATIO;
    }

    public double getPusherActualRPM() {
        return inputs.pusherVelocityRPS * 60.0 * FeederConstants.PUSHER_GEAR_RATIO;
    }

    public boolean panAtSpeed() {
        return Math.abs(getPanActualRPM() - PAN_RUNNING_RPM) < PAN_SPEED_TOLERANCE_RPM;
    }

    public double getPusherCurrent() {
        return inputs.pusherCurrentAmps;
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

        double currentTime = Timer.getFPGATimestamp();

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
                    jamStartTime = currentTime;
                }
                break;

            case FEEDING:
                io.setPanRPM(PAN_RUNNING_RPM);
                io.setPusherRPM(PUSHER_RUNNING_RPM);

                if (getPusherCurrent() > JAM_CURRENT_THRESHOLD) {
                    if (currentTime - jamStartTime > JAM_DETECT_TIME) {
                        state = RevolverState.DEJAM_REVERSING;
                        dejamStartTime = currentTime;
                    }
                } else {
                    jamStartTime = currentTime;
                }
                break;

            case DEJAM_REVERSING:
                io.setPanRPM(-40);
                io.setPusherRPM(-500);

                if (currentTime - dejamStartTime > DEJAM_REVERSE_TIME) {
                    state = RevolverState.DEJAM_PAUSE;
                    dejamStartTime = currentTime;
                }
                break;

            case DEJAM_PAUSE:
                io.setPanRPM(0);
                io.setPusherRPM(0);

                if (currentTime - dejamStartTime > DEJAM_PAUSE_TIME) {
                    state = RevolverState.SPINNING_UP;
                }
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
        Logger.recordOutput("Feeder/PusherCurrent", getPusherCurrent());
    }
}
