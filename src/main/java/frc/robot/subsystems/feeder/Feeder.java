package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {
    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    public enum FeederState {
        IDLE,
        SPINNING_UP,
        FEEDING
    }

    private boolean intaking = false;
    private boolean shooting = false;
    private boolean outtaking = false;
    private FeederState state = FeederState.IDLE;

    public Feeder(FeederIO io) {
        this.io = io;
    }

    public double getPanActualRPM() {
        return inputs.panVelocityRPS * 60.0 * FeederConstants.PAN_GEAR_RATIO;
    }

    public double getPusherActualRPM() {
        return inputs.pusherVelocityRPS * 60.0 * FeederConstants.PUSHER_GEAR_RATIO;
    }

    public boolean panAtSpeed() {
        return Math.abs(getPanActualRPM() - FeederConstants.PAN_RUNNING_RPM) < FeederConstants.PAN_SPEED_TOLERANCE_RPM;
    }

    public double getPusherCurrent() {
        return inputs.pusherCurrentAmps;
    }

    public void setIntaking(boolean intaking) {
        this.intaking = intaking;
    }

    public void setShooting(boolean shooting) {
        this.shooting = shooting;
    }

    public boolean isIntaking() {
        return intaking;
    }

    public boolean isShooting() {
        return shooting;
    }

    public void setOuttaking(boolean outtaking) {
        this.outtaking = outtaking;
    }

    public boolean isOuttaking() {
        return outtaking;
    }

    public FeederState getState() {
        return state;
    }

    @Override
    public void periodic() {
        // io.updateInputs(inputs);
        // Logger.processInputs("Feeder", inputs);

        // // Floor and wall run when intaking or shooting, reverse when outtaking
        // if (outtaking) {
        //     io.setFloorRPM(-FeederConstants.FLOOR_RPM);
        // } else if (intaking || shooting) {
        //     io.setFloorRPM(FeederConstants.FLOOR_RPM);
        // } else {
        //     io.setFloorRPM(FeederConstants.IDLE_RPM);
        // }

        // // State machine for pan and pusher (spinning up before feeding)
        // if (shooting) {
        //     switch (state) {
        //         case IDLE:
        //             state = FeederState.SPINNING_UP;
        //             // Fall through to start spinning immediately
        //         case SPINNING_UP:
        //             io.setPanRPM(FeederConstants.PAN_RUNNING_RPM);
        //             io.setPusherRPM(FeederConstants.IDLE_RPM);
        //             if (panAtSpeed()) {
        //                 state = FeederState.FEEDING;
        //             }
        //             break;
        //         case FEEDING:
        //             io.setPanRPM(FeederConstants.PAN_RUNNING_RPM);
        //             io.setPusherRPM(FeederConstants.PUSHER_RUNNING_RPM);
        //             break;
        //     }
        // } else {
        //     state = FeederState.IDLE;
        //     io.setPanRPM(FeederConstants.IDLE_RPM);
        //     io.setPusherRPM(FeederConstants.IDLE_RPM);
        // }

        // Logger.recordOutput("Feeder/Intaking", intaking);
        // Logger.recordOutput("Feeder/Shooting", shooting);
        // Logger.recordOutput("Feeder/Outtaking", outtaking);
        // Logger.recordOutput("Feeder/State", state.toString());
        // Logger.recordOutput("Feeder/PanActualRPM", getPanActualRPM());
        // Logger.recordOutput("Feeder/PusherActualRPM", getPusherActualRPM());
        // Logger.recordOutput("Feeder/PanAtSpeed", panAtSpeed());
        // Logger.recordOutput("Feeder/PusherCurrent", getPusherCurrent());
    }
}
