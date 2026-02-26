package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double lastExtendCommand = 0.0;
    private Mode mode = Mode.IDLE;

    public enum Mode {
        IDLE,
        EXTENSION,
        SPRINGY
    }

    public Intake(IntakeIO io) {
        this.io = io;
        io.setSpringyCurrentLimits(false);
    }

    public void setMode(Mode mode) {
        if (this.mode == mode) {
            return;
        }

        this.mode = mode;
        io.setSpringyCurrentLimits(mode == Mode.SPRINGY);
    }

    public void setExtensionMode() {
        setMode(Mode.EXTENSION);
    }

    public void setSpringyMode() {
        setMode(Mode.SPRINGY);
    }

    public void setIdleMode() {
        setMode(Mode.IDLE);
    }

    public void setExtendPercent(double percent) {
        if (!IntakeConstants.EXTENSION_ENABLED) {
            lastExtendCommand = 0.0;
            io.setExtendPercent(0.0);
            return;
        }

        if (percent < 0 && isRetracted()) {
            lastExtendCommand = 0.0;
            io.setExtendPercent(0.0);
            return;
        }

        lastExtendCommand = percent;
        io.setExtendPercent(percent);
    }

    public void stopExtend() {
        setExtendPercent(0.0);
    }

    public void setSpinPercent(double percent) {
        io.setSpinPercent(percent);
    }

    public void stopSpin() {
        io.setSpinPercent(0.0);
    }

    public void stopAll() {
        stopExtend();
        stopSpin();
    }

    public boolean isRetracted() {
        if (!IntakeConstants.EXTENSION_ENABLED) {
            return true;
        }
        return inputs.retractLimitSwitch;
    }

    public double getExtensionRotations() {
        return inputs.extendPositionRotations;
    }

    public Mode getMode() {
        return mode;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // if (!IntakeConstants.EXTENSION_ENABLED) {
        //     if (mode == Mode.EXTENSION || mode == Mode.SPRINGY) {
        //         stopExtend();
        //         setSpinPercent(IntakeConstants.SPIN_SPEED);
        //     } else {
        //         stopExtend();
        //         stopSpin();
        //     }
        //     Logger.recordOutput("Intake/Mode", mode.toString());
        //     return;
        // }

        // if (isRetracted()) {
        //     io.resetExtendPosition();
        // }

        // if (lastExtendCommand < 0 && isRetracted()) {
        //     lastExtendCommand = 0.0;
        //     io.setExtendPercent(0.0);
        // }

        // if (mode == Mode.EXTENSION && getExtensionRotations() >= IntakeConstants.SPRINGY_TRIGGER_ROTATIONS) {
        //     setSpringyMode();
        // }

        // switch (mode) {
        //     case EXTENSION:
        //         setExtendPercent(IntakeConstants.EXTEND_SPEED);
        //         setSpinPercent(IntakeConstants.SPIN_SPEED);
        //         break;
        //     case SPRINGY:
        //         stopExtend();
        //         setSpinPercent(IntakeConstants.SPRINGY_SPIN_SPEED);
        //         break;
        //     case IDLE:
        //     default:
        //         if (!isRetracted()) {
        //             setExtendPercent(IntakeConstants.RETRACT_SPEED);
        //         } else {
        //             stopExtend();
        //         }
        //         stopSpin();
        //         break;
        // }

        Logger.recordOutput("Intake/Mode", mode.toString());
        Logger.recordOutput("Intake/ExtendCommand", lastExtendCommand);
    }
}
