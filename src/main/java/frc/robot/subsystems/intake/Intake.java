
package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double lastExtendCommand = 0.0;
    private Mode mode = Mode.IDLE;

    // Reset constants
    private static final double RESET_RETRACT_PERCENT = -0.15;
    private static final double RESET_TIMEOUT_SECONDS = 3.0;

    // Dejam constants
    private static final double JAM_CURRENT_THRESHOLD = 45.0;
    private static final double JAM_DETECT_TIME = 0.25;
    private static final double DEJAM_REVERSE_TIME = 0.3;
    private static final double DEJAM_PAUSE_TIME = 0.15;

    // Dejam state
    private boolean dejamReversing = false;
    private boolean dejamPausing = false;
    private double jamStartTime = 0;
    private double dejamStartTime = 0;

    public enum Mode {
        IDLE,
        EXTENSION,
        EXTEND_ONLY,
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

    public void setExtendOnlyMode() {
        setMode(Mode.EXTEND_ONLY);
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

    public void setExtendPosition(double rotations) {
        io.setExtendPosition(rotations);
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

    // public void resetExtendPosition() {
    //     io.resetExtendPosition();
    // }

    public Mode getMode() {
        return mode;
    }

    /**
     * Returns a command that slowly retracts the intake until the limit switch
     * triggers, then resets the encoder to zero. Times out after RESET_TIMEOUT_SECONDS
     * and sets zero at that point as a fallback if the limit switch doesn't work.
     */
    public Command resetPositionCommand() {
        return Commands.run(() -> {
            io.setExtendPercent(RESET_RETRACT_PERCENT);
        }, this)
        .until(() -> inputs.retractLimitSwitch)
        .withTimeout(RESET_TIMEOUT_SECONDS)
        .finallyDo((interrupted) -> {
            io.setExtendPercent(0.0);
            io.resetExtendPosition();
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (!IntakeConstants.EXTENSION_ENABLED) {
            double currentTime = Timer.getFPGATimestamp();
            boolean spinning = (mode == Mode.EXTENSION || mode == Mode.SPRINGY) && mode != Mode.EXTEND_ONLY;

            if (spinning) {
                stopExtend();

                // Dejam state machine
                if (dejamReversing) {
                    setSpinPercent(-IntakeConstants.SPIN_SPEED);
                    if (currentTime - dejamStartTime > DEJAM_REVERSE_TIME) {
                        dejamReversing = false;
                        dejamPausing = true;
                        dejamStartTime = currentTime;
                    }
                } else if (dejamPausing) {
                    stopSpin();
                    if (currentTime - dejamStartTime > DEJAM_PAUSE_TIME) {
                        dejamPausing = false;
                        jamStartTime = currentTime;
                    }
                } else {
                    setSpinPercent(IntakeConstants.SPIN_SPEED);

                    // Jam detection
                    if (inputs.spinCurrentAmps > JAM_CURRENT_THRESHOLD) {
                        if (currentTime - jamStartTime > JAM_DETECT_TIME) {
                            dejamReversing = true;
                            dejamStartTime = currentTime;
                        }
                    } else {
                        jamStartTime = currentTime;
                    }
                }
            } else if (mode == Mode.EXTEND_ONLY) {
                stopExtend();
                stopSpin();
                dejamReversing = false;
                dejamPausing = false;
            } else {
                stopExtend();
                stopSpin();
                dejamReversing = false;
                dejamPausing = false;
            }

            Logger.recordOutput("Intake/Mode", mode.toString());
            Logger.recordOutput("Intake/DejamReversing", dejamReversing);
            Logger.recordOutput("Intake/DejamPausing", dejamPausing);
            Logger.recordOutput("Intake/SpinCurrent", inputs.spinCurrentAmps);
            return;
        }

        if (isRetracted()) {
            io.resetExtendPosition();
        }

        if (lastExtendCommand < 0 && isRetracted()) {
            lastExtendCommand = 0.0;
            io.setExtendPercent(0.0);
        }

        if (mode == Mode.EXTENSION && getExtensionRotations() >= IntakeConstants.SPRINGY_TRIGGER_ROTATIONS) {
            setSpringyMode();
        }

        switch (mode) {
            case EXTENSION:
                setExtendPosition(IntakeConstants.EXTENDED_POSITION_ROTATIONS);
                setSpinPercent(IntakeConstants.SPIN_SPEED);
                break;
            case EXTEND_ONLY:
                setExtendPosition(IntakeConstants.EXTENDED_POSITION_ROTATIONS);
                stopSpin();
                break;
            case SPRINGY:
                stopExtend();
                setSpinPercent(IntakeConstants.SPRINGY_SPIN_SPEED);
                break;
            case IDLE:
            default:
                setExtendPosition(IntakeConstants.RETRACTED_POSITION_ROTATIONS);
                stopSpin();
                break;
        }

        Logger.recordOutput("Intake/Mode", mode.toString());
        Logger.recordOutput("Intake/ExtendCommand", lastExtendCommand);
    }
}

