package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double lastExtendCommand = 0.0;

    private ExtendMode extendMode = ExtendMode.IDLE;
    private RollerMode rollerMode = RollerMode.INACTIVE;
    private boolean springyMode = false;

    private double oscillateTimer = 0.0;
    private boolean oscillateForward = true;
    private static final double OSCILLATE_PERIOD = 0.4; // seconds per direction

    public enum ExtendMode {
        IDLE,
        EXTENSION,
        OSCILLATING
    }

    public enum RollerMode {
        INACTIVE,
        ACTIVE
    }

    // Legacy Mode enum for backwards compatibility
    public enum Mode {
        IDLE,
        EXTENSION,
        EXTEND_ONLY,
        SPRINGY,
        OSCILLATING
    }

    public Intake(IntakeIO io) {
        this.io = io;
        io.setSpringyCurrentLimits(false);
    }

    // New state machine methods for ExtendMode
    public void setExtendMode(ExtendMode mode) {
        if (this.extendMode == mode) {
            return;
        }

        this.extendMode = mode;

        if (mode == ExtendMode.OSCILLATING) {
            oscillateTimer = 0.0;
            oscillateForward = true;
        }
    }

    public ExtendMode getExtendMode() {
        return extendMode;
    }

    // New state machine methods for RollerMode
    public void setRollerMode(RollerMode mode) {
        this.rollerMode = mode;
    }

    public RollerMode getRollerMode() {
        return rollerMode;
    }

    // Springy mode flag (affects current limits and spin speed)
    public void setSpringy(boolean springy) {
        this.springyMode = springy;
        io.setSpringyCurrentLimits(springy);
    }

    public boolean isSpringy() {
        return springyMode;
    }

    // Legacy setMode for backwards compatibility - maps to new state machines
    public void setMode(Mode mode) {
        switch (mode) {
            case IDLE:
                setExtendMode(ExtendMode.IDLE);
                setRollerMode(RollerMode.INACTIVE);
                setSpringy(false);
                break;
            case EXTENSION:
                setExtendMode(ExtendMode.EXTENSION);
                setRollerMode(RollerMode.ACTIVE);
                setSpringy(false);
                break;
            case EXTEND_ONLY:
                setExtendMode(ExtendMode.EXTENSION);
                setRollerMode(RollerMode.INACTIVE);
                setSpringy(false);
                break;
            case SPRINGY:
                setExtendMode(ExtendMode.IDLE);
                setRollerMode(RollerMode.ACTIVE);
                setSpringy(true);
                break;
            case OSCILLATING:
                setExtendMode(ExtendMode.OSCILLATING);
                setRollerMode(RollerMode.ACTIVE);
                setSpringy(false);
                break;
        }
    }

    // Legacy convenience methods - keep for backwards compatibility
    public void setExtensionMode() {
        setMode(Mode.EXTENSION);
    }

    public void setSpringyMode() {
        setMode(Mode.SPRINGY);
    }

    public void setExtendOnlyMode() {
        setMode(Mode.EXTEND_ONLY);
    }

    public void setOscillatingMode() {
        setMode(Mode.OSCILLATING);
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

    public void resetExtendPosition() {
        io.resetExtendPosition();
    }

    // Legacy getMode - derives from new state machines
    public Mode getMode() {
        if (springyMode) {
            return Mode.SPRINGY;
        }
        if (extendMode == ExtendMode.OSCILLATING) {
            return Mode.OSCILLATING;
        }
        if (extendMode == ExtendMode.EXTENSION) {
            return rollerMode == RollerMode.ACTIVE ? Mode.EXTENSION : Mode.EXTEND_ONLY;
        }
        return Mode.IDLE;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (!IntakeConstants.EXTENSION_ENABLED) {
            stopExtend();
            if (rollerMode == RollerMode.ACTIVE) {
                setSpinPercent(springyMode ? IntakeConstants.SPRINGY_SPIN_SPEED : IntakeConstants.SPIN_SPEED);
            } else {
                stopSpin();
            }
            Logger.recordOutput("Intake/ExtendMode", extendMode.toString());
            Logger.recordOutput("Intake/RollerMode", rollerMode.toString());
            Logger.recordOutput("Intake/Springy", springyMode);
            return;
        }

        if (isRetracted()) {
            io.resetExtendPosition();
        }

        if (lastExtendCommand < 0 && isRetracted()) {
            lastExtendCommand = 0.0;
            io.setExtendPercent(0.0);
        }

        // Handle extension state machine
        switch (extendMode) {
            case EXTENSION:
                setExtendPosition(IntakeConstants.EXTENDED_POSITION_ROTATIONS);
                break;
            case OSCILLATING:
                oscillateTimer += 0.02; // 20ms loop
                if (oscillateTimer >= OSCILLATE_PERIOD) {
                    oscillateTimer = 0.0;
                    oscillateForward = !oscillateForward;
                }
                double oscillateTarget = oscillateForward
                    ? IntakeConstants.EXTENDED_POSITION_ROTATIONS
                    : IntakeConstants.RETRACTED_POSITION_ROTATIONS;
                setExtendPosition(oscillateTarget);
                break;
            case IDLE:
            default:
                if (springyMode) {
                    stopExtend();
                } else {
                    setExtendPosition(IntakeConstants.RETRACTED_POSITION_ROTATIONS);
                }
                break;
        }

        // Handle roller state machine
        if (rollerMode == RollerMode.ACTIVE) {
            setSpinPercent(IntakeConstants.SPIN_SPEED);
        } else {
            stopSpin();
        }

        Logger.recordOutput("Intake/ExtendMode", extendMode.toString());
        Logger.recordOutput("Intake/RollerMode", rollerMode.toString());
        Logger.recordOutput("Intake/Springy", springyMode);
        Logger.recordOutput("Intake/ExtendCommand", lastExtendCommand);
    }
}
