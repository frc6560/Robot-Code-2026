package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private ExtendMode extendMode = ExtendMode.IDLE;
    private RollerMode rollerMode = RollerMode.INACTIVE;
    private boolean releaseRequested = false;
    private boolean releaseComplete = false;

    public enum ExtendMode {
        IDLE,
        EXTENSION,
        OSCILLATING
    }

    public enum RollerMode {
        INACTIVE,
        ACTIVE
    }

    public enum Mode {
        IDLE,
        EXTENSION,
        EXTEND_ONLY,
        SPRINGY,
        OSCILLATING
    }

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void setExtendMode(ExtendMode mode) {
        this.extendMode = mode;
        if (mode == ExtendMode.IDLE) {
            setRollerMode(RollerMode.INACTIVE);
            return;
        }

        // Any active intake mode should ensure the spring-loaded intake has been unlatched first.
        requestRelease();
        setRollerMode(RollerMode.ACTIVE);
    }

    public ExtendMode getExtendMode() {
        return extendMode;
    }

    public void setRollerMode(RollerMode mode) {
        this.rollerMode = mode;
    }

    public RollerMode getRollerMode() {
        return rollerMode;
    }

    public void requestRelease() {
        if (!releaseComplete) {
            releaseRequested = true;
        }
    }

    public boolean isReleaseComplete() {
        return releaseComplete;
    }

    public void setMode(Mode mode) {
        switch (mode) {
            case IDLE:
                setExtendMode(ExtendMode.IDLE);
                break;
            case EXTENSION:
                setExtendMode(ExtendMode.EXTENSION);
                break;
            case SPRINGY:
                setExtendMode(ExtendMode.EXTENSION);
                break;
            case OSCILLATING:
                setExtendMode(ExtendMode.OSCILLATING);
                break;
            case EXTEND_ONLY:
                extendMode = ExtendMode.EXTENSION;
                requestRelease();
                setRollerMode(RollerMode.INACTIVE);
                break;
        }
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

    public void setOscillatingMode() {
        setMode(Mode.OSCILLATING);
    }

    public void setIdleMode() {
        setMode(Mode.IDLE);
    }

    public void setExtendPercent(double percent) {
        io.setExtendIntakePercent(percent);
    }

    public void stopExtend() {
        io.stopExtendIntake();
    }

    public void setExtendPosition(double rotations) {
        io.setExtendIntakePosition(rotations);
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

    public double getExtensionRotations() {
        return inputs.extendIntakePositionRotations;
    }

    public void resetExtendPosition() {
        io.resetExtendIntakePosition();
        // Reset keeps the release routine testable between disable/enable cycles.
        releaseRequested = false;
        releaseComplete = false;
        extendMode = ExtendMode.IDLE;
    }

    public Mode getMode() {
        if (extendMode == ExtendMode.EXTENSION) {
            return rollerMode == RollerMode.ACTIVE ? Mode.EXTENSION : Mode.EXTEND_ONLY;
        }
        if (extendMode == ExtendMode.OSCILLATING) {
            return Mode.OSCILLATING;
        }
        return Mode.IDLE;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (releaseRequested && !releaseComplete) {
            // Hold the motor at the latch-pull position until the commanded quarter turn is reached.
            io.setExtendIntakePosition(IntakeConstants.EXTEND_INTAKE_RELEASE_ROTATIONS);
            if (Math.abs(inputs.extendIntakePositionRotations - IntakeConstants.EXTEND_INTAKE_RELEASE_ROTATIONS)
                    <= IntakeConstants.EXTEND_INTAKE_RELEASE_TOLERANCE_ROTATIONS) {
                releaseComplete = true;
                releaseRequested = false;
                io.stopExtendIntake();
            }
        } else {
            io.stopExtendIntake();
        }

        if (rollerMode == RollerMode.ACTIVE) {
            setSpinPercent(IntakeConstants.SPIN_SPEED);
        } else {
            stopSpin();
        }

        Logger.recordOutput("Intake/ExtendMode", extendMode.toString());
        Logger.recordOutput("Intake/RollerMode", rollerMode.toString());
        Logger.recordOutput("Intake/ReleaseRequested", releaseRequested);
        Logger.recordOutput("Intake/ReleaseComplete", releaseComplete);
        Logger.recordOutput("Intake/ExtendIntakePositionRotations", inputs.extendIntakePositionRotations);
    }
}
