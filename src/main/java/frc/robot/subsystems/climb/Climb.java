package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private ClimbState currentState = ClimbState.BOTH_DOWN;

    public enum ClimbState {
        BOTH_DOWN,    // Fully retracted
        LEFT_REACH,   // Left arm goes UP, Right arm stays DOWN
        MEET_MIDDLE,  // Both arms go to MID
        RIGHT_REACH   // Right arm goes UP, Left arm stays DOWN
    }

    public Climb(ClimbIO io) {
        this.io = io;
    }

    public void setState(ClimbState state) {
        this.currentState = state;
    }

    public void stop() {
        io.setLeftPercent(0.0);
        io.setRightPercent(0.0);
    }

    
    public double getLeftPosition() { return inputs.leftPositionRotations; }
    public double getRightPosition() { return inputs.rightPositionRotations; }

    private boolean isAtTarget(double current, double target) {
        return Math.abs(current - target) < ClimbConstants.POSITION_TOLERANCE; 
    }

    public boolean isLeftExtended() { return isAtTarget(getLeftPosition(), ClimbConstants.EXTEND_ROTATIONS); }
    public boolean isRightExtended() { return isAtTarget(getRightPosition(), ClimbConstants.EXTEND_ROTATIONS); }
    public boolean areBothAtMiddle() {
        return isAtTarget(getLeftPosition(), ClimbConstants.MID_ROTATIONS) && 
               isAtTarget(getRightPosition(), ClimbConstants.MID_ROTATIONS);
    }

    
    public Command autoClimbRoutine() {
        return Commands.sequence(
            Commands.runOnce(() -> setState(ClimbState.LEFT_REACH), this),
            Commands.waitUntil(this::isLeftExtended),

            Commands.runOnce(() -> setState(ClimbState.MEET_MIDDLE), this),
            Commands.waitUntil(this::areBothAtMiddle),

            Commands.runOnce(() -> setState(ClimbState.RIGHT_REACH), this),
            Commands.waitUntil(this::isRightExtended),

            Commands.runOnce(() -> setState(ClimbState.MEET_MIDDLE), this),
            Commands.waitUntil(this::areBothAtMiddle)
        ).withName("AutoClimb");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        if (!ClimbConstants.CLIMB_ENABLED) {
            stop();
            return;
        }

        if (inputs.leftLimitSwitch) io.resetLeftPosition();
        if (inputs.rightLimitSwitch) io.resetRightPosition();

        double leftTarget = 0.0;
        double rightTarget = 0.0;

        switch (currentState) {
            case LEFT_REACH:
                leftTarget = ClimbConstants.EXTEND_ROTATIONS;
                rightTarget = ClimbConstants.RETRACTED_ROTATIONS;
                break;
            case RIGHT_REACH:
                leftTarget = ClimbConstants.RETRACTED_ROTATIONS;
                rightTarget = ClimbConstants.EXTEND_ROTATIONS;
                break;
            case MEET_MIDDLE:
                leftTarget = ClimbConstants.MID_ROTATIONS;
                rightTarget = ClimbConstants.MID_ROTATIONS;
                break;
            case BOTH_DOWN:
            default:
                leftTarget = ClimbConstants.RETRACTED_ROTATIONS;
                rightTarget = ClimbConstants.RETRACTED_ROTATIONS;
                break;
        }

        if (leftTarget <= 0.0 && inputs.leftLimitSwitch) {
            io.setLeftPercent(0.0);
        } else {
            io.setLeftTarget(leftTarget);
        }

        if (rightTarget <= 0.0 && inputs.rightLimitSwitch) {
            io.setRightPercent(0.0);
        } else {
            io.setRightTarget(rightTarget);
        }

        Logger.recordOutput("Climb/State", currentState.toString());
        Logger.recordOutput("Climb/LeftTarget", leftTarget);
        Logger.recordOutput("Climb/RightTarget", rightTarget);
    }
}