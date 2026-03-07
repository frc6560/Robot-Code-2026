package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private ClimbState currentState = ClimbState.RETRACTED;
    private final Timer homingTimer = new Timer();
    private boolean isHoming = false;

    public enum ClimbState {
        RETRACTED, 
        EXTENDED,  
        PULL_UP,
        HOMING
    }

    public Climb(ClimbIO io) {
        this.io = io;
    }

    public void setState(ClimbState state) {
        if (state == ClimbState.HOMING && currentState != ClimbState.HOMING) {
            isHoming = false;
        }
        this.currentState = state;
    }

    public void stop() {
        io.setPercent(0.0);
    }

    public double getPosition() { return inputs.leftPositionRotations; }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        if (!ClimbConstants.CLIMB_ENABLED) {
            stop();
            return;
        }

        double target = 0.0;

        switch (currentState) {
            case HOMING:
                if (!isHoming) {
                    homingTimer.restart();
                    isHoming = true;
                }
                
                io.setVoltage(ClimbConstants.HOMING_VOLTS);

                // Zero if current spikes OR if the timer hits 2 seconds
                if (inputs.currentAmps[0] >= ClimbConstants.HOMING_CURRENT_AMPS || homingTimer.hasElapsed(ClimbConstants.HOMING_TIMEOUT_SECS)) {
                    io.zeroPosition();
                    setState(ClimbState.RETRACTED); 
                    homingTimer.stop();
                    isHoming = false;
                }
                return; 

            case EXTENDED:
                target = ClimbConstants.EXTENDED_ROTATIONS;
                break;
            case PULL_UP:
                target = ClimbConstants.PULL_UP_ROTATIONS;
                break;
            case RETRACTED:
            default:
                target = ClimbConstants.RETRACTED_ROTATIONS;
                break;
        }

        io.setTarget(target);

        Logger.recordOutput("Climb/State", currentState.toString());
        Logger.recordOutput("Climb/Target", target);
        Logger.recordOutput("Climb/HomingTimer", homingTimer.get());
    }
}