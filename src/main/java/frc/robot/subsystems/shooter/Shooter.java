package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double goalRPM = 0.0;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    /**
     * Set the target flywheel speed in RPM.
     * @param rpm Desired flywheel RPM
     */
    public void setGoal(double rpm) {
        goalRPM = rpm;
    }

    public void setIdle() {
        setGoal(ShooterConstants.FLYWHEEL_IDLE_RPM);
    }

    public void stop() {
        goalRPM = 0.0;
        io.stop();
    }

    public double getGoalRPM() {
        return goalRPM;
    }

    public double getCurrentRPM() {
        return inputs.leaderVelocityRPS * ShooterConstants.FLYWHEEL_GEAR_RATIO * 60.0;
    }

    public boolean atTarget() {
        if (Math.abs(goalRPM) < 60.0) return false;
        return Math.abs(getCurrentRPM() - goalRPM) < ShooterConstants.FLYWHEEL_RPM_TOLERANCE;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (goalRPM > 0) {
            double goalRPS = goalRPM / 60.0; // mechanism RPS
            io.setVelocityRPS(goalRPS);
        }

        Logger.recordOutput("Shooter/GoalRPM", goalRPM);
        Logger.recordOutput("Shooter/CurrentRPM", getCurrentRPM());
        Logger.recordOutput("Shooter/AtTarget", atTarget());
        Logger.recordOutput("Shooter/ErrorRPM", getCurrentRPM() - goalRPM);
        Logger.recordOutput("Shooter/ControlMode", inputs.controlMode);
    }
}
