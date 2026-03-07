package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double goalRPM = 0.0;
    private boolean sysIdMode = false;
    private final SysIdRoutine sysIdRoutine;

    public Shooter(ShooterIO io) {
        this.io = io;

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> io.setVoltage(volts.in(Volts)),
                null,
                this
            )
        );
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

    public void setSysIdMode(boolean enabled) {
        sysIdMode = enabled;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction)
            .beforeStarting(() -> sysIdMode = true)
            .finallyDo(() -> sysIdMode = false);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction)
            .beforeStarting(() -> sysIdMode = true)
            .finallyDo(() -> sysIdMode = false);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Only run velocity control if not in SysId mode
        if (!sysIdMode) {
            if (goalRPM > 0) {
                double goalRPS = goalRPM / 60.0; // mechanism RPS
                io.setVelocityRPS(goalRPS);
            }
            else{
                io.stop();
            }
        }

        Logger.recordOutput("Shooter/GoalRPM", goalRPM);
        Logger.recordOutput("Shooter/CurrentRPM", getCurrentRPM());
        Logger.recordOutput("Shooter/AtTarget", atTarget());
        Logger.recordOutput("Shooter/ErrorRPM", getCurrentRPM() - goalRPM);
        Logger.recordOutput("Shooter/ControlMode", inputs.controlMode);
    }
}
