package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private double goalDegrees = 0.0;
    private double goalVelocityDegreesPerSec = 0.0;
    private boolean useVelocityFeedforward = false;

    public Turret(TurretIO io) {
        this.io = io;
    }

    /**
     * Set the target angle for the turret (uses Motion Magic).
     * @param goal Target angle in degrees
     */
    public void setGoal(double goal) {
        goalDegrees = MathUtil.clamp(goal, TurretConstants.LOWER_SOFT_LIMIT, TurretConstants.UPPER_SOFT_LIMIT);
        goalVelocityDegreesPerSec = 0.0;
        useVelocityFeedforward = false;
    }

    /**
     * Set the target angle with velocity feedforward for tracking moving targets.
     * Uses PositionVoltage with velocity feedforward instead of Motion Magic.
     * @param goal Target angle in degrees
     * @param velocityDegreesPerSec Velocity feedforward in degrees per second
     */
    public void setGoalWithVelocity(double goal, double velocityDegreesPerSec) {
        goalDegrees = MathUtil.clamp(goal, TurretConstants.LOWER_SOFT_LIMIT, TurretConstants.UPPER_SOFT_LIMIT);
        goalVelocityDegreesPerSec = velocityDegreesPerSec;
        useVelocityFeedforward = true;
    }

    public double getGoalDegrees() {
        return goalDegrees;
    }

    /** Gets the turret angle in degrees */
    public double getTurretAngle() {
        return inputs.turretAngleDegrees;
    }

    /** Gets the turret velocity in degrees per second */
    public double getTurretVelocity() {
        return inputs.turretVelocityDegreesPerSec;
    }

    public double getAbsoluteEncoderRotations() {
        return inputs.absoluteEncoderPositionRotations;
    }

    public void stopMotor() {
        io.stop();
    }

    public void reseedEncoder() {
        io.seedMotorEncoder();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        // Run control based on whether velocity feedforward is being used
        if (useVelocityFeedforward) {
            io.setTargetAngleWithVelocity(goalDegrees, goalVelocityDegreesPerSec);
        } else {
            io.setTargetAngle(goalDegrees);
        }

        Logger.recordOutput("Turret/GoalDegrees", goalDegrees);
        Logger.recordOutput("Turret/GoalVelocityDegreesPerSec", goalVelocityDegreesPerSec);
        Logger.recordOutput("Turret/UsingVelocityFF", useVelocityFeedforward);
        Logger.recordOutput("Turret/CurrentAngleDegrees", getTurretAngle());
        Logger.recordOutput("Turret/VelocityDegreesPerSec", getTurretVelocity());
        Logger.recordOutput("Turret/ErrorDegrees", getTurretAngle() - goalDegrees);
    }
}
