package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private double goalDegrees = 0.0;

    public Turret(TurretIO io) {
        this.io = io;
    }

    /**
     * Set the target angle for the turret.
     * @param goal Target angle in degrees
     */
    public void setGoal(double goal) {
        goalDegrees = MathUtil.clamp(goal, TurretConstants.LOWER_SOFT_LIMIT, TurretConstants.UPPER_SOFT_LIMIT);
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

        // Run motion magic control
        io.setTargetAngle(goalDegrees);

        Logger.recordOutput("Turret/GoalDegrees", goalDegrees);
        Logger.recordOutput("Turret/CurrentAngleDegrees", getTurretAngle());
        Logger.recordOutput("Turret/VelocityDegreesPerSec", getTurretVelocity());
        Logger.recordOutput("Turret/ErrorDegrees", getTurretAngle() - goalDegrees);
    }
}
