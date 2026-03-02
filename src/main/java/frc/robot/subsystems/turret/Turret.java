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
     * Wraps an angle to the range [-180, 180).
     */
    private double wrapAngle(double angle) {
        angle = angle % 360;
        if (angle >= 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    /**
     * Given a wrapped -180 to 180 angle, calculates a desired unwrapped angle to move to, based upon the turret's current location.
     *
     * @param targetFieldAngle The desired field angle (will be wrapped to -180..180)
     * @return The optimal physical position within soft limits
     */
    private double calculateOptimalPosition(double targetFieldAngle) {
        double currentPosition = getTurretAngle();
        double wrappedTarget = wrapAngle(targetFieldAngle);

        // Find all valid physical positions that achieve this field angle
        // Candidates are: wrappedTarget, wrappedTarget + 360, wrappedTarget - 360
        double[] candidates = {
            wrappedTarget - 360,
            wrappedTarget,
            wrappedTarget + 360
        };

        // Filter to only valid positions within soft limits
        double bestPosition = wrappedTarget; // default fallback
        double bestScore = Double.MAX_VALUE;
        boolean foundValid = false;

        boolean needsWireProtection = currentPosition <= TurretConstants.WIRE_PROTECTION_LOWER
            || currentPosition >= TurretConstants.WIRE_PROTECTION_UPPER;

        for (double candidate : candidates) {
            if (candidate >= TurretConstants.LOWER_SOFT_LIMIT &&
                candidate <= TurretConstants.UPPER_SOFT_LIMIT) {

                double score;
                if (needsWireProtection) {
                    score = Math.abs(candidate);
                } else {
                    score = Math.abs(candidate - currentPosition);
                }

                if (score < bestScore) {
                    bestScore = score;
                    bestPosition = candidate;
                    foundValid = true;
                }
            }
        }

        // Clamp as final safety (should already be within limits if foundValid)
        return MathUtil.clamp(bestPosition, TurretConstants.LOWER_SOFT_LIMIT, TurretConstants.UPPER_SOFT_LIMIT);
    }

    /**
     * Set the target angle for the turret (uses Motion Magic).
     * Automatically calculates shortest path with wire protection.
     * @param goal Target field angle in degrees (will be wrapped and optimized)
     */
    public void setGoal(double goal) {
        goalDegrees = calculateOptimalPosition(goal);
        goalVelocityDegreesPerSec = 0.0;
        useVelocityFeedforward = false;
    }

    /**
     * Set the target angle with velocity feedforward for tracking moving targets.
     * Uses PositionVoltage with velocity feedforward instead of Motion Magic.
     * Automatically calculates shortest path with wire protection.
     * @param goal Target field angle in degrees (will be wrapped and optimized)
     * @param velocityDegreesPerSec Velocity feedforward in degrees per second
     */
    public void setGoalWithVelocity(double goal, double velocityDegreesPerSec) {
        goalDegrees = calculateOptimalPosition(goal);
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

    public boolean getAtTarget(){
        return Math.abs(getTurretAngle() - goalDegrees) < 2.5; // 1.5 degree tolerance
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
        Logger.recordOutput("Turret/AtTarget", getAtTarget());
        Logger.recordOutput("Turret/WireProtectionActive",
            getTurretAngle() <= TurretConstants.WIRE_PROTECTION_LOWER
            || getTurretAngle() >= TurretConstants.WIRE_PROTECTION_UPPER);
    }
}
