package frc.robot.utility.shooting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utility.shooting.FuelShotModel.Shot;
import frc.robot.Constants.ShotModelConstants;

/** Cached translation lead selection with a full inherited-velocity flight check before release. */
public final class MovingShotSolver {
    private MovingShotSolver() {}
    private static final int MAX_ITERATIONS = 6;
    private static final double RESIDUAL_METERS = 0.02;

    public record Solution(Shot shot, Translation2d lead, double range, boolean valid) {
        static Solution invalid() {
            return new Solution(Shot.invalid(), new Translation2d(), Double.NaN, false);
        }
    }

    public static boolean isMoving(Translation2d fieldVelocity) {
        return fieldVelocity.getNorm() > FuelShotConfig.MOVING_SHOT_MIN_SPEED_MPS;
    }

    public static Solution solve(FuelShotModel model, Translation2d toHub,
            Translation2d fieldVelocity) {
        if (!finite(toHub) || !finite(fieldVelocity)) return Solution.invalid();
        Shot estimate = model.estimate(toHub.getNorm());
        if (!estimate.valid()) return Solution.invalid();
        Translation2d lead = new Translation2d();
        double range = toHub.getNorm();
        for (int i = 0; i < MAX_ITERATIONS; i++) {
            lead = lead(fieldVelocity, estimate.flightSeconds());
            range = toHub.minus(lead).getNorm();
            estimate = model.estimate(range);
            if (!estimate.valid()) return Solution.invalid();
        }
        Shot shot = model.solve(range);
        if (!shot.valid()) return Solution.invalid();
        // Refuse a solution if the bounded iteration did not converge, including cache error.
        boolean converged = lead.getDistance(lead(fieldVelocity, shot.flightSeconds()))
            <= RESIDUAL_METERS;
        return new Solution(shot, lead, range, converged);
    }

    /** Validate measured RPM, flight time and heading against the actual predicted landing. */
    public static boolean scores(FuelShotModel model, Translation2d toHub,
            Translation2d fieldVelocity, Rotation2d measuredHeading, double rpm) {
        if (!Double.isFinite(rpm) || rpm < model.calibration().minRPM()
                || rpm > model.calibration().maxRPM()) return false;
        if (!finite(toHub) || !finite(fieldVelocity)
                || !Double.isFinite(measuredHeading.getRadians())) return false;
        var flight = model.movingFlight(rpm, measuredHeading.getRadians(),
            fieldVelocity.getX(), fieldVelocity.getY());
        if (!flight.reached()) return false;
        Translation2d target = toHub.minus(fieldVelocity.times(FuelShotConfig.MOVING_SHOT_RELEASE_DELAY_SECONDS));
        double range = target.getNorm();
        if (range < 0.75 || range > 8) return false;
        Translation2d landing = new Translation2d(flight.crossingX(), flight.crossingY());
        double risingProjection = (flight.risingX() * target.getX() + flight.risingY() * target.getY()) / range;
        double halfWidth = ShotModelConstants.HUB_OPENING_SPAN_METERS / 2
            - ShotModelConstants.BALL_DIAMETER_METERS / 2 - ShotModelConstants.HUB_RIM_MARGIN_METERS;
        return risingProjection <= range - ShotModelConstants.HUB_OPENING_SPAN_METERS / 2
                - ShotModelConstants.HUB_RIM_MARGIN_METERS
            && flight.entryDegrees() >= FuelShotConfig.MIN_ENTRY_DEGREES
            && landing.getDistance(target) <= halfWidth;
    }

    private static Translation2d lead(Translation2d velocity, double flightSeconds) {
        return velocity.times(flightSeconds + FuelShotConfig.MOVING_SHOT_RELEASE_DELAY_SECONDS);
    }

    private static boolean finite(Translation2d value) {
        return Double.isFinite(value.getX()) && Double.isFinite(value.getY());
    }
}
