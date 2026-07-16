package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.BLineConstants;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.EventTrigger;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.RangedConstraint;
import frc.robot.lib.BLine.Path.Waypoint;

/** Competition autonomous paths expressed natively as BLine elements. */
public final class BLinePaths {
    private BLinePaths() {}

    private static Waypoint waypoint(double x, double y, double headingRadians) {
        return new Waypoint(x, y, Rotation2d.fromRadians(headingRadians));
    }

    private static PathConstraints constraints(double maxVelocity, double maxAcceleration) {
        return new PathConstraints()
            .setMaxVelocityMetersPerSec(maxVelocity)
            .setMaxAccelerationMetersPerSec2(maxAcceleration)
            .setMaxVelocityDegPerSec(BLineConstants.GLOBAL_MAX_ANGULAR_VELOCITY_DEG_PER_SEC)
            .setMaxAccelerationDegPerSec2(
                BLineConstants.GLOBAL_MAX_ANGULAR_ACCELERATION_DEG_PER_SEC2)
            .setEndTranslationToleranceMeters(BLineConstants.END_TRANSLATION_TOLERANCE_METERS)
            .setEndRotationToleranceDeg(BLineConstants.END_ROTATION_TOLERANCE_DEGREES);
    }

    public static Path hpTrenchToCenter() {
        return new Path(
            constraints(4.5, 3.0).setMaxVelocityMetersPerSec(
                new RangedConstraint(4.5, 0, 3), new RangedConstraint(1.5, 4, Integer.MAX_VALUE)),
            waypoint(3.70, 0.75, 0.0),
            new EventTrigger(0.01 / 1.35734, "intake"),
            waypoint(5.91, 0.75, 0.0),
            waypoint(7.77, 1.80, Math.PI / 2.0),
            waypoint(7.77, 3.20, Math.PI / 2.0));
    }

    public static Path hpTrenchToShoot() {
        return new Path(
            constraints(4.5, 3.0),
            waypoint(7.77, 3.20, Math.PI / 2.0),
            new EventTrigger(1.28852 / 1.78852, "retract"),
            waypoint(5.91, 0.75, Math.PI),
            waypoint(3.70, 0.75, Math.PI));
    }

    public static Path hpTrenchToHp() {
        return new Path(
            constraints(2.0, 7.0).setMaxVelocityDegPerSec(120.0),
            waypoint(3.70, 0.75, Math.PI),
            new EventTrigger(0.01 / 1.38447, "intake"),
            new EventTrigger(1.28447 / 1.38447, "retract"),
            waypoint(1.60, 0.75, Math.PI));
    }

    public static Path hpHpToCenter() {
        return new Path(
            constraints(4.0, 4.0).setMaxVelocityMetersPerSec(
                new RangedConstraint(4.0, 0, 3), new RangedConstraint(2.0, 4, Integer.MAX_VALUE)),
            waypoint(1.60, 0.75, Math.PI),
            new EventTrigger(1.0, "intake"),
            waypoint(6.00, 0.75, Math.PI),
            waypoint(7.5493702888, 1.8712102175, Math.PI / 2.0),
            waypoint(6.54, 3.24, Math.PI / 2.0));
    }

    public static Path depotTrenchToCenter() {
        return new Path(
            constraints(4.5, 3.0).setMaxVelocityMetersPerSec(
                new RangedConstraint(4.5, 0, 3), new RangedConstraint(1.5, 4, Integer.MAX_VALUE)),
            waypoint(3.70, 7.40, 0.0),
            new EventTrigger(0.89010 / 1.39010, "intake"),
            waypoint(6.00, 7.40, 0.0),
            waypoint(7.77, 6.26, -Math.PI / 2.0),
            waypoint(7.77, 4.60, -Math.PI / 2.0));
    }

    public static Path depotTrenchToShoot() {
        return new Path(
            constraints(4.5, 3.0),
            waypoint(7.77, 4.60, -Math.PI / 2.0),
            new EventTrigger(1.40740 / 1.90740, "retract"),
            waypoint(5.91, 7.40, Math.PI),
            waypoint(3.70, 7.40, Math.PI));
    }

    public static Path depotTrenchToDepot() {
        return new Path(
            constraints(4.5, 3.0).setMaxVelocityMetersPerSec(
                new RangedConstraint(4.5, 0, 3), new RangedConstraint(1.0, 4, Integer.MAX_VALUE)),
            waypoint(3.70, 7.40, Math.PI),
            new EventTrigger(1.12597 / 1.42597, "intake"),
            waypoint(2.20, 6.00, Math.PI),
            new EventTrigger((3.04479 - 1.42597) / (3.14479 - 1.42597), "retract"),
            waypoint(0.65, 6.00, Math.PI));
    }

    public static Path depotPullout() {
        return new Path(
            constraints(4.5, 6.0),
            waypoint(0.65, 6.00, Math.PI),
            waypoint(1.23, 6.00, Math.PI));
    }

    public static Path depotDepotToCenter() {
        return new Path(
            constraints(4.5, 4.0),
            waypoint(1.23, 6.00, Math.PI),
            waypoint(2.93, 7.50, Math.PI),
            waypoint(5.50, 7.50, Math.PI),
            new EventTrigger((2.13338 - 1.93338) / (2.68904 - 1.93338), "intake"),
            waypoint(6.75, 5.79, Math.PI),
            waypoint(6.39, 4.00, -Math.PI / 2.0));
    }

    public static Path hpFirstSwipeBack() {
        return new Path(
            constraints(4.5, 3.0),
            waypoint(7.77, 3.20, Math.PI / 2.0),
            new EventTrigger(1.28852 / 1.78852, "retract"),
            waypoint(5.91, 0.75, 0.0),
            waypoint(3.70, 0.75, 0.0));
    }

    public static Path hpSecondSwipe() {
        return new Path(
            constraints(4.5, 3.5),
            waypoint(3.70, 0.75, 0.0),
            new EventTrigger(1.09372 / 1.39372, "intake"),
            waypoint(6.50, 0.75, 0.0),
            waypoint(8.04, 1.74, Math.PI / 4.0),
            waypoint(7.67, 3.83, Math.PI),
            waypoint(6.38, 3.81, 5.0 * Math.PI / 4.0),
            waypoint(6.07, 2.53, -Math.PI / 2.0),
            new EventTrigger((5.55285 - 5.25285) / (6.71151 - 5.25285), "retract"),
            waypoint(3.70, 2.53, -Math.PI / 2.0));
    }
}
