package frc.robot.utility.Pathing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.obstacle.AStarPlanner;
import frc.robot.utility.Pathing.obstacle.ObstacleField;

import java.util.List;

/** Builds a smooth path from start to end, routing around obstacles declared in the
 *  shared {@link ObstacleField}. All obstacle logic is delegated to the field + A*; the
 *  old reef-specific circle code has been removed. */
public class PathCalculator {
    public Setpoint startPose;
    public Setpoint endPose;

    public double startFinalControlLength;
    public double waypointInitialControlLength;
    public double waypointFinalControlLength;
    public double endInitialControlLength;

    // Default kinematic limits — callers that want tuning can extend this ctor.
    private final double maxVelocity;
    private final double maxAccel;
    private final double maxOmega;
    private final double maxAlpha;
    private final double maxCentripetal;

    private final ObstacleField field;
    private final boolean blockedStraightLine;

    public PathCalculator(Setpoint currentPose, Setpoint finalPose) {
        this(currentPose, finalPose, 5.0, 4.0, 3.14, 6.28, 3.0);
    }

    public PathCalculator(Setpoint currentPose, Setpoint finalPose,
                          double maxVelocity, double maxAccel,
                          double maxOmega, double maxAlpha, double maxCentripetal) {
        this.startPose = currentPose;
        this.endPose = finalPose;
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;
        this.maxOmega = maxOmega;
        this.maxAlpha = maxAlpha;
        this.maxCentripetal = maxCentripetal;

        this.field = ObstacleField.getInstance();
        this.blockedStraightLine = field.isSegmentBlocked(
                currentPose.getTranslation(), finalPose.getTranslation());
    }

    public boolean hasObstacle() {
        return blockedStraightLine;
    }

    /** Offsets a point along a direction vector (in radians). */
    public Pose2d getPoseDirectionFrom(Pose2d pose, double magnitude, double direction) {
        double x = pose.getX() + magnitude * Math.cos(direction);
        double y = pose.getY() + magnitude * Math.sin(direction);
        return new Pose2d(x, y, Rotation2d.fromRadians(direction));
    }

    /** Two control handles around a waypoint: one looking back, one looking forward. */
    public Pose2d[] getControlPoints(Pose2d waypoint) {
        Pose2d firstControlHeading = getPoseDirectionFrom(waypoint, waypointInitialControlLength,
                waypoint.getRotation().getRadians() + Math.PI);
        Pose2d secondControlHeading = getPoseDirectionFrom(waypoint, waypointFinalControlLength,
                waypoint.getRotation().getRadians());
        return new Pose2d[] {firstControlHeading, secondControlHeading};
    }

    /** Control-handle length heuristic: 1/3 of the Euclidean distance between the two poses,
     *  clamped to a reasonable range so short hops don't get zero-length handles. */
    public double calculateControlLengths(Pose2d firstPose, Pose2d secondPose) {
        Translation2d displacement = secondPose.getTranslation().minus(firstPose.getTranslation());
        return MathUtil.clamp(displacement.getNorm() / 3.0, 0.5, 3.0);
    }

    /** Single-segment path when the straight line is clear. */
    public Path calculateDirectPath() {
        Pose2d startPoseReal = startPose.getSetpointPose();
        Pose2d endPoseReal = endPose.getSetpointPose();
        double len = calculateControlLengths(startPoseReal, endPoseReal);

        // Heading of the control handles should aim along start->end, not the robot's
        // current rotation — the robot is holonomic so rotation is independent.
        Translation2d diff = endPoseReal.getTranslation().minus(startPoseReal.getTranslation());
        double heading = Math.atan2(diff.getY(), diff.getX());

        Pose2d startControl = getPoseDirectionFrom(startPoseReal, len, heading);
        Pose2d endControl = getPoseDirectionFrom(endPoseReal, len, heading + Math.PI);

        return new Path(startPose, endPose, startControl, endControl,
                maxVelocity, maxAccel, maxOmega, maxAlpha, maxCentripetal);
    }

    /** When the straight line is blocked, plan around obstacles via A* and stitch the result
     *  into a {@link PathGroup} with a single intermediate waypoint.
     *
     *  <p>If A* produces several intermediate waypoints, we collapse to the one closest to
     *  the arc-length midpoint of the A* polyline — a pragmatic compromise that keeps the
     *  existing 2-segment {@link PathGroup} interface. Tight multi-obstacle courses that
     *  need more than one intermediate will want a multi-segment chain; defer that until a
     *  real case shows up.
     *
     *  @return a {@link PathGroup}, or {@code null} if A* cannot find a route.
     */
    public PathGroup calculatePathGroup() {
        if (!blockedStraightLine) return null;

        List<Translation2d> waypoints = new AStarPlanner(field).plan(
                startPose.getTranslation(), endPose.getTranslation());
        if (waypoints.size() < 2) return null;

        Translation2d intermediate = pickIntermediate(waypoints);
        Pose2d startPoseReal = startPose.getSetpointPose();
        Pose2d endPoseReal = endPose.getSetpointPose();

        // Waypoint heading: tangent of the A*-midpoint segment, i.e. roughly from start to
        // end around the obstacle. This keeps the Bezier handles pointing the natural way.
        Translation2d incoming = intermediate.minus(startPoseReal.getTranslation());
        Translation2d outgoing = endPoseReal.getTranslation().minus(intermediate);
        double heading = Math.atan2(
                (incoming.getY() + outgoing.getY()) * 0.5,
                (incoming.getX() + outgoing.getX()) * 0.5);

        Pose2d waypointPose = new Pose2d(intermediate, Rotation2d.fromRadians(heading));
        Setpoint waypoint = new Setpoint(
                waypointPose.getX(), waypointPose.getY(), heading, 0, 0, 0);

        this.startFinalControlLength = calculateControlLengths(startPoseReal, waypointPose);
        this.waypointInitialControlLength = this.startFinalControlLength;
        this.waypointFinalControlLength = calculateControlLengths(waypointPose, endPoseReal);
        this.endInitialControlLength = this.waypointFinalControlLength;

        // Direct heading on start/end: aim at the waypoint rather than the robot's rotation.
        double startToWayHeading = Math.atan2(incoming.getY(), incoming.getX());
        double wayToEndHeading = Math.atan2(outgoing.getY(), outgoing.getX());

        Pose2d startControlPoint = getPoseDirectionFrom(startPoseReal, startFinalControlLength, startToWayHeading);
        Pose2d endControlPoint = getPoseDirectionFrom(endPoseReal, endInitialControlLength, wayToEndHeading + Math.PI);
        Pose2d[] waypointHandles = getControlPoints(waypointPose);

        Path firstPath = new Path(
                this.startPose, waypoint,
                startControlPoint, waypointHandles[0],
                maxVelocity, maxAccel, maxOmega, maxAlpha, maxCentripetal);

        Path secondPath = new Path(
                waypoint, this.endPose,
                waypointHandles[1], endControlPoint,
                maxVelocity, maxAccel, maxOmega, maxAlpha, maxCentripetal);

        // If either segment is still clipping an obstacle, we've failed our approximation.
        // Surface it to the caller so they can fall back rather than silently run into a wall.
        if (field.isCurveBlocked(firstPath::calculatePosition, firstPath.getArcLength(), 0.10)
                || field.isCurveBlocked(secondPath::calculatePosition, secondPath.getArcLength(), 0.10)) {
            return null;
        }

        return new PathGroup(firstPath, secondPath,
                maxVelocity, maxAccel, maxOmega, maxAlpha, maxCentripetal);
    }

    private static Translation2d pickIntermediate(List<Translation2d> waypoints) {
        if (waypoints.size() == 2) {
            // No intermediate — fall back to midpoint of the segment. Shouldn't happen
            // (straight line was blocked) but defensive.
            return waypoints.get(0).plus(waypoints.get(1)).div(2.0);
        }
        if (waypoints.size() == 3) return waypoints.get(1);

        // Cumulative arc length; pick the inner waypoint closest to the midpoint.
        double[] cum = new double[waypoints.size()];
        for (int i = 1; i < waypoints.size(); i++) {
            cum[i] = cum[i - 1] + waypoints.get(i).getDistance(waypoints.get(i - 1));
        }
        double target = cum[cum.length - 1] * 0.5;
        int bestIdx = 1;
        double bestErr = Double.POSITIVE_INFINITY;
        for (int i = 1; i < waypoints.size() - 1; i++) {
            double err = Math.abs(cum[i] - target);
            if (err < bestErr) {
                bestErr = err;
                bestIdx = i;
            }
        }
        return waypoints.get(bestIdx);
    }
}
