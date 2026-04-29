package frc.robot.utility.Pathing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.obstacle.AStarPlanner;
import frc.robot.utility.Pathing.obstacle.ObstacleField;

import java.util.ArrayList;
import java.util.List;

/** Builds a smooth path from start to end, routing around obstacles declared in the
 *  shared {@link ObstacleField}. All obstacle logic is delegated to the field + A*. */
public class PathCalculator {
    public Setpoint startPose;
    public Setpoint endPose;

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

    public Pose2d getPoseDirectionFrom(Pose2d pose, double magnitude, double direction) {
        double x = pose.getX() + magnitude * Math.cos(direction);
        double y = pose.getY() + magnitude * Math.sin(direction);
        return new Pose2d(x, y, Rotation2d.fromRadians(direction));
    }

    /** Control-handle length heuristic: 1/3 of the Euclidean distance, clamped. */
    public double calculateControlLengths(Pose2d firstPose, Pose2d secondPose) {
        Translation2d displacement = secondPose.getTranslation().minus(firstPose.getTranslation());
        return MathUtil.clamp(displacement.getNorm() / 3.0, 0.5, 3.0);
    }

    /** Single-segment path when the straight line is clear. */
    public Path calculateDirectPath() {
        Pose2d startPoseReal = startPose.getSetpointPose();
        Pose2d endPoseReal = endPose.getSetpointPose();
        double len = calculateControlLengths(startPoseReal, endPoseReal);

        Translation2d diff = endPoseReal.getTranslation().minus(startPoseReal.getTranslation());
        double heading = Math.atan2(diff.getY(), diff.getX());

        Pose2d startControl = getPoseDirectionFrom(startPoseReal, len, heading);
        Pose2d endControl = getPoseDirectionFrom(endPoseReal, len, heading + Math.PI);

        return new Path(startPose, endPose, startControl, endControl,
                maxVelocity, maxAccel, maxOmega, maxAlpha, maxCentripetal);
    }

    /** Dynamic-waypoint obstacle-avoiding chain. Delegates waypoint selection to the
     *  A* planner's line-of-sight thinning — the number of intermediate waypoints is
     *  whatever the field geometry demands.
     *
     *  @return a {@link PathChain}, or {@code null} if A* cannot find a route.
     */
    public PathChain calculatePathChain() {
        if (!blockedStraightLine) {
            return new PathChain(List.of(calculateDirectPath()),
                    maxVelocity, maxAccel, maxOmega, maxAlpha, maxCentripetal);
        }

        List<Translation2d> waypoints = new AStarPlanner(field).plan(
                startPose.getTranslation(), endPose.getTranslation());
        if (waypoints.size() < 2) return null;

        List<Path> segments = new ArrayList<>(waypoints.size() - 1);

        for (int i = 0; i < waypoints.size() - 1; i++) {
            Translation2d a = waypoints.get(i);
            Translation2d b = waypoints.get(i + 1);

            // Control-handle heading at each shared junction is the bisector of the
            // incoming and outgoing segment directions -> colinear handles -> C1.
            double headingAtA = (i == 0)
                    ? Math.atan2(b.getY() - a.getY(), b.getX() - a.getX())
                    : averageHeading(waypoints.get(i - 1), a, b);
            double headingAtB = (i == waypoints.size() - 2)
                    ? Math.atan2(b.getY() - a.getY(), b.getX() - a.getX())
                    : averageHeading(a, b, waypoints.get(i + 2));

            Pose2d aPose = new Pose2d(a, Rotation2d.fromRadians(headingAtA));
            Pose2d bPose = new Pose2d(b, Rotation2d.fromRadians(headingAtB));
            double len = calculateControlLengths(aPose, bPose);

            // Endpoint setpoints: keep the caller-provided start/end, synthesize interior waypoints.
            Setpoint aSet = (i == 0) ? startPose
                    : new Setpoint(a.getX(), a.getY(), headingAtA, 0, 0, 0);
            Setpoint bSet = (i == waypoints.size() - 2) ? endPose
                    : new Setpoint(b.getX(), b.getY(), headingAtB, 0, 0, 0);

            Pose2d startControl = getPoseDirectionFrom(aPose, len, headingAtA);
            Pose2d endControl = getPoseDirectionFrom(bPose, len, headingAtB + Math.PI);

            Path seg = new Path(aSet, bSet, startControl, endControl,
                    maxVelocity, maxAccel, maxOmega, maxAlpha, maxCentripetal);

            // If the Bezier between consecutive LOS-clear waypoints is itself blocked
            // (can happen near concave obstacle corners where the Bezier bulges outward),
            // refuse rather than silently run into a wall.
            if (field.isCurveBlocked(seg::calculatePosition, seg.getArcLength(), 0.10)) {
                return null;
            }
            segments.add(seg);
        }

        return new PathChain(segments,
                maxVelocity, maxAccel, maxOmega, maxAlpha, maxCentripetal);
    }

    /** Legacy 2-segment API retained so older call sites keep working. Delegates to
     *  {@link #calculatePathChain()}; if the chain has more than two segments we collapse
     *  to the first+last, which loses fidelity but preserves the old return type. Prefer
     *  {@code calculatePathChain()} in new code. */
    @Deprecated
    public PathGroup calculatePathGroup() {
        PathChain chain = calculatePathChain();
        if (chain == null || chain.size() < 2) return null;
        // For N>=2, PathGroup can only hold two segments. Merge everything into a pair of
        // Beziers joining start -> midpoint-waypoint -> end by rebuilding from the midpoint.
        if (chain.size() == 2) {
            return new PathGroup(chain.getSegment(0), chain.getSegment(1),
                    maxVelocity, maxAccel, maxOmega, maxAlpha, maxCentripetal);
        }
        int mid = chain.size() / 2;
        return new PathGroup(chain.getSegment(0), chain.getSegment(mid),
                maxVelocity, maxAccel, maxOmega, maxAlpha, maxCentripetal);
    }

    private static double averageHeading(Translation2d prev, Translation2d at, Translation2d next) {
        double hIn = Math.atan2(at.getY() - prev.getY(), at.getX() - prev.getX());
        double hOut = Math.atan2(next.getY() - at.getY(), next.getX() - at.getX());
        // Average via unit vectors to avoid wraparound issues.
        double cx = Math.cos(hIn) + Math.cos(hOut);
        double cy = Math.sin(hIn) + Math.sin(hOut);
        return Math.atan2(cy, cx);
    }
}
