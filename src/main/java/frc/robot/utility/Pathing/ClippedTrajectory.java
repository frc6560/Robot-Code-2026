package frc.robot.utility.Pathing;

import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.obstacle.ObstacleField;
import frc.robot.utility.Pathing.profile.VelocityProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

/**
 * A safety wrapper around a {@link PathChain} that projects any sampled point which
 * lands inside an {@link ObstacleField} keep-out cell to the nearest free cell. The
 * resulting polyline is a strict-staircase approximation that hugs the boundary of the
 * keep-out zone instead of cutting the corner.
 *
 * <p>Caveats the caller should understand before relying on this:
 * <ul>
 *   <li>Projected polylines are <b>not</b> smooth — they inherit the grid resolution.
 *       Sharp kinks get fed into the velocity profile as large discrete curvatures, so
 *       the robot auto-slows at each kink. If that's too jittery for competition, use
 *       the unclipped {@link PathChain} and tune waypoints manually; this is a guardrail,
 *       not a primary planner.</li>
 *   <li>Rotation is handled exactly like {@link PathChain}: a single trapezoidal profile
 *       from the chain's first segment start heading to its last segment end heading.</li>
 *   <li>If the original chain is already clear, this behaves identically to
 *       {@link PathChain} (projection is a no-op) but with slightly more per-tick work.</li>
 * </ul>
 */
public class ClippedTrajectory {
    private final Translation2d[] points;
    private final double[] arcLengths;
    private final double totalLength;

    private final VelocityProfile profile;
    private double currentArc;

    private final TrapezoidProfile.State startRotation;
    private final TrapezoidProfile.State currentRotationState;
    private final TrapezoidProfile.State endRotation;
    private final TrapezoidProfile rotationProfile;

    private final double startHeading;
    private final double endHeading;

    /**
     * @param chain            underlying chain to sample
     * @param field            obstacle field (typically the shared singleton)
     * @param sampleStepMeters approximate spacing between samples along the chain
     *                         (smaller = closer-fitting projection, more work)
     * @param maxVelocity      straight-line speed cap
     * @param maxAt            tangential accel cap
     * @param maxOmega         rotational velocity cap
     * @param maxAlpha         rotational accel cap
     * @param maxCentripetal   centripetal accel cap; enforces slowdowns at kinks
     */
    public ClippedTrajectory(PathChain chain, ObstacleField field,
                             double sampleStepMeters,
                             double maxVelocity, double maxAt,
                             double maxOmega, double maxAlpha,
                             double maxCentripetal) {
        // 1. Densely sample the chain by stepping along each segment's arc length, then
        //    project any sample that lands in a blocked cell to the nearest free cell.
        int totalSamples = Math.max(2, (int) Math.ceil(chain.getTotalLength() / Math.max(sampleStepMeters, 1e-3)));
        Translation2d[] raw = new Translation2d[totalSamples + 1];
        double stride = chain.getTotalLength() / totalSamples;

        for (int i = 0; i <= totalSamples; i++) {
            double s = Math.min(i * stride, chain.getTotalLength());
            raw[i] = sampleChainAtArc(chain, s);
            if (field.isBlocked(raw[i].getX(), raw[i].getY())) {
                raw[i] = field.nearestFreeCell(raw[i].getX(), raw[i].getY());
            }
        }
        // Always anchor the exact start/end of the original chain (unless they were
        // themselves blocked, in which case projection already handled them).
        raw[0] = chain.getSegment(0).getStartPose().getTranslation();
        if (!field.isBlocked(raw[0].getX(), raw[0].getY())) {
            raw[0] = chain.getSegment(0).getStartPose().getTranslation();
        } else {
            raw[0] = field.nearestFreeCell(raw[0].getX(), raw[0].getY());
        }
        Translation2d endPt = chain.getSegment(chain.size() - 1).getEndPose().getTranslation();
        raw[totalSamples] = field.isBlocked(endPt.getX(), endPt.getY())
                ? field.nearestFreeCell(endPt.getX(), endPt.getY()) : endPt;

        // 2. De-duplicate consecutive identical points (projection often collapses a run
        //    of blocked samples onto one nearest-free cell).
        Translation2d[] dedup = new Translation2d[totalSamples + 1];
        int n = 0;
        for (Translation2d p : raw) {
            if (n == 0 || !p.equals(dedup[n - 1])) dedup[n++] = p;
        }
        if (n < 2) {
            // degenerate — fabricate a two-point stub so the object is still usable.
            this.points = new Translation2d[] { dedup[0], dedup[0] };
        } else {
            this.points = new Translation2d[n];
            System.arraycopy(dedup, 0, this.points, 0, n);
        }

        // 3. Cumulative arc length and discrete curvature (turning angle / avg segment length).
        this.arcLengths = new double[this.points.length];
        double[] curvatures = new double[this.points.length];
        for (int i = 1; i < this.points.length; i++) {
            arcLengths[i] = arcLengths[i - 1] + this.points[i].getDistance(this.points[i - 1]);
        }
        this.totalLength = arcLengths[arcLengths.length - 1];

        for (int i = 1; i < this.points.length - 1; i++) {
            Translation2d a = this.points[i - 1];
            Translation2d b = this.points[i];
            Translation2d c = this.points[i + 1];
            double dx1 = b.getX() - a.getX(), dy1 = b.getY() - a.getY();
            double dx2 = c.getX() - b.getX(), dy2 = c.getY() - b.getY();
            double len1 = Math.hypot(dx1, dy1), len2 = Math.hypot(dx2, dy2);
            if (len1 < 1e-6 || len2 < 1e-6) { curvatures[i] = 0.0; continue; }
            // Signed turning angle between consecutive segments.
            double cross = dx1 * dy2 - dy1 * dx2;
            double dot = dx1 * dx2 + dy1 * dy2;
            double theta = Math.atan2(cross, dot);
            // Discrete curvature estimate: angle change per average segment length.
            curvatures[i] = theta / (0.5 * (len1 + len2));
        }
        curvatures[0] = curvatures.length > 1 ? curvatures[1] : 0.0;
        curvatures[curvatures.length - 1] = curvatures.length > 1
                ? curvatures[curvatures.length - 2] : 0.0;

        // 4. Joint velocity profile over the clipped polyline.
        double vStart = chain.getSegment(0).getStartVelocity();
        double vEnd = chain.getSegment(chain.size() - 1).getEndVelocity();
        this.profile = new VelocityProfile(
                arcLengths, curvatures, maxVelocity, maxAt, maxCentripetal, vStart, vEnd);
        this.currentArc = 0.0;

        this.startHeading = chain.getSegment(0).getStartPose().getRotation().getRadians();
        this.endHeading = chain.getSegment(chain.size() - 1).getEndPose().getRotation().getRadians();
        this.startRotation = new TrapezoidProfile.State(startHeading, 0);
        this.currentRotationState = new TrapezoidProfile.State(startHeading, 0);
        this.endRotation = new TrapezoidProfile.State(endHeading, 0);
        this.rotationProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxOmega, maxAlpha));
    }

    public double getTotalLength() { return totalLength; }
    public int sampleCount() { return points.length; }

    public Setpoint calculate(double currentRotation, double dt) {
        double vPlanned = profile.velocityAt(currentArc);
        currentArc = Math.min(currentArc + vPlanned * dt, totalLength);
        double commandedVelocity = profile.velocityAt(currentArc);

        // Interpolate position and tangent along the polyline.
        int idx = searchArc(currentArc);
        double denom = arcLengths[idx + 1] - arcLengths[idx];
        double frac = denom > 1e-9 ? (currentArc - arcLengths[idx]) / denom : 0.0;
        Translation2d a = points[idx], b = points[idx + 1];
        Translation2d target = new Translation2d(
                a.getX() + (b.getX() - a.getX()) * frac,
                a.getY() + (b.getY() - a.getY()) * frac);
        double dx = b.getX() - a.getX(), dy = b.getY() - a.getY();
        double norm = Math.hypot(dx, dy) + 1e-6;
        Translation2d tangent = new Translation2d(dx / norm, dy / norm);

        // Rotation — same angle-modulus trick the rest of the library uses.
        double errorToGoal = MathUtil.angleModulus(endRotation.position - currentRotation);
        double errorToSetpoint = MathUtil.angleModulus(startRotation.position - currentRotation);
        endRotation.position = currentRotation + errorToGoal;
        currentRotationState.position = currentRotation + errorToSetpoint;

        State rotSetpoint = rotationProfile.calculate(dt, currentRotationState, endRotation);
        currentRotationState.position = rotSetpoint.position;
        currentRotationState.velocity = rotSetpoint.velocity;

        return new Setpoint(
                target.getX(), target.getY(),
                rotSetpoint.position,
                tangent.getX() * commandedVelocity,
                tangent.getY() * commandedVelocity,
                rotSetpoint.velocity);
    }

    private int searchArc(double arc) {
        if (arc <= arcLengths[0]) return 0;
        if (arc >= arcLengths[arcLengths.length - 1]) return arcLengths.length - 2;
        int lo = 0, hi = arcLengths.length - 1;
        while (lo + 1 < hi) {
            int mid = (lo + hi) >>> 1;
            if (arcLengths[mid] <= arc) lo = mid; else hi = mid;
        }
        return lo;
    }

    /** Samples the chain at a given arc length by walking segment offsets. */
    private static Translation2d sampleChainAtArc(PathChain chain, double arc) {
        double cumulative = 0.0;
        for (int i = 0; i < chain.size(); i++) {
            double segLen = chain.getSegment(i).getArcLength();
            if (arc <= cumulative + segLen || i == chain.size() - 1) {
                double local = Math.min(Math.max(arc - cumulative, 0.0), segLen);
                double t = chain.getSegment(i).getTimeForArcLength(local);
                return chain.getSegment(i).calculatePosition(t);
            }
            cumulative += segLen;
        }
        // Unreachable.
        return chain.getSegment(chain.size() - 1).getEndPose().getTranslation();
    }
}
