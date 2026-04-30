package frc.robot.utility.Pathing.profile;

/**
 * A planned velocity profile v(s) over a single parametric segment. Respects:
 *   - v_max           (straight-line speed cap)
 *   - a_max           (tangential accel/decel cap)
 *   - v_curv(s)=sqrt(a_centripetal / |k(s)|)   (curvature cap)
 *   - v_start, v_end  (boundary speeds)
 *
 * <p>Built with the standard two-pass trick: first a forward pass bounded by the curvature
 * cap that cannot accelerate faster than a_max; then a backward pass bounded by v_end that
 * cannot decelerate faster than a_max. The pointwise min of the two is the feasible profile.
 *
 * <p>Unlike the raw {@link edu.wpi.first.math.trajectory.TrapezoidProfile}, this plans the
 * curvature-aware decel ahead of a corner instead of clamping reactively (which silently
 * violates the accel cap).
 *
 * <p>Intended use: construct once per path segment, then call {@link #velocityAt(double)}
 * per control loop. O(N) to build, O(log N) per query.
 */
public class VelocityProfile {
    private final double[] s;
    private final double[] v;

    public VelocityProfile(double[] arcLengths,
                           double[] curvatures,
                           double vMax,
                           double aMax,
                           double maxCentripetal,
                           double vStart,
                           double vEnd) {
        if (arcLengths.length != curvatures.length) {
            throw new IllegalArgumentException("arc/curvature length mismatch");
        }
        int n = arcLengths.length;
        this.s = arcLengths.clone();
        this.v = new double[n];

        // curvature cap per sample
        double[] vCap = new double[n];
        for (int i = 0; i < n; i++) {
            double k = Math.abs(curvatures[i]);
            double cap = vMax;
            if (maxCentripetal > 0.0 && k > 1e-6) {
                cap = Math.min(cap, Math.sqrt(maxCentripetal / k));
            }
            vCap[i] = cap;
        }

        // forward pass
        v[0] = Math.min(vStart, vCap[0]);
        for (int i = 1; i < n; i++) {
            double ds = Math.max(s[i] - s[i - 1], 0.0);
            double reach = Math.sqrt(v[i - 1] * v[i - 1] + 2.0 * aMax * ds);
            v[i] = Math.min(vCap[i], reach);
        }

        // backward pass (clamp end condition by curvature at the last sample too)
        double endV = Math.min(vEnd, vCap[n - 1]);
        v[n - 1] = Math.min(v[n - 1], endV);
        for (int i = n - 2; i >= 0; i--) {
            double ds = Math.max(s[i + 1] - s[i], 0.0);
            double reach = Math.sqrt(v[i + 1] * v[i + 1] + 2.0 * aMax * ds);
            v[i] = Math.min(v[i], reach);
        }
    }

    public double getTotalLength() {
        return s[s.length - 1];
    }

    public double getTotalTime() {
        double t = 0.0;
        for (int i = 1; i < s.length; i++) {
            double ds = s[i] - s[i - 1];
            double vAvg = 0.5 * (v[i - 1] + v[i]);
            if (vAvg > 1e-6) t += ds / vAvg;
        }
        return t;
    }

    /** Interpolated planned velocity at arc length {@code arc}. */
    public double velocityAt(double arc) {
        if (arc <= s[0]) return v[0];
        if (arc >= s[s.length - 1]) return v[v.length - 1];
        int lo = 0, hi = s.length - 1;
        while (lo + 1 < hi) {
            int mid = (lo + hi) >>> 1;
            if (s[mid] <= arc) lo = mid;
            else hi = mid;
        }
        double denom = s[lo + 1] - s[lo];
        double frac = denom > 1e-12 ? (arc - s[lo]) / denom : 0.0;
        return v[lo] + frac * (v[lo + 1] - v[lo]);
    }
}
