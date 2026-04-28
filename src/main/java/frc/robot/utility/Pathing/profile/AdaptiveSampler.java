package frc.robot.utility.Pathing.profile;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.function.DoubleFunction;
import java.util.function.DoubleUnaryOperator;

/**
 * Builds a parameter-to-arc-length table for a parametric curve by recursive midpoint
 * subdivision. Curves get dense samples where they actually bend and sparse samples on
 * straight runs — no wasted resolution.
 *
 * <p>Decoupled from {@link frc.robot.utility.Pathing.Path} on purpose: accepts plain
 * function refs so it can be unit-tested with any curve.
 */
public class AdaptiveSampler {
    private final double[] ts;
    private final double[] ss;
    private final double[] ks;
    private final double totalLength;

    /**
     * @param position        t in [0,1] -> point on the curve
     * @param curvature       t in [0,1] -> signed curvature
     * @param chordTolerance  meters; midpoint deviation below this stops subdivision
     * @param minDepth        forces at least 2^minDepth segments even on straight sections
     * @param maxDepth        cap to keep recursion bounded (typical 12 -> up to 4096 samples)
     */
    public AdaptiveSampler(DoubleFunction<Translation2d> position,
                           DoubleUnaryOperator curvature,
                           double chordTolerance,
                           int minDepth,
                           int maxDepth) {
        ArrayList<Double> tList = new ArrayList<>(256);
        tList.add(0.0);
        subdivide(position, tList, 0.0, 1.0, chordTolerance, 0, minDepth, maxDepth);

        int n = tList.size();
        this.ts = new double[n];
        this.ss = new double[n];
        this.ks = new double[n];

        Translation2d prev = position.apply(tList.get(0));
        ts[0] = tList.get(0);
        ss[0] = 0.0;
        ks[0] = curvature.applyAsDouble(tList.get(0));
        for (int i = 1; i < n; i++) {
            double t = tList.get(i);
            Translation2d cur = position.apply(t);
            ts[i] = t;
            ss[i] = ss[i - 1] + cur.getDistance(prev);
            ks[i] = curvature.applyAsDouble(t);
            prev = cur;
        }
        this.totalLength = ss[n - 1];
    }

    private void subdivide(DoubleFunction<Translation2d> pos, ArrayList<Double> out,
                           double a, double b, double tol,
                           int depth, int minDepth, int maxDepth) {
        double m = 0.5 * (a + b);
        if (depth >= maxDepth) {
            out.add(b);
            return;
        }
        if (depth >= minDepth) {
            Translation2d pa = pos.apply(a);
            Translation2d pb = pos.apply(b);
            Translation2d pm = pos.apply(m);
            double err = pm.getDistance(pa.plus(pb).div(2.0));
            if (err < tol) {
                out.add(b);
                return;
            }
        }
        subdivide(pos, out, a, m, tol, depth + 1, minDepth, maxDepth);
        subdivide(pos, out, m, b, tol, depth + 1, minDepth, maxDepth);
    }

    public double getArcLength() {
        return totalLength;
    }

    public int size() {
        return ts.length;
    }

    public double[] tSamples() {
        return ts;
    }

    public double[] sSamples() {
        return ss;
    }

    public double[] kSamples() {
        return ks;
    }

    public double getTimeForArcLength(double s) {
        s = clamp(s, 0.0, totalLength);
        int idx = searchArc(s);
        double denom = ss[idx + 1] - ss[idx];
        double frac = denom > 1e-12 ? (s - ss[idx]) / denom : 0.0;
        return ts[idx] + frac * (ts[idx + 1] - ts[idx]);
    }

    public double getCurvatureAtArcLength(double s) {
        s = clamp(s, 0.0, totalLength);
        int idx = searchArc(s);
        double denom = ss[idx + 1] - ss[idx];
        double frac = denom > 1e-12 ? (s - ss[idx]) / denom : 0.0;
        return ks[idx] + frac * (ks[idx + 1] - ks[idx]);
    }

    private int searchArc(double s) {
        int lo = 0, hi = ss.length - 1;
        while (lo + 1 < hi) {
            int mid = (lo + hi) >>> 1;
            if (ss[mid] <= s) lo = mid;
            else hi = mid;
        }
        return lo;
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
