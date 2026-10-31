package frc.robot.utility;

import java.util.function.DoublePredicate;
import java.util.function.DoubleSupplier;

/** A command-start boost with a fixed deadline, independently verified at each hardware transition. */
public final class StartupCurrentLimit {
    private final double continuous, startup, seconds;
    private final DoubleSupplier clock;
    private final DoublePredicate apply;
    private final Runnable stop;
    private boolean active;
    private double start;
    private double applied;
    private double desired;

    /** The caller configures the continuous limit before constructing this helper. */
    public StartupCurrentLimit(double continuous, double startup, double seconds,
            DoubleSupplier clock, DoublePredicate apply, Runnable stop) {
        this.continuous = continuous; this.startup = startup; this.seconds = seconds;
        this.clock = clock; this.apply = apply; this.stop = stop;
        applied = desired = continuous;
    }

    /** Repeated requests/speed changes do not restart the timer. Explicit new rack targets may. */
    public boolean update(boolean requested, boolean newMove, boolean needsMotion) {
        return update(requested, newMove, needsMotion, false);
    }

    /** Explicit force-retract mode can retain the startup allowance until its wall stop. */
    public boolean update(boolean requested, boolean newMove, boolean needsMotion, boolean forceRetracting) {
        double now = clock.getAsDouble();
        if (requested && (!active || newMove)) start = now;
        active = requested;
        desired = requested && needsMotion && (forceRetracting || now < start + seconds) ? startup : continuous;
        if (applied != desired) {
            if (!apply.test(desired)) {
                applied = Double.NaN; // A partial multi-motor update must also be retried.
                stop.run();
                return false;
            }
            applied = desired;
        }
        return true;
    }

    public double appliedAmps() { return applied; }
    public boolean confirmed() { return Double.isFinite(applied) && applied == desired; }
}
