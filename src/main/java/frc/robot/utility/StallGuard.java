package frc.robot.utility;

/** Latches a sustained high-current, low-motion condition until explicitly reset while disabled. */
public final class StallGuard {
    private final double duration;
    private double blockedSince = Double.NaN;
    private boolean tripped;
    public StallGuard(double durationSeconds) { duration = durationSeconds; }
    public boolean update(boolean commanded, boolean blocked, double now) {
        if (!commanded || !blocked) blockedSince = Double.NaN;
        else if (!Double.isFinite(blockedSince)) blockedSince = now;
        else if (now - blockedSince >= duration) tripped = true;
        return tripped;
    }
    public boolean isTripped() { return tripped; }
    public void reset() { tripped = false; blockedSince = Double.NaN; }
}
