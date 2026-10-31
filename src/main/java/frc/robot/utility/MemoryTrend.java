package frc.robot.utility;

/** Fixed-storage trend over at most 60 seconds; values are nonnegative scalars, timestamps are seconds. */
final class MemoryTrend {
    private static final int CAPACITY = 61;
    private final double[] times = new double[CAPACITY];
    private final double[] values = new double[CAPACITY];
    private int head, count;
    private double baseline = Double.NaN;
    private double lastTime = Double.NaN;

    void add(double time, double value) {
        if (!Double.isFinite(time) || !Double.isFinite(value) || value < 0) {
            head = count = 0;
            return;
        }
        if (Double.isFinite(lastTime) && time <= lastTime) {
            head = count = 0;
            baseline = Double.NaN;
        }
        lastTime = time;
        if (!Double.isFinite(baseline)) baseline = value;
        while (count > 0 && time - times[head] > 60) {
            head = (head + 1) % CAPACITY;
            count--;
        }
        if (count == CAPACITY) { head = (head + 1) % CAPACITY; count--; }
        int slot = (head + count++) % CAPACITY;
        times[slot] = time;
        values[slot] = value;
    }
    private double latest() { return count == 0 ? Double.NaN : values[(head + count - 1) % CAPACITY]; }
    double sinceStart() { return latest() - baseline; }
    double delta() { return count < 2 ? Double.NaN : latest() - values[head]; }
    double seconds() { return count < 2 ? 0 : times[(head + count - 1) % CAPACITY] - times[head]; }
    double changePerMinute() { return seconds() > 0 ? delta() * 60 / seconds() : Double.NaN; }
    double bytesPerMinute() { return changePerMinute(); }
    double minimum() {
        if (count == 0) return Double.NaN;
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < count; i++) min = Math.min(min, values[(head + i) % CAPACITY]);
        return min;
    }
}
