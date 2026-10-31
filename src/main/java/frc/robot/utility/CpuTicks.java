package frc.robot.utility;

/** Fixed-storage Linux /proc/stat accounting. Guest counters are already included in user/nice. */
final class CpuTicks {
    private final long[] previous = new long[8];
    final long[] incoming = new long[8];
    boolean initialized, seen;
    double busy = Double.NaN, user = Double.NaN, kernel = Double.NaN,
        idle = Double.NaN, ioWait = Double.NaN, steal = Double.NaN;

    void reset() {
        initialized = seen = false;
        busy = user = kernel = idle = ioWait = steal = Double.NaN;
    }
    void accept() {
        seen = true;
        double total = 0;
        boolean valid = initialized;
        for (int i = 0; i < 8; i++) {
            if (incoming[i] < previous[i]) valid = false;
            total += incoming[i] - previous[i];
        }
        if (valid && total > 0) {
            user = 100.0 * ((incoming[0] - previous[0]) + (double) (incoming[1] - previous[1])) / total;
            kernel = 100.0 * ((incoming[2] - previous[2]) + (double) (incoming[5] - previous[5])
                + (incoming[6] - previous[6])) / total;
            idle = 100.0 * (incoming[3] - previous[3]) / total;
            ioWait = 100.0 * (incoming[4] - previous[4]) / total;
            steal = 100.0 * (incoming[7] - previous[7]) / total;
            busy = Math.min(100, user + kernel + steal);
        } else busy = user = kernel = idle = ioWait = steal = Double.NaN;
        System.arraycopy(incoming, 0, previous, 0, 8);
        initialized = true;
    }

    /** No split/boxing. Require the first eight counters; ignore optional guest fields. */
    static boolean parse(String line, long[] values) {
        int i = 0;
        if (!line.startsWith("cpu")) return false;
        i = 3;
        while (i < line.length() && line.charAt(i) >= '0' && line.charAt(i) <= '9') i++;
        if (i >= line.length() || !Character.isWhitespace(line.charAt(i))) return false;
        for (int field = 0; field < 8; field++) {
            while (i < line.length() && Character.isWhitespace(line.charAt(i))) i++;
            int start = i;
            long value = 0;
            while (i < line.length() && line.charAt(i) >= '0' && line.charAt(i) <= '9') {
                int digit = line.charAt(i++) - '0';
                if (value > (Long.MAX_VALUE - digit) / 10) return false;
                value = value * 10 + digit;
            }
            if (i == start || (i < line.length() && !Character.isWhitespace(line.charAt(i)))) return false;
            values[field] = value;
        }
        return true;
    }
}
