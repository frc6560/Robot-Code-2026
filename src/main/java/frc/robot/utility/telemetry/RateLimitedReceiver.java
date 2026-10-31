package frc.robot.utility.telemetry;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

/** Caps all recorded frames, including AdvantageKit's built-in fields, to two per second. */
public final class RateLimitedReceiver implements LogDataReceiver {
    private final LogDataReceiver delegate;
    private long nextTimestamp = Long.MIN_VALUE;
    public RateLimitedReceiver(LogDataReceiver delegate) { this.delegate = delegate; }
    @Override public void start() { delegate.start(); }
    @Override public void end() { delegate.end(); }
    @Override public void putTable(LogTable table) throws InterruptedException {
        long timestamp = table.getTimestamp();
        if (timestamp < nextTimestamp) return;
        delegate.putTable(table);
        nextTimestamp = timestamp + RateLimitedTelemetry.PERIOD_MICROS;
    }
}
