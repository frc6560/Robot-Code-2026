package frc.robot.utility.telemetry;

import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Latest-value logging. Sensor acquisition and motor control never wait for telemetry. */
public final class RateLimitedTelemetry {
    public static final long PERIOD_MICROS = 500_000;
    private static final LatestValueLimiter outputs = new LatestValueLimiter(
        RobotController::getFPGATime, RateLimitedTelemetry::writeOutput);
    private static final LatestValueLimiter inputs = new LatestValueLimiter(
        RobotController::getFPGATime, (key, value) -> Logger.processInputs(key, (LoggableInputs) value));
    private static final LatestValueLimiter dashboard = new LatestValueLimiter(
        RobotController::getFPGATime, RateLimitedTelemetry::writeDashboard);
    private RateLimitedTelemetry() {}

    public static void recordOutput(String key, Object value) { outputs.offer(key, value); }
    public static void processInputs(String key, LoggableInputs value) {
        // Replay input consumption must still run on every control iteration.
        if (Logger.hasReplaySource()) Logger.processInputs(key, value);
        else inputs.offer(key, value);
    }
    public static void putNumber(String key, double value) { putDashboard(key, value); }
    public static void putString(String key, String value) { putDashboard(key, value); }
    public static void putBoolean(String key, boolean value) { putDashboard(key, value); }
    private static void putDashboard(String key, Object value) {
        // Desktop control tests inspect dashboard state immediately. Recorded logs remain capped.
        if (RobotBase.isSimulation()) writeDashboard(key, value);
        else dashboard.offer(key, value);
    }
    private static void writeDashboard(String key, Object value) {
        if (value instanceof Boolean v) SmartDashboard.putBoolean(key, v);
        else if (value instanceof Number v) SmartDashboard.putNumber(key, v.doubleValue());
        else if (value instanceof String v) SmartDashboard.putString(key, v);
    }
    /** Flush pending end/interruption states even when their command has stopped executing. */
    public static void flush() {
        outputs.flush();
        inputs.flush();
        dashboard.flush();
    }
    private static void writeOutput(String key, Object value) {
        try {
            if (value instanceof Boolean v) Logger.recordOutput(key, v.booleanValue());
            else if (value instanceof Integer v) Logger.recordOutput(key, v.intValue());
            else if (value instanceof Long v) Logger.recordOutput(key, v.longValue());
            else if (value instanceof Number v) Logger.recordOutput(key, v.doubleValue());
            else if (value instanceof String v) Logger.recordOutput(key, v);
            else if (value instanceof String[] v) Logger.recordOutput(key, v);
            else if (value instanceof double[] v) Logger.recordOutput(key, v);
            else if (value instanceof boolean[] v) Logger.recordOutput(key, v);
            else if (value instanceof WPISerializable v) Logger.recordOutput(key, v);
            else if (value instanceof StructSerializable[] v) Logger.recordOutput(key, v);
            else DriverStation.reportWarning("Unsupported telemetry type for " + key, false);
        } catch (RuntimeException error) {
            // A diagnostic serialization failure must not terminate mechanism control.
            DriverStation.reportWarning("Telemetry " + key + ": " + error, false);
        }
    }
}
