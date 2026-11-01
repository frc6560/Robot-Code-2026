package frc.robot.utility.telemetry;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

/** Disable application-managed file recorders before hardware devices are constructed. */
public final class StreamingLogPolicy {
    private StreamingLogPolicy() {}

    public static boolean disablePersistentRecorders() {
        // Disabling auto-start alone does not stop a recorder that is already running.
        var autoStatus = SignalLogger.enableAutoLogging(false);
        var stopStatus = SignalLogger.stop();
        // stop() does not create a log. logNetworkTables(false)/logConsoleOutput(false)
        // would implicitly START DataLogManager if it had never been started.
        DataLogManager.stop();
        boolean success = autoStatus.isOK() && stopStatus.isOK();
        if (!success) {
            DriverStation.reportError("Could not confirm CTRE file recording disabled: auto="
                + autoStatus + ", stop=" + stopStatus, false);
        }
        return success;
    }
}
