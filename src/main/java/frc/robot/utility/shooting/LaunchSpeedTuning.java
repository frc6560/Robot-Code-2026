package frc.robot.utility.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.telemetry.RateLimitedTelemetry;

/** Fixed launch-speed scale. Dashboard and legacy saved preferences never select this value. */
public final class LaunchSpeedTuning {
    public static final double DEFAULT_COEFFICIENT = 1.025;
    /** Legacy dashboard key, now published for display only. */
    public static final String REQUEST_KEY = "Shooter/Launch Speed Coefficient";
    /** Legacy preference key, retained for compatibility; never read or written. */
    public static final String PREFERENCE_KEY = "ShooterLaunchSpeedCoefficientV2";
    private static double nextTelemetry;
    private LaunchSpeedTuning() {}

    public static void initialize() {
        SmartDashboard.getEntry(REQUEST_KEY).clearPersistent();
        publish();
    }

    /** The same fixed scale applies before shooting, while disabled, and during every shot. */
    public static double update() {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (now >= nextTelemetry) {
            nextTelemetry = now + 0.5;
            publish();
        }
        return DEFAULT_COEFFICIENT;
    }

    private static void publish() {
        RateLimitedTelemetry.putNumber(REQUEST_KEY, DEFAULT_COEFFICIENT);
        RateLimitedTelemetry.putString("Shooter/Launch Speed Coefficient Status",
            "Fixed in code: 1.025; dashboard edits ignored");
        RateLimitedTelemetry.recordOutput("Fuel/Tuning/AppliedCoefficient", DEFAULT_COEFFICIENT);
    }
}
