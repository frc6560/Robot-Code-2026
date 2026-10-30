package frc.robot.utility.shooting;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/** Dashboard edit is a request; the last accepted value has a separate persistent home. */
public final class LaunchSpeedTuning {
    public static final double DEFAULT_COEFFICIENT = 1.05;
    public static final String REQUEST_KEY = "Shooter/Launch Speed Coefficient";
    public static final String PREFERENCE_KEY = "ShooterLaunchSpeedCoefficientV2";
    private static double nextTelemetry;
    private LaunchSpeedTuning() {}

    private static boolean valid(double value) {
        return Double.isFinite(value) && value >= 0.5 && value <= 1.5;
    }

    public static void initialize() {
        double saved = Preferences.getDouble(PREFERENCE_KEY, Double.NaN);
        if (!valid(saved)) {
            // Start this calibration at 1.05; subsequent dashboard edits persist.
            saved = DEFAULT_COEFFICIENT;
            Preferences.setDouble(PREFERENCE_KEY, saved);
        }
        SmartDashboard.putNumber(REQUEST_KEY, saved);
        SmartDashboard.getEntry(REQUEST_KEY).setPersistent();
        SmartDashboard.putNumber("Shooter/Saved Launch Speed Coefficient", saved);
    }

    /** Runs while disabled as well as enabled; no shoot-button press is needed to save. */
    public static double update() {
        double saved = Preferences.getDouble(PREFERENCE_KEY, DEFAULT_COEFFICIENT);
        if (!valid(saved)) saved = DEFAULT_COEFFICIENT;
        double requested = SmartDashboard.getNumber(REQUEST_KEY, Double.NaN);
        boolean accepted = valid(requested);
        if (accepted && requested != saved) {
            saved = requested;
            Preferences.setDouble(PREFERENCE_KEY, saved);
        }
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (now >= nextTelemetry) {
            nextTelemetry = now + 0.2;
        SmartDashboard.putNumber("Shooter/Saved Launch Speed Coefficient", saved);
        SmartDashboard.putString("Shooter/Launch Speed Coefficient Status", accepted
            ? "Saved on robot" : "Edit not received as a number in [0.5, 1.5]; keeping saved value");
        Logger.recordOutput("Fuel/Tuning/RequestedCoefficient", requested);
        Logger.recordOutput("Fuel/Tuning/SavedCoefficient", saved);
        Logger.recordOutput("Fuel/Tuning/RequestValid", accepted);
        }
        return saved;
    }
}
