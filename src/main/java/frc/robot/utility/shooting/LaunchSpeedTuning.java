package frc.robot.utility.shooting;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/** Dashboard edit is a request; the last accepted value has a separate persistent home. */
public final class LaunchSpeedTuning {
    public static final String REQUEST_KEY = "Shooter/Launch Speed Coefficient";
    public static final String PREFERENCE_KEY = "ShooterLaunchSpeedCoefficient";
    private LaunchSpeedTuning() {}

    private static boolean valid(double value) {
        return Double.isFinite(value) && value >= 0.5 && value <= 1.5;
    }

    public static void initialize() {
        double saved = Preferences.getDouble(PREFERENCE_KEY, Double.NaN);
        if (!valid(saved)) {
            // Migrate a valid value from the previous dashboard-only implementation.
            saved = SmartDashboard.getNumber(REQUEST_KEY, 1.0);
            if (!valid(saved)) saved = 1.0;
            Preferences.setDouble(PREFERENCE_KEY, saved);
        }
        SmartDashboard.putNumber(REQUEST_KEY, saved);
        SmartDashboard.getEntry(REQUEST_KEY).setPersistent();
        SmartDashboard.putNumber("Shooter/Saved Launch Speed Coefficient", saved);
    }

    /** Runs while disabled as well as enabled; no shoot-button press is needed to save. */
    public static double update() {
        double saved = Preferences.getDouble(PREFERENCE_KEY, 1.0);
        if (!valid(saved)) saved = 1.0;
        double requested = SmartDashboard.getNumber(REQUEST_KEY, Double.NaN);
        boolean accepted = valid(requested);
        if (accepted && requested != saved) {
            saved = requested;
            Preferences.setDouble(PREFERENCE_KEY, saved);
        }
        SmartDashboard.putNumber("Shooter/Saved Launch Speed Coefficient", saved);
        SmartDashboard.putString("Shooter/Launch Speed Coefficient Status", accepted
            ? "Saved on robot" : "Edit not received as a number in [0.5, 1.5]; keeping saved value");
        Logger.recordOutput("Fuel/Tuning/RequestedCoefficient", requested);
        Logger.recordOutput("Fuel/Tuning/SavedCoefficient", saved);
        Logger.recordOutput("Fuel/Tuning/RequestValid", accepted);
        return saved;
    }
}
