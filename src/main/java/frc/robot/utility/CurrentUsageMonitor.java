package frc.robot.utility;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** Rank battery-side current across existing controllers, never creates duplicate devices. */
public final class CurrentUsageMonitor {
    public record Reading(String name, double supplyAmps) {}
    private static final Map<String, StatusSignal<Current>> signals = new LinkedHashMap<>();
    private CurrentUsageMonitor() {}

    public static void register(String name, TalonFX motor) {
        StatusSignal<Current> signal = motor.getSupplyCurrent();
        signal.setUpdateFrequency(50);
        signals.put(name + " (ID " + motor.getDeviceID() + ")", signal);
    }

    static List<Reading> rank(List<Reading> readings) {
        return readings.stream().filter(r -> Double.isFinite(r.supplyAmps()) && r.supplyAmps() >= 0)
            .sorted(Comparator.comparingDouble(Reading::supplyAmps).reversed()
                .thenComparing(Reading::name)).toList();
    }

    public static void update() {
        if (!signals.isEmpty()) BaseStatusSignal.refreshAll(signals.values().toArray(new BaseStatusSignal[0]));
        List<Reading> readings = new ArrayList<>();
        signals.forEach((name, signal) -> {
            boolean fresh = signal.getStatus().isOK() && signal.getTimestamp().getLatency() < 0.25;
            Logger.recordOutput("Power/Motors/" + name + "/Fresh", fresh);
            Logger.recordOutput("Power/Motors/" + name + "/SupplyAmps", fresh ? signal.getValueAsDouble() : Double.NaN);
            if (fresh) readings.add(new Reading(name, Math.max(0, signal.getValueAsDouble())));
        });
        List<Reading> sorted = rank(readings);
        for (int i = 0; i < 3; i++) {
            String key = "Power/Top" + (i + 1) + "CurrentUser";
            String name = i < sorted.size() ? sorted.get(i).name() : "No fresh motor data";
            double amps = i < sorted.size() ? sorted.get(i).supplyAmps() : 0;
            String summary = i < sorted.size()
                ? String.format(java.util.Locale.ROOT, "%s: %.1f A", name, amps) : name;
            SmartDashboard.putString(key, summary);
            SmartDashboard.putNumber(key + "Amps", amps);
            Logger.recordOutput(key, summary);
            Logger.recordOutput(key + "Amps", amps);
        }
        SmartDashboard.putNumber("Power/TrackedMotorCount", signals.size());
        SmartDashboard.putNumber("Power/FreshMotorCount", sorted.size());
        Logger.recordOutput("Power/TrackedMotorCount", signals.size());
        Logger.recordOutput("Power/FreshMotorCount", sorted.size());
        Logger.recordOutput("Power/TotalTrackedSupplyAmps", sorted.stream().mapToDouble(Reading::supplyAmps).sum());
    }
}
