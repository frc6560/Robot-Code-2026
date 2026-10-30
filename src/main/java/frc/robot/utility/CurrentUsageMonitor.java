package frc.robot.utility;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
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
    private record MotorSignals(String bus, StatusSignal<Current> current, StatusSignal<Temperature> temperature) {}
    private static final Map<String, MotorSignals> signals = new LinkedHashMap<>();
    public record TemperatureReading(String name, double celsius) {}
    private CurrentUsageMonitor() {}

    public static void register(String name, TalonFX motor) {
        StatusSignal<Current> signal = motor.getSupplyCurrent();
        signal.setUpdateFrequency(20);
        StatusSignal<Temperature> temperature = motor.getDeviceTemp();
        temperature.setUpdateFrequency(10);
        signals.put(name + " (ID " + motor.getDeviceID() + ")",
            new MotorSignals(motor.getNetwork().getName(), signal, temperature));
    }

    static List<Reading> rank(List<Reading> readings) {
        return readings.stream().filter(r -> Double.isFinite(r.supplyAmps()) && r.supplyAmps() >= 0)
            .sorted(Comparator.comparingDouble(Reading::supplyAmps).reversed()
                .thenComparing(Reading::name)).toList();
    }

    static List<TemperatureReading> rankTemperatures(List<TemperatureReading> readings) {
        return readings.stream().filter(r -> Double.isFinite(r.celsius()) && r.celsius() >= -40)
            .sorted(Comparator.comparingDouble(TemperatureReading::celsius).reversed()
                .thenComparing(TemperatureReading::name)).toList();
    }

    private static String lastError = "";

    private static double nextUpdate;

    public static void update() {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (now < nextUpdate) return;
        nextUpdate = now + 0.5; // Slow diagnostics: 2 Hz, not every control cycle.

        try {
            updateReadings();
            Logger.recordOutput("Power/MonitorStatus", "Running");
            SmartDashboard.putString("Power/MonitorStatus", "Running");
            lastError = "";
        } catch (RuntimeException error) {
            String message = error.toString();
            Logger.recordOutput("Power/MonitorStatus", message);
            SmartDashboard.putString("Power/MonitorStatus", message);
            if (!message.equals(lastError)) {
                edu.wpi.first.wpilibj.DriverStation.reportError("Power monitor: " + message, error.getStackTrace());
                lastError = message;
            }
        }
    }

    private static void updateReadings() {
        // Phoenix rejects a combined refresh spanning roboRIO and CANivore networks.
        Map<String, List<BaseStatusSignal>> byBus = new LinkedHashMap<>();
        signals.values().forEach(m -> {
            var group = byBus.computeIfAbsent(m.bus(), ignored -> new ArrayList<>());
            group.add(m.current());
            group.add(m.temperature());
        });
        byBus.forEach((bus, group) -> Logger.recordOutput("Power/RefreshStatus/" + bus,
            BaseStatusSignal.refreshAll(group).toString()));
        List<Reading> readings = new ArrayList<>();
        List<TemperatureReading> temperatures = new ArrayList<>();
        signals.forEach((name, motor) -> {
            var signal = motor.current();
            var temperature = motor.temperature();
            boolean tempFresh = temperature.getStatus().isOK() && temperature.getTimestamp().getLatency() < 2.0;
            Logger.recordOutput("Power/Motors/" + name + "/TemperatureFresh", tempFresh);
            Logger.recordOutput("Power/Motors/" + name + "/TemperatureStatus", temperature.getStatus().toString());
            Logger.recordOutput("Power/Motors/" + name + "/DeviceTempCelsius",
                tempFresh ? temperature.getValueAsDouble() : Double.NaN);
            if (tempFresh) temperatures.add(new TemperatureReading(name, temperature.getValueAsDouble()));
            boolean fresh = signal.getStatus().isOK() && signal.getTimestamp().getLatency() < 0.5;
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
        List<TemperatureReading> hottest = rankTemperatures(temperatures);
        for (int i = 0; i < 3; i++) {
            String key = "Power/Top" + (i + 1) + "HottestDevice";
            String summary = i < hottest.size() ? String.format(java.util.Locale.ROOT,
                "%s: %.1f C", hottest.get(i).name(), hottest.get(i).celsius()) : "No fresh temperature data";
            double celsius = i < hottest.size() ? hottest.get(i).celsius() : Double.NaN;
            SmartDashboard.putString(key, summary);
            SmartDashboard.putNumber(key + "Celsius", celsius);
            Logger.recordOutput(key, summary);
            Logger.recordOutput(key + "Celsius", celsius);
        }
        SmartDashboard.putNumber("Power/FreshTemperatureCount", hottest.size());
        SmartDashboard.putNumber("Power/TrackedMotorCount", signals.size());
        SmartDashboard.putNumber("Power/FreshMotorCount", sorted.size());
        Logger.recordOutput("Power/TrackedMotorCount", signals.size());
        Logger.recordOutput("Power/FreshMotorCount", sorted.size());
        Logger.recordOutput("Power/TotalTrackedSupplyAmps", sorted.stream().mapToDouble(Reading::supplyAmps).sum());
    }
}
