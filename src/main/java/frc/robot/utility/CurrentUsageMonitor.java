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
import frc.robot.utility.telemetry.RateLimitedTelemetry;

/** Rank battery-side current across existing controllers, never creates duplicate devices. */
public final class CurrentUsageMonitor {
    public record Reading(String name, double supplyAmps) {}
    private record MotorSignals(String bus, StatusSignal<Current> current, StatusSignal<Temperature> temperature) {}
    private static final Map<String, MotorSignals> signals = new LinkedHashMap<>();
    private static TrackedMotor[] motors = new TrackedMotor[0];
    private static BusGroup[] buses = new BusGroup[0];
    private static final TrackedMotor[] topCurrent = new TrackedMotor[3], topTemperature = new TrackedMotor[3];
    private static final String[] currentKeys = {"Power/Top1CurrentUser", "Power/Top2CurrentUser", "Power/Top3CurrentUser"};
    private static final String[] temperatureKeys = {"Power/Top1HottestDevice", "Power/Top2HottestDevice", "Power/Top3HottestDevice"};
    private static final String[] ampsKeys = {"Power/Top1CurrentUserAmps", "Power/Top2CurrentUserAmps", "Power/Top3CurrentUserAmps"};
    private static final String[] temperatureValueKeys = {"Power/Top1HottestDeviceCelsius", "Power/Top2HottestDeviceCelsius", "Power/Top3HottestDeviceCelsius"};
    private record BusGroup(String key, BaseStatusSignal[] signals) {}
    static final class TrackedMotor {
        final String name, temperatureFreshKey, temperatureStatusKey, temperatureKey, freshKey, currentKey;
        final MotorSignals signals;
        double amps, temperature;
        TrackedMotor(String name, MotorSignals signals) {
            this.name = name; this.signals = signals;
            String prefix = "Power/Motors/" + name;
            temperatureFreshKey = prefix + "/TemperatureFresh";
            temperatureStatusKey = prefix + "/TemperatureStatus";
            temperatureKey = prefix + "/DeviceTempCelsius";
            freshKey = prefix + "/Fresh";
            currentKey = prefix + "/SupplyAmps";
        }
    }
    public record TemperatureReading(String name, double celsius) {}
    private CurrentUsageMonitor() {}

    public static void register(String name, TalonFX motor) {
        StatusSignal<Current> signal = motor.getSupplyCurrent();
        signal.setUpdateFrequency(20);
        StatusSignal<Temperature> temperature = motor.getDeviceTemp();
        temperature.setUpdateFrequency(10);
        signals.put(name + " (ID " + motor.getDeviceID() + ")",
            new MotorSignals(motor.getNetwork().getName(), signal, temperature));
        // Registration occurs during initialization. Build fixed buffers here, never in update().
        motors = signals.entrySet().stream().map(e -> new TrackedMotor(e.getKey(), e.getValue()))
            .toArray(TrackedMotor[]::new);
        Map<String, List<BaseStatusSignal>> groups = new LinkedHashMap<>();
        for (TrackedMotor tracked : motors) {
            var group = groups.computeIfAbsent(tracked.signals.bus(), ignored -> new ArrayList<>());
            group.add(tracked.signals.current());
            group.add(tracked.signals.temperature());
        }
        buses = groups.entrySet().stream().map(e -> new BusGroup("Power/RefreshStatus/" + e.getKey(),
            e.getValue().toArray(BaseStatusSignal[]::new))).toArray(BusGroup[]::new);
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
            RateLimitedTelemetry.recordOutput("Power/MonitorStatus", "Running");
            RateLimitedTelemetry.putString("Power/MonitorStatus", "Running");
            lastError = "";
        } catch (RuntimeException error) {
            String message = error.toString();
            RateLimitedTelemetry.recordOutput("Power/MonitorStatus", message);
            RateLimitedTelemetry.putString("Power/MonitorStatus", message);
            if (!message.equals(lastError)) {
                edu.wpi.first.wpilibj.DriverStation.reportError("Power monitor: " + message, error.getStackTrace());
                lastError = message;
            }
        }
    }

    static void insert(TrackedMotor[] top, TrackedMotor candidate, boolean temperature) {
        double value = temperature ? candidate.temperature : candidate.amps;
        for (int i = 0; i < top.length; i++) {
            TrackedMotor existing = top[i];
            double other = existing == null ? Double.NEGATIVE_INFINITY : temperature ? existing.temperature : existing.amps;
            if (existing == null || value > other || (value == other && candidate.name.compareTo(existing.name) < 0)) {
                for (int j = top.length - 1; j > i; j--) top[j] = top[j - 1];
                top[i] = candidate;
                return;
            }
        }
    }

    private static void updateReadings() {
        for (BusGroup bus : buses)
            RateLimitedTelemetry.recordOutput(bus.key(), BaseStatusSignal.refreshAll(bus.signals()).toString());
        java.util.Arrays.fill(topCurrent, null);
        java.util.Arrays.fill(topTemperature, null);
        int freshCount = 0, tempCount = 0;
        double totalAmps = 0;
        for (TrackedMotor motor : motors) {
            var current = motor.signals.current();
            var temperature = motor.signals.temperature();
            boolean tempFresh = temperature.getStatus().isOK() && temperature.getTimestamp().getLatency() < 2;
            motor.temperature = tempFresh ? temperature.getValueAsDouble() : Double.NaN;
            RateLimitedTelemetry.recordOutput(motor.temperatureFreshKey, tempFresh);
            RateLimitedTelemetry.recordOutput(motor.temperatureStatusKey, temperature.getStatus().toString());
            RateLimitedTelemetry.recordOutput(motor.temperatureKey, motor.temperature);
            if (Double.isFinite(motor.temperature) && motor.temperature >= -40) {
                tempCount++; insert(topTemperature, motor, true);
            }
            boolean fresh = current.getStatus().isOK() && current.getTimestamp().getLatency() < 0.5;
            motor.amps = fresh ? Math.max(0, current.getValueAsDouble()) : Double.NaN;
            RateLimitedTelemetry.recordOutput(motor.freshKey, fresh);
            RateLimitedTelemetry.recordOutput(motor.currentKey, motor.amps);
            if (Double.isFinite(motor.amps)) {
                freshCount++; totalAmps += motor.amps; insert(topCurrent, motor, false);
            }
        }
        for (int i = 0; i < 3; i++) {
            String name = topCurrent[i] == null ? "No fresh motor data" : topCurrent[i].name;
            double amps = topCurrent[i] == null ? 0 : topCurrent[i].amps;
            RateLimitedTelemetry.putString(currentKeys[i], name);
            RateLimitedTelemetry.recordOutput(currentKeys[i], name);
            RateLimitedTelemetry.putNumber(ampsKeys[i], amps);
            RateLimitedTelemetry.recordOutput(ampsKeys[i], amps);
            String hot = topTemperature[i] == null ? "No fresh temperature data" : topTemperature[i].name;
            double celsius = topTemperature[i] == null ? Double.NaN : topTemperature[i].temperature;
            RateLimitedTelemetry.putString(temperatureKeys[i], hot);
            RateLimitedTelemetry.recordOutput(temperatureKeys[i], hot);
            RateLimitedTelemetry.putNumber(temperatureValueKeys[i], celsius);
            RateLimitedTelemetry.recordOutput(temperatureValueKeys[i], celsius);
        }
        RateLimitedTelemetry.putNumber("Power/FreshTemperatureCount", tempCount);
        RateLimitedTelemetry.putNumber("Power/TrackedMotorCount", motors.length);
        RateLimitedTelemetry.putNumber("Power/FreshMotorCount", freshCount);
        RateLimitedTelemetry.recordOutput("Power/TrackedMotorCount", motors.length);
        RateLimitedTelemetry.recordOutput("Power/FreshMotorCount", freshCount);
        RateLimitedTelemetry.recordOutput("Power/TotalTrackedSupplyAmps", totalAmps);
    }
}
