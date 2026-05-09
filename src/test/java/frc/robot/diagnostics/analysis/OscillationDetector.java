package frc.robot.diagnostics.analysis;

import frc.robot.diagnostics.capture.ModuleSnapshot;
import frc.robot.diagnostics.capture.TickSnapshot;

import java.util.*;
import java.util.function.ToDoubleFunction;

public class OscillationDetector {
    private static final int WINDOW_SIZE = 25; // 500ms
    private static final int MIN_ZERO_CROSSINGS = 5;

    public List<Map<String, Object>> detect(List<TickSnapshot> snapshots) {
        List<Map<String, Object>> anomalies = new ArrayList<>();

        detectSignal(snapshots, anomalies, "Turret",
            s -> s.superstructure().turretAngleDeg() - s.superstructure().turretTargetDeg());
        detectSignal(snapshots, anomalies, "Hood",
            s -> s.superstructure().hoodAngleDeg() - s.superstructure().hoodTargetDeg());
        detectSignal(snapshots, anomalies, "Shooter",
            s -> s.superstructure().shooterRPM() - s.superstructure().shooterTargetRPM());

        for (int moduleIdx = 0; moduleIdx < 4; moduleIdx++) {
            final int idx = moduleIdx;
            detectSignal(snapshots, anomalies, "DriveModule" + idx,
                s -> s.drive().modules().get(idx).driveAppliedVolts());
        }

        return anomalies;
    }

    private void detectSignal(List<TickSnapshot> snapshots, List<Map<String, Object>> anomalies,
                              String subsystem, ToDoubleFunction<TickSnapshot> signalExtractor) {
        if (snapshots.size() < WINDOW_SIZE) return;

        double[] signal = snapshots.stream().mapToDouble(signalExtractor).toArray();

        int anomalyStart = -1;
        for (int i = WINDOW_SIZE; i < signal.length; i++) {
            int crossings = countZeroCrossings(signal, i - WINDOW_SIZE, i);
            if (crossings >= MIN_ZERO_CROSSINGS) {
                if (anomalyStart == -1) anomalyStart = i - WINDOW_SIZE;
            } else {
                if (anomalyStart != -1) {
                    double peakError = 0;
                    for (int j = anomalyStart; j < i; j++) {
                        peakError = Math.max(peakError, Math.abs(signal[j]));
                    }
                    if (peakError > 0.5) {
                        Map<String, Object> details = new LinkedHashMap<>();
                        details.put("peak_amplitude", Math.round(peakError * 100.0) / 100.0);
                        details.put("approx_frequency_hz", crossings / (WINDOW_SIZE * 0.02 * 2.0));
                        anomalies.add(AnomalyDetector.makeAnomaly(
                            "WARNING", "OSCILLATION", subsystem,
                            anomalyStart, i,
                            subsystem + " error oscillating with peak " + String.format("%.1f", peakError),
                            details));
                    }
                    anomalyStart = -1;
                }
            }
        }
    }

    private int countZeroCrossings(double[] signal, int start, int end) {
        int crossings = 0;
        for (int i = start + 1; i < end; i++) {
            if ((signal[i - 1] >= 0 && signal[i] < 0) || (signal[i - 1] < 0 && signal[i] >= 0)) {
                crossings++;
            }
        }
        return crossings;
    }
}
