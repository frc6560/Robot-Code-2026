package frc.robot.diagnostics.analysis;

import frc.robot.diagnostics.capture.TickSnapshot;

import java.util.*;

public class DriftDetector {
    private static final double HEADING_DRIFT_RATE_THRESHOLD = 5.0; // deg/s when robot should be stationary
    private static final double STATIONARY_VELOCITY_THRESHOLD = 0.05; // m/s

    public List<Map<String, Object>> detect(List<TickSnapshot> snapshots) {
        List<Map<String, Object>> anomalies = new ArrayList<>();

        int stationaryStart = -1;
        double headingAtStart = 0;

        for (int i = 0; i < snapshots.size(); i++) {
            TickSnapshot snap = snapshots.get(i);
            double speed = Math.hypot(snap.drive().vxMps(), snap.drive().vyMps());

            if (speed < STATIONARY_VELOCITY_THRESHOLD) {
                if (stationaryStart == -1) {
                    stationaryStart = i;
                    headingAtStart = snap.drive().headingDeg();
                }
            } else {
                if (stationaryStart != -1) {
                    checkDrift(snapshots, anomalies, stationaryStart, i, headingAtStart);
                }
                stationaryStart = -1;
            }
        }

        if (stationaryStart != -1) {
            checkDrift(snapshots, anomalies, stationaryStart, snapshots.size() - 1, headingAtStart);
        }

        return anomalies;
    }

    private void checkDrift(List<TickSnapshot> snapshots, List<Map<String, Object>> anomalies,
                            int start, int end, double headingAtStart) {
        int duration = end - start;
        if (duration < 25) return; // ignore short stops

        double headingAtEnd = snapshots.get(end).drive().headingDeg();
        double driftDeg = Math.abs(headingAtEnd - headingAtStart);
        double durationSec = duration * 0.02;
        double driftRate = driftDeg / durationSec;

        if (driftRate > HEADING_DRIFT_RATE_THRESHOLD) {
            Map<String, Object> details = new LinkedHashMap<>();
            details.put("drift_deg", Math.round(driftDeg * 100.0) / 100.0);
            details.put("drift_rate_deg_per_s", Math.round(driftRate * 100.0) / 100.0);
            details.put("stationary_duration_s", Math.round(durationSec * 100.0) / 100.0);
            anomalies.add(AnomalyDetector.makeAnomaly(
                "ERROR", "DRIFT", "Drive",
                start, end,
                "Heading drift of " + String.format("%.1f", driftDeg) + " deg over "
                    + String.format("%.1f", durationSec) + "s while stationary",
                details));
        }
    }
}
