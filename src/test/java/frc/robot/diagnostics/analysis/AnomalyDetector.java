package frc.robot.diagnostics.analysis;

import frc.robot.diagnostics.capture.TickSnapshot;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class AnomalyDetector {

    public List<Map<String, Object>> analyze(List<TickSnapshot> snapshots) {
        List<Map<String, Object>> anomalies = new ArrayList<>();
        anomalies.addAll(new OscillationDetector().detect(snapshots));
        anomalies.addAll(new SaturationDetector().detect(snapshots));
        anomalies.addAll(new DriftDetector().detect(snapshots));
        return anomalies;
    }

    static Map<String, Object> makeAnomaly(String severity, String category,
                                            String subsystem, int firstTick, int lastTick,
                                            String description, Map<String, Object> details) {
        Map<String, Object> anomaly = new LinkedHashMap<>();
        anomaly.put("severity", severity);
        anomaly.put("category", category);
        anomaly.put("subsystem", subsystem);
        anomaly.put("first_tick", firstTick);
        anomaly.put("last_tick", lastTick);
        anomaly.put("time_range_s", String.format("%.2f-%.2f", firstTick * 0.02, lastTick * 0.02));
        anomaly.put("description", description);
        if (details != null) anomaly.put("details", details);
        return anomaly;
    }
}
