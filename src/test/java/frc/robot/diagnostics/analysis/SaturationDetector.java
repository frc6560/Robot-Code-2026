package frc.robot.diagnostics.analysis;

import frc.robot.diagnostics.capture.ModuleSnapshot;
import frc.robot.diagnostics.capture.TickSnapshot;

import java.util.*;

public class SaturationDetector {
    private static final double VOLTAGE_THRESHOLD = 11.5;
    private static final int MIN_DURATION_TICKS = 25; // 500ms

    public List<Map<String, Object>> detect(List<TickSnapshot> snapshots) {
        List<Map<String, Object>> anomalies = new ArrayList<>();

        for (int moduleIdx = 0; moduleIdx < 4; moduleIdx++) {
            detectModuleSaturation(snapshots, anomalies, moduleIdx, true);
            detectModuleSaturation(snapshots, anomalies, moduleIdx, false);
        }

        return anomalies;
    }

    private void detectModuleSaturation(List<TickSnapshot> snapshots,
                                        List<Map<String, Object>> anomalies,
                                        int moduleIdx, boolean isDrive) {
        int saturatedStart = -1;

        for (int i = 0; i < snapshots.size(); i++) {
            ModuleSnapshot mod = snapshots.get(i).drive().modules().get(moduleIdx);
            double volts = isDrive ? mod.driveAppliedVolts() : mod.turnAppliedVolts();

            if (Math.abs(volts) >= VOLTAGE_THRESHOLD) {
                if (saturatedStart == -1) saturatedStart = i;
            } else {
                if (saturatedStart != -1 && (i - saturatedStart) >= MIN_DURATION_TICKS) {
                    String component = isDrive ? "Drive" : "Turn";
                    Map<String, Object> details = new LinkedHashMap<>();
                    details.put("duration_ms", (i - saturatedStart) * 20);
                    details.put("module_index", moduleIdx);
                    details.put("component", component);
                    anomalies.add(AnomalyDetector.makeAnomaly(
                        "WARNING", "SATURATION", "Module" + moduleIdx + component,
                        saturatedStart, i,
                        "Module " + moduleIdx + " " + component.toLowerCase()
                            + " motor saturated for " + ((i - saturatedStart) * 20) + "ms",
                        details));
                }
                saturatedStart = -1;
            }
        }
    }
}
