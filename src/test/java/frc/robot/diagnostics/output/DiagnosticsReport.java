package frc.robot.diagnostics.output;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.diagnostics.capture.TickSnapshot;
import frc.robot.diagnostics.capture.TrajectoryExpectation;

import java.util.List;
import java.util.Map;

public class DiagnosticsReport {
    private final String scenarioName;
    private final double durationSeconds;
    private final List<TickSnapshot> snapshots;
    private final List<Map<String, Object>> anomalies;
    private TrajectoryExpectation trajectory;

    public DiagnosticsReport(String scenarioName, double durationSeconds,
                             List<TickSnapshot> snapshots, List<Map<String, Object>> anomalies) {
        this.scenarioName = scenarioName;
        this.durationSeconds = durationSeconds;
        this.snapshots = snapshots;
        this.anomalies = anomalies;
    }

    public void setTrajectory(TrajectoryExpectation trajectory) {
        this.trajectory = trajectory;
    }

    public String getScenarioName() { return scenarioName; }
    public double getDurationSeconds() { return durationSeconds; }
    public List<TickSnapshot> getSnapshots() { return snapshots; }
    public List<Map<String, Object>> getAnomalies() { return anomalies; }
    public TrajectoryExpectation getTrajectory() { return trajectory; }
    public boolean hasTrajectory() { return trajectory != null && !trajectory.isEmpty(); }

    public List<TickSnapshot> getSampledSnapshots(int interval) {
        return snapshots.stream()
            .filter(s -> s.tick() % interval == 0)
            .toList();
    }
}
