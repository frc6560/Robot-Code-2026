package frc.robot.diagnostics.output;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.diagnostics.capture.*;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.Instant;
import java.util.*;

public class DiagnosticsReporter {

    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().serializeSpecialFloatingPointValues().create();

    public static void writeReport(DiagnosticsReport report, String outputDir) {
        File dir = new File(outputDir);
        dir.mkdirs();

        String baseName = report.getScenarioName().replaceAll("[^a-zA-Z0-9_]", "_");

        Map<String, Object> fullOutput = buildFullOutput(report);
        writeJson(new File(dir, baseName + ".json"), fullOutput);

        Map<String, Object> summaryOutput = buildSummaryOutput(report);
        writeJson(new File(dir, baseName + "_summary.json"), summaryOutput);
    }

    private static Map<String, Object> buildFullOutput(DiagnosticsReport report) {
        Map<String, Object> output = new LinkedHashMap<>();

        Map<String, Object> metadata = new LinkedHashMap<>();
        metadata.put("scenario", report.getScenarioName());
        metadata.put("duration_seconds", report.getDurationSeconds());
        metadata.put("tick_count", report.getSnapshots().size());
        metadata.put("tick_period_ms", 20);
        metadata.put("timestamp", Instant.now().toString());
        output.put("metadata", metadata);

        output.put("summary", buildSummaryStats(report));
        output.put("anomalies", report.getAnomalies());
        output.put("snapshots", serializeSnapshots(report.getSnapshots()));

        return output;
    }

    private static Map<String, Object> buildSummaryOutput(DiagnosticsReport report) {
        Map<String, Object> output = new LinkedHashMap<>();

        Map<String, Object> metadata = new LinkedHashMap<>();
        metadata.put("scenario", report.getScenarioName());
        metadata.put("duration_seconds", report.getDurationSeconds());
        metadata.put("tick_count", report.getSnapshots().size());
        metadata.put("timestamp", Instant.now().toString());
        output.put("metadata", metadata);

        output.put("summary", buildSummaryStats(report));
        output.put("anomalies", report.getAnomalies());

        // Sampled snapshots (every 25th tick = 500ms intervals)
        output.put("snapshots", serializeSnapshots(report.getSampledSnapshots(25)));

        return output;
    }

    private static Map<String, Object> buildSummaryStats(DiagnosticsReport report) {
        Map<String, Object> summary = new LinkedHashMap<>();
        List<TickSnapshot> snapshots = report.getSnapshots();

        if (snapshots.isEmpty()) {
            summary.put("error", "No snapshots captured");
            return summary;
        }

        TickSnapshot first = snapshots.get(0);
        TickSnapshot last = snapshots.get(snapshots.size() - 1);

        // Final pose
        Map<String, Double> finalPose = new LinkedHashMap<>();
        finalPose.put("x", last.drive().poseX());
        finalPose.put("y", last.drive().poseY());
        finalPose.put("heading_deg", last.drive().headingDeg());
        summary.put("final_pose", finalPose);

        // Initial pose
        Map<String, Double> initialPose = new LinkedHashMap<>();
        initialPose.put("x", first.drive().poseX());
        initialPose.put("y", first.drive().poseY());
        initialPose.put("heading_deg", first.drive().headingDeg());
        summary.put("initial_pose", initialPose);

        // Max speed
        double maxSpeed = snapshots.stream()
            .mapToDouble(s -> Math.hypot(s.drive().vxMps(), s.drive().vyMps()))
            .max().orElse(0);
        summary.put("max_speed_mps", round(maxSpeed));

        // Total distance traveled
        double totalDist = 0;
        for (int i = 1; i < snapshots.size(); i++) {
            double dx = snapshots.get(i).drive().poseX() - snapshots.get(i - 1).drive().poseX();
            double dy = snapshots.get(i).drive().poseY() - snapshots.get(i - 1).drive().poseY();
            totalDist += Math.hypot(dx, dy);
        }
        summary.put("total_distance_m", round(totalDist));

        // Heading drift (difference between start and end if robot should return)
        double headingDelta = last.drive().headingDeg() - first.drive().headingDeg();
        summary.put("heading_change_deg", round(headingDelta));

        // Max module current across all ticks
        double maxCurrent = snapshots.stream()
            .flatMap(s -> s.drive().modules().stream())
            .mapToDouble(ModuleSnapshot::driveCurrentAmps)
            .max().orElse(0);
        summary.put("max_drive_current_amps", round(maxCurrent));

        // Command events summary
        Set<String> commandsRun = new LinkedHashSet<>();
        for (TickSnapshot snap : snapshots) {
            for (CommandEvent event : snap.commands().events()) {
                if ("STARTED".equals(event.eventType())) {
                    commandsRun.add(event.name());
                }
            }
        }
        summary.put("commands_executed", commandsRun);

        summary.put("anomaly_count", report.getAnomalies().size());

        // Trajectory tracking metrics
        if (report.hasTrajectory()) {
            TrajectoryExpectation traj = report.getTrajectory();
            Map<String, Object> tracking = new LinkedHashMap<>();

            double maxError = 0;
            double sumError = 0;
            int errorCount = 0;
            double trajDuration = traj.getTotalDuration();

            for (TickSnapshot snap : snapshots) {
                double t = snap.timestampSeconds();
                if (t > trajDuration) break;
                Pose2d actual = new Pose2d(snap.drive().poseX(), snap.drive().poseY(),
                    edu.wpi.first.math.geometry.Rotation2d.fromDegrees(snap.drive().headingDeg()));
                double error = traj.computeTrackingError(t, actual);
                maxError = Math.max(maxError, error);
                sumError += error;
                errorCount++;
            }

            tracking.put("max_error_m", round(maxError));
            tracking.put("avg_error_m", errorCount > 0 ? round(sumError / errorCount) : 0.0);
            tracking.put("trajectory_duration_s", round(trajDuration));

            Pose2d expectedFinal = traj.getFinalPose();
            tracking.put("expected_final_pose", Map.of(
                "x", round(expectedFinal.getX()),
                "y", round(expectedFinal.getY()),
                "heading_deg", round(expectedFinal.getRotation().getDegrees())
            ));

            double finalPosError = Math.hypot(
                last.drive().poseX() - expectedFinal.getX(),
                last.drive().poseY() - expectedFinal.getY());
            tracking.put("final_position_error_m", round(finalPosError));

            double finalHeadingError = Math.abs(
                last.drive().headingDeg() - expectedFinal.getRotation().getDegrees());
            tracking.put("final_heading_error_deg", round(finalHeadingError));

            summary.put("trajectory_tracking", tracking);
        }

        return summary;
    }

    private static List<Map<String, Object>> serializeSnapshots(List<TickSnapshot> snapshots) {
        List<Map<String, Object>> list = new ArrayList<>();
        for (TickSnapshot snap : snapshots) {
            Map<String, Object> entry = new LinkedHashMap<>();
            entry.put("tick", snap.tick());
            entry.put("t", round(snap.timestampSeconds()));

            // Drive
            Map<String, Object> drive = new LinkedHashMap<>();
            Map<String, Double> pose = new LinkedHashMap<>();
            pose.put("x", round(snap.drive().poseX()));
            pose.put("y", round(snap.drive().poseY()));
            pose.put("heading_deg", round(snap.drive().headingDeg()));
            drive.put("pose", pose);

            Map<String, Double> vel = new LinkedHashMap<>();
            vel.put("vx", round(snap.drive().vxMps()));
            vel.put("vy", round(snap.drive().vyMps()));
            vel.put("omega", round(snap.drive().omegaRadPerSec()));
            drive.put("velocity", vel);

            List<Map<String, Object>> modules = new ArrayList<>();
            for (ModuleSnapshot mod : snap.drive().modules()) {
                Map<String, Object> m = new LinkedHashMap<>();
                m.put("idx", mod.index());
                m.put("speed_radps", round(mod.speedRadPerSec()));
                m.put("angle_deg", round(mod.angleDeg()));
                m.put("drive_amps", round(mod.driveCurrentAmps()));
                m.put("drive_volts", round(mod.driveAppliedVolts()));
                m.put("turn_amps", round(mod.turnCurrentAmps()));
                m.put("turn_volts", round(mod.turnAppliedVolts()));
                modules.add(m);
            }
            drive.put("modules", modules);
            entry.put("drive", drive);

            // Superstructure
            Map<String, Object> superstructure = new LinkedHashMap<>();
            superstructure.put("hood_deg", round(snap.superstructure().hoodAngleDeg()));
            superstructure.put("hood_target_deg", round(snap.superstructure().hoodTargetDeg()));
            superstructure.put("hood_at_target", snap.superstructure().hoodAtTarget());
            superstructure.put("shooter_rpm", round(snap.superstructure().shooterRPM()));
            superstructure.put("shooter_target_rpm", round(snap.superstructure().shooterTargetRPM()));
            superstructure.put("shooter_at_target", snap.superstructure().shooterAtTarget());
            superstructure.put("turret_deg", round(snap.superstructure().turretAngleDeg()));
            superstructure.put("turret_target_deg", round(snap.superstructure().turretTargetDeg()));
            superstructure.put("turret_at_target", snap.superstructure().turretAtTarget());
            superstructure.put("feeder_state", snap.superstructure().feederState());
            superstructure.put("intake_active", snap.superstructure().intakeActive());
            entry.put("superstructure", superstructure);

            // Commands
            Map<String, Object> commands = new LinkedHashMap<>();
            List<String> running = snap.commands().running().stream()
                .map(ScheduledCommandInfo::name).toList();
            commands.put("running", running);
            if (!snap.commands().events().isEmpty()) {
                List<Map<String, Object>> events = new ArrayList<>();
                for (CommandEvent e : snap.commands().events()) {
                    Map<String, Object> ev = new LinkedHashMap<>();
                    ev.put("name", e.name());
                    ev.put("event", e.eventType());
                    ev.put("t", round(e.timestamp()));
                    events.add(ev);
                }
                commands.put("events", events);
            }
            entry.put("commands", commands);

            list.add(entry);
        }
        return list;
    }

    private static double round(double value) {
        return Math.round(value * 1000.0) / 1000.0;
    }

    private static void writeJson(File file, Object data) {
        try (FileWriter writer = new FileWriter(file)) {
            writer.write(GSON.toJson(data));
        } catch (IOException e) {
            throw new RuntimeException("Failed to write diagnostics: " + file.getAbsolutePath(), e);
        }
    }
}
