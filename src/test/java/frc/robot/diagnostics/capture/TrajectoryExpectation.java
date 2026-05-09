package frc.robot.diagnostics.capture;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryExpectation {
    private final List<TimestampedPose> samples;
    private final String name;

    public record TimestampedPose(double t, double x, double y, double headingRad) {
        public Pose2d toPose2d() {
            return new Pose2d(x, y, new Rotation2d(headingRad));
        }
    }

    public TrajectoryExpectation(String name, List<TimestampedPose> samples) {
        this.name = name;
        this.samples = samples;
    }

    public static TrajectoryExpectation fromFile(String trajectoryName) {
        File trajFile = new File(Filesystem.getDeployDirectory(), "choreo/" + trajectoryName + ".traj");
        if (!trajFile.exists()) {
            return new TrajectoryExpectation(trajectoryName, List.of());
        }

        try (FileReader reader = new FileReader(trajFile)) {
            JsonObject root = new Gson().fromJson(reader, JsonObject.class);
            JsonArray samplesArray = root.getAsJsonObject("trajectory").getAsJsonArray("samples");

            List<TimestampedPose> samples = new ArrayList<>();
            for (JsonElement elem : samplesArray) {
                JsonObject s = elem.getAsJsonObject();
                samples.add(new TimestampedPose(
                    s.get("t").getAsDouble(),
                    s.get("x").getAsDouble(),
                    s.get("y").getAsDouble(),
                    s.get("heading").getAsDouble()
                ));
            }
            return new TrajectoryExpectation(trajectoryName, samples);
        } catch (Exception e) {
            return new TrajectoryExpectation(trajectoryName, List.of());
        }
    }

    public static TrajectoryExpectation chain(TrajectoryExpectation... segments) {
        List<TimestampedPose> all = new ArrayList<>();
        double timeOffset = 0;
        for (TrajectoryExpectation seg : segments) {
            for (TimestampedPose sample : seg.samples) {
                all.add(new TimestampedPose(
                    sample.t + timeOffset,
                    sample.x, sample.y, sample.headingRad
                ));
            }
            if (!seg.samples.isEmpty()) {
                timeOffset = all.get(all.size() - 1).t;
            }
        }
        return new TrajectoryExpectation("chained", all);
    }

    public Pose2d getInitialPose() {
        if (samples.isEmpty()) return new Pose2d();
        return samples.get(0).toPose2d();
    }

    public Pose2d getFinalPose() {
        if (samples.isEmpty()) return new Pose2d();
        return samples.get(samples.size() - 1).toPose2d();
    }

    public double getTotalDuration() {
        if (samples.isEmpty()) return 0;
        return samples.get(samples.size() - 1).t;
    }

    public Pose2d interpolateAtTime(double t) {
        if (samples.isEmpty()) return new Pose2d();
        if (t <= samples.get(0).t) return samples.get(0).toPose2d();
        if (t >= samples.get(samples.size() - 1).t) return samples.get(samples.size() - 1).toPose2d();

        for (int i = 1; i < samples.size(); i++) {
            if (samples.get(i).t >= t) {
                TimestampedPose a = samples.get(i - 1);
                TimestampedPose b = samples.get(i);
                double alpha = (t - a.t) / (b.t - a.t);
                return new Pose2d(
                    a.x + alpha * (b.x - a.x),
                    a.y + alpha * (b.y - a.y),
                    new Rotation2d(a.headingRad + alpha * (b.headingRad - a.headingRad))
                );
            }
        }
        return samples.get(samples.size() - 1).toPose2d();
    }

    public double computeTrackingError(double t, Pose2d actualPose) {
        Pose2d expected = interpolateAtTime(t);
        return actualPose.getTranslation().getDistance(expected.getTranslation());
    }

    public String getName() { return name; }
    public List<TimestampedPose> getSamples() { return samples; }
    public boolean isEmpty() { return samples.isEmpty(); }
}
