package frc.robot.utility.Pathing.serialization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.Path;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * Plain-text read/write for a single {@link Path}. Kept dependency-free so it drops onto
 * the robot without adding Gson/Jackson to the classpath. The GUI writes this format into
 * {@code src/main/deploy/pathing/*.path}; the robot loads it at boot.
 *
 * <p>Format:
 * <pre>
 *   PATH v1
 *   startPose x=.. y=.. theta=.. vx=.. vy=.. omega=..
 *   endPose   x=.. y=.. theta=.. vx=.. vy=.. omega=..
 *   startControl x=.. y=.. theta=..
 *   endControl   x=.. y=.. theta=..
 *   limits maxVel=.. maxAt=.. maxOmega=.. maxAlpha=.. maxCentripetal=..
 * </pre>
 */
public final class PathIO {
    private PathIO() {}

    public static void write(Path path, File file,
                             double maxVelocity, double maxAt,
                             double maxOmega, double maxAlpha, double maxCentripetal) throws IOException {
        try (FileWriter w = new FileWriter(file)) {
            w.write("PATH v1\n");
            writeSetpoint(w, "startPose", poseToSetpoint(path.getStartPose(), path.getStartVelocity()));
            writeSetpoint(w, "endPose",   poseToSetpoint(path.getEndPose(),   path.getEndVelocity()));
            writePose(w, "startControl", path.getStartControlHeading());
            writePose(w, "endControl",   path.getEndControlHeading());
            w.write(String.format(
                    "limits maxVel=%.6f maxAt=%.6f maxOmega=%.6f maxAlpha=%.6f maxCentripetal=%.6f%n",
                    maxVelocity, maxAt, maxOmega, maxAlpha, maxCentripetal));
        }
    }

    public static Path read(File file) throws IOException {
        try (BufferedReader r = new BufferedReader(new FileReader(file))) {
            String header = r.readLine();
            if (header == null || !header.startsWith("PATH v1")) {
                throw new IOException("bad header: " + header);
            }
            Setpoint startPose = readSetpoint(r.readLine(), "startPose");
            Setpoint endPose = readSetpoint(r.readLine(), "endPose");
            Pose2d startCtrl = readPose(r.readLine(), "startControl");
            Pose2d endCtrl = readPose(r.readLine(), "endControl");
            Map<String, Double> limits = parseKVs(r.readLine(), "limits");
            return new Path(startPose, endPose, startCtrl, endCtrl,
                    limits.get("maxVel"),
                    limits.get("maxAt"),
                    limits.get("maxOmega"),
                    limits.get("maxAlpha"),
                    limits.get("maxCentripetal"));
        }
    }

    private static void writeSetpoint(FileWriter w, String tag, Setpoint s) throws IOException {
        w.write(String.format("%s x=%.6f y=%.6f theta=%.6f vx=%.6f vy=%.6f omega=%.6f%n",
                tag, s.x, s.y, s.theta, s.vx, s.vy, s.omega));
    }

    private static void writePose(FileWriter w, String tag, Pose2d p) throws IOException {
        w.write(String.format("%s x=%.6f y=%.6f theta=%.6f%n",
                tag, p.getX(), p.getY(), p.getRotation().getRadians()));
    }

    private static Setpoint readSetpoint(String line, String expectTag) {
        Map<String, Double> kv = parseKVs(line, expectTag);
        return new Setpoint(kv.get("x"), kv.get("y"), kv.get("theta"),
                kv.getOrDefault("vx", 0.0), kv.getOrDefault("vy", 0.0), kv.getOrDefault("omega", 0.0));
    }

    private static Pose2d readPose(String line, String expectTag) {
        Map<String, Double> kv = parseKVs(line, expectTag);
        return new Pose2d(kv.get("x"), kv.get("y"),
                new Rotation2d(kv.getOrDefault("theta", 0.0)));
    }

    private static Map<String, Double> parseKVs(String line, String expectTag) {
        if (line == null) throw new IllegalArgumentException("unexpected EOF expecting " + expectTag);
        String[] tokens = line.trim().split("\\s+");
        if (tokens.length == 0 || !tokens[0].equals(expectTag)) {
            throw new IllegalArgumentException("expected " + expectTag + " got: " + line);
        }
        Map<String, Double> out = new HashMap<>();
        for (int i = 1; i < tokens.length; i++) {
            int eq = tokens[i].indexOf('=');
            if (eq <= 0) continue;
            out.put(tokens[i].substring(0, eq), Double.parseDouble(tokens[i].substring(eq + 1)));
        }
        return out;
    }

    private static Setpoint poseToSetpoint(Pose2d p, double speed) {
        // GUI doesn't know the tangent direction precisely — the path's tangent at the
        // endpoint defines it. We persist just the scalar speed; vx/vy get recomputed on
        // load from the Bezier tangent if needed. For now, zero out linear velocity
        // components — the constructor only uses startPose.getSpeed().
        double vx = speed;  // stored as pure magnitude; tangent inferred from handles
        return new Setpoint(p.getX(), p.getY(), p.getRotation().getRadians(), vx, 0.0, 0.0);
    }
}
