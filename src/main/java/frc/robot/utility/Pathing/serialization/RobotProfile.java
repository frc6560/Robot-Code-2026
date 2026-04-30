package frc.robot.utility.Pathing.serialization;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * Kinematic limits and footprint used as defaults when the pathing library plans a new
 * trajectory. Persisted at {@code deploy/pathing/robot-profile.json}.
 *
 * <p>These are the numbers that make or break a path: wrong limits and the robot can't
 * follow its own trajectory. Tune from SysID data (for velocity / accel caps) and from
 * measurement (for footprint).
 *
 * <p>Non-goals: PID gains, feedforward constants, SysID results. Those live with the
 * drivetrain; the pathing library has no business with them.
 */
public final class RobotProfile {
    public static final String FILENAME = "pathing/robot-profile.json";

    /** Default profile — conservative numbers for a typical FRC swerve. Overridden by
     *  whatever is present on disk at startup. */
    public static final RobotProfile DEFAULT = new RobotProfile(
            5.0,              // maxVelocity m/s
            4.0,              // maxAccel m/s^2
            Math.PI,          // maxOmega rad/s
            2 * Math.PI,      // maxAlpha rad/s^2
            3.0,              // maxCentripetal m/s^2
            0.812,            // footprintLength m
            0.812             // footprintWidth m
    );

    public final double maxVelocity;
    public final double maxAccel;
    public final double maxOmega;
    public final double maxAlpha;
    public final double maxCentripetal;
    public final double footprintLength;
    public final double footprintWidth;

    public RobotProfile(double maxVelocity, double maxAccel, double maxOmega, double maxAlpha,
                        double maxCentripetal, double footprintLength, double footprintWidth) {
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;
        this.maxOmega = maxOmega;
        this.maxAlpha = maxAlpha;
        this.maxCentripetal = maxCentripetal;
        this.footprintLength = footprintLength;
        this.footprintWidth = footprintWidth;
    }

    private static volatile RobotProfile INSTANCE;

    public static RobotProfile getInstance() {
        RobotProfile local = INSTANCE;
        if (local == null) {
            synchronized (RobotProfile.class) {
                local = INSTANCE;
                if (local == null) {
                    local = tryLoad();
                    INSTANCE = local;
                }
            }
        }
        return local;
    }

    /** Always returns a valid profile. Probes the source-tree path first, then the robot's
     *  deploy directory. The deploy-directory lookup is done reflectively — referencing
     *  {@code edu.wpi.first.wpilibj.Filesystem} textually would trigger the JNI loader
     *  chain and crash the GUI on a laptop. By reflection, the class is only resolved
     *  when we actually call into it (robot-only). */
    private static RobotProfile tryLoad() {
        // Source-tree probe — used by the Swing GUI on a laptop.
        File srcFile = new File("src/main/deploy/" + FILENAME);
        if (srcFile.exists()) {
            try { return read(srcFile); }
            catch (IOException e) {
                System.err.println("[RobotProfile] failed to load " + srcFile + ": " + e);
            }
        }
        // Common FRC deploy locations on the roboRIO. These are absolute so they work
        // without loading any WPILib class.
        String[] roboRioCandidates = {
                "/home/lvuser/deploy/" + FILENAME,
                "/home/lvuser/robot-deploy/" + FILENAME
        };
        for (String path : roboRioCandidates) {
            File f = new File(path);
            if (f.exists()) {
                try { return read(f); }
                catch (IOException e) {
                    System.err.println("[RobotProfile] failed to load " + f + ": " + e);
                }
            }
        }
        return DEFAULT;
    }

    /** Force a new profile into the singleton slot. Useful for GUI previews and tests. */
    public static void setInstance(RobotProfile profile) {
        synchronized (RobotProfile.class) {
            INSTANCE = profile;
        }
    }

    public void write(File file) throws IOException {
        File parent = file.getParentFile();
        if (parent != null && !parent.exists()) parent.mkdirs();
        try (FileWriter w = new FileWriter(file)) {
            w.write("{\n");
            w.write(String.format(Locale.ROOT, "  \"maxVelocity\": %.6f,%n", maxVelocity));
            w.write(String.format(Locale.ROOT, "  \"maxAccel\": %.6f,%n", maxAccel));
            w.write(String.format(Locale.ROOT, "  \"maxOmega\": %.6f,%n", maxOmega));
            w.write(String.format(Locale.ROOT, "  \"maxAlpha\": %.6f,%n", maxAlpha));
            w.write(String.format(Locale.ROOT, "  \"maxCentripetal\": %.6f,%n", maxCentripetal));
            w.write(String.format(Locale.ROOT, "  \"footprintLength\": %.6f,%n", footprintLength));
            w.write(String.format(Locale.ROOT, "  \"footprintWidth\": %.6f%n", footprintWidth));
            w.write("}\n");
        }
    }

    public static RobotProfile read(File file) throws IOException {
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) sb.append(line).append('\n');
            String src = sb.toString();
            return new RobotProfile(
                    num(src, "maxVelocity", DEFAULT.maxVelocity),
                    num(src, "maxAccel", DEFAULT.maxAccel),
                    num(src, "maxOmega", DEFAULT.maxOmega),
                    num(src, "maxAlpha", DEFAULT.maxAlpha),
                    num(src, "maxCentripetal", DEFAULT.maxCentripetal),
                    num(src, "footprintLength", DEFAULT.footprintLength),
                    num(src, "footprintWidth", DEFAULT.footprintWidth));
        }
    }

    /** Lenient number extractor — missing fields fall back to {@code fallback} rather than
     *  failing the whole load, so an older profile can still be opened. */
    private static double num(String src, String key, double fallback) {
        int i = src.indexOf('"' + key + '"');
        if (i < 0) return fallback;
        int colon = src.indexOf(':', i);
        if (colon < 0) return fallback;
        int start = colon + 1;
        while (start < src.length() && Character.isWhitespace(src.charAt(start))) start++;
        int end = start;
        while (end < src.length()) {
            char ch = src.charAt(end);
            if (Character.isDigit(ch) || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E') end++;
            else break;
        }
        if (end == start) return fallback;
        try { return Double.parseDouble(src.substring(start, end)); }
        catch (NumberFormatException e) { return fallback; }
    }
}
