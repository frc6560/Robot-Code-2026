package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/** Headless-only 2026 wall/hub collision checks. Does not model ramps, trenches or deployed intake. */
public final class CompetitionAutoDiagnostic {
    private final StringBuilder trace = new StringBuilder(
        "time,x,y,heading,wall_clearance_model,wall_clearance_0812,hub_clearance,shooting,feeding,speed\n");
    private double minWall = Double.POSITIVE_INFINITY, minConservativeWall = Double.POSITIVE_INFINITY;
    private double minHub = Double.POSITIVE_INFINITY;
    private double firstShot = -1, feedingSeconds;
    private int wallContactSamples, hubContactSamples;
    private boolean finished;

    public static boolean requested() {
        String selection = System.getenv("BLINE_HEADLESS_AUTO");
        return "1".equals(System.getenv("BLINE_HEADLESS_DIAGNOSTIC")) && selection != null
            && selection.matches("(?i)(blue|red)-(left|center|right)");
    }

    /** Spawn clear of obstacles, at the selected first waypoint, before physics starts. */
    public static Pose2d initialPose() {
        String choice = System.getenv("BLINE_HEADLESS_AUTO");
        String source = choice.endsWith("left") ? "non-human" : choice.endsWith("right") ? "human" : "hub";
        var path = frc.robot.autonomous.AutoCommands.competitionPath(source, choice.startsWith("red-"));
        var first = (frc.robot.lib.BLine.Path.Waypoint) path.getElement(0);
        return new Pose2d(first.translationTarget().translation(), first.rotationTarget().rotation());
    }

    /** Must run before constructing SwerveSubsystem, which registers its body in this arena. */
    public static void configureArena() { SimulatedArena.overrideInstance(new WallHubArena()); }

    private static final class WallHubArena extends SimulatedArena {
        private static final class Map extends FieldMap {
            Map() {
                double l = FieldConstants.FIELD_LENGTH, w = FieldConstants.FIELD_WIDTH;
                addBorderLine(new Translation2d(), new Translation2d(l, 0));
                addBorderLine(new Translation2d(l, 0), new Translation2d(l, w));
                addBorderLine(new Translation2d(l, w), new Translation2d(0, w));
                addBorderLine(new Translation2d(0, w), new Translation2d());
                for (Translation2d hub : new Translation2d[] {
                        FieldConstants.BLUE_HUB_CENTER, FieldConstants.RED_HUB_CENTER}) {
                    addRectangularObstacle(47 * 0.0254, 47 * 0.0254, new Pose2d(hub, new Rotation2d()));
                }
            }
        }
        WallHubArena() { super(new Map()); }
        @Override public void placeGamePiecesOnField() {}
    }

    public void sample(double seconds, SwerveSubsystem drive, boolean shooting, boolean feeding) {
        if (finished) return;
        Pose2d pose = drive.getSwerveDrive().getSimulationDriveTrainPose().orElse(drive.getPose());
        // These are the exact bumper dimensions YAGSL uses for its MapleSim body.
        var config = drive.getSwerveDrive().swerveDriveConfiguration;
        double length = config.getTracklength() + 0.127, width = config.getTrackwidth() + 0.127;
        double wall = wallClearance(pose, length, width);
        double conservativeWall = wallClearance(pose, 0.812, 0.812);
        double hub = Math.min(hubSeparation(pose, length, width, FieldConstants.BLUE_HUB_CENTER),
            hubSeparation(pose, length, width, FieldConstants.RED_HUB_CENTER));
        minWall = Math.min(minWall, wall);
        minConservativeWall = Math.min(minConservativeWall, conservativeWall);
        minHub = Math.min(minHub, hub);
        if (wall <= 0.003) wallContactSamples++;
        if (hub <= 0.003) hubContactSamples++;
        if (shooting && firstShot < 0) firstShot = seconds;
        if (feeding) feedingSeconds += 0.02;
        var velocity = drive.getRobotVelocity();
        trace.append(String.format(Locale.ROOT, "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%s,%.6f%n",
            seconds, pose.getX(), pose.getY(), pose.getRotation().getRadians(), wall, conservativeWall,
            hub, shooting, feeding, Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond)));
    }

    private static double wallClearance(Pose2d pose, double length, double width) {
        double c = Math.abs(pose.getRotation().getCos()), s = Math.abs(pose.getRotation().getSin());
        double halfX = (length * c + width * s) / 2, halfY = (length * s + width * c) / 2;
        return Math.min(Math.min(pose.getX() - halfX, FieldConstants.FIELD_LENGTH - pose.getX() - halfX),
            Math.min(pose.getY() - halfY, FieldConstants.FIELD_WIDTH - pose.getY() - halfY));
    }

    /** Separating-axis clearance for the oriented bumper rectangle and axis-aligned hub square. */
    private static double hubSeparation(Pose2d pose, double length, double width, Translation2d hub) {
        double c = pose.getRotation().getCos(), s = pose.getRotation().getSin();
        double dx = hub.getX() - pose.getX(), dy = hub.getY() - pose.getY();
        double h = 47 * 0.0254 / 2;
        double separation = Double.NEGATIVE_INFINITY;
        for (double[] axis : new double[][] {{1, 0}, {0, 1}, {c, s}, {-s, c}}) {
            double bumperRadius = length / 2 * Math.abs(axis[0] * c + axis[1] * s)
                + width / 2 * Math.abs(-axis[0] * s + axis[1] * c);
            double hubRadius = h * (Math.abs(axis[0]) + Math.abs(axis[1]));
            separation = Math.max(separation,
                Math.abs(dx * axis[0] + dy * axis[1]) - bumperRadius - hubRadius);
        }
        return separation;
    }

    public void finish(String selection) {
        if (finished) return;
        finished = true;
        String report = String.format(Locale.ROOT,
            "selection=%s minWallMeters=%.5f minWall0812Meters=%.5f minHubSeparationMeters=%.5f "
                + "wallContactSamples=%d hubContactSamples=%d firstShotSeconds=%.3f feedingSeconds=%.3f%n",
            selection, minWall, minConservativeWall, minHub, wallContactSamples, hubContactSamples, firstShot, feedingSeconds);
        System.out.print("COMPETITION_SIM_RESULT: " + report);
        String output = System.getenv("BLINE_COMPARE_LOG_DIR");
        if (output != null) {
            try {
                Files.createDirectories(Path.of(output));
                Files.writeString(Path.of(output, "trace.csv"), trace);
                Files.writeString(Path.of(output, "summary.txt"), report);
            } catch (java.io.IOException e) { throw new RuntimeException(e); }
        }
    }
}
