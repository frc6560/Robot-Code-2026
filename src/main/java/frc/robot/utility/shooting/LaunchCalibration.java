package frc.robot.utility.shooting;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import frc.robot.Constants.ShooterConstants;

/** Measured launch conditions at one fixed tunnel speed. Never extrapolates. */
public final class LaunchCalibration {
    public record Point(double rpm, double speed, double backspinRps) {}
    public record Launch(double speed, double backspinRps) {}
    private final List<Point> points;
    private final double tunnelRPM;

    public LaunchCalibration(List<Point> points, double tunnelRPM) {
        if (points.size() < 3 || !Double.isFinite(tunnelRPM) || tunnelRPM <= 0 || tunnelRPM > 3000) {
            throw new IllegalArgumentException("Need at least 3 RPM points and tunnel RPM in (0, 3000]");
        }
        this.points = List.copyOf(points);
        this.tunnelRPM = tunnelRPM;
        boolean measuredSpin = Double.isFinite(points.get(0).backspinRps());
        double lastRPM = 0, lastSpeed = 0;
        for (Point p : points) {
            if (!Double.isFinite(p.rpm()) || !Double.isFinite(p.speed())
                    || p.rpm() <= lastRPM || p.rpm() < ShooterConstants.FLYWHEEL_IDLE_RPM
                    || p.rpm() > ShooterConstants.MAX_RPM || p.speed() <= lastSpeed
                    || p.speed() > 30 || Double.isInfinite(p.backspinRps())
                    || measuredSpin != Double.isFinite(p.backspinRps())
                    || (measuredSpin && Math.abs(p.backspinRps()) > 200)) {
                throw new IllegalArgumentException("Invalid/nonmonotonic calibration or mixed measured/missing spin");
            }
            lastRPM = p.rpm();
            lastSpeed = p.speed();
        }
    }

    public static LaunchCalibration load(Path path) throws IOException {
        List<String> lines = Files.readAllLines(path);
        if (lines.isEmpty() || !lines.get(0).trim().equals("rpm,exit_speed_mps,backspin_rps,tunnel_rpm")) {
            throw new IllegalArgumentException("Unexpected launch calibration CSV header");
        }
        List<Point> points = new ArrayList<>();
        double tunnel = Double.NaN;
        for (int i = 1; i < lines.size(); i++) {
            if (lines.get(i).isBlank()) continue;
            String[] cells = lines.get(i).split(",", -1);
            if (cells.length != 4) throw new IllegalArgumentException("Invalid CSV row " + (i + 1));
            double rowTunnel = Double.parseDouble(cells[3]);
            if (!Double.isNaN(tunnel) && rowTunnel != tunnel) throw new IllegalArgumentException("Tunnel RPM must be fixed");
            tunnel = rowTunnel;
            points.add(new Point(Double.parseDouble(cells[0]), Double.parseDouble(cells[1]),
                cells[2].isBlank() ? Double.NaN : Double.parseDouble(cells[2])));
        }
        return new LaunchCalibration(points, tunnel);
    }

    public double minRPM() { return points.get(0).rpm(); }
    public double maxRPM() { return points.get(points.size() - 1).rpm(); }
    public double tunnelRPM() { return tunnelRPM; }
    public boolean hasMeasuredSpin() { return Double.isFinite(points.get(0).backspinRps()); }

    public Launch at(double rpm) {
        if (!Double.isFinite(rpm) || rpm < minRPM() || rpm > maxRPM()) {
            throw new IllegalArgumentException("RPM outside measured calibration");
        }
        for (int i = 1; i < points.size(); i++) {
            Point a = points.get(i - 1), b = points.get(i);
            if (rpm <= b.rpm()) {
                double t = (rpm - a.rpm()) / (b.rpm() - a.rpm());
                return new Launch(a.speed() + t * (b.speed() - a.speed()),
                    hasMeasuredSpin() ? a.backspinRps() + t * (b.backspinRps() - a.backspinRps()) : Double.NaN);
            }
        }
        throw new IllegalStateException("Missing calibration interval");
    }
}
