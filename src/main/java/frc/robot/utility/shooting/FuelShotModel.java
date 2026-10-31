package frc.robot.utility.shooting;

import frc.robot.utility.shooting.FuelShotConfig;
import frc.robot.Constants.ShotModelConstants;

/** Fresh stationary fixed-hood model driven by measured exit speeds, not motor surface speed. */
public final class FuelShotModel {
    public record MovingFlight(double crossingX, double crossingY, double entryDegrees,
        double time, double risingX, double risingY, boolean reached) {}
    public record Flight(double crossingX, double entryDegrees, double time, double risingX, boolean reached) {}
    public record Shot(double rpm, double minimumRPM, double maximumRPM, double flightSeconds,
                       boolean valid) {
        public static Shot invalid() { return new Shot(0, 0, 0, 0, false); }
    }
    private final LaunchCalibration calibration;
    public record SolveDiagnostics(double rangeMeters, int sampled, int reachedHeight,
        int clearedNearRim, int landedInOpening, int passedEntryAngle, String reason) {}
    private SolveDiagnostics diagnostics = new SolveDiagnostics(Double.NaN, 0, 0, 0, 0, 0, "Not solved yet");
    public SolveDiagnostics diagnostics() { return diagnostics; }
    private double launchSpeedCoefficient = 1.0;

    /** Update launch-speed scale and invalidate all cached trajectory predictions. */
    public boolean setLaunchSpeedCoefficient(double coefficient) {
        if (!Double.isFinite(coefficient) || coefficient < 0.5 || coefficient > 1.5
                || coefficient == launchSpeedCoefficient) return false;
        launchSpeedCoefficient = coefficient;
        sampledFlights.clear();
        for (double rpm : sampledRPM) sampledFlights.add(flight(rpm));
        return true;
    }

    private final java.util.List<Double> sampledRPM = new java.util.ArrayList<>();
    private final java.util.List<Flight> sampledFlights = new java.util.ArrayList<>();
    private static final double DT = 0.005;
    private static final double RADIUS = ShotModelConstants.BALL_DIAMETER_METERS / 2;
    private static final double PLANE = ShotModelConstants.HUB_RIM_HEIGHT_METERS + RADIUS;
    private static final double HALF_WIDTH = ShotModelConstants.HUB_OPENING_SPAN_METERS / 2
        - RADIUS - ShotModelConstants.HUB_RIM_MARGIN_METERS;
    private static final double AERO = 0.5 * 1.225 * Math.PI * RADIUS * RADIUS / ShotModelConstants.BALL_MASS_KG;

    public FuelShotModel(LaunchCalibration calibration) {
        this.calibration = calibration;
        for (double rpm = calibration.minRPM(); ; rpm = Math.min(calibration.maxRPM(), rpm + 10)) {
            sampledRPM.add(rpm);
            sampledFlights.add(flight(rpm));
            if (rpm == calibration.maxRPM()) break;
        }
    }
    public LaunchCalibration calibration() { return calibration; }

    /** Bounded work: scan the calibrated RPM span, then refine the selected contiguous band. */
    public Shot solve(double range) {
        return solve(range, false);
    }

    /** Cache-only estimate for bounded moving-shot lead iterations; final shots use solve(). */
    public Shot estimate(double range) {
        return solve(range, true);
    }

    private Shot solve(double range, boolean cacheOnly) {
        if (!Double.isFinite(range) || range < 0.75 || range > 8.0) {
            diagnostics = new SolveDiagnostics(range, 0, 0, 0, 0, 0, "Range outside solver limits (0.75–8 m)");
            return Shot.invalid();
        }
        // A fine grid finds narrow bands; midpoint refinement below never assumes a global monotonic trajectory.
        double step = 10.0;
        double bestLow = 0, bestHigh = -1, start = 0, previous = calibration.minRPM();
        boolean inBand = false;
        int reached = 0, cleared = 0, landed = 0, entered = 0;
        for (int index = 0; index < sampledRPM.size(); index++) {
            double rpm = sampledRPM.get(index);
            Flight candidate = sampledFlights.get(index);
            boolean valid = flightScores(range, candidate);
            // Progressive counts explain exactly where candidate trajectories are eliminated.
            if (candidate.reached()) {
                reached++;
                if (candidate.risingX() <= range - ShotModelConstants.HUB_OPENING_SPAN_METERS / 2
                        - ShotModelConstants.HUB_RIM_MARGIN_METERS) {
                    cleared++;
                    if (Math.abs(candidate.crossingX() - range) <= HALF_WIDTH) {
                        landed++;
                        if (candidate.entryDegrees() >= FuelShotConfig.MIN_ENTRY_DEGREES) entered++;
                    }
                }
            }
            if (valid && !inBand) { start = rpm; inBand = true; }
            if (inBand && (!valid || rpm == calibration.maxRPM())) {
                double end = valid ? rpm : previous;
                if (end - start > bestHigh - bestLow) { bestLow = start; bestHigh = end; }
                inBand = false;
            }
            if (rpm == calibration.maxRPM()) break;
            previous = rpm;
        }
        String reason = reached == 0 ? "No sampled RPM reaches hub height"
            : cleared == 0 ? "No sampled trajectory clears near rim"
            : landed == 0 ? "No rim-clearing trajectory lands in opening"
            : entered == 0 ? "Entry angle too shallow (requires "
                + FuelShotConfig.MIN_ENTRY_DEGREES + " deg)" : "Reachable";
        diagnostics = new SolveDiagnostics(range, sampledRPM.size(), reached, cleared, landed, entered, reason);
        if (bestHigh < bestLow) return Shot.invalid();
        // Grid endpoints are already valid; avoid dozens of full integrations to refine sub-RPM edges.
        double low = bestLow;
        double high = bestHigh;
        double target = (low + high) / 2;
        if (cacheOnly) {
            int index = Math.min(sampledRPM.size() - 2,
                (int) ((target - calibration.minRPM()) / 10));
            double fraction = (target - sampledRPM.get(index))
                / (sampledRPM.get(index + 1) - sampledRPM.get(index));
            double time = fraction == 0 ? sampledFlights.get(index).time()
                : fraction == 1 ? sampledFlights.get(index + 1).time()
                : sampledFlights.get(index).time() * (1 - fraction)
                    + sampledFlights.get(index + 1).time() * fraction;
            return new Shot(target, low, high, time, Double.isFinite(time));
        }
        Flight flight = flight(target);
        boolean valid = scores(range, target);
        if (!valid) diagnostics = new SolveDiagnostics(range, sampledRPM.size(), reached, cleared,
            landed, entered, "Refined RPM failed trajectory check");
        return new Shot(target, low, high, flight.time(), valid);
    }

    private double refine(double range, double inside, double outside) {
        if (scores(range, outside)) return outside;
        for (int i = 0; i < 12; i++) {
            double mid = (inside + outside) / 2;
            if (scores(range, mid)) inside = mid; else outside = mid;
        }
        return inside;
    }

    public boolean scores(double range, double rpm) {
        if (!Double.isFinite(range) || range < 0.75 || range > 8 || !Double.isFinite(rpm)
                || rpm < calibration.minRPM() || rpm > calibration.maxRPM()) return false;
        Flight f = flight(rpm);
        return flightScores(range, f);
    }

    private boolean flightScores(double range, Flight f) {
        return f.reached() && f.risingX() <= range - ShotModelConstants.HUB_OPENING_SPAN_METERS / 2
            - ShotModelConstants.HUB_RIM_MARGIN_METERS
            && Math.abs(f.crossingX() - range) <= HALF_WIDTH
            && f.entryDegrees() >= FuelShotConfig.MIN_ENTRY_DEGREES;
    }

    /** RK4 gravity + quadratic drag + backspin lift. Positive backspin gives upward lift. */
    public Flight flight(double rpm) {
        LaunchCalibration.Launch base = calibration.at(rpm);
        LaunchCalibration.Launch launch = new LaunchCalibration.Launch(
            base.speed() * launchSpeedCoefficient, base.backspinRps());
        double angle = Math.toRadians(ShotModelConstants.LAUNCH_ELEVATION_DEGREES
            + FuelShotConfig.LAUNCH_ANGLE_OFFSET_DEGREES);
        double spin = Double.isFinite(launch.backspinRps()) ? launch.backspinRps() * 2 * Math.PI
            : launch.speed() / RADIUS; // Explicit rolling estimate until spin is measured.
        double[] s = {0, ShotModelConstants.RELEASE_HEIGHT_METERS,
            launch.speed() * Math.cos(angle), launch.speed() * Math.sin(angle), spin};
        double[] a = new double[5], b = new double[5], c = new double[5], d = new double[5], tmp = new double[5];
        double risingX = Double.POSITIVE_INFINITY;
        for (int n = 0; n < 800; n++) {
            double oldX = s[0], oldZ = s[1], oldVX = s[2], oldVZ = s[3];
            derivative(s, a);
            for (int i = 0; i < 5; i++) tmp[i] = s[i] + a[i] * DT / 2;
            derivative(tmp, b);
            for (int i = 0; i < 5; i++) tmp[i] = s[i] + b[i] * DT / 2;
            derivative(tmp, c);
            for (int i = 0; i < 5; i++) tmp[i] = s[i] + c[i] * DT;
            derivative(tmp, d);
            for (int i = 0; i < 5; i++) s[i] += DT * (a[i] + 2*b[i] + 2*c[i] + d[i]) / 6;
            if (oldZ < PLANE && s[1] >= PLANE) {
                risingX = oldX + (PLANE - oldZ) / (s[1] - oldZ) * (s[0] - oldX);
            }
            if (oldZ >= PLANE && s[1] <= PLANE && s[3] < 0) {
                double t = (oldZ - PLANE) / (oldZ - s[1]);
                double vx = oldVX + t * (s[2] - oldVX), vz = oldVZ + t * (s[3] - oldVZ);
                return new Flight(oldX + t * (s[0] - oldX), Math.toDegrees(Math.atan2(-vz, vx)),
                    (n + t) * DT, risingX, vx > 0);
            }
            if (s[1] < 0 || s[0] > 15) break;
        }
        return new Flight(Double.NaN, Double.NaN, Double.NaN, Double.NaN, false);
    }

    /** Bounded 3D release validation. Drag/lift use air-relative velocity including chassis motion.
     * Heading defines the launch direction and horizontal backspin axis. Stationary solve/flight
     * continue to use the original 2D path and cache without calling this method.
     */
    public MovingFlight movingFlight(double rpm, double heading, double fieldVX, double fieldVY) {
        LaunchCalibration.Launch base = calibration.at(rpm);
        double speed = base.speed() * launchSpeedCoefficient;
        double elevation = Math.toRadians(ShotModelConstants.LAUNCH_ELEVATION_DEGREES
            + FuelShotConfig.LAUNCH_ANGLE_OFFSET_DEGREES);
        double spin = Double.isFinite(base.backspinRps()) ? base.backspinRps() * 2 * Math.PI
            : speed / RADIUS;
        double[] state = {0, 0, ShotModelConstants.RELEASE_HEIGHT_METERS,
            speed * Math.cos(elevation) * Math.cos(heading) + fieldVX,
            speed * Math.cos(elevation) * Math.sin(heading) + fieldVY,
            speed * Math.sin(elevation), spin};
        double[] a = new double[7], b = new double[7], c = new double[7], d = new double[7], temp = new double[7];
        double axisX = Math.sin(heading), axisY = -Math.cos(heading);
        double risingX = Double.NaN, risingY = Double.NaN;
        for (int n = 0; n < 800; n++) {
            double oldX = state[0], oldY = state[1], oldZ = state[2];
            double oldVX = state[3], oldVY = state[4], oldVZ = state[5];
            movingDerivative(state, a, axisX, axisY);
            for (int i = 0; i < 7; i++) temp[i] = state[i] + a[i] * DT / 2;
            movingDerivative(temp, b, axisX, axisY);
            for (int i = 0; i < 7; i++) temp[i] = state[i] + b[i] * DT / 2;
            movingDerivative(temp, c, axisX, axisY);
            for (int i = 0; i < 7; i++) temp[i] = state[i] + c[i] * DT;
            movingDerivative(temp, d, axisX, axisY);
            for (int i = 0; i < 7; i++) state[i] += DT * (a[i] + 2*b[i] + 2*c[i] + d[i]) / 6;
            if (oldZ < PLANE && state[2] >= PLANE) {
                double fraction = (PLANE - oldZ) / (state[2] - oldZ);
                risingX = oldX + fraction * (state[0] - oldX);
                risingY = oldY + fraction * (state[1] - oldY);
            }
            if (oldZ >= PLANE && state[2] <= PLANE && state[5] < 0) {
                double fraction = (oldZ - PLANE) / (oldZ - state[2]);
                double vx = oldVX + fraction * (state[3] - oldVX);
                double vy = oldVY + fraction * (state[4] - oldVY);
                double vz = oldVZ + fraction * (state[5] - oldVZ);
                return new MovingFlight(oldX + fraction * (state[0] - oldX),
                    oldY + fraction * (state[1] - oldY),
                    Math.toDegrees(Math.atan2(-vz, Math.hypot(vx, vy))), (n + fraction) * DT,
                    risingX, risingY, Double.isFinite(risingX) && Double.isFinite(risingY));
            }
            if (state[2] < 0 || Math.hypot(state[0], state[1]) > 15) break;
        }
        return new MovingFlight(Double.NaN, Double.NaN, Double.NaN, Double.NaN,
            Double.NaN, Double.NaN, false);
    }

    private static void movingDerivative(double[] state, double[] out, double axisX, double axisY) {
        double speed = Math.sqrt(state[3]*state[3] + state[4]*state[4] + state[5]*state[5]);
        double drag = AERO * FuelShotConfig.DRAG_COEFFICIENT * speed;
        double cl = speed < 1e-6 ? 0 : Math.min(0.35,
            FuelShotConfig.LIFT_SLOPE * RADIUS * Math.abs(state[6]) / speed);
        double lift = AERO * cl * speed * Math.signum(state[6]);
        out[0] = state[3]; out[1] = state[4]; out[2] = state[5];
        out[3] = -drag * state[3] + lift * axisY * state[5];
        out[4] = -drag * state[4] - lift * axisX * state[5];
        out[5] = -9.80665 - drag * state[5] + lift * (axisX * state[4] - axisY * state[3]);
        out[6] = -FuelShotConfig.SPIN_DECAY_PER_SECOND * state[6];
    }

    private static void derivative(double[] s, double[] out) {
        double speed = Math.hypot(s[2], s[3]);
        double drag = AERO * FuelShotConfig.DRAG_COEFFICIENT * speed;
        double cl = speed < 1e-6 ? 0 : Math.min(0.35,
            FuelShotConfig.LIFT_SLOPE * RADIUS * Math.abs(s[4]) / speed);
        double lift = AERO * cl * speed * Math.signum(s[4]);
        out[0] = s[2]; out[1] = s[3];
        out[2] = -drag * s[2] - lift * s[3];
        out[3] = -9.80665 - drag * s[3] + lift * s[2];
        out[4] = -FuelShotConfig.SPIN_DECAY_PER_SECOND * s[4];
    }
}
