package frc.robot.utility.Shooter;

import java.util.LinkedHashMap;
import java.util.Map;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotModelConstants;

/**
 * Solves shots for the single-flywheel, fixed-hood shooter with the drag and Magnus model used by
 * the shot-calibrator app. The launch angle is fixed, so flywheel RPM is the only control: each
 * distance has a band of RPMs that score, and the solver commands the middle of it.
 */
public final class PhysicsShotSolver {
    public record Solution(
        double targetDistanceMeters,
        double flywheelRPM,
        double launchElevationDegrees,
        double timeOfFlightSeconds,
        double entryAngleDegrees,
        double crossingXMeters,
        double openingClearanceMeters,
        double nearRimClearanceMeters,
        double rpmToleranceLower,
        double rpmToleranceUpper,
        boolean valid
    ) {}

    /** A pass: the RPM that lands the ball on the carpet at the given distance, and when. */
    public record PassSolution(double distanceMeters, double flywheelRPM, double timeOfFlightSeconds, boolean valid) {}

    /** Where the ball first falls through a given height, and what it looked like there. */
    private record Crossing(
        double timeSeconds,
        double xMeters,
        double vxMetersPerSecond,
        double vzMetersPerSecond,
        double probeHeightMeters
    ) {}

    private static final int BISECTION_STEPS = 18;
    private static final double SEARCH_DT_SECONDS = 0.006;
    private static final double REFINEMENT_DT_SECONDS = 0.004;
    private static final double MAX_TIME_SECONDS = 4.0;
    private static final double CACHE_RESOLUTION_METERS = 0.025;
    private static final int MAX_CACHE_ENTRIES = 192;

    private static final double BALL_RADIUS_METERS = ShotModelConstants.BALL_DIAMETER_METERS / 2.0;
    private static final double BALL_AREA_SQUARE_METERS = Math.PI * BALL_RADIUS_METERS * BALL_RADIUS_METERS;
    private static final double DYNAMIC_ACCELERATION_FACTOR =
        0.5
            * ShotModelConstants.AIR_DENSITY_KG_PER_CUBIC_METER
            * BALL_AREA_SQUARE_METERS
            / ShotModelConstants.BALL_MASS_KG;

    private final Map<Long, Solution> solutionCache = new LinkedHashMap<>(32, 0.75f, true) {
        @Override
        protected boolean removeEldestEntry(Map.Entry<Long, Solution> eldest) {
            return size() > MAX_CACHE_ENTRIES;
        }
    };

    /**
     * Searches for the middle of the scoring RPM band at this range. Too slow for every loop; the
     * robot uses {@link #solveRuntime(double)}, and this is what that policy is checked against.
     * The band search is cached in 2.5 cm range bins and the result re-checked at the exact range.
     */
    public synchronized Solution solve(double distanceMeters) {
        if (!isDistanceInRange(distanceMeters)) {
            return invalidSolution(distanceMeters);
        }

        long cacheKey = Math.round(distanceMeters / CACHE_RESOLUTION_METERS);
        // Clamp: a bin center can land a hair outside the range in floating point (242 * 0.025).
        double binDistance = clamp(
            cacheKey * CACHE_RESOLUTION_METERS,
            ShotModelConstants.MIN_DISTANCE_METERS,
            ShotModelConstants.MAX_DISTANCE_METERS);
        Solution cached = solutionCache.computeIfAbsent(cacheKey, ignored -> solveBand(binDistance));
        if (!cached.valid()) {
            return cached;
        }
        Solution exact = evaluate(distanceMeters, cached.flywheelRPM());
        if (exact.valid()) {
            return withTolerances(exact, cached);
        }
        // A range-bin boundary can put the cached command just outside the band; solve exactly.
        return solveBand(distanceMeters);
    }

    /**
     * Constant-time robot-loop policy fitted to {@link #solve(double)} over the full range. The
     * command is re-simulated here, so flight time and validity still come from the equation.
     */
    public Solution solveRuntime(double distanceMeters) {
        if (!isDistanceInRange(distanceMeters)) {
            return invalidSolution(distanceMeters);
        }
        double rpm = (ShotModelConstants.RPM_POLICY_DISTANCE_SQUARED * distanceMeters
                + ShotModelConstants.RPM_POLICY_DISTANCE) * distanceMeters
            + ShotModelConstants.RPM_POLICY_CONSTANT;
        rpm = clamp(rpm, ShooterConstants.FLYWHEEL_IDLE_RPM, ShooterConstants.MAX_RPM);
        return evaluate(distanceMeters, rpm);
    }

    public static double launchElevationDegrees() {
        return ShotModelConstants.LAUNCH_ELEVATION_DEGREES + ShotModelConstants.HOOD_OFFSET_DEGREES;
    }

    /** Evaluates one RPM with the full equation without searching. */
    public Solution evaluate(double distanceMeters, double flywheelRPM) {
        if (!isDistanceInRange(distanceMeters)
                || !Double.isFinite(flywheelRPM)
                || flywheelRPM < ShooterConstants.FLYWHEEL_IDLE_RPM
                || flywheelRPM > ShooterConstants.MAX_RPM) {
            return invalidSolution(distanceMeters);
        }

        double usableHalfSpan = usableHalfSpanMeters();
        double nearRimX = distanceMeters - ShotModelConstants.HUB_OPENING_SPAN_METERS / 2.0;
        Crossing crossing = fallThrough(
            flywheelRPM,
            ShotModelConstants.HUB_BALL_CENTER_HEIGHT_METERS,
            nearRimX,
            maxRangeMeters(distanceMeters),
            REFINEMENT_DT_SECONDS
        );
        if (crossing == null) {
            return commandWithoutEntry(distanceMeters, flywheelRPM);
        }

        double centerOffset = Math.abs(crossing.xMeters() - distanceMeters);
        // Falling through the plane before reaching the near rim means the ball hit the HUB's side.
        double nearRimClearance = Double.isNaN(crossing.probeHeightMeters())
            ? Double.NEGATIVE_INFINITY
            : crossing.probeHeightMeters() - ShotModelConstants.HUB_BALL_CENTER_HEIGHT_METERS;
        double entryAngle = Math.toDegrees(
            Math.atan2(-crossing.vzMetersPerSecond(), Math.max(Math.abs(crossing.vxMetersPerSecond()), 1e-9)));
        boolean scoring = usableHalfSpan > 0.0
            && crossing.vxMetersPerSecond() > 0.0
            && centerOffset <= usableHalfSpan
            && nearRimClearance >= ShotModelConstants.HUB_RIM_MARGIN_METERS
            && entryAngle >= ShotModelConstants.MIN_ENTRY_ANGLE_DEGREES;

        return new Solution(
            distanceMeters,
            flywheelRPM,
            launchElevationDegrees(),
            crossing.timeSeconds(),
            entryAngle,
            crossing.xMeters(),
            usableHalfSpan - centerOffset,
            nearRimClearance,
            0.0,
            0.0,
            scoring
        );
    }

    /** RPM that lands the ball on the carpet at this distance; invalid past the flywheel's reach. */
    public PassSolution solvePass(double distanceMeters) {
        if (!Double.isFinite(distanceMeters) || distanceMeters <= 0.0) {
            return new PassSolution(distanceMeters, ShooterConstants.FLYWHEEL_IDLE_RPM, Double.NaN, false);
        }
        double maxRange = Math.max(20.0, 1.5 * distanceMeters);
        double low = ShooterConstants.FLYWHEEL_IDLE_RPM;
        double high = ShooterConstants.MAX_RPM;
        if (landingDistance(high, maxRange) < distanceMeters || landingDistance(low, maxRange) > distanceMeters) {
            return new PassSolution(distanceMeters, ShooterConstants.FLYWHEEL_IDLE_RPM, Double.NaN, false);
        }
        for (int i = 0; i < BISECTION_STEPS; i++) {
            double middle = 0.5 * (low + high);
            if (landingDistance(middle, maxRange) < distanceMeters) {
                low = middle;
            } else {
                high = middle;
            }
        }
        double rpm = 0.5 * (low + high);
        Crossing landing = fallThrough(rpm, BALL_RADIUS_METERS, Double.NaN, maxRange, REFINEMENT_DT_SECONDS);
        if (landing == null) {
            return new PassSolution(distanceMeters, ShooterConstants.FLYWHEEL_IDLE_RPM, Double.NaN, false);
        }
        return new PassSolution(distanceMeters, rpm, landing.timeSeconds(), true);
    }

    private Solution solveBand(double distanceMeters) {
        double low = ShooterConstants.FLYWHEEL_IDLE_RPM;
        double high = ShooterConstants.MAX_RPM;
        double maxRange = maxRangeMeters(distanceMeters);
        double planeHeight = ShotModelConstants.HUB_BALL_CENTER_HEIGHT_METERS;

        // Where the falling ball crosses the scoring plane moves out monotonically with speed,
        // which is what makes the speed that lands on the HUB center bisectable.
        if (planeCrossingX(high, planeHeight, maxRange) < distanceMeters) {
            return invalidSolution(distanceMeters);
        }
        double lower = low;
        double upper = high;
        for (int i = 0; i < BISECTION_STEPS; i++) {
            double middle = 0.5 * (lower + upper);
            if (planeCrossingX(middle, planeHeight, maxRange) < distanceMeters) {
                lower = middle;
            } else {
                upper = middle;
            }
        }
        double nominal = 0.5 * (lower + upper);
        if (!scores(distanceMeters, nominal)) {
            return invalidSolution(distanceMeters);
        }

        double bandLow = bisectEdge(distanceMeters, nominal, low);
        double bandHigh = bisectEdge(distanceMeters, nominal, high);
        double rpm = 0.5 * (bandLow + bandHigh);
        Solution solution = evaluate(distanceMeters, rpm);
        if (!solution.valid()) {
            solution = evaluate(distanceMeters, nominal);
            rpm = nominal;
        }
        if (!solution.valid()) {
            return invalidSolution(distanceMeters);
        }
        return new Solution(
            solution.targetDistanceMeters(),
            solution.flywheelRPM(),
            solution.launchElevationDegrees(),
            solution.timeOfFlightSeconds(),
            solution.entryAngleDegrees(),
            solution.crossingXMeters(),
            solution.openingClearanceMeters(),
            solution.nearRimClearanceMeters(),
            rpm - bandLow,
            bandHigh - rpm,
            true
        );
    }

    /** Walks the boundary between a scoring RPM and a missing one. */
    private double bisectEdge(double distanceMeters, double inside, double outside) {
        for (int i = 0; i < BISECTION_STEPS; i++) {
            double middle = 0.5 * (inside + outside);
            if (scores(distanceMeters, middle)) {
                inside = middle;
            } else {
                outside = middle;
            }
        }
        return inside;
    }

    private boolean scores(double distanceMeters, double rpm) {
        return evaluate(distanceMeters, rpm).valid();
    }

    /** Range at which the falling ball crosses this height; -inf if it never gets that high. */
    private static double planeCrossingX(double rpm, double heightMeters, double maxRange) {
        Crossing crossing = fallThrough(rpm, heightMeters, Double.NaN, maxRange, SEARCH_DT_SECONDS);
        if (crossing != null) {
            return crossing.xMeters();
        }
        return reachesHeight(rpm, heightMeters) ? Double.POSITIVE_INFINITY : Double.NEGATIVE_INFINITY;
    }

    private static double landingDistance(double rpm, double maxRange) {
        return planeCrossingX(rpm, BALL_RADIUS_METERS, maxRange);
    }

    private static boolean reachesHeight(double rpm, double heightMeters) {
        double[] release = releaseState(rpm);
        double vz = release[3];
        double apex = ShotModelConstants.RELEASE_HEIGHT_METERS
            + vz * vz / (2.0 * ShotModelConstants.GRAVITY_METERS_PER_SECOND_SQUARED);
        // Drag only lowers the apex and lift only raises it a little; this just separates a ball
        // that flew past maxRange still climbing from one that never got close.
        return apex >= heightMeters;
    }

    /** Initial state [x, z, vx, vz, spin] for this flywheel speed. */
    private static double[] releaseState(double flywheelRPM) {
        double surfaceSpeed = Math.PI * ShotModelConstants.FLYWHEEL_DIAMETER_METERS * flywheelRPM / 60.0;
        double exitSpeed = ShotModelConstants.VELOCITY_TRANSFER * surfaceSpeed / 2.0;
        // Rolling between the wheel and the static hood leaves the ball with backspin (positive).
        double spin = ShotModelConstants.SPIN_TRANSFER * surfaceSpeed / (2.0 * BALL_RADIUS_METERS);
        double launch = Math.toRadians(launchElevationDegrees());
        return new double[] {
            0.0,
            ShotModelConstants.RELEASE_HEIGHT_METERS,
            exitSpeed * Math.cos(launch),
            exitSpeed * Math.sin(launch),
            spin
        };
    }

    /**
     * Integrates until the ball falls through {@code heightMeters}. If {@code probeX} is a number,
     * also records the ball's height when it passed that x (NaN if it never got there first).
     * Returns null if the ball never falls through that height within {@code maxRange}.
     */
    private static Crossing fallThrough(
        double flywheelRPM,
        double heightMeters,
        double probeX,
        double maxRange,
        double dtSeconds
    ) {
        double[] state = releaseState(flywheelRPM);
        double[] previous = state.clone();
        double[] k1 = new double[5];
        double[] k2 = new double[5];
        double[] k3 = new double[5];
        double[] k4 = new double[5];
        double[] temporary = new double[5];
        double probeHeight = Double.NaN;
        int steps = (int) Math.ceil(MAX_TIME_SECONDS / dtSeconds);

        for (int step = 0; step < steps; step++) {
            System.arraycopy(state, 0, previous, 0, state.length);
            rk4Step(state, dtSeconds, k1, k2, k3, k4, temporary);

            if (Double.isNaN(probeHeight) && !Double.isNaN(probeX)
                    && previous[0] <= probeX && state[0] >= probeX && state[0] > previous[0]) {
                double fraction = (probeX - previous[0]) / (state[0] - previous[0]);
                probeHeight = lerp(previous[1], state[1], fraction);
            }

            if (previous[1] >= heightMeters && state[1] < heightMeters && state[3] < 0.0) {
                double fraction = (heightMeters - previous[1]) / (state[1] - previous[1]);
                return new Crossing(
                    (step + fraction) * dtSeconds,
                    lerp(previous[0], state[0], fraction),
                    lerp(previous[2], state[2], fraction),
                    lerp(previous[3], state[3], fraction),
                    probeHeight
                );
            }

            if (state[1] < 0.0 || state[0] > maxRange) {
                break;
            }
        }
        return null;
    }

    private static void rk4Step(
        double[] state,
        double dt,
        double[] k1,
        double[] k2,
        double[] k3,
        double[] k4,
        double[] temporary
    ) {
        derivative(state, k1);
        addScaled(state, k1, 0.5 * dt, temporary);
        derivative(temporary, k2);
        addScaled(state, k2, 0.5 * dt, temporary);
        derivative(temporary, k3);
        addScaled(state, k3, dt, temporary);
        derivative(temporary, k4);

        for (int index = 0; index < state.length; index++) {
            state[index] += dt * (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index]) / 6.0;
        }
    }

    private static void derivative(double[] state, double[] result) {
        double vx = state[2];
        double vz = state[3];
        double omega = state[4];
        double relativeVx = vx - ShotModelConstants.WIND_X_METERS_PER_SECOND;
        double relativeVz = vz;
        double relativeSpeed = Math.hypot(relativeVx, relativeVz);
        double accelerationX = 0.0;
        double accelerationZ = -ShotModelConstants.GRAVITY_METERS_PER_SECOND_SQUARED;

        if (relativeSpeed > 1e-8) {
            double dragFactor =
                -DYNAMIC_ACCELERATION_FACTOR
                    * ShotModelConstants.DRAG_COEFFICIENT
                    * ShotModelConstants.DRAG_SCALE
                    * relativeSpeed;
            accelerationX += dragFactor * relativeVx;
            accelerationZ += dragFactor * relativeVz;

            double spinRatio = BALL_RADIUS_METERS * Math.abs(omega) / relativeSpeed;
            double liftCoefficient = Math.min(
                ShotModelConstants.MAX_LIFT_COEFFICIENT,
                ShotModelConstants.LIFT_SLOPE * spinRatio
            );
            double liftFactor =
                DYNAMIC_ACCELERATION_FACTOR
                    * liftCoefficient
                    * relativeSpeed
                    * Math.signum(omega);
            accelerationX += liftFactor * -relativeVz;
            accelerationZ += liftFactor * relativeVx;
        }

        result[0] = vx;
        result[1] = vz;
        result[2] = accelerationX;
        result[3] = accelerationZ;
        result[4] = -ShotModelConstants.SPIN_DECAY_PER_SECOND * omega;
    }

    private static void addScaled(double[] state, double[] derivative, double scale, double[] result) {
        for (int index = 0; index < state.length; index++) {
            result[index] = state[index] + derivative[index] * scale;
        }
    }

    private static Solution withTolerances(Solution exact, Solution band) {
        return new Solution(
            exact.targetDistanceMeters(),
            exact.flywheelRPM(),
            exact.launchElevationDegrees(),
            exact.timeOfFlightSeconds(),
            exact.entryAngleDegrees(),
            exact.crossingXMeters(),
            exact.openingClearanceMeters(),
            exact.nearRimClearanceMeters(),
            band.rpmToleranceLower(),
            band.rpmToleranceUpper(),
            true
        );
    }

    private static Solution invalidSolution(double distanceMeters) {
        return commandWithoutEntry(distanceMeters, ShooterConstants.FLYWHEEL_IDLE_RPM);
    }

    private static Solution commandWithoutEntry(double distanceMeters, double flywheelRPM) {
        return new Solution(
            distanceMeters,
            flywheelRPM,
            launchElevationDegrees(),
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NEGATIVE_INFINITY,
            Double.NEGATIVE_INFINITY,
            0.0,
            0.0,
            false
        );
    }

    private static double usableHalfSpanMeters() {
        return Math.max(
            ShotModelConstants.HUB_OPENING_SPAN_METERS / 2.0
                - BALL_RADIUS_METERS
                - ShotModelConstants.HUB_RIM_MARGIN_METERS,
            0.0
        );
    }

    private static double maxRangeMeters(double distanceMeters) {
        return Math.max(10.0, distanceMeters * 1.25);
    }

    private static double lerp(double start, double end, double fraction) {
        return start + fraction * (end - start);
    }

    private static boolean isDistanceInRange(double distanceMeters) {
        return Double.isFinite(distanceMeters)
            && distanceMeters >= ShotModelConstants.MIN_DISTANCE_METERS
            && distanceMeters <= ShotModelConstants.MAX_DISTANCE_METERS;
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
