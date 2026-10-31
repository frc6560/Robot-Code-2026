package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ShotModelConstants;
import frc.robot.commands.scoring.FuelShootCommand;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.shooting.*;
import java.io.File;
import java.nio.file.*;
import java.util.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/** Runs production aim/shooter/feed commands against YAGSL MapleSim and WPILib FlywheelSim.
 * No mocked alignment or measured RPM. YAGSL uses simplified module motor loops, not Talon firmware. Ballistics are checked separately in 3D, including inherited
 * ground velocity. Indexer readiness is idealized; ball loading, release jitter and vision are absent.
 * Re-run: ./gradlew movingShotSimulation --rerun-tasks
 */
@Tag("moving-shot-simulation")
class MovingShotClosedLoopSimulationTest {
    private static final double DT = 0.02;
    private static final boolean STRESS = "stress".equals(System.getProperty("movingShotSuite"));
    private static final double DURATION = STRESS ? 10 : 6;
    private static final double MOTION_END = STRESS ? 8.5 : 4.5;
    private static final Path OUTPUT = Path.of(STRESS
        ? "build/reports/moving-shot-stress-simulation" : "build/reports/moving-shot-simulation");
    private static final class FlatArena extends SimulatedArena {
        private static final class Map extends FieldMap {}
        FlatArena() { super(new Map()); }
        @Override public void placeGamePiecesOnField() {}
    }
    /** Motor-controller stand-in at 1 kHz: the stock ShooterIOSim only implements P.
     * This models P/D in motor rot/s units with the existing plant and stator limit.
     * It does not reproduce Talon firmware filtering, CAN latency, or supply-current limiting.
     */
    private static final class MotorLoopFlywheelIO implements ShooterIO {
        private final FlywheelSim plant = new FlywheelSim(LinearSystemId.createFlywheelSystem(
            Constants.ShooterConstants.FLYWHEEL_GEARBOX, Constants.ShooterConstants.FLYWHEEL_MOI, 1),
            Constants.ShooterConstants.FLYWHEEL_GEARBOX);
        private final boolean includeD;
        MotorLoopFlywheelIO(boolean includeD) { this.includeD = includeD; }
        private ShooterIOOutputs requested = new ShooterIOOutputs();
        private double previousError, position, volts;
        private boolean havePreviousError;

        @Override public void applyOutputs(ShooterIOOutputs outputs) { requested = outputs; }

        @Override public void updateInputs(ShooterIOInputs inputs) {
            var motor = Constants.ShooterConstants.FLYWHEEL_GEARBOX;
            double ratio = Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO;
            for (int i = 0; i < 20; i++) {
                double velocity = plant.getAngularVelocityRadPerSec();
                double backEmf = velocity / motor.KvRadPerSecPerVolt;
                if (requested.mode == ShooterIOOutputMode.VELOCITY) {
                    double error = (requested.velocityRadsPerSec - velocity) / (2 * Math.PI * ratio);
                    double derivative = havePreviousError ? (error - previousError) / 0.001 : 0;
                    volts = requested.feedforwardVolts + requested.kP * error + (includeD ? requested.kD * derivative : 0);
                    previousError = error;
                    havePreviousError = true;
                } else {
                    havePreviousError = false;
                    volts = requested.mode == ShooterIOOutputMode.COAST ? backEmf : requested.voltage;
                }
                double headroom = Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT
                    * Constants.ShooterConstants.FLYWHEEL_MOTOR_COUNT * motor.rOhms;
                volts = MathUtil.clamp(MathUtil.clamp(volts, backEmf - headroom, backEmf + headroom), -12, 12);
                plant.setInputVoltage(volts);
                plant.update(0.001);
                position += plant.getAngularVelocityRadPerSec() / (2 * Math.PI * ratio) * .001;
            }
            inputs.leaderConnected = inputs.followerConnected = true;
            inputs.velocityRadsPerSec = plant.getAngularVelocityRadPerSec();
            inputs.leaderVelocityRPS = inputs.followerVelocityRPS = inputs.velocityRadsPerSec / (2 * Math.PI * ratio);
            inputs.leaderPositionRotations = position;
            inputs.leaderAppliedVolts = inputs.followerAppliedVolts = volts;
            inputs.leaderCurrentAmps = inputs.followerCurrentAmps = Math.abs(plant.getCurrentDrawAmps()) / 2;
        }
    }

    private record RunResult(String csv, String scenario, int movingShots, int movingScored) {}

    private record Landing(double miss, double entry, boolean clearedRim, boolean scored) {}

    @Test void independentBallisticsMatchesStationaryModel() {
        var model = new FuelShotModel(LaunchCalibration.ideal());
        model.setLaunchSpeedCoefficient(LaunchSpeedTuning.DEFAULT_COEFFICIENT);
        var pose = new Pose2d();
        var target = new Translation2d(3.5, 0);
        double rpm = model.solve(3.5).rpm();
        var flight = model.flight(rpm);
        var independent = landing(model, pose, new ChassisSpeeds(), target, rpm);
        assertEquals(Math.abs(flight.crossingX() - 3.5), independent.miss(), 0.0001);
        assertEquals(flight.entryDegrees(), independent.entry(), 0.001);
    }

    @Test void runConfiguredControllersAndExportTraces() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        SimulatedArena.overrideInstance(new FlatArena());
        SwerveSubsystem drive = null;
        try {
            drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
            drive.getSwerveDrive().stopOdometryThread();
            Files.createDirectories(OUTPUT);
            StringBuilder summary = new StringBuilder(
                "scenario,first_feed_s,moving_feed_fraction,moving_rpm_error_p95,moving_heading_error_p95_deg,shots,scored,moving_shots,moving_scored,mean_miss_m,max_miss_m,command_p95_ms,command_max_ms\n");
            List<String> scenarios = STRESS
                ? List.of("stationary", "blue-orbit", "red-orbit", "blue-figure-eight",
                    "red-figure-eight", "near-weave", "far-weave", "stationary-p-only-reference")
                : List.of("stationary", "approach", "retreat", "sideways-1mps", "sideways-2mps",
                    "diagonal", "reversal", "move-stop", "red-sideways", "stationary-p-only-reference");
            List<RunResult> results = new ArrayList<>();
            for (String scenario : scenarios) {
                RunResult result = run(drive, scenario);
                results.add(result);
                summary.append(result.csv());
            }
            Files.writeString(OUTPUT.resolve("summary.csv"), summary);
            Files.writeString(OUTPUT.resolve("configuration.txt"), String.format(Locale.ROOT,
                "Aim PID: %.3f, %.3f, %.3f; angular limits: %.3f rad/s, %.3f rad/s^2%n"
                + "Aim tolerance: %.3f deg; tracking-rate tolerance: %.3f deg/s%n"
                + "Shooter PID: %.3f, %.3f, %.3f; FF kS=%.6f kV=%.6f kA=%.6f%n"
                + "Launch coefficient: %.3f; battery: 12.0 V; robot loop: 20 ms; motor loop: 1 ms; MapleSim substeps: 5%n"
                + "Indexer idealized ready; flat field; no vision noise, wheel impacts, supply sag, release delay or ball loading. Motor P/D stand-in omits firmware filtering and CAN latency.%n",
                Constants.DrivebaseConstants.HUB_AIM_KP, Constants.DrivebaseConstants.HUB_AIM_KI,
                Constants.DrivebaseConstants.HUB_AIM_KD,
                Constants.DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC,
                Constants.DrivebaseConstants.HUB_AIM_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ,
                Math.toDegrees(Constants.DrivebaseConstants.HUB_AIM_TOLERANCE_RADIANS),
                Math.toDegrees(Constants.DrivebaseConstants.HUB_AIM_VELOCITY_TOLERANCE_RAD_PER_SEC),
                Constants.ShooterConstants.kP, Constants.ShooterConstants.kI, Constants.ShooterConstants.kD,
                Constants.ShooterConstants.FF_kS, Constants.ShooterConstants.FF_kV, Constants.ShooterConstants.FF_kA,
                LaunchSpeedTuning.DEFAULT_COEFFICIENT));
            Files.writeString(OUTPUT.resolve("suite.txt"), STRESS
                ? "stress: 10-second runs; move from 1.5 to 8.5 seconds; orbit/figure-eight/weave; near range 2.7 m, far range 4 m, otherwise 3.5 m.\n"
                : "standard: 6-second runs; move from 1.5 to 4.5 seconds; straight-line/reversal/stop.\n");
            System.out.println("MOVING_SHOT_SIMULATION\n" + summary);
            // Export every scenario before checking performance so a failed run remains reviewable.
            for (RunResult result : results) {
                assertEquals(result.movingShots(), result.movingScored(),
                    result.scenario() + " released a moving predicted miss in independent ballistics");
                if (STRESS && !result.scenario().equals("stationary")
                        && !result.scenario().endsWith("-reference")) {
                    assertTrue(result.movingShots() > 0, result.scenario() + " never released while moving");
                }
            }
        } finally {
            if (drive != null) {
                drive.driveFieldOriented(new ChassisSpeeds());
                CommandScheduler.getInstance().unregisterSubsystem(drive);
                // YAGSL's simulation has a null hardware IMU; close modules directly instead.
                for (var module : drive.getSwerveDrive().getModules()) module.close();
            }
            DriverStationSim.resetData();
            DriverStationSim.notifyNewData();
            SimHooks.resumeTiming();
        }
    }

    private RunResult run(SwerveSubsystem drive, String scenario) throws Exception {
        boolean red = scenario.startsWith("red-");
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        RoboRioSim.setVInVoltage(12);
        Preferences.setDouble(LaunchSpeedTuning.PREFERENCE_KEY, LaunchSpeedTuning.DEFAULT_COEFFICIENT);
        SmartDashboard.putNumber(LaunchSpeedTuning.REQUEST_KEY, LaunchSpeedTuning.DEFAULT_COEFFICIENT);
        SmartDashboard.putBoolean("Fuel/Calibration Mode", false);
        var hub = red ? Constants.FieldConstants.RED_HUB_CENTER : Constants.FieldConstants.BLUE_HUB_CENTER;
        double startRange = scenario.equals("near-weave") ? 2.7 : scenario.equals("far-weave") ? 4.0 : 3.5;
        var start = new Pose2d(hub.plus(new Translation2d(red ? startRange : -startRange, 0)),
            Rotation2d.fromDegrees(red ? 215 : 35));
        drive.getSwerveDrive().getMapleSimDrive().orElseThrow().setRobotSpeeds(new ChassisSpeeds());
        drive.resetOdometry(start);
        var shooter = new Shooter(new MotorLoopFlywheelIO(!scenario.endsWith("-reference")));
        var indexer = new Indexer(new IndexerIO() {}) {
            @Override public boolean isTunnelReady() { return true; }
        };
        var model = new FuelShotModel(LaunchCalibration.ideal());
        model.setLaunchSpeedCoefficient(LaunchSpeedTuning.DEFAULT_COEFFICIENT);
        ChassisSpeeds[] translation = {new ChassisSpeeds()};
        var command = FuelShootCommand.aimedShot(drive, shooter, indexer, () -> translation[0],
            () -> false, model, "Ideal with live default coefficient", false);
        StringBuilder trace = new StringBuilder("time,x,y,heading_deg,vx,vy,omega,goal_rpm,measured_rpm,heading_error_deg,feeding,status,command_ms\n");
        StringBuilder shots = new StringBuilder("time,speed,miss_m,entry_deg,cleared_rim,scored\n");
        double firstFeed = -1, nextShot = 0;
        int movingCycles = 0, movingFeed = 0, shotCount = 0, scored = 0, movingShots = 0, movingScored = 0;
        double missSum = 0, maxMiss = 0;
        List<Double> rpmErrors = new ArrayList<>(), headingErrors = new ArrayList<>(), commandTimes = new ArrayList<>();
        command.initialize();
        try {
            for (int tick = 0; tick < Math.round(DURATION / DT); tick++) {
                double t = tick * DT;
                translation[0] = velocity(scenario, t, drive.getPose(), hub);
                shooter.periodic();
                long begin = System.nanoTime();
                command.execute();
                double elapsed = (System.nanoTime() - begin) / 1e6;
                commandTimes.add(elapsed);
                var pose = drive.getPose();
                var realPose = drive.getSwerveDrive().getSimulationDriveTrainPose().orElseThrow();
                var fieldVelocity = drive.getSwerveDrive().getMapleSimDrive().orElseThrow()
                    .getDriveTrainSimulatedChassisSpeedsFieldRelative();
                double speed = Math.hypot(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
                double targetHeading = SmartDashboard.getNumber("Hub Aim/Target Heading Deg", 0);
                double error = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(targetHeading)
                    - realPose.getRotation().getRadians()));
                boolean feeding = command.isFeeding();
                if (feeding && firstFeed < 0) firstFeed = t;
                // Exclude the first 0.5 s of translation; report settled moving tracking separately.
                if (t >= 2 && t < MOTION_END && speed > 0.15) {
                    movingCycles++;
                    if (feeding) movingFeed++;
                    rpmErrors.add(Math.abs(shooter.getGoalRPM() - shooter.getCurrentRPM()));
                    headingErrors.add(Math.abs(error));
                }
                if (feeding && t >= nextShot) {
                    var landing = landing(model, realPose, fieldVelocity, hub, shooter.getCurrentRPM());
                    shotCount++;
                    if (landing.scored()) scored++;
                    if (speed > 0.15) {
                        movingShots++;
                        if (landing.scored()) movingScored++;
                    }
                    missSum += landing.miss(); maxMiss = Math.max(maxMiss, landing.miss());
                    shots.append(String.format(Locale.ROOT, "%.3f,%.6f,%.6f,%.6f,%s,%s%n",
                        t, speed, landing.miss(), landing.entry(), landing.clearedRim(), landing.scored()));
                    nextShot = t + 0.2;
                }
                trace.append(String.format(Locale.ROOT,
                    "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.6f,%s,%s,%.6f%n",
                    t, pose.getX(), pose.getY(), realPose.getRotation().getDegrees(),
                    fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond,
                    fieldVelocity.omegaRadiansPerSecond, shooter.getGoalRPM(), shooter.getCurrentRPM(),
                    error, feeding, SmartDashboard.getString("Fuel/Status", ""), elapsed));
                assertTrue(Double.isFinite(shooter.getGoalRPM()));
                SimHooks.stepTiming(DT);
                // Manual update advances MapleSim once, with five physics substeps; no Notifier races.
                drive.getSwerveDrive().updateOdometry();
            }
        } finally {
            command.end(false);
            CommandScheduler.getInstance().unregisterSubsystem(shooter, indexer);
        }
        // This is a diagnostic: zero releases is an important result, not an export failure.
        if (scenario.equals("stationary")) assertEquals(shotCount, scored);
        if (scenario.endsWith("-reference")) {
            assertTrue(firstFeed >= 0, "P-only control case never became ready");
            assertEquals(shotCount, scored);
        }
        Files.writeString(OUTPUT.resolve(scenario + "-trace.csv"), trace);
        Files.writeString(OUTPUT.resolve(scenario + "-shots.csv"), shots);
        return new RunResult(String.format(Locale.ROOT, "%s,%.3f,%.4f,%.3f,%.3f,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f%n",
            scenario, firstFeed, movingCycles == 0 ? Double.NaN : (double) movingFeed / movingCycles,
            percentile(rpmErrors, .95), percentile(headingErrors, .95), shotCount, scored, movingShots, movingScored,
            shotCount == 0 ? Double.NaN : missSum / shotCount, maxMiss,
            percentile(commandTimes, .95), percentile(commandTimes, 1)), scenario, movingShots, movingScored);
    }

    private static double percentile(List<Double> values, double p) {
        if (values.isEmpty()) return Double.NaN;
        var sorted = new ArrayList<>(values); Collections.sort(sorted);
        return sorted.get(Math.min(sorted.size() - 1, (int) Math.ceil(sorted.size() * p) - 1));
    }

    private static ChassisSpeeds velocity(String scenario, double t, Pose2d pose, Translation2d hub) {
        if (t < 1.5 || t >= MOTION_END) return new ChassisSpeeds();
        // Ramp commands over 0.3 seconds rather than teleporting the physical drivetrain velocity.
        double scale = Math.min(1, (t - 1.5) / 0.3);
        double x = 0, y = 0;
        double phase = (t - 1.5) * 2 * Math.PI / 7;
        if (scenario.endsWith("-orbit")) {
            // Reverse smoothly along a curved arc; stay on the same side of the hub.
            Translation2d radial = pose.getTranslation().minus(hub);
            double omega = 0.4 * Math.cos(phase);
            return new ChassisSpeeds(-radial.getY() * omega * scale,
                radial.getX() * omega * scale, 0);
        }
        switch (scenario) {
            case "approach" -> x = 0.5;
            case "retreat" -> x = -0.5;
            case "sideways-1mps", "red-sideways" -> y = 1;
            case "sideways-2mps" -> y = 2;
            case "diagonal" -> { x = 0.4; y = 0.8; }
            case "reversal" -> y = t < 3 ? 1 : -1;
            case "move-stop" -> y = t < 3 ? 1 : 0;
            case "blue-figure-eight", "red-figure-eight" -> {
                x = 0.7 * Math.cos(2 * phase) * (scenario.startsWith("red-") ? -1 : 1);
                y = 1.1 * Math.cos(phase);
            }
            case "near-weave" -> { x = -0.15; y = 0.8 * Math.sin(phase); }
            case "far-weave" -> { x = 0.1; y = 1.4 * Math.sin(phase); }
        }
        return new ChassisSpeeds(x * scale, y * scale, 0);
    }

    /** Independent RK4 integration of gravity, drag and Magnus lift with inherited field velocity.
     * Uses the same provisional aerodynamic constants, not the lead solver's translated trajectory.
     */
    private static Landing landing(FuelShotModel model, Pose2d pose, ChassisSpeeds velocity,
            Translation2d hub, double rpm) {
        var launch = model.calibration().at(rpm);
        double speed = launch.speed() * LaunchSpeedTuning.DEFAULT_COEFFICIENT;
        double radius = ShotModelConstants.BALL_DIAMETER_METERS / 2;
        double elevation = Math.toRadians(ShotModelConstants.LAUNCH_ELEVATION_DEGREES
            + FuelShotConfig.LAUNCH_ANGLE_OFFSET_DEGREES);
        double heading = pose.getRotation().getRadians();
        var release = FuelShootCommand.releasePosition(pose);
        double spin = Double.isFinite(launch.backspinRps()) ? launch.backspinRps() * 2 * Math.PI : speed / radius;
        double[] s = {release.getX(), release.getY(), ShotModelConstants.RELEASE_HEIGHT_METERS,
            speed * Math.cos(elevation) * Math.cos(heading) + velocity.vxMetersPerSecond,
            speed * Math.cos(elevation) * Math.sin(heading) + velocity.vyMetersPerSecond,
            speed * Math.sin(elevation), spin};
        double plane = ShotModelConstants.HUB_RIM_HEIGHT_METERS + radius;
        double halfWidth = ShotModelConstants.HUB_OPENING_SPAN_METERS / 2 - radius
            - ShotModelConstants.HUB_RIM_MARGIN_METERS;
        boolean cleared = false;
        double h = 0.0025;
        for (int n = 0; n < 1600; n++) {
            double[] old = s.clone();
            double[] a = derivative(s, heading), b = derivative(add(s, a, h / 2), heading),
                c = derivative(add(s, b, h / 2), heading), d = derivative(add(s, c, h), heading);
            for (int i = 0; i < s.length; i++) s[i] += h * (a[i] + 2*b[i] + 2*c[i] + d[i]) / 6;
            if (old[2] < plane && s[2] >= plane) {
                double f = (plane - old[2]) / (s[2] - old[2]);
                double distance = new Translation2d(old[0] + f * (s[0] - old[0]),
                    old[1] + f * (s[1] - old[1])).getDistance(hub);
                cleared = distance >= ShotModelConstants.HUB_OPENING_SPAN_METERS / 2
                    + ShotModelConstants.HUB_RIM_MARGIN_METERS;
            }
            if (old[2] >= plane && s[2] <= plane && s[5] < 0) {
                double f = (old[2] - plane) / (old[2] - s[2]);
                double miss = new Translation2d(old[0] + f * (s[0] - old[0]),
                    old[1] + f * (s[1] - old[1])).getDistance(hub);
                double entry = Math.toDegrees(Math.atan2(-(old[5] + f * (s[5] - old[5])),
                    Math.hypot(old[3] + f * (s[3] - old[3]), old[4] + f * (s[4] - old[4]))));
                return new Landing(miss, entry, cleared,
                    cleared && miss <= halfWidth && entry >= FuelShotConfig.MIN_ENTRY_DEGREES);
            }
            if (s[2] < 0) break;
        }
        return new Landing(100, 0, false, false);
    }

    private static double[] add(double[] state, double[] derivative, double scale) {
        double[] result = state.clone();
        for (int i = 0; i < result.length; i++) result[i] += derivative[i] * scale;
        return result;
    }

    private static double[] derivative(double[] s, double heading) {
        double radius = ShotModelConstants.BALL_DIAMETER_METERS / 2;
        double aero = 0.5 * 1.225 * Math.PI * radius * radius / ShotModelConstants.BALL_MASS_KG;
        double speed = Math.sqrt(s[3]*s[3] + s[4]*s[4] + s[5]*s[5]);
        double drag = aero * FuelShotConfig.DRAG_COEFFICIENT * speed;
        double cl = speed < 1e-6 ? 0 : Math.min(0.35, FuelShotConfig.LIFT_SLOPE * radius * Math.abs(s[6]) / speed);
        double lift = aero * cl * speed * Math.signum(s[6]);
        double ax = Math.sin(heading), ay = -Math.cos(heading);
        return new double[] {s[3], s[4], s[5], -drag*s[3] + lift*ay*s[5],
            -drag*s[4] - lift*ax*s[5], -9.80665 - drag*s[5] + lift*(ax*s[4] - ay*s[3]),
            -FuelShotConfig.SPIN_DECAY_PER_SECOND*s[6]};
    }
}
