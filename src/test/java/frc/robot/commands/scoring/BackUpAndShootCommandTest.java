package frc.robot.commands.scoring;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.autonomous.AutoCommands;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.shooting.*;
import java.io.File;
import org.junit.jupiter.api.*;

class BackUpAndShootCommandTest {
    static SwerveSubsystem drive;
    Shooter shooter;
    Indexer indexer;
    Intake intake;
    Path plannedPath;
    BackUpAndShootCommand command;
    boolean pit;
    boolean indexerReady;

    @BeforeAll static void initialize() {
        assertTrue(HAL.initialize(500, 0));
        drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
    }

    @BeforeEach void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        SmartDashboard.putNumber(LaunchSpeedTuning.REQUEST_KEY, 1.0);
        // These dashboard settings must never bypass the autonomous trajectory checks.
        SmartDashboard.putBoolean("Fuel/Calibration Mode", true);
        shooter = new Shooter(new ShooterIO() {}) {
            @Override public boolean atTarget() { return true; }
            @Override public boolean isHardwareReady() { return true; }
            @Override public double getCurrentRPM() { return getGoalRPM(); }
        };
        indexerReady = true;
        indexer = new Indexer(new IndexerIO() {}) {
            @Override public boolean isTunnelReady() { return indexerReady; }
        };
        intake = new Intake(new IntakeIO() {});
        var factory = new AutoCommands(drive, intake, shooter);
        command = new BackUpAndShootCommand(drive, shooter, indexer,
            new FuelShotModel(LaunchCalibration.ideal()), () -> pit, "Ideal", path -> {
                plannedPath = path;
                return factory.followFieldPath(path);
            });
    }

    void pose(double range) {
        drive.resetOdometry(new Pose2d(Constants.FieldConstants.BLUE_HUB_CENTER.getX() - range,
            Constants.FieldConstants.BLUE_HUB_CENTER.getY(), new Rotation2d()));
    }

    void cycles(int count) {
        for (int i = 0; i < count && !command.isFinished(); i++) {
            SimHooks.stepTiming(0.02);
            command.execute();
        }
    }

    String phase() { return SmartDashboard.getString("Auto/Back Up and Shoot/Phase", ""); }

    @Test void bLineRetreatsExactlyTwoPointFourMetersBeforeUnloadingAndCanBeReused() {
        pose(0.8);
        command.initialize();
        cycles(20);
        assertEquals("RETREAT", phase());
        assertNotNull(plannedPath);
        assertFalse(((Path.Waypoint) plannedPath.getElement(0)).rotationTarget().profiledRotation());
        assertFalse(((Path.Waypoint) plannedPath.getElement(1)).rotationTarget().profiledRotation());
        assertEquals(1.0, plannedPath.getPathConstraints().getMaxVelocityMetersPerSec().get().get(0).value());
        assertEquals(0.5, plannedPath.getPathConstraints().getMaxAccelerationMetersPerSec2().get().get(0).value());
        assertEquals(0, indexer.getTunnelTargetRPM());
        // A reachable shot before reaching the fixed endpoint must still not feed.
        pose(2.5);
        cycles(30);
        assertEquals("RETREAT", phase());
        assertEquals(0, indexer.getTunnelTargetRPM());
        Pose2d end = new Pose2d(plannedPath.getTranslations().get(1), new Rotation2d());
        assertEquals(2.4, end.getTranslation().getDistance(plannedPath.getTranslations().get(0)), 1e-9);
        drive.resetOdometry(end);
        cycles(100);
        assertEquals("SHOOT", phase());
        assertTrue(indexer.getTunnelTargetRPM() > 0);
        cycles(350);
        assertEquals("DONE", phase());
        assertTrue(command.isFinished());
        assertEquals(0, shooter.getGoalRPM());
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());
        command.end(false);
        pose(0.8);
        command.initialize();
        assertFalse(command.isFinished());
        cycles(20);
        command.end(true);
        assertEquals(0, indexer.getTunnelTargetRPM());
        assertEquals(0, shooter.getGoalRPM());
    }

    @Test void plannerUsesFixedDistanceOnBothAlliancesAndRejectsInsufficientSpace() {
        var model = new FuelShotModel(LaunchCalibration.ideal());
        for (var alliance : DriverStation.Alliance.values()) {
            boolean blue = alliance == DriverStation.Alliance.Blue;
            var hub = blue ? Constants.FieldConstants.BLUE_HUB_CENTER : Constants.FieldConstants.RED_HUB_CENTER;
            var start = new Pose2d(hub.getX() + (blue ? -0.8 : 0.8), hub.getY(), new Rotation2d());
            var end = BackUpAndShootCommand.findRetreatEnd(start, alliance, model);
            assertNotNull(end);
            assertEquals(2.4, end.getTranslation().getDistance(start.getTranslation()), 1e-9);
            assertTrue(model.solve(FuelShootCommand.releasePosition(end).getDistance(hub)).valid());
        }
        var tooCloseToWall = new Pose2d(1.0, 4.03, new Rotation2d());
        assertNull(BackUpAndShootCommand.findRetreatEnd(tooCloseToWall, DriverStation.Alliance.Blue, model));
    }

    @Test void unknownAlliancePitAndTimeoutStopEverything() {
        pose(0.8);
        indexerReady = false;
        command.initialize();
        cycles(20);
        drive.resetOdometry(new Pose2d(plannedPath.getTranslations().get(1), new Rotation2d()));
        cycles(800);
        assertEquals("ABORTED", phase());
        assertEquals(0, indexer.getTunnelTargetRPM());
        DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
        DriverStationSim.notifyNewData();
        command.initialize(); cycles(1);
        assertEquals("ABORTED", phase());
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        command.initialize(); pit = true; cycles(1);
        assertEquals("ABORTED", phase());
        assertEquals(0, shooter.getGoalRPM());
    }

    @AfterEach void cleanup() {
        command.end(true);
        CommandScheduler.getInstance().unregisterSubsystem(shooter, indexer, intake);
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }

    @AfterAll static void cleanupDrive() { CommandScheduler.getInstance().unregisterSubsystem(drive); }
}
