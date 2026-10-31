package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.scoring.FuelShootCommand;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.shooting.*;
import java.io.File;
import org.junit.jupiter.api.*;

class EventPathAutoTest {
    static SwerveSubsystem drive;
    Shooter shooter;
    Indexer indexer;
    Intake intake;
    AutoCommands factory;
    FuelShotModel model;
    Command auto;
    double roller, extension;
    @BeforeAll static void hal() {
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
        SmartDashboard.putBoolean("Fuel/Calibration Mode", false);
        shooter = new Shooter(new ShooterIO() {}) {
            @Override public boolean atTarget() { return getGoalRPM() > 1000; }
            @Override public boolean isHardwareReady() { return true; }
            @Override public double getCurrentRPM() { return getGoalRPM(); }
        };
        indexer = new Indexer(new IndexerIO() {}) {
            @Override public boolean isTunnelReady() { return true; }
        };
        intake = new Intake(new IntakeIO() {
            @Override public void updateInputs(IntakeIOInputs input) {
                input.configured = input.deployConnected = input.deployHomed = true;
                input.rollerConnected = input.rollerConfigured = true;
                input.deployPositionRotations = extension;
            }
            @Override public void setRollerRPM(double rpm) { roller = rpm; }
        });
        model = new FuelShotModel(LaunchCalibration.ideal());
        factory = new AutoCommands(drive, intake, shooter);
    }
    void cycles(int count) {
        for (int i = 0; i < count; i++) {
            auto.execute();
            intake.periodic();
            SimHooks.stepTiming(0.02);
        }
    }
    @Test void revDoesNotFeedAndFinalShotMatchesTeleopAndPersistsUntilAutoEnds() {
        var path = new Path("hub");
        auto = factory.getCompetitionAuto("hub", false, indexer, () -> false, model, "Ideal");
        auto.initialize();
        cycles(1);
        assertEquals(1000, shooter.getGoalRPM(), 1e-6);
        assertEquals(0, indexer.getTunnelTargetRPM());
        var start = path.getTranslations().get(0);
        var end = path.getTranslations().get(1);
        drive.resetOdometry(new Pose2d(start.interpolate(end, 0.6), new edu.wpi.first.math.geometry.Rotation2d()));
        cycles(5);
        assertTrue(shooter.getGoalRPM() > 1000);
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());
        drive.resetOdometry(new Pose2d(end, new edu.wpi.first.math.geometry.Rotation2d()));
        cycles(30);
        double rpm = shooter.getGoalRPM();
        double tunnel = indexer.getTunnelTargetRPM();
        double floor = indexer.getFloorTargetRPM();
        assertTrue(tunnel > 0 && floor > 0);
        cycles(350);
        assertFalse(auto.isFinished()); // No six-second cutoff on the primary path autos.
        assertEquals(tunnel, indexer.getTunnelTargetRPM());
        auto.end(true);
        assertEquals(0, indexer.getTunnelTargetRPM());
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(1000, shooter.getGoalRPM(), 1e-6);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        var teleop = FuelShootCommand.aimedShot(drive, shooter, indexer, ChassisSpeeds::new,
            () -> false, model, "Ideal", false);
        teleop.initialize();
        for (int i = 0; i < 30; i++) { teleop.execute(); SimHooks.stepTiming(0.02); }
        assertEquals(rpm, shooter.getGoalRPM(), 1e-6);
        assertEquals(tunnel, indexer.getTunnelTargetRPM());
        assertEquals(floor, indexer.getFloorTargetRPM());
        teleop.end(true);
    }
    @Test void intakeEventPersistsRollersAtFortyPercentAndCancellationClearsIt() {
        auto = factory.getCompetitionAuto("non-human", false, indexer, () -> false, model, "Ideal");
        var path = new Path("non-human");
        auto.initialize();
        var start = path.getTranslations().get(0);
        var next = path.getTranslations().get(1);
        drive.resetOdometry(new Pose2d(start.interpolate(next, 0.3), new edu.wpi.first.math.geometry.Rotation2d(Math.PI)));
        extension = frc.robot.Constants.IntakeConstants.EXTENDED_POSITION_ROTATIONS * 0.4;
        cycles(10);
        assertEquals(2500, roller);
        cycles(20);
        assertEquals(2500, roller);
        auto.end(true);
        intake.periodic();
        assertEquals(0, roller);
    }
    @Test void allThreePathsLoadAndUseRegisteredEventNames() {
        for (String name : new String[] {"hub", "human", "non-human"}) {
            var path = new Path(name);
            assertTrue(path.isValid());
            assertTrue(path.getPathElements().stream().anyMatch(
                e -> e instanceof Path.EventTrigger event && event.libKey().equals("rev")));
            assertTrue(path.getPathElements().stream().anyMatch(
                e -> e instanceof Path.EventTrigger event && event.libKey().equals("shoot")));
        }
    }
    @Test void redChoiceResetsToRotatedPoseAndNeverFlipsAgainOnRestart() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();
        auto = factory.getCompetitionAuto("hub", true, indexer, () -> false, model, "Ideal");
        var red = AutoCommands.competitionPath("hub", true);
        var start = (Path.Waypoint) red.getElement(0);
        var end = (Path.Waypoint) red.getElement(red.getPathElements().size() - 1);
        for (int run = 0; run < 2; run++) {
            auto.initialize();
            assertEquals(start.translationTarget().translation().getX(), drive.getPose().getX(), 1e-6);
            assertEquals(start.translationTarget().translation().getY(), drive.getPose().getY(), 1e-6);
            assertEquals(0, edu.wpi.first.math.MathUtil.angleModulus(
                drive.getPose().getRotation().minus(start.rotationTarget().rotation()).getRadians()), 1e-6);
            drive.resetOdometry(new Pose2d(end.translationTarget().translation(), end.rotationTarget().rotation()));
            cycles(30);
            assertTrue(indexer.getTunnelTargetRPM() > 0);
            auto.end(true);
        }
    }

    @Test void chooserExposesAllSixAllianceAndPositionChoices() {
        var selections = new java.util.HashSet<String>();
        var chooser = new AutoModeChooser(factory, (name, red) -> {
            selections.add(name + ":" + red);
            return factory.getCompetitionAuto(name, red, indexer, () -> false, model, "Ideal");
        }, edu.wpi.first.wpilibj2.command.Commands.none());
        assertEquals(java.util.Set.of("hub:false", "hub:true", "human:false", "human:true",
            "non-human:false", "non-human:true"), selections);
        SmartDashboard.putData("Test/CompetitionChooser", chooser.getAutoChooser());
        SmartDashboard.updateValues();
        String[] options = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
            .getTable("SmartDashboard").getSubTable("Test/CompetitionChooser")
            .getEntry("options").getStringArray(new String[0]);
        assertTrue(java.util.Arrays.asList(options).containsAll(java.util.List.of(
            "Blue Left", "Blue Center", "Blue Right", "Red Left", "Red Center", "Red Right")));
        assertEquals("Blue Center", chooser.getAutoChooser().getSelected().getName());
    }

    @AfterEach void cleanup() {
        if (auto != null) auto.end(true);
        CommandScheduler.getInstance().unregisterSubsystem(shooter, indexer, intake);
        DriverStationSim.resetData(); DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }
    @AfterAll static void cleanupDrive() { CommandScheduler.getInstance().unregisterSubsystem(drive); }
}
