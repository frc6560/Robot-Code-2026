package frc.robot.commands.scoring;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.utility.shooting.*;
import java.util.List;
import org.junit.jupiter.api.*;

class FuelShootCommandTest {
    class FakeShooter extends Shooter {
        double measured = 3000;
        boolean connected = true;
        FakeShooter() { super(new ShooterIO() {}); }
        @Override public boolean atTarget() { return true; }
        @Override public double getCurrentRPM() { return measured; }
        @Override public boolean isHardwareReady() { return connected; }
    }
    class FakeIndexer extends Indexer {
        boolean connected = true;
        FakeIndexer() { super(new IndexerIO() {}); }
        @Override public boolean isTunnelReady() { return connected; }
    }
    FakeShooter shooter;
    FakeIndexer indexer;
    boolean aimReady, pit;
    Pose2d pose;
    ChassisSpeeds speeds;
    FuelShootCommand command;

    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }
    @BeforeEach void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        pose = new Pose2d(Constants.FieldConstants.BLUE_HUB_CENTER.getX() - 3.5,
            Constants.FieldConstants.BLUE_HUB_CENTER.getY(), new Rotation2d());
        speeds = new ChassisSpeeds();
        shooter = new FakeShooter(); indexer = new FakeIndexer();
        aimReady = false; pit = false;
        SmartDashboard.putBoolean("Fuel/Calibration Mode", true);
        SmartDashboard.putNumber("Fuel/Test Flywheel RPM", 3000);
        SmartDashboard.putNumber("Fuel/Test Tunnel RPM", 1500);
    }
    void start(FuelShotModel model) {
        command = new FuelShootCommand(shooter, indexer, () -> pose, () -> speeds,
            new Command() {}, () -> aimReady, () -> pit, model, "No calibration");
        command.initialize();
    }
    void cycles(int count) {
        for (int i = 0; i < count; i++) { command.execute(); SimHooks.stepTiming(0.02); }
    }
    @AfterEach void cleanup() {
        if (command != null) command.end(true);
        CommandScheduler.getInstance().unregisterSubsystem(shooter, indexer);
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }
    @Test void oneHoldRequiresAimMeasuredSpeedAndDwellThenStopsImmediately() {
        start(null); cycles(15);
        assertEquals(3000, shooter.getGoalRPM(), 1e-6);
        assertEquals(0, indexer.getTunnelTargetRPM());
        aimReady = true; cycles(2);
        assertEquals(0, indexer.getTunnelTargetRPM());
        cycles(10); assertEquals(1500, indexer.getTunnelTargetRPM());
        shooter.measured = 2700; cycles(1);
        assertEquals(0, indexer.getTunnelTargetRPM());
        shooter.measured = 3000; cycles(10);
        assertEquals(1500, indexer.getTunnelTargetRPM());
        aimReady = false; cycles(1); assertEquals(0, indexer.getTunnelTargetRPM());
        aimReady = true; cycles(10); command.end(false);
        assertEquals(0, indexer.getTunnelTargetRPM()); assertEquals(0, shooter.getGoalRPM());
    }
    @Test void startupDoesNotOverwriteDashboardTuningAndMarksItPersistent() {
        SmartDashboard.putNumber(FuelShootCommand.LAUNCH_COEFFICIENT_KEY, 1.06);
        try {
            FuelShootCommand.publishLaunchSpeedCoefficient();
            assertEquals(1.06, SmartDashboard.getNumber(FuelShootCommand.LAUNCH_COEFFICIENT_KEY, 0));
            assertTrue(SmartDashboard.getEntry(FuelShootCommand.LAUNCH_COEFFICIENT_KEY).isPersistent());
            start(null); cycles(1);
            assertEquals(1.06, SmartDashboard.getNumber("Shooter/Applied Launch Speed Coefficient", 0));
            assertEquals(1.06, SmartDashboard.getNumber(FuelShootCommand.LAUNCH_COEFFICIENT_KEY, 0));
        } finally {
            SmartDashboard.getEntry(FuelShootCommand.LAUNCH_COEFFICIENT_KEY).clearPersistent();
            SmartDashboard.putNumber(FuelShootCommand.LAUNCH_COEFFICIENT_KEY, 1.0);
        }
    }
    @Test void motionAlliancePitFaultsAndDisableBlockFeed() {
        start(null); aimReady = true;
        speeds = new ChassisSpeeds(0.3, 0, 0); cycles(10);
        assertEquals(3000, shooter.getGoalRPM(), 1e-6); // Spin while approaching.
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());
        speeds = new ChassisSpeeds(); cycles(10);
        pit = true; cycles(1); assertEquals(0, indexer.getTunnelTargetRPM());
        assertEquals(0, shooter.getGoalRPM());
        pit = false; indexer.connected = false; cycles(10);
        assertEquals(0, indexer.getTunnelTargetRPM());
        indexer.connected = true; shooter.connected = false; cycles(10);
        assertEquals(0, indexer.getTunnelTargetRPM());
        shooter.connected = true;
        DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
        DriverStationSim.notifyNewData(); cycles(10); assertEquals(0, indexer.getTunnelTargetRPM());
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData();
        cycles(10); assertEquals(0, indexer.getTunnelTargetRPM());
    }
    @Test void missingMeasurementsUseExplicitFixedRPMAndBadSettingsStillBlock() {
        SmartDashboard.putBoolean("Fuel/Calibration Mode", false);
        start(null); aimReady = true; cycles(20);
        assertEquals(1500, indexer.getTunnelTargetRPM());
        assertEquals("Fixed RPM (uncalibrated)", SmartDashboard.getString("Fuel/Mode", ""));
        assertEquals(800, indexer.getFloorTargetRPM());
        command.end(false);
        SmartDashboard.putBoolean("Fuel/Calibration Mode", true);
        SmartDashboard.putNumber("Fuel/Test Flywheel RPM", Double.NaN);
        start(null); cycles(20);
        assertEquals(0, indexer.getTunnelTargetRPM()); assertEquals(0, shooter.getGoalRPM());
    }
    @Test void normalModeCommandsCalibratedShotAndUsesRealTunnel() {
        var model = new FuelShotModel(new LaunchCalibration(List.of(
            new LaunchCalibration.Point(1000, 2.66, Double.NaN),
            new LaunchCalibration.Point(2500, 6.65, Double.NaN),
            new LaunchCalibration.Point(3800, 10.11, Double.NaN)), 1400));
        var shot = model.solve(3.5);
        assertTrue(shot.valid());
        SmartDashboard.putBoolean("Fuel/Calibration Mode", false);
        start(model); aimReady = true; shooter.measured = shot.rpm();
        cycles(15);
        assertEquals(shot.rpm(), shooter.getGoalRPM(), 1e-6);
        assertEquals(1400, indexer.getTunnelTargetRPM());
        assertEquals(800, indexer.getFloorTargetRPM());
        shooter.measured = 1000; cycles(1);
        assertEquals(0, indexer.getTunnelTargetRPM());
        assertEquals(0, indexer.getFloorTargetRPM());
    }
}
