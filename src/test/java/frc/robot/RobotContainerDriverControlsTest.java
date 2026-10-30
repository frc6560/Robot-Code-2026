package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

class RobotContainerDriverControlsTest {
    private static final int DRIVER_PORT = 1;
    private static final int BUTTON_BOARD_PORT = 0;
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void resetState() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test
    void floorStartsOnEnableButTunnelRequiresExplicitFeed() throws Exception {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
        DriverStationSim.setJoystickAxisCount(DRIVER_PORT, 6);
        DriverStationSim.setJoystickButtonCount(DRIVER_PORT, 10);
        DriverStationSim.setJoystickIsXbox(DRIVER_PORT, true);
        DriverStationSim.setJoystickButtonCount(BUTTON_BOARD_PORT, 11);
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kLeftBumper.value, false);
        DriverStationSim.notifyNewData();

        RobotContainer container = new RobotContainer();
        // Neither vision nor alliance information is needed for these manual controls.
        container.getDrivebase().resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d()));
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.run();

        Shooter shooter = getField(container, "shooter", Shooter.class);
        Indexer indexer = getField(container, "indexer", Indexer.class);
        Intake intake = getField(container, "intake", Intake.class);
        Turret turret = getField(container, "turret", Turret.class);
        runCycles(150);
        assertTargets(shooter, indexer, 0.0, 300.0, 0.0);
        assertEquals(false, SmartDashboard.getBoolean("Intake/RackRequested", true));
        assertEquals(0.0, SmartDashboard.getNumber("Intake/DeployAppliedVolts", -1));

        assertEquals(DRIVER_PORT, SmartDashboard.getNumber("Swerve/DriverControllerPort", -1));
        // Slot 0 has no axes: drive and rack must work with only the slot 1 Xbox.
        assertEquals(0, edu.wpi.first.wpilibj.DriverStation.getStickAxisCount(BUTTON_BOARD_PORT));
        // The default drive must accept both translation and rotation with no shot held.
        DriverStationSim.setJoystickAxis(DRIVER_PORT, XboxController.Axis.kLeftY.value, -0.6);
        DriverStationSim.setJoystickAxis(DRIVER_PORT, XboxController.Axis.kRightX.value, 0.3);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertTrue(Math.hypot(SmartDashboard.getNumber("Swerve/RequestedVxMPS", 0),
            SmartDashboard.getNumber("Swerve/RequestedVyMPS", 0)) > 0.1);
        assertTrue(Math.abs(SmartDashboard.getNumber("Swerve/RequestedOmegaRadPerSec", 0)) > 0.1);
        assertTrue(SmartDashboard.getString("Swerve/ControlStatus", "")
            .contains("Driver field-oriented drive"));
        DriverStationSim.setJoystickAxis(DRIVER_PORT, XboxController.Axis.kLeftY.value, 0.0);
        DriverStationSim.setJoystickAxis(DRIVER_PORT, XboxController.Axis.kRightX.value, 0.0);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertEquals(0.0, shooter.getCurrentRPM(), 150.0);
        // Rack triggers must select and retain endpoints without feeding the tunnel.
        DriverStationSim.setJoystickAxis(DRIVER_PORT, XboxController.Axis.kLeftTrigger.value, 1.0);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertEquals(0.0, SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        assertTargets(shooter, indexer, 0.0, 300.0, 0.0);
        DriverStationSim.setJoystickAxis(DRIVER_PORT, XboxController.Axis.kLeftTrigger.value, 0.0);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertEquals(0.0, SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        DriverStationSim.setJoystickAxis(DRIVER_PORT, XboxController.Axis.kRightTrigger.value, 1.0);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertEquals(Constants.IntakeConstants.EXTENDED_POSITION_ROTATIONS,
            SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        assertTargets(shooter, indexer, 0.0, 300.0, 0.0);
        assertTrue(SmartDashboard.getBoolean("Intake/RackRequested", false));
        assertEquals("None", SmartDashboard.getString("Intake/BlockedReason", ""));
        DriverStationSim.setJoystickAxis(DRIVER_PORT, XboxController.Axis.kRightTrigger.value, 0.0);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertEquals(Constants.IntakeConstants.EXTENDED_POSITION_ROTATIONS,
            SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        double turretGoalBeforeTrigger = turret.getGoalDegrees();

        setControls(true, false);
        runCycles(150);
        assertTargets(shooter, indexer, 3000.0, 300.0, 0.0);
        assertEquals(3000.0, shooter.getCurrentRPM(), 200.0);

        setControls(true, true);
        assertTargets(shooter, indexer, 3000.0, 300.0, 1500.0);
        // Releasing the left bumper must not cancel the bumper's shooter/floor command.
        setControls(true, false);
        assertTargets(shooter, indexer, 3000.0, 300.0, 0.0);
        setControls(true, true);
        setControls(false, true);
        assertTargets(shooter, indexer, 0.0, 0.0, 1500.0);
        runCycles(150);
        // Also verify a bumper-only hold/release returns to the zero-RPM idle goal.
        setControls(true, false);
        runCycles(150);
        assertEquals(3000.0, shooter.getCurrentRPM(), 200.0);
        setControls(false, false);
        assertTargets(shooter, indexer, 0.0, 300.0, 0.0);
        runCycles(150);

        // Left bumper alone feeds without changing the shooter's idle goal.
        setControls(false, true);
        assertTargets(shooter, indexer, 0.0, 0.0, 1500.0);
        assertEquals(turretGoalBeforeTrigger, turret.getGoalDegrees(), 0.01);

        setControls(false, false);
        setDriver2Controls(true, false);
        runCycles(150);
        assertTargets(shooter, indexer, 3000.0, 0.0, 0.0);
        assertEquals(3000.0, shooter.getCurrentRPM(), 200.0);
        setDriver2Controls(true, true);
        assertTargets(shooter, indexer, 3000.0, 300.0, 1500.0);
        setDriver2Controls(false, true);
        runCycles(150);
        assertTargets(shooter, indexer, 0.0, 300.0, 1500.0);
        setDriver2Controls(true, true);
        setDriver2Controls(true, false);
        assertTargets(shooter, indexer, 3000.0, 0.0, 0.0);
        setDriver2Controls(false, false);
        assertTargets(shooter, indexer, 0.0, 300.0, 0.0);
        setDriver2Button(4, true);
        assertEquals(true, intake.isActive());
        setDriver2Button(4, false);
        assertEquals(false, intake.isShootingOscillationActive());
        setDriver2Button(6, true);
        assertEquals(true, intake.isActive());
        setDriver2Button(6, false);
        setDriver2Controls(false, true);
        assertTargets(shooter, indexer, 0.0, 300.0, 1500.0);
        setControls(false, true);
        assertTargets(shooter, indexer, 0.0, 300.0, 1500.0);
        setDriver2Controls(false, false);
        assertTargets(shooter, indexer, 0.0, 0.0, 1500.0);

        setControls(true, true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertTargets(shooter, indexer, 1000.0, 0.0, 0.0);

        // No held controls: only the floor restarts on enable, including autonomous.
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kRightBumper.value, false);
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kLeftBumper.value, false);
        DriverStationSim.notifyNewData();
        runCycles(3);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertEquals(300.0, indexer.getFloorTargetRPM());
        assertEquals(0.0, indexer.getTunnelTargetRPM());

        // Pit coast stops automatic output; exiting it resumes only the floor.
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        runCycles(3);
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kBack.value, true);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertEquals(0.0, indexer.getFloorTargetRPM());
        assertEquals(0.0, indexer.getTunnelTargetRPM());
        assertTrue(SmartDashboard.getString("Swerve/ControlStatus", "").contains("Pit coast mode"));
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kBack.value, false);
        DriverStationSim.notifyNewData();
        runCycles(3);
        // Teleop initialization uses this same reset to avoid carrying coast across modes.
        container.disablePitCoastMode();
        runCycles(3);
        assertEquals(300.0, indexer.getFloorTargetRPM());
        assertEquals(0.0, indexer.getTunnelTargetRPM());
        assertEquals(false, SmartDashboard.getBoolean("Swerve/PitCoastMode", true));
        assertTrue(SmartDashboard.getString("Swerve/ControlStatus", "")
            .contains("Driver field-oriented drive"));
        assertEquals(false, SmartDashboard.getBoolean("Intake/RackRequested", true));
        // Y owns shooter/tunnel even while the manual calibration bumper is held.
        SmartDashboard.putBoolean("Fuel/Calibration Mode", false);
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kBack.value, false);
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kY.value, true);
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kLeftBumper.value, true);
        DriverStationSim.notifyNewData();
        runCycles(4);
        assertEquals("FuelShootCommand", CommandScheduler.getInstance().requiring(indexer).getName());
        assertEquals(0, indexer.getTunnelTargetRPM()); // No calibration/alliance, so never releases.
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kY.value, false);
        DriverStationSim.notifyNewData();
        runCycles(4);
        assertEquals(0, indexer.getTunnelTargetRPM()); // Left bumper cannot resume ungated feed on Y release.
        setControls(false, false);
        setControls(false, true);
        assertEquals(1500, indexer.getTunnelTargetRPM()); // Explicit new manual hold still works.
        setControls(false, false);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertEquals(0.0, indexer.getFloorTargetRPM());
        assertEquals(0.0, indexer.getTunnelTargetRPM());
    }

    private static void setControls(boolean bumper, boolean trigger) {
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kRightBumper.value, bumper);
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kLeftBumper.value, trigger);
        DriverStationSim.notifyNewData();
        runCycles(3);
    }

    private static void setDriver2Controls(boolean rampUp, boolean floor) {
        DriverStationSim.setJoystickButton(BUTTON_BOARD_PORT, 11, rampUp);
        DriverStationSim.setJoystickButton(BUTTON_BOARD_PORT, 3, floor);
        DriverStationSim.notifyNewData();
        runCycles(3);
    }

    private static void setDriver2Button(int button, boolean pressed) {
        DriverStationSim.setJoystickButton(BUTTON_BOARD_PORT, button, pressed);
        DriverStationSim.notifyNewData();
        runCycles(3);
    }

    private static void runCycles(int count) {
        for (int i = 0; i < count; i++) {
            CommandScheduler.getInstance().run();
        }
    }

    private static void assertTargets(Shooter shooter, Indexer indexer,
            double shooterRPM, double floorRPM, double tunnelRPM) {
        assertEquals(shooterRPM, shooter.getGoalRPM(), 1e-6);
        assertEquals(floorRPM, indexer.getFloorTargetRPM(), 1e-6);
        assertEquals(tunnelRPM, indexer.getTunnelTargetRPM(), 1e-6);
    }

    private static <T> T getField(RobotContainer container, String name, Class<T> type)
            throws ReflectiveOperationException {
        Field field = RobotContainer.class.getDeclaredField(name);
        field.setAccessible(true);
        return type.cast(field.get(container));
    }
}
