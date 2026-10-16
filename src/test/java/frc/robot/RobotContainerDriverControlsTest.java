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
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

class RobotContainerDriverControlsTest {
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
    void indexerRunsThroughoutTeleopWithoutOverridingShooterControls() throws Exception {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
        DriverStationSim.setJoystickAxisCount(0, 6);
        DriverStationSim.setJoystickButtonCount(0, 10);
        DriverStationSim.setJoystickIsXbox(0, true);
        DriverStationSim.setJoystickButtonCount(1, 11);
        DriverStationSim.setJoystickAxis(0, XboxController.Axis.kRightTrigger.value, 0.0);
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
        assertTargets(shooter, indexer, 1000.0, 1000.0, 1500.0);
        assertEquals(1000.0, shooter.getCurrentRPM(), 150.0);
        double turretGoalBeforeTrigger = turret.getGoalDegrees();

        setControls(true, false);
        runCycles(150);
        assertTargets(shooter, indexer, 3000.0, 1000.0, 1500.0);
        assertEquals(3000.0, shooter.getCurrentRPM(), 200.0);

        setControls(true, true);
        assertTargets(shooter, indexer, 3000.0, 1000.0, 1500.0);
        // Releasing the trigger must not cancel the bumper's shooter/floor command.
        setControls(true, false);
        assertTargets(shooter, indexer, 3000.0, 1000.0, 1500.0);
        setControls(true, true);
        setControls(false, true);
        assertTargets(shooter, indexer, 1000.0, 1000.0, 1500.0);
        runCycles(150);
        assertEquals(1000.0, shooter.getCurrentRPM(), 150.0);
        // Also verify a bumper-only hold/release settles back to measured idle speed.
        setControls(true, false);
        runCycles(150);
        assertEquals(3000.0, shooter.getCurrentRPM(), 200.0);
        setControls(false, false);
        assertTargets(shooter, indexer, 1000.0, 1000.0, 1500.0);
        runCycles(150);
        assertEquals(1000.0, shooter.getCurrentRPM(), 150.0);

        // Trigger holds do not interrupt the automatic indexer or change the shooter's idle goal.
        setControls(false, true);
        assertTargets(shooter, indexer, 1000.0, 1000.0, 1500.0);
        assertEquals(turretGoalBeforeTrigger, turret.getGoalDegrees(), 0.01);

        setControls(false, false);
        setDriver2Controls(true, false);
        runCycles(150);
        assertTargets(shooter, indexer, 3000.0, 1000.0, 1500.0);
        assertEquals(3000.0, shooter.getCurrentRPM(), 200.0);
        setDriver2Controls(true, true);
        assertTargets(shooter, indexer, 3000.0, 1000.0, 1500.0);
        setDriver2Controls(false, true);
        runCycles(150);
        assertTargets(shooter, indexer, 1000.0, 1000.0, 1500.0);
        assertEquals(1000.0, shooter.getCurrentRPM(), 150.0);
        setDriver2Controls(true, true);
        setDriver2Controls(true, false);
        assertTargets(shooter, indexer, 3000.0, 1000.0, 1500.0);
        setDriver2Controls(false, false);
        assertTargets(shooter, indexer, 1000.0, 1000.0, 1500.0);
        setDriver2Button(4, true);
        assertEquals(Intake.State.EXTENDED_SPINNING, intake.getState());
        setDriver2Button(4, false);
        assertEquals(Intake.State.EXTENDED_STOPPED, intake.getState());
        setDriver2Button(6, true);
        assertEquals(Intake.State.RETRACTED_STOPPED, intake.getState());
        setDriver2Button(6, false);
        setDriver2Controls(false, true);
        assertTargets(shooter, indexer, 1000.0, 1000.0, 1500.0);
        setControls(false, true);
        assertTargets(shooter, indexer, 1000.0, 1000.0, 1500.0);
        setDriver2Controls(false, false);
        assertTargets(shooter, indexer, 1000.0, 1000.0, 1500.0);

        setControls(true, true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertTargets(shooter, indexer, 1000.0, 0.0, 0.0);

        // Re-enable with every control released: both stages must start automatically.
        setControls(false, false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertTargets(shooter, indexer, 1000.0, 1000.0, 1500.0);

        // Pit coast stops the indexer until toggled off again.
        setDriverButton(XboxController.Button.kBack.value, true);
        assertTargets(shooter, indexer, 1000.0, 0.0, 0.0);
        setDriverButton(XboxController.Button.kBack.value, false);
        setDriverButton(XboxController.Button.kBack.value, true);
        setDriverButton(XboxController.Button.kBack.value, false);
        assertTargets(shooter, indexer, 1000.0, 1000.0, 1500.0);

        // Leaving teleop also stops both stages even while the robot stays enabled.
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertTargets(shooter, indexer, 1000.0, 0.0, 0.0);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(true);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertTargets(shooter, indexer, 1000.0, 0.0, 0.0);
        DriverStationSim.setTest(false);
        DriverStationSim.notifyNewData();
        runCycles(3);
        assertTargets(shooter, indexer, 1000.0, 1000.0, 1500.0);
    }

    private static void setDriverButton(int button, boolean pressed) {
        DriverStationSim.setJoystickButton(0, button, pressed);
        DriverStationSim.notifyNewData();
        runCycles(3);
    }

    private static void setControls(boolean bumper, boolean trigger) {
        DriverStationSim.setJoystickButton(0, XboxController.Button.kRightBumper.value, bumper);
        DriverStationSim.setJoystickAxis(0, XboxController.Axis.kRightTrigger.value, trigger ? 1.0 : 0.0);
        DriverStationSim.notifyNewData();
        runCycles(3);
    }

    private static void setDriver2Controls(boolean rampUp, boolean floor) {
        DriverStationSim.setJoystickButton(1, 11, rampUp);
        DriverStationSim.setJoystickButton(1, 3, floor);
        DriverStationSim.notifyNewData();
        runCycles(3);
    }

    private static void setDriver2Button(int button, boolean pressed) {
        DriverStationSim.setJoystickButton(1, button, pressed);
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
