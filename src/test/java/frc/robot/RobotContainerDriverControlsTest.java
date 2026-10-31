package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;

class RobotContainerDriverControlsTest {
    private static final int DRIVER_PORT = Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
    private static final int BUTTON_BOARD_PORT = Constants.OperatorConstants.BUTTON_BOARD_PORT;
    @BeforeAll static void initializeHal() { assertTrue(HAL.initialize(500, 0)); }
    @AfterEach void resetState() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test void driverControlsOwnIntakeAimAndManualFeed() throws Exception {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
        DriverStationSim.setJoystickAxisCount(DRIVER_PORT, 6);
        DriverStationSim.setJoystickButtonCount(DRIVER_PORT, 10);
        DriverStationSim.setJoystickIsXbox(DRIVER_PORT, true);
        DriverStationSim.setJoystickButtonCount(BUTTON_BOARD_PORT, 11);
        DriverStationSim.notifyNewData();
        RobotContainer container = new RobotContainer();
        assertEquals("Blue Center", container.getAutonomousCommand().getName());
        // This test checks button ownership and fixed-RPM feeding, not the trajectory solver.
        SmartDashboard.putBoolean("Fuel/Calibration Mode", true);
        SmartDashboard.putNumber("Fuel/Test Flywheel RPM", 3000);
        container.getDrivebase().resetOdometry(new Pose2d());
        Shooter shooter = getField(container, "shooter", Shooter.class);
        Indexer indexer = getField(container, "indexer", Indexer.class);
        runCycles(4);
        assertEquals(1000, shooter.getGoalRPM(), 1e-6);
        setBoard(11, true);
        assertEquals(1000, shooter.getGoalRPM(), 1e-6);
        setBoard(11, false);
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());

        // Driver X routes the explicit reset to the front camera only.
        for (String camera : new String[] {"Front", "Left", "Right"}) {
            SmartDashboard.putString("Vision/" + camera + "/ResetStatus", "Not requested");
        }
        setButton(XboxController.Button.kX, true);
        assertNotEquals("Not requested", SmartDashboard.getString("Vision/Front/ResetStatus", "Not requested"));
        assertEquals("Not requested", SmartDashboard.getString("Vision/Left/ResetStatus", ""));
        assertEquals("Not requested", SmartDashboard.getString("Vision/Right/ResetStatus", ""));
        setButton(XboxController.Button.kX, false);

        Intake intake = getField(container, "intake", Intake.class);
        // LT collects without feeding either indexer. Release stops the rack too.
        setTrigger(XboxController.Axis.kLeftTrigger, true);
        assertEquals(Constants.IntakeConstants.EXTENDED_POSITION_ROTATIONS,
            SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        assertEquals(0, indexer.getTunnelTargetRPM());
        assertEquals(0, indexer.getFloorTargetRPM());
        setTrigger(XboxController.Axis.kLeftTrigger, false);
        assertFalse(SmartDashboard.getBoolean("Intake/RackRequested", true));
        setTrigger(XboxController.Axis.kLeftTrigger, true);
        setButton(XboxController.Button.kRightBumper, true);
        assertEquals(0, SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        assertTrue(SmartDashboard.getBoolean("Intake/ForceRetractRequested", false));
        setButton(XboxController.Button.kRightBumper, false);
        assertFalse(SmartDashboard.getBoolean("Intake/RackRequested", true));
        assertEquals(0, SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        setTrigger(XboxController.Axis.kLeftTrigger, false);

        // Releasing RB must not create a fresh extension press from a held LT.
        setButton(XboxController.Button.kRightBumper, true);
        setTrigger(XboxController.Axis.kLeftTrigger, true);
        setButton(XboxController.Button.kRightBumper, false);
        assertFalse(SmartDashboard.getBoolean("Intake/RackRequested", true));
        assertEquals(0, SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        setTrigger(XboxController.Axis.kLeftTrigger, false);

        setButton(XboxController.Button.kLeftBumper, true);
        assertTrue(SmartDashboard.getBoolean("Intake/ForceExtendRequested", false));
        assertFalse(SmartDashboard.getBoolean("Intake/ForceRetractRequested", true));
        assertEquals(Constants.IntakeConstants.EXTENDED_POSITION_ROTATIONS,
            SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        setButton(XboxController.Button.kLeftBumper, false);
        assertFalse(SmartDashboard.getBoolean("Intake/RackRequested", true));
        assertFalse(SmartDashboard.getBoolean("Intake/ForceExtendRequested", true));

        setTrigger(XboxController.Axis.kLeftTrigger, true);
        setButton(XboxController.Button.kA, true);
        assertTrue(intake.isExtensionLimitReached());
        assertTrue(intake.atDeployTarget());
        setButton(XboxController.Button.kA, false);
        setTrigger(XboxController.Axis.kLeftTrigger, false);
        assertFalse(intake.isExtensionLimitReached());

        setBoard(3, true);
        assertEquals(1500, indexer.getTunnelTargetRPM());
        assertEquals(800, indexer.getFloorTargetRPM());
        setTrigger(XboxController.Axis.kRightTrigger, true);
        runCycles(150);
        assertEquals("FuelShootCommand", CommandScheduler.getInstance().requiring(indexer).getName());
        assertSame(CommandScheduler.getInstance().requiring(indexer),
            CommandScheduler.getInstance().requiring(container.getDrivebase()));
        assertEquals(3000, shooter.getGoalRPM(), 1e-6);
        assertEquals(0, indexer.getTunnelTargetRPM()); // Unknown alliance: spin, but do not feed.
        assertEquals(0, indexer.getFloorTargetRPM());
        assertTrue(SmartDashboard.getBoolean("Intake/ForceRetractRequested", false));
        assertSame(CommandScheduler.getInstance().requiring(indexer),
            CommandScheduler.getInstance().requiring(intake));
        setBoard(3, false);
        assertEquals("FuelShootCommand", CommandScheduler.getInstance().requiring(indexer).getName());
        // Keep RT held after the wall spike: aiming remains active but rack output stays off.
        SimHooks.pauseTiming();
        try {
            for (int i = 0; i < 130; i++) {
                SimHooks.stepTiming(0.02);
                runCycles(1);
            }
            assertTrue(intake.isRetractionLimitReached());
            for (int i = 0; i < 150; i++) {
                SimHooks.stepTiming(0.02);
                runCycles(1);
            }
            assertTrue(intake.isRetractionLimitReached());
            assertEquals(0, SmartDashboard.getNumber("Intake/DeployAppliedVolts", 99));
            assertEquals("FuelShootCommand", CommandScheduler.getInstance().requiring(indexer).getName());
        } finally {
            SimHooks.resumeTiming();
        }
        setTrigger(XboxController.Axis.kRightTrigger, false);
        runCycles(3); // Releasing RT stops retraction and resets its current allowance.
        assertFalse(SmartDashboard.getBoolean("Intake/RackRequested", true));
        assertEquals(1000, shooter.getGoalRPM(), 1e-6);
        assertEquals(0, indexer.getTunnelTargetRPM());
        assertEquals(0, indexer.getFloorTargetRPM());

        // A new hold starts the combined command again; Y is no longer needed.
        setTrigger(XboxController.Axis.kRightTrigger, true);
        assertEquals(3000, shooter.getGoalRPM(), 1e-6);
        setTrigger(XboxController.Axis.kRightTrigger, false);

        // Explicit operator feed runs both indexers; RB still takes ownership.
        setBoard(3, true);
        assertEquals(1500, indexer.getTunnelTargetRPM());
        assertEquals(800, indexer.getFloorTargetRPM());
        setTrigger(XboxController.Axis.kRightTrigger, true);
        assertEquals("FuelShootCommand", CommandScheduler.getInstance().requiring(indexer).getName());
        setTrigger(XboxController.Axis.kRightTrigger, false);
        assertEquals(0, indexer.getTunnelTargetRPM()); // Held board input cannot resume after RB release.
        setBoard(3, false); setBoard(3, true);
        assertEquals(1500, indexer.getTunnelTargetRPM());
        assertEquals(800, indexer.getFloorTargetRPM());
        setBoard(3, false);

        // A held teleop intake trigger must not cancel the newly scheduled autonomous.
        DriverStationSim.setJoystickAxis(DRIVER_PORT, XboxController.Axis.kLeftTrigger.value, 1.0);
        DriverStationSim.notifyNewData(); runCycles(3);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        var auto = container.getAutonomousCommand();
        auto.schedule();
        runCycles(4);
        assertTrue(auto.isScheduled());
        assertEquals(1000, shooter.getGoalRPM(), 1e-6);
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData(); runCycles(4);
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());
    }
    private static void setTrigger(XboxController.Axis axis, boolean pressed) {
        DriverStationSim.setJoystickAxis(DRIVER_PORT, axis.value, pressed ? 1.0 : 0.0);
        DriverStationSim.notifyNewData(); runCycles(4);
    }
    private static void setButton(XboxController.Button button, boolean pressed) {
        DriverStationSim.setJoystickButton(DRIVER_PORT, button.value, pressed);
        DriverStationSim.notifyNewData(); runCycles(4);
    }
    private static void setBoard(int button, boolean pressed) {
        DriverStationSim.setJoystickButton(BUTTON_BOARD_PORT, button, pressed);
        DriverStationSim.notifyNewData(); runCycles(4);
    }
    private static void runCycles(int count) {
        for (int i = 0; i < count; i++) CommandScheduler.getInstance().run();
    }
    private static <T> T getField(RobotContainer container, String name, Class<T> type)
            throws ReflectiveOperationException {
        Field field = RobotContainer.class.getDeclaredField(name);
        field.setAccessible(true);
        return type.cast(field.get(container));
    }
}
