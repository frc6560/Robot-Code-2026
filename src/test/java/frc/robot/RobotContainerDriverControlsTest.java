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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

class RobotContainerDriverControlsTest {
    private static final int DRIVER_PORT = Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
    private static final int BUTTON_BOARD_PORT = Constants.OperatorConstants.BUTTON_BOARD_PORT;
    @BeforeAll static void initializeHal() { assertTrue(HAL.initialize(500, 0)); }
    @AfterEach void resetState() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test void rightBumperOwnsAimSpinFeedAndIdleIndexersStayStopped() throws Exception {
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

        // Rack triggers keep their independent endpoint selection.
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kRightBumper.value, true);
        DriverStationSim.notifyNewData(); runCycles(3);
        assertEquals(Constants.IntakeConstants.EXTENDED_POSITION_ROTATIONS,
            SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        assertEquals(0, indexer.getTunnelTargetRPM());
        // Retraction must take over even if the bumpers overlap. Releasing LB
        // while RB remains held must not synthesize a new extension press.
        setButton(XboxController.Button.kLeftBumper, true);
        assertEquals(Constants.IntakeConstants.RETRACTED_POSITION_ROTATIONS,
            SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        setButton(XboxController.Button.kLeftBumper, false);
        assertEquals(Constants.IntakeConstants.RETRACTED_POSITION_ROTATIONS,
            SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kRightBumper.value, false);
        DriverStationSim.notifyNewData(); runCycles(3);

        setButton(XboxController.Button.kLeftBumper, true);
        setButton(XboxController.Button.kRightBumper, true);
        setButton(XboxController.Button.kLeftBumper, false);
        assertEquals(Constants.IntakeConstants.RETRACTED_POSITION_ROTATIONS,
            SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        setButton(XboxController.Button.kRightBumper, false);
        setButton(XboxController.Button.kRightBumper, true);
        assertEquals(Constants.IntakeConstants.EXTENDED_POSITION_ROTATIONS,
            SmartDashboard.getNumber("Intake/DeployTargetRotations", -1));
        setButton(XboxController.Button.kRightBumper, false);

        setTrigger(XboxController.Axis.kLeftTrigger, true);
        assertEquals(1500, indexer.getTunnelTargetRPM()); // Standalone manual feed, independent of alliance/aim.
        assertEquals(800, indexer.getFloorTargetRPM());
        setTrigger(XboxController.Axis.kRightTrigger, true);
        runCycles(150);
        assertEquals("FuelShootCommand", CommandScheduler.getInstance().requiring(indexer).getName());
        assertSame(CommandScheduler.getInstance().requiring(indexer),
            CommandScheduler.getInstance().requiring(container.getDrivebase()));
        assertEquals(3000, shooter.getGoalRPM(), 1e-6);
        assertEquals(0, indexer.getTunnelTargetRPM()); // Unknown alliance: spin, but do not feed.
        assertEquals(0, indexer.getFloorTargetRPM());
        setTrigger(XboxController.Axis.kLeftTrigger, false);
        assertEquals("FuelShootCommand", CommandScheduler.getInstance().requiring(indexer).getName());
        setTrigger(XboxController.Axis.kRightTrigger, false);
        runCycles(3); // Return to idle when RB releases its requirements.
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
        DriverStationSim.setJoystickButton(DRIVER_PORT, XboxController.Button.kRightBumper.value, true);
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
