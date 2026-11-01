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

    @org.junit.jupiter.api.Disabled("Shooter and intake intentionally not initialized during memory investigation")
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
        assertEquals(0, shooter.getGoalRPM(), 1e-6);
        setBoard(11, true);
        assertEquals(0, shooter.getGoalRPM(), 1e-6);
        setBoard(11, false);
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());

        assertIdleAt(container, shooter, AllianceStationID.Blue1, 0.0, 1000);
        assertIdleAt(container, shooter, AllianceStationID.Blue1, 4.70, 0);
        assertIdleAt(container, shooter, AllianceStationID.Red1, 16.0, 1000);
        assertIdleAt(container, shooter, AllianceStationID.Red1, 11.80, 0);
        assertIdleAt(container, shooter, AllianceStationID.Unknown, 0.0, 0);

        Intake intake = getField(container, "intake", Intake.class);
        // LT collects without feeding either indexer. Release stops the rack too.
        setTrigger(XboxController.Axis.kLeftTrigger, true);
        assertTrue(SmartDashboard.getBoolean("Intake/ForceExtendRequested", false));
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
        assertEquals(-Constants.IntakeConstants.ROLLER_RPM,
            SmartDashboard.getNumber("Intake/RollerCommandedRPM", 0));
        assertEquals(-Constants.IndexerConstants.FLOOR_RUNNING_RPM, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());
        assertFalse(SmartDashboard.getBoolean("Intake/RackRequested", true));
        setTrigger(XboxController.Axis.kLeftTrigger, true);
        assertEquals(-Constants.IntakeConstants.ROLLER_RPM,
            SmartDashboard.getNumber("Intake/RollerCommandedRPM", 0));
        setTrigger(XboxController.Axis.kRightTrigger, true);
        assertEquals(-Constants.IndexerConstants.FLOOR_RUNNING_RPM, indexer.getFloorTargetRPM());
        setTrigger(XboxController.Axis.kRightTrigger, false);
        setTrigger(XboxController.Axis.kLeftTrigger, false);
        setButton(XboxController.Button.kLeftBumper, false);
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, SmartDashboard.getNumber("Intake/RollerCommandedRPM", 99));

        setTrigger(XboxController.Axis.kLeftTrigger, true);
        setButton(XboxController.Button.kA, true);
        assertTrue(intake.isExtensionLimitReached());
        assertTrue(intake.atDeployTarget());
        setButton(XboxController.Button.kA, false);
        setTrigger(XboxController.Axis.kLeftTrigger, false);
        assertFalse(intake.isExtensionLimitReached());

        setBoard(3, true);
        assertEquals(1500, indexer.getTunnelTargetRPM());
        assertEquals(1200, indexer.getFloorTargetRPM());
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
        assertEquals(0, shooter.getGoalRPM(), 1e-6);
        assertEquals(0, indexer.getTunnelTargetRPM());
        assertEquals(0, indexer.getFloorTargetRPM());

        // A new hold starts the combined command again; Y is no longer needed.
        setTrigger(XboxController.Axis.kRightTrigger, true);
        assertEquals(3000, shooter.getGoalRPM(), 1e-6);
        setTrigger(XboxController.Axis.kRightTrigger, false);

        // Explicit operator feed runs both indexers; RB still takes ownership.
        setBoard(3, true);
        assertEquals(1500, indexer.getTunnelTargetRPM());
        assertEquals(1200, indexer.getFloorTargetRPM());
        setTrigger(XboxController.Axis.kRightTrigger, true);
        assertEquals("FuelShootCommand", CommandScheduler.getInstance().requiring(indexer).getName());
        setTrigger(XboxController.Axis.kRightTrigger, false);
        assertEquals(0, indexer.getTunnelTargetRPM()); // Held board input cannot resume after RB release.
        setBoard(3, false); setBoard(3, true);
        assertEquals(1500, indexer.getTunnelTargetRPM());
        assertEquals(1200, indexer.getFloorTargetRPM());
        setBoard(3, false);

        // Midzone has zero idle, but explicit RT still requests the passing flywheel RPM.
        assertIdleAt(container, shooter, AllianceStationID.Blue1, 8.0, 0);
        setTrigger(XboxController.Axis.kRightTrigger, true);
        assertEquals(frc.robot.utility.shooting.AlliancePass.RPM, shooter.getGoalRPM(), 1e-6);
        setTrigger(XboxController.Axis.kRightTrigger, false);
        runCycles(3);
        assertEquals(0, shooter.getGoalRPM(), 1e-6);

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
    @Test void disabledMechanismControlsLeaveDrivingAvailableAndIndexerStopped() throws Exception {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setJoystickAxisCount(DRIVER_PORT, 6);
        DriverStationSim.setJoystickButtonCount(DRIVER_PORT, 10);
        DriverStationSim.setJoystickIsXbox(DRIVER_PORT, true);
        DriverStationSim.setJoystickButtonCount(BUTTON_BOARD_PORT, 11);
        DriverStationSim.notifyNewData();
        RobotContainer container = new RobotContainer();
        Indexer indexer = getField(container, "indexer", Indexer.class);
        runCycles(4);
        CommandScheduler scheduler = CommandScheduler.getInstance();
        var driveCommand = scheduler.requiring(container.getDrivebase());
        assertNotNull(driveCommand);
        assertEquals("Driver field-oriented drive", driveCommand.getName());
        // Former intake/shoot/feed controls must not schedule mechanism commands or seize driving.
        setTrigger(XboxController.Axis.kLeftTrigger, true);
        setTrigger(XboxController.Axis.kRightTrigger, true);
        for (var button : new XboxController.Button[] {XboxController.Button.kLeftBumper,
                XboxController.Button.kRightBumper, XboxController.Button.kA,
                XboxController.Button.kB, XboxController.Button.kY}) {
            setButton(button, true);
            assertSame(driveCommand, scheduler.requiring(container.getDrivebase()));
            assertEquals(0, indexer.getFloorTargetRPM());
            assertEquals(0, indexer.getTunnelTargetRPM());
            setButton(button, false);
        }
        setBoard(3, true);
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());
        var auto = container.getAutonomousCommand();
        assertEquals("Mechanisms disabled - Idle", auto.getName());
        assertTrue(auto.getRequirements().isEmpty());
        setBoard(3, false);
        setTrigger(XboxController.Axis.kLeftTrigger, false);
        setTrigger(XboxController.Axis.kRightTrigger, false);
    }

    @Test void xDoesNotRequestVisionResetWhileLimelightsAreDisabled() {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setJoystickAxisCount(DRIVER_PORT, 6);
        DriverStationSim.setJoystickButtonCount(DRIVER_PORT, 10);
        DriverStationSim.setJoystickIsXbox(DRIVER_PORT, true);
        DriverStationSim.notifyNewData();
        new RobotContainer();
        // Vision is disabled: driver X must not request a camera pose reset.
        for (String camera : new String[] {"Front", "Left", "Right"}) {
            SmartDashboard.putString("Vision/" + camera + "/ResetStatus", "Not requested");
        }
        setButton(XboxController.Button.kX, true);
        assertEquals("Not requested", SmartDashboard.getString("Vision/Front/ResetStatus", ""));
        assertEquals("Not requested", SmartDashboard.getString("Vision/Left/ResetStatus", ""));
        assertEquals("Not requested", SmartDashboard.getString("Vision/Right/ResetStatus", ""));
        setButton(XboxController.Button.kX, false);
    }

    @org.junit.jupiter.api.Disabled("Shooter and intake intentionally not initialized during memory investigation")
    @Test void yUsesCalculatorWithoutTakingDrivetrain() throws Exception {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setJoystickAxisCount(DRIVER_PORT, 6);
        DriverStationSim.setJoystickButtonCount(DRIVER_PORT, 10);
        DriverStationSim.setJoystickIsXbox(DRIVER_PORT, true);
        DriverStationSim.notifyNewData();
        RobotContainer container = new RobotContainer();
        Shooter shooter = getField(container, "shooter", Shooter.class);
        Indexer indexer = getField(container, "indexer", Indexer.class);
        Intake intake = getField(container, "intake", Intake.class);
        runCycles(4);
        // Y uses the real calculator while manual steering retains the drivetrain.
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        SmartDashboard.putBoolean("Fuel/Calibration Mode", false);
        container.getDrivebase().resetOdometry(new Pose2d(
            Constants.FieldConstants.BLUE_HUB_CENTER.getX() - 3.5,
            Constants.FieldConstants.BLUE_HUB_CENTER.getY(),
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180)));
        var model = getField(container, "fuelShotModel", frc.robot.utility.shooting.FuelShotModel.class);
        model.setLaunchSpeedCoefficient(frc.robot.utility.shooting.LaunchSpeedTuning.update());
        double calculatedRPM = model.solve(3.5).rpm();
        SmartDashboard.putBoolean("Hub Aim/Active", false);
        setButton(XboxController.Button.kY, true);
        assertEquals("FuelShootManuallyAligned", CommandScheduler.getInstance().requiring(indexer).getName());
        assertEquals(calculatedRPM, shooter.getGoalRPM(), 1e-6);
        assertNotSame(CommandScheduler.getInstance().requiring(indexer),
            CommandScheduler.getInstance().requiring(container.getDrivebase()));
        assertSame(CommandScheduler.getInstance().requiring(indexer),
            CommandScheduler.getInstance().requiring(intake));
        assertFalse(SmartDashboard.getBoolean("Hub Aim/Active", true));
        // Switching from Y to RT takes ownership; releasing RT resumes a still-held Y.
        setTrigger(XboxController.Axis.kRightTrigger, true);
        assertEquals("FuelShootCommand", CommandScheduler.getInstance().requiring(indexer).getName());
        setTrigger(XboxController.Axis.kRightTrigger, false);
        assertEquals("FuelShootManuallyAligned", CommandScheduler.getInstance().requiring(indexer).getName());
        setButton(XboxController.Button.kY, false);
        assertEquals(0, indexer.getTunnelTargetRPM());
        assertEquals(0, indexer.getFloorTargetRPM());

    }

    @org.junit.jupiter.api.Disabled("Shooter and intake intentionally not initialized during memory investigation")
    @Test void bWaitsForSixtyRpmToleranceWithoutAimOrIntakeAndStopsOnRelease() throws Exception {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
        DriverStationSim.setJoystickAxisCount(DRIVER_PORT, 6);
        DriverStationSim.setJoystickButtonCount(DRIVER_PORT, 10);
        DriverStationSim.setJoystickIsXbox(DRIVER_PORT, true);
        DriverStationSim.notifyNewData();
        RobotContainer container = new RobotContainer();
        Shooter shooter = getField(container, "shooter", Shooter.class);
        Indexer indexer = getField(container, "indexer", Indexer.class);
        Intake intake = getField(container, "intake", Intake.class);
        runCycles(4);
        setButton(XboxController.Button.kB, true);
        assertEquals(2700, shooter.getGoalRPM(), 1e-6);
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());
        var raw = CommandScheduler.getInstance().requiring(indexer);
        assertEquals("FuelShootRaw2700", raw.getName());
        Field inputsField = Shooter.class.getDeclaredField("inputs");
        inputsField.setAccessible(true);
        var inputs = (frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs) inputsField.get(shooter);
        inputs.leaderConnected = inputs.followerConnected = true;
        inputs.velocityRadsPerSec = edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(2641);
        raw.execute();
        assertEquals(1200, indexer.getFloorTargetRPM());
        assertEquals(1500, indexer.getTunnelTargetRPM());
        inputs.velocityRadsPerSec = edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(2639);
        raw.execute();
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());
        inputs.velocityRadsPerSec = edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(2759);
        raw.execute();
        assertEquals(1500, indexer.getTunnelTargetRPM());
        inputs.velocityRadsPerSec = edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(2761);
        raw.execute();
        assertEquals(0, indexer.getTunnelTargetRPM());
        assertNotSame(raw, CommandScheduler.getInstance().requiring(container.getDrivebase()));
        assertNotSame(raw, CommandScheduler.getInstance().requiring(intake));
        assertFalse(SmartDashboard.getBoolean("Intake/ForceRetractRequested", false));
        setTrigger(XboxController.Axis.kRightTrigger, true);
        assertSame(raw, CommandScheduler.getInstance().requiring(indexer));
        assertEquals(2700, shooter.getGoalRPM(), 1e-6);
        setTrigger(XboxController.Axis.kRightTrigger, false);
        setButton(XboxController.Button.kB, false);
        assertEquals(0, shooter.getGoalRPM(), 1e-6);
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());
        setButton(XboxController.Button.kB, true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData(); runCycles(4);
        assertEquals(0, indexer.getFloorTargetRPM());
        assertEquals(0, indexer.getTunnelTargetRPM());
    }

    private static void assertIdleAt(RobotContainer container, Shooter shooter,
            AllianceStationID alliance, double x, double expected) {
        DriverStationSim.setAllianceStationId(alliance);
        DriverStationSim.notifyNewData();
        container.getDrivebase().resetOdometry(new Pose2d(x, 4.0, new edu.wpi.first.math.geometry.Rotation2d()));
        runCycles(4);
        assertEquals(expected, shooter.getGoalRPM(), 1e-6);
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
