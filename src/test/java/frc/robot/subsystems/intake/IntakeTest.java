package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import org.junit.jupiter.api.*;

class IntakeTest {
    private static class FakeIO implements IntakeIO {
        double target;
        double roller;
        boolean stopped;
        boolean ready = true;
        boolean configured = true;
        boolean homed = true;
        @Override public void updateInputs(IntakeIOInputs inputs) {
            inputs.deployConnected = ready;
            inputs.configured = ready && configured;
            inputs.deployHomed = ready && homed;
            // No roller is connected or required for the rack-only test.
        }
        @Override public void setDeployPosition(double position) { target = position; stopped = false; }
        @Override public void setRollerRPM(double rpm) { roller = rpm; }
        @Override public void stop() { stopped = true; }
    }
    private FakeIO io;
    private Intake intake;

    @BeforeEach void setup() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
        io = new FakeIO();
        intake = new Intake(io);
    }

    @AfterEach void cleanup() {
        CommandScheduler.getInstance().unregisterSubsystem(intake);
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }

    private void teleop(boolean enabled) {
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
        intake.periodic();
    }

    @Test void extendsToEightyPercentAndHoldsWithoutTimedRetraction() {
        teleop(true);
        assertEquals(IntakeConstants.ENABLE_POSITION_ROTATIONS, io.target);
        assertFalse(io.stopped);
        assertTrue(intake.isReady());
        SimHooks.stepTiming(20.0);
        intake.periodic();
        assertEquals(IntakeConstants.ENABLE_POSITION_ROTATIONS, io.target);
        assertEquals(Intake.State.EXTENDED_STOPPED, intake.getState());
        assertEquals(0.0, io.roller);
        assertEquals(1000.0, SmartDashboard.getNumber("Intake/RollerRequestedRPM", -1));
        assertEquals(0.0, SmartDashboard.getNumber("Intake/RollerCommandedRPM", -1));
        assertTrue(SmartDashboard.getString("Intake/RollerBlockedReason", "").contains("conflicts"));
    }

    @Test void delayedMotorReadinessStillExtendsToEightyPercent() {
        io.ready = false;
        teleop(true);
        SimHooks.stepTiming(5.0);
        intake.periodic();
        assertTrue(io.stopped);
        assertTrue(SmartDashboard.getString("Intake/BlockedReason", "").contains("26"));
        io.ready = true;
        intake.periodic();
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.ENABLE_POSITION_ROTATIONS, io.target);
        SimHooks.stepTiming(20.0);
        intake.periodic();
        assertEquals(IntakeConstants.ENABLE_POSITION_ROTATIONS, io.target);
    }

    @Test void configurationAndZeroFailuresHaveVisibleReasonsAndBlockMotion() {
        io.configured = false;
        teleop(true);
        assertTrue(io.stopped);
        assertTrue(SmartDashboard.getString("Intake/BlockedReason", "").contains("configuration failed"));
        io.configured = true;
        io.homed = false;
        intake.periodic();
        assertTrue(io.stopped);
        assertTrue(SmartDashboard.getString("Intake/BlockedReason", "").contains("zero failed"));
        io.homed = true;
        intake.periodic();
        assertFalse(io.stopped);
        assertEquals("None", SmartDashboard.getString("Intake/BlockedReason", ""));
    }

    @Test void disableStopsImmediatelyAndReenableResumesSameTarget() {
        teleop(true);
        SimHooks.stepTiming(0.5);
        teleop(false);
        assertTrue(io.stopped);
        assertEquals(0.0, io.roller);
        assertEquals(0.0, SmartDashboard.getNumber("Intake/RollerRequestedRPM", -1));
        teleop(true);
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.ENABLE_POSITION_ROTATIONS, io.target);
        SimHooks.stepTiming(20.0);
        intake.periodic();
        assertEquals(IntakeConstants.ENABLE_POSITION_ROTATIONS, io.target);
    }

    @Test void disabledStopsAndAllEnabledModesHoldEightyPercent() {
        intake.periodic();
        assertTrue(io.stopped);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        intake.periodic();
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.ENABLE_POSITION_ROTATIONS, io.target);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(true);
        DriverStationSim.notifyNewData();
        intake.periodic();
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.ENABLE_POSITION_ROTATIONS, io.target);
    }

    @Test void legacyIntakeAndShotCallsCannotOverrideEnableTarget() {
        teleop(true);
        intake.activate();
        intake.activateOuttake();
        intake.setRetracted();
        intake.setExtendedStopped();
        intake.startShootingOscillation();
        intake.periodic();
        assertEquals(IntakeConstants.ENABLE_POSITION_ROTATIONS, io.target);
        assertEquals(0.0, io.roller);
        assertFalse(intake.isActive());
        assertFalse(intake.isShootingOscillationActive());
        SimHooks.stepTiming(20.0);
        intake.activate();
        intake.periodic();
        assertEquals(IntakeConstants.ENABLE_POSITION_ROTATIONS, io.target);
    }

    @Test void feedbackFaultAndPitCoastStopOutput() {
        teleop(true);
        io.ready = false;
        intake.periodic();
        assertTrue(io.stopped);
        assertFalse(intake.isReady());
        io.ready = true;
        intake.setPitCoastMode(true);
        intake.periodic();
        assertTrue(io.stopped);
        intake.setPitCoastMode(false);
        intake.periodic();
        assertFalse(io.stopped);
    }
}
