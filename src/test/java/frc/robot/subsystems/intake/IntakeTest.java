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
        int positionRequests;
        boolean ready = true;
        boolean configured = true;
        boolean homed = true;
        boolean rollersConnected = true;
        boolean rollersConfigured = true;
        @Override public void updateInputs(IntakeIOInputs inputs) {
            inputs.deployConnected = ready;
            inputs.configured = ready && configured;
            inputs.deployHomed = ready && homed;
            inputs.rollerConnected = rollersConnected;
            inputs.rollerConfigured = rollersConfigured;
        }
        @Override public void setDeployPosition(double position) {
            target = position;
            stopped = false;
            positionRequests++;
        }
        @Override public void setRollerRPM(double rpm) { roller = rpm; }
        @Override public void stop() { stopped = true; roller = 0.0; }
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

    @Test void enablingNeverEnergizesRackWithoutRequestButStillRunsRollers() {
        teleop(true);
        SimHooks.stepTiming(20.0);
        intake.periodic();
        assertTrue(io.stopped);
        assertEquals(0, io.positionRequests);
        assertFalse(SmartDashboard.getBoolean("Intake/RackRequested", true));
        assertEquals("Waiting for LT/RT rack request", SmartDashboard.getString("Intake/BlockedReason", ""));
        assertEquals(IntakeConstants.ROLLER_RPM, io.roller);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        intake.periodic();
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(true);
        DriverStationSim.notifyNewData();
        intake.periodic();
        assertEquals(0, io.positionRequests);
        assertTrue(io.stopped);
    }

    @Test void rackSelectionsHoldUntilAnotherSelectionWithoutChangingRollers() {
        teleop(true);
        intake.requestRackExtended();
        intake.periodic();
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
        SimHooks.stepTiming(20.0);
        intake.periodic();
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
        assertEquals(Intake.State.EXTENDED_SPINNING, intake.getState());
        intake.requestRackRetracted();
        intake.periodic();
        assertEquals(IntakeConstants.RETRACTED_POSITION_ROTATIONS, io.target);
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.ROLLER_RPM, io.roller);
    }

    @Test void disablingClearsRequestAndReenableRequiresNewSelection() {
        teleop(true);
        intake.requestRackExtended();
        intake.periodic();
        teleop(false);
        assertTrue(io.stopped);
        assertEquals(0.0, io.roller);
        int requestsBeforeReenable = io.positionRequests;
        teleop(true);
        assertTrue(io.stopped);
        assertEquals(requestsBeforeReenable, io.positionRequests);
        intake.requestRackRetracted();
        intake.periodic();
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.RETRACTED_POSITION_ROTATIONS, io.target);
    }

    @Test void configurationHomingAndConnectionFaultsBlockRequestedMotion() {
        teleop(true);
        intake.requestRackExtended();
        io.configured = false;
        intake.periodic();
        assertTrue(io.stopped);
        assertTrue(SmartDashboard.getString("Intake/BlockedReason", "").contains("configuration failed"));
        io.configured = true;
        io.homed = false;
        intake.periodic();
        assertTrue(io.stopped);
        assertTrue(SmartDashboard.getString("Intake/BlockedReason", "").contains("zero failed"));
        io.homed = true;
        io.ready = false;
        intake.periodic();
        assertTrue(io.stopped);
        assertEquals(0, io.positionRequests);
        io.ready = true;
        intake.periodic();
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
    }

    @Test void pitCoastClearsRackRequestAndStopsRollers() {
        teleop(true);
        intake.requestRackExtended();
        intake.periodic();
        intake.setPitCoastMode(true);
        intake.periodic();
        assertTrue(io.stopped);
        assertEquals(0.0, io.roller);
        intake.setPitCoastMode(false);
        intake.periodic();
        assertTrue(io.stopped);
        assertEquals(IntakeConstants.ROLLER_RPM, io.roller);
        intake.requestRackRetracted();
        intake.periodic();
        assertFalse(io.stopped);
    }

    @Test void legacyIntakeAndShotCallsCannotMoveRack() {
        teleop(true);
        intake.activate();
        intake.activateOuttake();
        intake.setRetracted();
        intake.setExtendedStopped();
        intake.startShootingOscillation();
        intake.periodic();
        assertTrue(io.stopped);
        assertEquals(0, io.positionRequests);
        assertEquals(IntakeConstants.ROLLER_RPM, io.roller);
        assertFalse(intake.isShootingOscillationActive());
    }

    @Test void rollersRunWithoutRackHomingButStopOnTheirOwnFaults() {
        io.homed = false;
        teleop(true);
        assertTrue(io.stopped);
        assertEquals(IntakeConstants.ROLLER_RPM, io.roller);
        io.rollersConnected = false;
        intake.periodic();
        assertEquals(0.0, io.roller);
        io.rollersConnected = true;
        io.rollersConfigured = false;
        intake.periodic();
        assertEquals(0.0, io.roller);
        io.rollersConfigured = true;
        intake.periodic();
        assertEquals(IntakeConstants.ROLLER_RPM, io.roller);
    }
}
