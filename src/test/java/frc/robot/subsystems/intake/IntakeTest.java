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
        double position, stator, velocity;
        @Override public void updateInputs(IntakeIOInputs inputs) {
            inputs.deployPositionRotations = position;
            inputs.deployVelocityRPS = velocity;
            inputs.deployStatorCurrentAmps = stator;
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

    @Test void enablingNeverEnergizesRackOrRollersWithoutExtension() {
        teleop(true);
        SimHooks.stepTiming(20.0);
        intake.periodic();
        assertTrue(io.stopped);
        assertEquals(0, io.positionRequests);
        assertFalse(SmartDashboard.getBoolean("Intake/RackRequested", true));
        assertEquals("Waiting for LT/RT rack request", SmartDashboard.getString("Intake/BlockedReason", ""));
        assertEquals(0, io.roller);
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

    @Test void rackSelectionsHoldAndRollersWaitForMeasuredFullExtension() {
        teleop(true);
        intake.requestRackExtended();
        intake.periodic();
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
        SimHooks.stepTiming(20.0);
        intake.periodic();
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
        assertEquals(Intake.State.EXTENDED_STOPPED, intake.getState());
        assertEquals(0, io.roller);
        io.position = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        intake.periodic();
        assertEquals(0, io.roller); // Endpoint must settle before rollers can start.
        SimHooks.stepTiming(IntakeConstants.ROLLER_EXTENSION_SETTLE_SECONDS + 0.01);
        intake.periodic();
        assertEquals(IntakeConstants.ROLLER_RPM, io.roller);
        assertEquals(Intake.State.EXTENDED_SPINNING, intake.getState());
        intake.requestRackRetracted();
        assertEquals(0, io.roller); // Stop immediately, before the next periodic cycle.
        intake.periodic();
        assertEquals(IntakeConstants.RETRACTED_POSITION_ROTATIONS, io.target);
        assertFalse(io.stopped);
        assertEquals(0, io.roller);
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
        assertEquals(0, io.roller);
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
        assertEquals(0, io.roller);
        assertFalse(intake.isShootingOscillationActive());
    }

    @Test void rollersRequireHomingFullPositionAndHealthyRollerControllers() {
        teleop(true);
        intake.requestRackExtended();
        io.position = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        intake.periodic();
        assertEquals(0, io.roller); // Endpoint must settle before rollers can start.
        SimHooks.stepTiming(IntakeConstants.ROLLER_EXTENSION_SETTLE_SECONDS + 0.01);
        intake.periodic();
        assertEquals(IntakeConstants.ROLLER_RPM, io.roller);
        io.homed = false;
        intake.periodic();
        assertEquals(0, io.roller);
        io.homed = true; io.rollersConnected = false;
        intake.periodic(); assertEquals(0, io.roller);
        io.rollersConnected = true; io.rollersConfigured = false;
        intake.periodic(); assertEquals(0, io.roller);
        io.rollersConfigured = true;
        intake.periodic();
        SimHooks.stepTiming(IntakeConstants.ROLLER_EXTENSION_SETTLE_SECONDS + 0.01);
        intake.periodic(); assertEquals(IntakeConstants.ROLLER_RPM, io.roller);
        io.position -= 1;
        intake.periodic(); assertEquals(0, io.roller); // Slipping out of extension stops them again.
        io.position = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        io.velocity = 1;
        intake.periodic();
        SimHooks.stepTiming(1); intake.periodic(); assertEquals(0, io.roller);
        io.position = Double.NaN; io.velocity = 0;
        intake.periodic(); assertEquals(0, io.roller);
    }

    @Test void sustainedRackStallStopsAndLatchesUntilDisabled() {
        teleop(true);
        intake.requestRackExtended();
        io.stator = IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
        intake.periodic();
        SimHooks.stepTiming(0.5); intake.periodic();
        assertFalse(io.stopped); // Allow breakaway effort briefly.
        SimHooks.stepTiming(0.6); intake.periodic();
        assertTrue(io.stopped);
        assertTrue(SmartDashboard.getBoolean("Intake/DeployStallFault", false));
        io.stator = 0;
        intake.requestRackExtended(); intake.periodic();
        assertTrue(io.stopped); // A repeated button cannot defeat the latched fault.
        teleop(false); teleop(true);
        intake.requestRackExtended(); intake.periodic();
        assertFalse(io.stopped);
    }
}
