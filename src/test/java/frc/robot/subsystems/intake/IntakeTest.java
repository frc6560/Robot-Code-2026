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
        double statorLimit = IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
        boolean currentLimitOK = true;
        boolean currentFresh = true;
        int deployStops;
        int forceRequests, wallZeros;
        int forceExtendRequests;
        @Override public void updateInputs(IntakeIOInputs inputs) {
            inputs.deployPositionRotations = position;
            inputs.deployVelocityRPS = velocity;
            inputs.deployStatorCurrentAmps = stator;
            inputs.deployCurrentFresh = currentFresh;
            inputs.deployStatorLimitAmps = statorLimit;
            inputs.deployCurrentLimitOK = currentLimitOK;
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
        @Override public void stopDeploy() { stopped = true; deployStops++; }
        @Override public void stop() { stopped = true; roller = 0.0; }
        @Override public void forceRetract() { target = 0; stopped = false; forceRequests++; }
        @Override public void forceExtend() {
            target = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
            stopped = false;
            forceExtendRequests++;
        }
        @Override public void finishForceRetract() { stopDeploy(); position = 0; wallZeros++; }
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
        assertEquals("Waiting for intake control", SmartDashboard.getString("Intake/BlockedReason", ""));
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

    @Test void extendedRackCannotStartRollersWithoutHeldTriggerAndReleaseStopsImmediately() {
        teleop(true);
        intake.requestRackExtended();
        io.position = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        intake.periodic();
        SimHooks.stepTiming(1);
        intake.periodic();
        assertEquals(0, io.roller);
        intake.setRollersRequested(true);
        intake.periodic();
        SimHooks.stepTiming(IntakeConstants.ROLLER_EXTENSION_SETTLE_SECONDS + 0.01);
        intake.periodic();
        assertEquals(IntakeConstants.ROLLER_RPM, io.roller);
        intake.setRollersRequested(false);
        assertEquals(0, io.roller);
        intake.periodic();
        SimHooks.stepTiming(1);
        intake.periodic();
        assertEquals(0, io.roller);
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
        assertEquals("Intake trigger not held", SmartDashboard.getString("Intake/RollerBlockedReason", ""));
    }

    @Test void rackSelectionsHoldAndRollersStartAtFortyPercentWhileMoving() {
        teleop(true);
        intake.requestRackExtended();
        intake.setRollersRequested(true);
        intake.periodic();
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
        SimHooks.stepTiming(20.0);
        intake.periodic();
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
        assertEquals(Intake.State.EXTENDED_STOPPED, intake.getState());
        assertEquals(0, io.roller);
        io.position = IntakeConstants.EXTENDED_POSITION_ROTATIONS * 0.4;
        io.velocity = 2.0;
        intake.periodic();
        assertEquals(2500, io.roller);
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

    @Test void rollersRequireConfirmedContinuousCurrentLimit() {
        teleop(true);
        intake.requestRackExtended();
        intake.setRollersRequested(true);
        io.position = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        io.statorLimit = 60.0;
        intake.periodic();
        SimHooks.stepTiming(2.0);
        intake.periodic();
        assertEquals(0, io.roller);
        assertEquals("Rack continuous current limit not confirmed",
            SmartDashboard.getString("Intake/RollerBlockedReason", ""));
        io.statorLimit = IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
        io.currentLimitOK = false;
        intake.periodic();
        SimHooks.stepTiming(1.0);
        intake.periodic();
        assertEquals(0, io.roller);
        io.currentLimitOK = true;
        intake.periodic();
        assertEquals(2500, io.roller);
        SimHooks.stepTiming(IntakeConstants.ROLLER_EXTENSION_SETTLE_SECONDS + 0.01);
        intake.periodic();
        assertEquals(IntakeConstants.ROLLER_RPM, io.roller);
        io.statorLimit = 60.0;
        intake.periodic();
        assertEquals(0, io.roller);
    }

    @Test void disablingClearsRequestAndReenableRequiresNewSelection() {
        teleop(true);
        intake.requestRackExtended();
        intake.setRollersRequested(true);
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
        intake.setRollersRequested(true);
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
        intake.setRollersRequested(true);
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


    @Test void rollersRequireFortyPercentAndHealthyControllers() {
        teleop(true);
        intake.requestRackExtended();
        intake.setRollersRequested(true);
        io.position = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        intake.periodic();
        assertEquals(2500, io.roller);
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
        io.position = IntakeConstants.EXTENDED_POSITION_ROTATIONS * 0.39;
        intake.periodic(); assertEquals(0, io.roller); // Slipping out of extension stops them again.
        io.position = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        io.velocity = 1;
        intake.periodic();
        SimHooks.stepTiming(1); intake.periodic(); assertEquals(2500, io.roller);
        io.position = Double.NaN; io.velocity = 0;
        intake.periodic(); assertEquals(0, io.roller);
    }

    private void currentFor(double amps, double seconds) {
        io.stator = amps;
        intake.periodic();
        SimHooks.stepTiming(seconds);
        intake.periodic();
    }

    @Test void sustainedTwoSecondSpikeLatchesRackOffDespiteEncoderAndKeepsRollersRunning() {
        teleop(true);
        intake.requestRackExtended();
        intake.setRollersRequested(true);
        io.position = IntakeConstants.EXTENDED_POSITION_ROTATIONS * 0.2;
        currentFor(45, 2.0);
        assertEquals(1, intake.getDeploySpikeCount());
        assertTrue(intake.isExtensionLimitReached());
        assertTrue(io.stopped);
        assertEquals(2500, io.roller); // Physical limit overrides the inaccurate encoder fraction.
        int requests = io.positionRequests;
        currentFor(0, 1.0);
        assertEquals(requests, io.positionRequests); // Must not resume pushing the hard stop.
        intake.setRollersRequested(false);
        intake.periodic();
        assertEquals(0, io.roller);
        assertEquals(requests, io.positionRequests);
        intake.requestRackRetracted();
        intake.periodic();
        assertFalse(intake.isExtensionLimitReached());
        assertFalse(io.stopped);
        intake.requestRackExtended();
        intake.setRollersRequested(true);
        intake.periodic();
        assertEquals(0, intake.getDeploySpikeCount());
    }

    @Test void retractionKeepsZeroTargetDespiteRepeatedHighCurrentSpikes() {
        teleop(true);
        intake.requestRackExtended();
        io.position = 7.0;
        intake.periodic();
        intake.requestRackRetracted();
        currentFor(35, 0.10);
        currentFor(10, 0.15);
        currentFor(35, 0.10);
        assertEquals(0, intake.getDeploySpikeCount());
        assertFalse(intake.isExtensionLimitReached());
        assertEquals(IntakeConstants.RETRACTED_POSITION_ROTATIONS, io.target);
        assertFalse(io.stopped);
        assertEquals(0, io.roller);
    }

    @Test void driverConfirmationStopsRackAtPartialEncoderAndPreservesRollerRequest() {
        teleop(true);
        intake.requestRackExtended();
        intake.setRollersRequested(true);
        io.position = 1.0;
        intake.periodic();
        assertEquals(0, io.roller);
        int requests = io.positionRequests;
        intake.confirmFullyExtended();
        assertTrue(io.stopped); // Immediate stop, without waiting for periodic.
        intake.periodic();
        assertTrue(intake.atDeployTarget());
        assertTrue(intake.isExtensionLimitReached());
        assertEquals(requests, io.positionRequests);
        assertEquals(IntakeConstants.ROLLER_RPM, io.roller);
        assertEquals(1.0, io.position); // Confirmation does not rewrite the encoder.
        intake.requestRackRetracted();
        intake.periodic();
        assertFalse(intake.isExtensionLimitReached());
        assertFalse(io.stopped);
        assertEquals(0, io.target);
        assertEquals(0, io.roller);
        intake.requestRackExtended();
        intake.periodic();
        assertFalse(intake.isExtensionLimitReached());
        intake.confirmFullyExtended();
        teleop(false);
        assertFalse(intake.isExtensionLimitReached());
    }

    @Test void driverConfirmationCannotBypassDisabledPitCoastAutoOrMissingReference() {
        intake.confirmFullyExtended();
        assertFalse(intake.isExtensionLimitReached());
        teleop(true);
        intake.confirmFullyExtended();
        assertTrue(intake.isExtensionLimitReached());
        assertEquals(0, io.roller); // Does not invent a roller request.
        intake.setPitCoastMode(true);
        intake.confirmFullyExtended();
        assertFalse(intake.isExtensionLimitReached());
        intake.setPitCoastMode(false);
        io.homed = false;
        intake.periodic();
        intake.confirmFullyExtended();
        assertFalse(intake.isExtensionLimitReached());
        io.homed = true;
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        intake.periodic();
        intake.confirmFullyExtended();
        assertFalse(intake.isExtensionLimitReached());
    }

    @Test void fortyAmpsDoesNotCountButOverFortyForTwoSecondsDoes() {
        teleop(true);
        intake.requestRackExtended();
        currentFor(40, 3.0);
        assertEquals(0, intake.getDeploySpikeCount());
        currentFor(45, 2.0);
        assertEquals(1, intake.getDeploySpikeCount());
    }

    @Test void shortNoiseStaleCurrentAndDropsToFortyResetTheContinuousTimer() {
        teleop(true);
        intake.requestRackExtended();
        currentFor(45, 1.99);
        currentFor(0, 0.10);
        assertEquals(0, intake.getDeploySpikeCount());
        io.currentFresh = false;
        currentFor(45, 2.5);
        assertEquals(0, intake.getDeploySpikeCount());
        io.currentFresh = true;
        currentFor(45, 1.99);
        currentFor(40, 0.02);
        currentFor(45, 1.99);
        assertEquals(0, intake.getDeploySpikeCount());
        assertFalse(intake.isExtensionLimitReached());
        currentFor(45, 0.01);
        assertEquals(1, intake.getDeploySpikeCount());
        assertTrue(intake.isExtensionLimitReached());
        intake.requestRackExtended(); // A new trigger press starts a fresh count.
        assertEquals(0, intake.getDeploySpikeCount());
    }

    @Test void forceRetractIgnoresEncoderEndpointAndStopsAndZerosOnlyOnSustainedWallSpike() {
        teleop(true);
        io.position = -1.0; // The old encoder zero must not stop forced retraction.
        intake.requestForceRetracted();
        intake.periodic();
        assertEquals(1, io.forceRequests);
        assertFalse(intake.atDeployTarget());
        currentFor(45, 1.99);
        assertFalse(io.stopped);
        assertEquals(0, io.wallZeros);
        currentFor(40, 0.02); // Break continuity.
        currentFor(45, 1.99);
        assertFalse(io.stopped);
        currentFor(45, 0.01);
        assertTrue(io.stopped);
        assertTrue(intake.isRetractionLimitReached());
        assertTrue(intake.atDeployTarget());
        assertEquals(1, io.wallZeros);
        assertEquals(0, io.position);
        int requests = io.forceRequests;
        currentFor(45, 5.0);
        assertEquals(requests, io.forceRequests);
        assertEquals(1, io.wallZeros);
        intake.stop(); // Control release clears latch and stops immediately.
        assertFalse(intake.isRetractionLimitReached());
        intake.requestForceRetracted();
        intake.periodic();
        assertEquals(requests + 1, io.forceRequests);
        assertFalse(io.stopped);
        intake.stop();
        intake.periodic();
        assertTrue(io.stopped);
        assertEquals(requests + 1, io.forceRequests);
    }

    @Test void forceExtensionGoesOutFromHalfwayAndLatchesOffUntilRelease() {
        teleop(true);
        io.position = IntakeConstants.EXTENDED_POSITION_ROTATIONS / 2;
        intake.requestForceExtended();
        intake.periodic();
        assertEquals(1, io.forceExtendRequests);
        assertEquals(0, io.forceRequests);
        assertFalse(intake.atDeployTarget());
        currentFor(45, 1.99);
        assertFalse(io.stopped);
        currentFor(45, 0.01);
        assertTrue(io.stopped);
        assertTrue(intake.isExtensionLimitReached());
        assertTrue(intake.atDeployTarget());
        assertEquals(0, io.wallZeros); // An extended stall must never establish the retracted zero.
        int requests = io.forceExtendRequests;
        currentFor(45, 5.0);
        assertEquals(requests, io.forceExtendRequests);
        intake.stop();
        assertFalse(intake.isExtensionLimitReached());
        intake.requestForceExtended();
        intake.periodic();
        assertEquals(requests + 1, io.forceExtendRequests);
        intake.requestRackExtended();
        intake.periodic();
        assertEquals(requests + 1, io.forceExtendRequests);
        assertTrue(io.positionRequests > 0);
    }

    @Test void simulatedForceExtensionStopsAtExtendedWall() {
        teleop(true);
        var simulatedIO = new IntakeIOSim();
        var simulatedIntake = new Intake(simulatedIO);
        try {
            simulatedIntake.periodic();
            simulatedIntake.requestForceExtended();
            for (int i = 0; i < 300; i++) {
                SimHooks.stepTiming(0.02);
                simulatedIntake.periodic();
            }
            assertTrue(simulatedIntake.isExtensionLimitReached());
            assertFalse(simulatedIntake.isRetractionLimitReached());
            var feedback = new IntakeIO.IntakeIOInputs();
            simulatedIO.updateInputs(feedback);
            assertEquals(0, feedback.deployAppliedVolts);
            assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, feedback.deployPositionRotations, 1e-6);
        } finally {
            simulatedIntake.stop();
            CommandScheduler.getInstance().unregisterSubsystem(simulatedIntake);
        }
    }

    @Test void simulatedForceRetractReachesWallAndStopsAfterTwoContinuousSeconds() {
        teleop(true);
        var simulatedIO = new IntakeIOSim();
        var simulatedIntake = new Intake(simulatedIO);
        try {
            simulatedIntake.periodic();
            simulatedIntake.requestForceRetracted();
            for (int i = 0; i < 99; i++) {
                SimHooks.stepTiming(0.02);
                simulatedIntake.periodic();
            }
            assertFalse(simulatedIntake.isRetractionLimitReached());
            for (int i = 0; i < 5; i++) {
                SimHooks.stepTiming(0.02);
                simulatedIntake.periodic();
            }
            assertTrue(simulatedIntake.isRetractionLimitReached());
            assertTrue(simulatedIntake.atDeployTarget());
            simulatedIntake.periodic();
            var feedback = new IntakeIO.IntakeIOInputs();
            simulatedIO.updateInputs(feedback);
            assertEquals(0, feedback.deployAppliedVolts);
            assertEquals(0, feedback.deployPositionRotations);
        } finally {
            simulatedIntake.stop();
            CommandScheduler.getInstance().unregisterSubsystem(simulatedIntake);
        }
    }

    @Test void forceRetractStopsOnLostFeedbackDisableAndPitCoast() {
        teleop(true);
        intake.requestForceRetracted();
        intake.periodic();
        io.ready = false;
        intake.periodic();
        assertTrue(io.stopped);
        io.ready = true;
        intake.periodic();
        assertFalse(io.stopped);
        intake.setPitCoastMode(true);
        intake.periodic();
        assertTrue(io.stopped);
        intake.setPitCoastMode(false);
        intake.requestForceRetracted();
        intake.periodic();
        teleop(false);
        teleop(true);
        assertTrue(io.stopped);
        assertFalse(intake.isRetractionLimitReached());
    }

    @Test void sustainedLowSpeedAndHighCurrentDoNotCancelRackRequest() {
        teleop(true);
        intake.requestRackExtended();
        intake.setRollersRequested(true);
        io.stator = IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
        intake.periodic();
        SimHooks.stepTiming(2.0); intake.periodic();
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
        assertFalse(SmartDashboard.getBoolean("Intake/DeployStallFault", true));
        assertEquals(0, io.roller); // Extension interlock remains active.
        intake.requestRackRetracted(); intake.periodic();
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.RETRACTED_POSITION_ROTATIONS, io.target);
    }
}
