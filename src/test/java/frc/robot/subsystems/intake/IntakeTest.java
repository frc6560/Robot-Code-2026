package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class IntakeTest {
    @BeforeAll static void initializeHal() { assertTrue(HAL.initialize(500, 0)); }

    private static class FakeIO implements IntakeIO {
        double volts;
        double current = 5;
        boolean connected = true;
        boolean valid = true;
        @Override public void updateInputs(IntakeIOInputs inputs) {
            inputs.motorConnected = connected;
            inputs.motorCurrentValid = valid;
            inputs.motorStatorCurrentAmps = current;
        }
        @Override public void setMotorVoltage(double volts) { this.volts = volts; }
        @Override public void stop() { volts = 0; }
    }

    private static class Fixture implements AutoCloseable {
        final FakeIO io = new FakeIO();
        double now;
        boolean enabled = true;
        final Intake intake = new Intake(io, () -> now, () -> enabled,
            new IntakeEndStopTracker.Settings(25, 20, .15, .10, 2, .10));
        void step(double time, double amps) { now = time; io.current = amps; intake.periodic(); }
        @Override public void close() {
            intake.setPitCoastMode(true);
            CommandScheduler.getInstance().unregisterSubsystem(intake);
        }
    }

    @Test void cyclesOnlyAfterConfirmedEndStopsAndPausesBeforeReversing() {
        try (var f = new Fixture()) {
            f.intake.activate();
            f.step(0, 5);
            assertTrue(f.io.volts < 0, "Unknown position first homes toward the rear");
            assertEquals("UNKNOWN", f.intake.getInferredEndpoint());
            f.step(.20, 22);
            f.step(.30, 22);
            assertEquals(0, f.io.volts);
            assertEquals("RETRACTED", f.intake.getInferredEndpoint());
            f.step(.32, 0);
            assertEquals(0, f.io.volts, "Pause before reversal");
            f.step(.41, 5);
            assertTrue(f.io.volts > 0);
            f.step(.60, 30);
            f.step(.70, 30);
            assertEquals(0, f.io.volts);
            assertEquals("EXTENDED", f.intake.getInferredEndpoint());
            f.step(.81, 5);
            assertTrue(f.io.volts < 0);
            f.step(1.0, 22);
            f.step(1.1, 22);
            assertEquals(0, f.io.volts);
            assertEquals("RETRACTED", f.intake.getInferredEndpoint());
            assertEquals(1, f.intake.getCompletedCycles());
            f.intake.setExtendedStopped();
            f.step(1.5, 0);
            assertEquals(0, f.io.volts, "Releasing the button ends cycling");
        }
    }

    @Test void retractCommandStopsAtRearAndDoesNotRestartAgainstIt() {
        try (var f = new Fixture()) {
            f.intake.setRetracted();
            f.step(0, 5);
            f.step(.2, 22);
            f.step(.3, 22);
            assertTrue(f.intake.atDeployTarget());
            f.step(1.5, 0);
            assertEquals(0, f.io.volts);
            assertEquals("RETRACTED", f.intake.getInferredEndpoint());
        }
    }

    @Test void timeoutStopsCyclingUntilAnExplicitReset() {
        try (var f = new Fixture()) {
            f.intake.activate();
            f.step(0, 5);
            f.step(2, 5);
            assertEquals(0, f.io.volts);
            assertFalse(f.intake.getFault().isEmpty());
            f.step(3, 5);
            assertEquals(0, f.io.volts);
            assertEquals(0, f.intake.getCompletedCycles());
            f.intake.clearFault();
            assertFalse(f.intake.isActive());
            f.intake.activate();
            f.step(3.2, 5);
            assertTrue(f.io.volts < 0);
        }
    }

    @Test void disconnectAndDisabledOutputsCannotRunTheIntake() {
        try (var f = new Fixture()) {
            f.intake.activate();
            f.step(0, 5);
            f.io.connected = false;
            f.step(.2, 5);
            assertEquals(0, f.io.volts);
            assertFalse(f.intake.getFault().isEmpty());
            f.intake.clearFault();
            f.io.connected = true;
            f.enabled = false;
            f.intake.activate();
            f.step(.4, 50);
            assertEquals(0, f.io.volts);
            assertEquals("UNKNOWN", f.intake.getInferredEndpoint());
            assertEquals("", f.intake.getFault());
        }
    }

    @Test void shootingCyclesAndCoastOrReleaseStopsOutputs() {
        try (var f = new Fixture()) {
            f.intake.setExtendedStopped();
            f.intake.startShootingOscillation();
            f.step(0, 5);
            assertTrue(f.io.volts < 0);
            f.intake.stopShootingOscillation();
            f.step(.2, 5);
            assertEquals(0, f.io.volts);
            f.intake.activate();
            f.step(.3, 5);
            f.intake.setPitCoastMode(true);
            f.step(.4, 5);
            assertEquals(0, f.io.volts);
            assertFalse(f.intake.isShootingOscillationActive());
        }
    }

    @Test void brokenIntakeStaysDisabledThroughNormalPublicConstructor() {
        var io = new FakeIO();
        var intake = new Intake(io);
        try {
            intake.activate();
            intake.startShootingOscillation();
            intake.periodic();
            assertEquals(0, io.volts);
            assertEquals("UNKNOWN", intake.getInferredEndpoint());
        } finally { CommandScheduler.getInstance().unregisterSubsystem(intake); }
    }

    @Test void simulationCompletesRepeatedTripsWithoutAnEncoder() {
        double[] clock = {0};
        var intake = new Intake(new IntakeIOSim(), () -> clock[0], () -> true,
            new IntakeEndStopTracker.Settings(25, 25, .15, .1, 10, .15));
        try {
            intake.activate();
            for (int i = 0; i < 1600; i++) {
                clock[0] = i * .02;
                intake.periodic();
            }
            assertTrue(intake.getCompletedCycles() >= 2);
            assertEquals("", intake.getFault());
        } finally { CommandScheduler.getInstance().unregisterSubsystem(intake); }
    }
}
