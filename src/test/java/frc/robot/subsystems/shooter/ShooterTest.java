package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.ShooterConstants;

class ShooterTest {
    private static final double SHOT_RPM = 3500.0;

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void enable() {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void reset() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    private static void step(Shooter shooter, int cycles) {
        for (int i = 0; i < cycles; i++) {
            shooter.periodic();
        }
    }

    @Test
    void setpointIsRateLimitedAndMeasuredSpeedFollowsIt() {
        Shooter shooter = new Shooter(new ShooterIOSim());
        shooter.setGoal(SHOT_RPM);

        step(shooter, 1);
        assertTrue(shooter.getSetpointRPM() < SHOT_RPM, "setpoint should ramp, not jump");
        assertFalse(shooter.atTarget());

        int cycles = 0;
        while (!shooter.atTarget() && cycles < 500) {
            step(shooter, 1);
            cycles++;
            assertTrue(shooter.getSetpointRPM() <= SHOT_RPM + 1e-6);
        }
        assertTrue(shooter.atTarget(), "setpoint never reached the goal");
        assertTrue(cycles > 5, "spin-up from rest can't be near-instant at this inertia");

        step(shooter, 50);
        assertTrue(
            shooter.withinTolerance(ShooterConstants.FLYWHEEL_RPM_TOLERANCE),
            "measured " + shooter.getCurrentRPM() + " RPM vs goal " + SHOT_RPM);
    }

    @Test
    void bangBangReachesGoal() {
        Shooter shooter = new Shooter(new ShooterIOSim());
        shooter.setGoal(SHOT_RPM, true);
        step(shooter, 250);

        assertTrue(shooter.atTarget());
        assertTrue(
            shooter.withinTolerance(ShooterConstants.FLYWHEEL_RPM_TOLERANCE),
            "measured " + shooter.getCurrentRPM() + " RPM vs goal " + SHOT_RPM);
    }

    @Test
    void idleIsNeverReadyAndSpinsTheWheel() {
        Shooter shooter = new Shooter(new ShooterIOSim());
        shooter.setIdle();
        step(shooter, 150);

        assertFalse(shooter.atTarget());
        assertEquals(ShooterConstants.FLYWHEEL_IDLE_RPM, shooter.getCurrentRPM(), 150.0);
    }

    @Test
    void stopReseedsSetpointFromMeasuredSpeed() {
        Shooter shooter = new Shooter(new ShooterIOSim());
        shooter.setGoal(SHOT_RPM);
        step(shooter, 150);

        shooter.stop();
        step(shooter, 1);
        assertFalse(shooter.atTarget());
        assertEquals(shooter.getCurrentRPM(), shooter.getSetpointRPM(), 50.0);
    }

    @Test
    void disabledCoastsRegardlessOfGoal() {
        Shooter shooter = new Shooter(new ShooterIOSim());
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        shooter.setGoal(SHOT_RPM);
        step(shooter, 50);

        assertFalse(shooter.atTarget());
        assertEquals(0.0, shooter.getCurrentRPM(), 1.0);
    }
}
