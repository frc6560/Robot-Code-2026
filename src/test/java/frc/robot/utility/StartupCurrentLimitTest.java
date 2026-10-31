package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.*;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class StartupCurrentLimitTest {
    double now;
    boolean accept = true;
    int stops;
    List<Double> writes = new ArrayList<>();
    StartupCurrentLimit limit = new StartupCurrentLimit(40, 60, 1, () -> now,
        amps -> { writes.add(amps); return accept; }, () -> stops++);

    @Test void repeatedRequestsCannotExtendOneSecondAndNoRepeatedConfigWrites() {
        assertTrue(limit.update(true, false, true));
        assertEquals(60, limit.appliedAmps());
        now = 0.5; limit.update(true, false, true);
        now = 0.999; limit.update(true, false, true);
        assertEquals(List.of(60.0), writes);
        now = 1.0; limit.update(true, false, true);
        assertEquals(40, limit.appliedAmps());
        now = 10; limit.update(true, false, true);
        assertEquals(List.of(60.0, 40.0), writes);
        assertEquals(0, stops); // No stall cutoff: the request remains usable at 40 A.
    }

    @Test void stoppingImmediatelyRestores40AndNewStartGetsItsOwnWindow() {
        limit.update(true, false, true);
        now = 0.2; limit.update(false, false, false);
        assertEquals(40, limit.appliedAmps());
        now = 0.4; limit.update(true, false, true);
        assertEquals(60, limit.appliedAmps());
        now = 1.4; limit.update(true, false, true);
        assertEquals(40, limit.appliedAmps());
    }

    @Test void rackGoalChangeRestartsButAlreadyReachedGoalUses40() {
        limit.update(true, false, true);
        now = 2; limit.update(true, false, true);
        assertEquals(40, limit.appliedAmps());
        limit.update(true, true, true); // Explicitly select a new rack position.
        assertEquals(60, limit.appliedAmps());
        now = 2.1; limit.update(true, false, false); // Arrived early: no boost while holding.
        assertEquals(40, limit.appliedAmps());
        now = 3; limit.update(true, false, true); // Drift does not create another startup window.
        assertEquals(40, limit.appliedAmps());
    }

    @Test void failedReductionStopsMotorAndRetries40WithoutRenewingBoost() {
        limit.update(true, false, true);
        now = 1; accept = false;
        assertFalse(limit.update(true, false, true));
        assertFalse(limit.confirmed());
        assertEquals(1, stops);
        now = 1.02; accept = true;
        assertTrue(limit.update(true, false, true));
        assertEquals(40, limit.appliedAmps());
        assertEquals(List.of(60.0, 40.0, 40.0), writes);
    }

    @Test void failedBoostCannotLeaveMotorRunningAndRetriesDoNotExtendDeadline() {
        accept = false;
        assertFalse(limit.update(true, false, true));
        assertEquals(1, stops);
        now = 1.5; accept = true;
        assertTrue(limit.update(true, false, true));
        assertEquals(40, limit.appliedAmps());
        assertEquals(List.of(60.0, 40.0), writes);
    }
}
