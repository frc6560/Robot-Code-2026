package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.*;
import frc.robot.subsystems.intake.IntakeEndStopTracker.Direction;
import frc.robot.subsystems.intake.IntakeEndStopTracker.Endpoint;
import org.junit.jupiter.api.Test;

class IntakeEndStopTrackerTest {
    private IntakeEndStopTracker tracker() {
        return new IntakeEndStopTracker(new IntakeEndStopTracker.Settings(25, 20, .15, .10, 2, .10));
    }

    @Test void startupAndShortSpikesDoNotCreateAnEndpoint() {
        var tracker = tracker();
        assertEquals(Direction.EXTENDING, tracker.update(Direction.EXTENDING, 35, true, 0));
        tracker.update(Direction.EXTENDING, 35, true, .10);
        tracker.update(Direction.EXTENDING, 5, true, .16);
        tracker.update(Direction.EXTENDING, 35, true, .30);
        tracker.update(Direction.EXTENDING, 5, true, .38);
        tracker.update(Direction.EXTENDING, 35, true, .40);
        assertEquals(Direction.EXTENDING, tracker.update(Direction.EXTENDING, 35, true, .48));
        assertEquals(Endpoint.UNKNOWN, tracker.endpoint());
        assertEquals(Direction.STOPPED, tracker.update(Direction.EXTENDING, 35, true, .50));
        assertEquals(Endpoint.EXTENDED, tracker.endpoint());
        // A held command cannot repeatedly push against the detected stop.
        assertEquals(Direction.STOPPED, tracker.update(Direction.EXTENDING, 0, true, .90));
        assertEquals(1, tracker.endStopCount());
    }

    @Test void reversalPausesAndThenDetectsTheOtherEndpoint() {
        var tracker = tracker();
        tracker.update(Direction.EXTENDING, 5, true, 0);
        tracker.update(Direction.EXTENDING, 30, true, .20);
        tracker.update(Direction.EXTENDING, 30, true, .30);
        assertEquals(Direction.STOPPED, tracker.update(Direction.RETRACTING, 30, true, .32));
        assertEquals(Direction.RETRACTING, tracker.update(Direction.RETRACTING, 30, true, .41));
        assertEquals(Endpoint.UNKNOWN, tracker.endpoint());
        tracker.update(Direction.RETRACTING, 22, true, .60);
        assertEquals(Direction.STOPPED, tracker.update(Direction.RETRACTING, 22, true, .70));
        assertEquals(Endpoint.RETRACTED, tracker.endpoint());
        assertEquals(2, tracker.endStopCount());
    }

    @Test void timeoutLatchesAFaultInsteadOfGuessingAnEndpointOrReversing() {
        var tracker = tracker();
        tracker.update(Direction.EXTENDING, 5, true, 0);
        assertEquals(Direction.STOPPED, tracker.update(Direction.EXTENDING, 5, true, 2));
        assertEquals(Endpoint.UNKNOWN, tracker.endpoint());
        assertFalse(tracker.fault().isEmpty());
        assertEquals(Direction.STOPPED, tracker.update(Direction.RETRACTING, 5, true, 3));
        tracker.clearFault(3);
        assertEquals(Direction.RETRACTING, tracker.update(Direction.RETRACTING, 5, true, 3.2));
    }

    @Test void badOrMissingCurrentFeedbackStopsMotion() {
        for (double current : new double[] { 5, Double.NaN, -1 }) {
            var tracker = tracker();
            tracker.update(Direction.EXTENDING, 5, true, 0);
            boolean valid = current != 5;
            assertEquals(Direction.STOPPED, tracker.update(Direction.EXTENDING, current, valid, .2));
            assertEquals(Endpoint.UNKNOWN, tracker.endpoint());
            assertFalse(tracker.fault().isEmpty());
        }
    }

    @Test void unconfiguredDetectionCannotMoveTheMotor() {
        var tracker = new IntakeEndStopTracker(new IntakeEndStopTracker.Settings(0, 0, .15, .1, 2, .1));
        assertEquals(Direction.STOPPED, tracker.update(Direction.EXTENDING, 5, true, 0));
        assertFalse(tracker.fault().isEmpty());
    }

    @Test void disablingInvalidatesThePositionReference() {
        var tracker = tracker();
        tracker.update(Direction.EXTENDING, 5, true, 0);
        tracker.update(Direction.EXTENDING, 30, true, .2);
        tracker.update(Direction.EXTENDING, 30, true, .3);
        assertEquals(Endpoint.EXTENDED, tracker.endpoint());
        tracker.invalidate(.4);
        assertEquals(Endpoint.UNKNOWN, tracker.endpoint());
        assertEquals(Direction.STOPPED, tracker.moving());
    }
}
