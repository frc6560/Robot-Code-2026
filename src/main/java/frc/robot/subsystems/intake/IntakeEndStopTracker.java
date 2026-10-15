package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;

/** Infers endpoints from sustained motor current. A jam can look like an endpoint. */
final class IntakeEndStopTracker {
    enum Direction { STOPPED, EXTENDING, RETRACTING }
    enum Endpoint { UNKNOWN, EXTENDED, RETRACTED }

    record Settings(double extendCurrentAmps, double retractCurrentAmps,
                    double startupIgnoreSeconds, double spikeSeconds,
                    double travelTimeoutSeconds, double reversalPauseSeconds) {
        boolean configured() {
            return Double.isFinite(extendCurrentAmps) && extendCurrentAmps > 0
                && Double.isFinite(retractCurrentAmps) && retractCurrentAmps > 0
                && extendCurrentAmps < IntakeConstants.STATOR_CURRENT_LIMIT_AMPS
                && retractCurrentAmps < IntakeConstants.STATOR_CURRENT_LIMIT_AMPS
                && Double.isFinite(startupIgnoreSeconds) && startupIgnoreSeconds >= 0
                && Double.isFinite(spikeSeconds) && spikeSeconds > 0
                && Double.isFinite(travelTimeoutSeconds)
                && travelTimeoutSeconds > startupIgnoreSeconds + spikeSeconds
                && Double.isFinite(reversalPauseSeconds) && reversalPauseSeconds >= 0;
        }
    }

    private final Settings settings;
    private Direction moving = Direction.STOPPED;
    private Direction lastDirection = Direction.STOPPED;
    private Endpoint endpoint = Endpoint.UNKNOWN;
    private double startedAt;
    private double highCurrentSince = Double.NaN;
    private double stoppedAt = Double.NEGATIVE_INFINITY;
    private String fault = "";
    private int endStopCount;

    IntakeEndStopTracker(Settings settings) {
        this.settings = settings;
    }

    /** Returns the direction permitted this cycle; STOPPED always means zero motor output. */
    Direction update(Direction requested, double currentAmps, boolean feedbackValid, double now) {
        if (requested == Direction.STOPPED || !fault.isEmpty()) {
            stopMotion(now);
            return Direction.STOPPED;
        }
        if (!settings.configured()) {
            fail("Current detection is not configured", now);
            return Direction.STOPPED;
        }
        if (!feedbackValid || !Double.isFinite(currentAmps) || currentAmps < 0) {
            fail("Motor current feedback unavailable", now);
            return Direction.STOPPED;
        }
        Endpoint target = requested == Direction.EXTENDING ? Endpoint.EXTENDED : Endpoint.RETRACTED;
        if (endpoint == target) {
            return Direction.STOPPED;
        }
        if (moving != Direction.STOPPED && moving != requested) {
            stopMotion(now);
        }
        if (moving == Direction.STOPPED) {
            if (lastDirection != Direction.STOPPED && lastDirection != requested
                    && now - stoppedAt < settings.reversalPauseSeconds()) {
                return Direction.STOPPED;
            }
            moving = requested;
            lastDirection = requested;
            endpoint = Endpoint.UNKNOWN;
            startedAt = now;
            highCurrentSince = Double.NaN;
        }
        double elapsed = now - startedAt;
        if (elapsed >= settings.travelTimeoutSeconds()) {
            fail("Travel timed out before an end stop was detected", now);
            return Direction.STOPPED;
        }
        double threshold = requested == Direction.EXTENDING
            ? settings.extendCurrentAmps() : settings.retractCurrentAmps();
        if (elapsed < settings.startupIgnoreSeconds() || currentAmps < threshold) {
            highCurrentSince = Double.NaN;
        } else {
            if (Double.isNaN(highCurrentSince)) {
                highCurrentSince = now;
            }
            if (now - highCurrentSince + 1e-9 >= settings.spikeSeconds()) {
                endpoint = target;
                endStopCount++;
                stopMotion(now);
                return Direction.STOPPED;
            }
        }
        return moving;
    }

    private void stopMotion(double now) {
        if (moving != Direction.STOPPED) {
            stoppedAt = now;
        }
        moving = Direction.STOPPED;
        highCurrentSince = Double.NaN;
    }

    private void fail(String reason, double now) {
        fault = reason;
        endpoint = Endpoint.UNKNOWN;
        stopMotion(now);
    }

    /** Disabling/coasting loses the endpoint reference because the mechanism may be moved by hand. */
    void invalidate(double now) {
        stopMotion(now);
        endpoint = Endpoint.UNKNOWN;
    }

    void clearFault(double now) {
        fault = "";
        invalidate(now);
    }

    Endpoint endpoint() { return endpoint; }
    Direction moving() { return moving; }
    String fault() { return fault; }
    int endStopCount() { return endStopCount; }
}
