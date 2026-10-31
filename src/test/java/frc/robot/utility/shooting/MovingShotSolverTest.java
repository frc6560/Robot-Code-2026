package frc.robot.utility.shooting;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class MovingShotSolverTest {
    @Test void cacheEstimatePreservesSelectionAndApproximatesFlightTime() {
        var model = new FuelShotModel(LaunchCalibration.ideal());
        for (double range = 2; range < 6; range += 0.07) {
            var actual = model.solve(range);
            var estimate = model.estimate(range);
            assertEquals(actual.valid(), estimate.valid());
            if (actual.valid()) {
                assertEquals(actual.rpm(), estimate.rpm());
                assertEquals(actual.flightSeconds(), estimate.flightSeconds(), 0.001);
            }
        }
    }

    @Test void approachReducesRpmRetreatIncreasesItAndSidewaysMotionLeadsOpposite() {
        var model = new FuelShotModel(LaunchCalibration.ideal());
        var target = new Translation2d(3.5, 0);
        double stationary = model.solve(3.5).rpm();
        var approach = MovingShotSolver.solve(model, target, new Translation2d(0.5, 0));
        var retreat = MovingShotSolver.solve(model, target, new Translation2d(-0.5, 0));
        var sideways = MovingShotSolver.solve(model, target, new Translation2d(0, 0.5));
        assertTrue(approach.valid());
        assertTrue(retreat.valid());
        assertTrue(sideways.valid());
        assertTrue(approach.shot().rpm() < stationary);
        assertTrue(retreat.shot().rpm() > stationary);
        assertTrue(target.minus(sideways.lead()).getY() < 0);
        assertTrue(sideways.shot().rpm() >= stationary);
        assertTrue(MovingShotSolver.scores(model, target, new Translation2d(0, 0.5), target.minus(sideways.lead()).getAngle(),
            sideways.shot().rpm()));
    }

    @Test void releaseGateUsesActual3DLandingAtTheOpeningBoundary() {
        var model = new FuelShotModel(LaunchCalibration.ideal());
        var velocity = new Translation2d(0.3, 0.8);
        double rpm = 2800;
        var flight = model.movingFlight(rpm, 0, velocity.getX(), velocity.getY());
        assertTrue(flight.reached());
        var landing = new Translation2d(flight.crossingX(), flight.crossingY());
        double halfWidth = frc.robot.Constants.ShotModelConstants.HUB_OPENING_SPAN_METERS / 2
            - frc.robot.Constants.ShotModelConstants.BALL_DIAMETER_METERS / 2
            - frc.robot.Constants.ShotModelConstants.HUB_RIM_MARGIN_METERS;
        assertTrue(MovingShotSolver.scores(model, landing, velocity, new Rotation2d(), rpm));
        assertTrue(MovingShotSolver.scores(model, landing.plus(new Translation2d(0, halfWidth - .005)),
            velocity, new Rotation2d(), rpm));
        assertFalse(MovingShotSolver.scores(model, landing.plus(new Translation2d(0, halfWidth + .005)),
            velocity, new Rotation2d(), rpm));
    }

    @Test void measuredHeadingBlocksAnOffTargetReleaseEvenWhenRpmAndLeadAreValid() {
        var model = new FuelShotModel(LaunchCalibration.ideal());
        var target = new Translation2d(3.5, 0);
        var velocity = new Translation2d(0, 1);
        var solution = MovingShotSolver.solve(model, target, velocity);
        assertTrue(solution.valid());
        Rotation2d correct = target.minus(solution.lead()).getAngle();
        assertTrue(MovingShotSolver.scores(model, target, velocity, correct, solution.shot().rpm()));
        assertFalse(MovingShotSolver.scores(model, target, velocity,
            correct.plus(Rotation2d.fromDegrees(15)), solution.shot().rpm()));
    }

    @Test void diagonalMotionAtDrivingSpeedProducesAConsistentLanding() {
        var model = new FuelShotModel(LaunchCalibration.ideal());
        var target = new Translation2d(3.5, 0);
        for (double x : new double[] {-1, 0, 1}) {
            for (double y : new double[] {-1, 1}) {
                var velocity = new Translation2d(x, y);
                var solution = MovingShotSolver.solve(model, target, velocity);
                assertTrue(solution.valid(), "Velocity " + velocity);
                assertTrue(MovingShotSolver.scores(model, target, velocity, target.minus(solution.lead()).getAngle(),
                    solution.shot().rpm()), "Landing for " + velocity);
            }
        }
    }

    @Test void compensationRotatesWithTheFieldAndRejectsInvalidOrUnreachableInputs() {
        var model = new FuelShotModel(LaunchCalibration.ideal());
        var target = new Translation2d(3.5, 0);
        var velocity = new Translation2d(0.25, 0.5);
        var original = MovingShotSolver.solve(model, target, velocity);
        var rotation = Rotation2d.fromDegrees(90);
        var rotated = MovingShotSolver.solve(model, target.rotateBy(rotation), velocity.rotateBy(rotation));
        assertTrue(original.valid());
        assertTrue(rotated.valid());
        assertEquals(original.shot().rpm(), rotated.shot().rpm());
        assertEquals(0, original.lead().rotateBy(rotation).getDistance(rotated.lead()), 1e-9);
        assertFalse(MovingShotSolver.solve(model, target, new Translation2d(Double.NaN, 0)).valid());
        assertFalse(MovingShotSolver.solve(model, target, new Translation2d(100, 0)).valid());
        assertFalse(MovingShotSolver.scores(model, target, velocity, target.minus(original.lead()).getAngle(), Double.NaN));
        assertFalse(MovingShotSolver.scores(model, target, velocity, target.minus(original.lead()).getAngle(), 99999));
        assertFalse(MovingShotSolver.scores(model, target, velocity,
            Rotation2d.fromDegrees(90), original.shot().rpm()));
        assertFalse(MovingShotSolver.isMoving(new Translation2d()));
        assertFalse(MovingShotSolver.isMoving(new Translation2d(0.03, 0.03)));
    }
}
