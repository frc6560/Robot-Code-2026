package frc.robot.utility.Shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotModelConstants;
import frc.robot.utility.Shooter.PhysicsShotSolver.PassSolution;
import frc.robot.utility.Shooter.PhysicsShotSolver.Solution;

class PhysicsShotSolverTest {
    private static final double FIFTEEN_FEET_METERS = 4.572;

    @Test
    void launchesAlongTheFixedHood() {
        assertEquals(48.36, PhysicsShotSolver.launchElevationDegrees(), 1e-9);
    }

    @Test
    void matchesCalibratorAtFifteenFeet() {
        // Reference from the shot-calibrator's "Powered wheel + fixed hood" model with the same
        // constants: scoring band 2864.8-3083.6 RPM, center 2974.2, 0.959 s, 35.7 deg entry.
        Solution solution = new PhysicsShotSolver().solve(FIFTEEN_FEET_METERS);

        assertTrue(solution.valid());
        assertEquals(2974.2, solution.flywheelRPM(), 2.0);
        assertEquals(2974.2 - 2864.8, solution.rpmToleranceLower(), 3.0);
        assertEquals(3083.6 - 2974.2, solution.rpmToleranceUpper(), 3.0);
        assertEquals(0.9588, solution.timeOfFlightSeconds(), 2e-3);
        assertEquals(35.74, solution.entryAngleDegrees(), 0.1);
        assertTrue(solution.openingClearanceMeters() > 0.0);
        assertTrue(solution.nearRimClearanceMeters() >= ShotModelConstants.HUB_RIM_MARGIN_METERS);
    }

    @Test
    void returnsScoringCommandsAcrossConfiguredRange() {
        PhysicsShotSolver solver = new PhysicsShotSolver();

        for (double distance : new double[] {
            ShotModelConstants.MIN_DISTANCE_METERS,
            4.0,
            ShotModelConstants.MAX_DISTANCE_METERS
        }) {
            Solution solution = solver.solve(distance);
            assertTrue(solution.valid(), "Expected valid shot at " + distance + " m");
            assertTrue(solution.flywheelRPM() >= ShooterConstants.FLYWHEEL_IDLE_RPM);
            assertTrue(solution.flywheelRPM() <= ShooterConstants.MAX_RPM);
            assertTrue(solution.entryAngleDegrees() >= ShotModelConstants.MIN_ENTRY_ANGLE_DEGREES);
            // Even the tightest band (at the minimum distance) leaves room for flywheel error.
            assertTrue(solution.rpmToleranceLower() > 40.0, "lower tolerance at " + distance + " m");
            assertTrue(solution.rpmToleranceUpper() > 40.0, "upper tolerance at " + distance + " m");
        }
    }

    @Test
    void rejectsInvalidDistance() {
        PhysicsShotSolver solver = new PhysicsShotSolver();
        Solution solution = solver.solve(Double.NaN);

        assertFalse(solution.valid());
        assertEquals(ShooterConstants.FLYWHEEL_IDLE_RPM, solution.flywheelRPM());
        assertFalse(solver.solveRuntime(ShotModelConstants.MIN_DISTANCE_METERS - 0.001).valid());
        assertFalse(solver.solveRuntime(ShotModelConstants.MAX_DISTANCE_METERS + 0.001).valid());
        assertFalse(solver.evaluate(ShotModelConstants.MAX_DISTANCE_METERS + 0.001, 3000.0).valid());
        assertFalse(solver.evaluate(FIFTEEN_FEET_METERS, ShooterConstants.MAX_RPM + 1.0).valid());
    }

    @Test
    void generatedQuadraticPolicyScoresAcrossConfiguredRange() {
        PhysicsShotSolver solver = new PhysicsShotSolver();
        for (int index = 0; index <= 1000; index++) {
            double distance = ShotModelConstants.MIN_DISTANCE_METERS
                + index * (ShotModelConstants.MAX_DISTANCE_METERS - ShotModelConstants.MIN_DISTANCE_METERS) / 1000.0;
            Solution solution = solver.solveRuntime(distance);
            assertTrue(solution.valid(), "Generated policy missed at " + distance + " m");
            assertTrue(solution.openingClearanceMeters() >= 0.0);
            assertTrue(solution.nearRimClearanceMeters() >= ShotModelConstants.HUB_RIM_MARGIN_METERS);
            assertTrue(solution.entryAngleDegrees() >= ShotModelConstants.MIN_ENTRY_ANGLE_DEGREES);
        }
    }

    @Test
    void runtimePolicyStaysNearBandCenter() {
        PhysicsShotSolver solver = new PhysicsShotSolver();
        for (double distance : new double[] {
            ShotModelConstants.MIN_DISTANCE_METERS, 3.5, 4.0, FIFTEEN_FEET_METERS, 5.0, 5.5,
            ShotModelConstants.MAX_DISTANCE_METERS
        }) {
            Solution center = solver.solve(distance);
            Solution runtime = solver.solveRuntime(distance);

            assertTrue(center.valid(), "Band search missed at " + distance + " m");
            assertTrue(runtime.valid(), "Runtime policy missed at " + distance + " m");
            assertTrue(
                Math.abs(runtime.flywheelRPM() - center.flywheelRPM()) <= 30.0,
                "Runtime RPM " + runtime.flywheelRPM() + " departed from band center "
                    + center.flywheelRPM() + " at " + distance + " m"
            );
        }
    }

    @Test
    void passesLandOnTheCarpetAndNeedMoreSpeedFarther() {
        PhysicsShotSolver solver = new PhysicsShotSolver();
        double previousRpm = 0.0;
        double previousTime = 0.0;
        for (double distance = 3.0; distance <= 9.0; distance += 1.0) {
            PassSolution pass = solver.solvePass(distance);
            assertTrue(pass.valid(), "Expected a pass at " + distance + " m");
            assertTrue(pass.flywheelRPM() > previousRpm);
            assertTrue(pass.timeOfFlightSeconds() > previousTime);
            previousRpm = pass.flywheelRPM();
            previousTime = pass.timeOfFlightSeconds();
        }
        assertFalse(solver.solvePass(ShotModelConstants.PASS_MAX_DISTANCE_METERS).valid(),
            "A 12 m pass should be out of reach at MAX_RPM");
    }
}
