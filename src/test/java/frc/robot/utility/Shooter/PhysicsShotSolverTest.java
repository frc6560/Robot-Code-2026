package frc.robot.utility.Shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotModelConstants;
import frc.robot.utility.Shooter.PhysicsShotSolver.Solution;

class PhysicsShotSolverTest {
    private static final double FIFTEEN_FEET_METERS = 4.572;

    @Test
    void convertsRearReferencedHoodCommandToLaunchElevation() {
        assertEquals(64.9, PhysicsShotSolver.launchElevationDegrees(25.1), 1e-9);
        assertEquals(45.0, PhysicsShotSolver.launchElevationDegrees(45.0), 1e-9);
    }

    @Test
    void matchesCalibratorAtFifteenFeet() {
        Solution solution = new PhysicsShotSolver().solve(FIFTEEN_FEET_METERS);

        assertTrue(solution.valid());
        assertEquals(2750.0, solution.flywheelRPM(), 1e-9);
        assertEquals(27.942857142857143, solution.hoodCommandDegrees(), 1e-9);
        assertEquals(62.05714285714286, solution.launchElevationDegrees(), 1e-9);
        assertEquals(1.2919535997040075, solution.timeOfFlightSeconds(), 2e-4);
        assertEquals(56.42044609938561, solution.entryAngleDegrees(), 0.03);
        assertTrue(solution.openingClearanceMeters() > 0.0);
        assertTrue(solution.nearRimClearanceMeters() >= ShotModelConstants.HUB_RIM_MARGIN_METERS);
    }

    @Test
    void returnsScoringCommandsAcrossConfiguredRange() {
        PhysicsShotSolver solver = new PhysicsShotSolver();

        for (double distance : new double[] {
            ShotModelConstants.MIN_DISTANCE_METERS,
            3.0,
            ShotModelConstants.MAX_DISTANCE_METERS
        }) {
            Solution solution = solver.solve(distance);
            assertTrue(solution.valid(), "Expected valid shot at " + distance + " m");
            assertTrue(solution.flywheelRPM() >= ShooterConstants.FLYWHEEL_IDLE_RPM);
            assertTrue(solution.flywheelRPM() <= ShooterConstants.MAX_RPM);
            assertTrue(solution.hoodCommandDegrees() >= HoodConstants.HOOD_MIN_ANGLE);
            assertTrue(solution.hoodCommandDegrees() <= HoodConstants.HOOD_MAX_ANGLE);
            assertTrue(solution.entryAngleDegrees() >= ShotModelConstants.MIN_ENTRY_ANGLE_DEGREES);
        }
    }

    @Test
    void rejectsInvalidDistance() {
        PhysicsShotSolver solver = new PhysicsShotSolver();
        Solution solution = solver.solve(Double.NaN);

        assertFalse(solution.valid());
        assertEquals(ShooterConstants.FLYWHEEL_IDLE_RPM, solution.flywheelRPM());
        assertEquals(HoodConstants.HOOD_MIN_ANGLE, solution.hoodCommandDegrees());
        assertFalse(solver.solveRuntime(ShotModelConstants.MIN_DISTANCE_METERS - 0.001).valid());
        assertFalse(solver.solveRuntime(ShotModelConstants.MAX_DISTANCE_METERS + 0.001).valid());
        assertFalse(solver.evaluate(
            ShotModelConstants.MAX_DISTANCE_METERS + 0.001,
            2500.0,
            30.0
        ).valid());
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
    void runtimePolicyStaysNearRobustSolverAtRepresentativeDistances() {
        PhysicsShotSolver solver = new PhysicsShotSolver();
        for (double distance : new double[] {
            ShotModelConstants.MIN_DISTANCE_METERS,
            2.0,
            3.0,
            4.0,
            FIFTEEN_FEET_METERS,
            5.0,
            ShotModelConstants.MAX_DISTANCE_METERS
        }) {
            Solution robust = solver.solve(distance);
            Solution runtime = solver.solveRuntime(distance);

            assertTrue(robust.valid(), "Robust solver missed at " + distance + " m");
            assertTrue(runtime.valid(), "Runtime policy missed at " + distance + " m");
            assertTrue(
                Math.abs(runtime.flywheelRPM() - robust.flywheelRPM()) <= 150.0,
                "Runtime RPM departed from robust solution at " + distance + " m"
            );
            assertTrue(
                Math.abs(runtime.hoodCommandDegrees() - robust.hoodCommandDegrees()) <= 3.0,
                "Runtime hood command departed from robust solution at " + distance + " m"
            );
        }
    }
}
