package frc.robot.utility.Shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;

class ShotCalculatorTest {
    private static final double FIFTEEN_FEET_METERS = 4.572;

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void resetDriverStation() {
        DriverStationSim.resetData();
    }

    @Test
    void stationaryHubShotUsesPhysicsPolicyOutputs() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();

        ShotCalculator calculator = new ShotCalculator();
        calculator.calculate(robotPoseAtBlueHubDistance(FIFTEEN_FEET_METERS), new ChassisSpeeds());

        PhysicsShotSolver.Solution expected = new PhysicsShotSolver().solveRuntime(FIFTEEN_FEET_METERS);
        assertTrue(calculator.isShotValid());
        assertEquals(expected.flywheelRPM(), calculator.getFlywheelRPM(), 1e-6);
        assertEquals(expected.hoodCommandDegrees(), calculator.getHoodAzimuth(), 1e-6);
        assertEquals(expected.timeOfFlightSeconds(), calculator.getTimeOfFlightSeconds(), 1e-5);
        assertEquals(FIFTEEN_FEET_METERS, calculator.getDistanceToVirtualTarget(), 1e-9);
        assertTrue(calculator.measuredControlsScore(
            calculator.getFlywheelRPM(),
            calculator.getHoodAzimuth()
        ));
        assertFalse(calculator.measuredControlsScore(
            calculator.getFlywheelRPM() - 200.0,
            calculator.getHoodAzimuth()
        ));
        assertFalse(calculator.measuredControlsScore(
            calculator.getFlywheelRPM() + 200.0,
            calculator.getHoodAzimuth()
        ));
    }

    @Test
    void movingShotUsesEquationFlightTimeForVirtualTarget() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();

        Pose2d robotPose = robotPoseAtBlueHubDistance(FIFTEEN_FEET_METERS);
        ShotCalculator calculator = new ShotCalculator();
        calculator.calculate(robotPose, new ChassisSpeeds(1.0, 0.0, 0.0));

        assertTrue(calculator.isShotValid());
        assertEquals(
            FieldConstants.BLUE_HUB_CENTER.getX() - calculator.getTimeOfFlightSeconds(),
            calculator.getVirtualTargetPose().getX(),
            0.005
        );
        assertEquals(FieldConstants.BLUE_HUB_CENTER.getY(), calculator.getVirtualTargetPose().getY(), 1e-9);
    }

    @Test
    void rejectsShotOutsideConfiguredDistance() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();

        ShotCalculator calculator = new ShotCalculator();
        calculator.calculate(robotPoseAtBlueHubDistance(7.0), new ChassisSpeeds());

        assertFalse(calculator.isShotValid());
        assertEquals(500.0, calculator.getFlywheelRPM(), 1e-9);
        assertEquals(25.1, calculator.getHoodAzimuth(), 1e-9);
        assertFalse(calculator.measuredControlsScore(2500.0, 30.0));
    }

    @Test
    void ungatedShotUsesNearestCalibratedSetpointOutsideConfiguredDistance() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();

        ShotCalculator calculator = new ShotCalculator();
        calculator.calculateUngated(robotPoseAtBlueHubDistance(7.0), new ChassisSpeeds());

        PhysicsShotSolver.Solution expected =
            new PhysicsShotSolver().solveRuntime(Constants.ShotModelConstants.MAX_DISTANCE_METERS);
        assertFalse(calculator.isShotValid());
        assertEquals(expected.flywheelRPM(), calculator.getFlywheelRPM(), 1e-6);
        assertEquals(expected.hoodCommandDegrees(), calculator.getHoodAzimuth(), 1e-6);
        assertTrue(calculator.getFlywheelRPM() > Constants.ShooterConstants.FLYWHEEL_IDLE_RPM);
        assertTrue(calculator.getHoodAzimuth() > Constants.HoodConstants.HOOD_MIN_ANGLE);
        assertFalse(calculator.measuredControlsScore(
            calculator.getFlywheelRPM(), calculator.getHoodAzimuth()));
    }

    @Test
    void rejectsNonFiniteLocalizationWithoutProducingUnsafeOutputs() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();

        Object[][] invalidInputs = new Object[][] {
            {null, new ChassisSpeeds()},
            {new Pose2d(), null},
            {new Pose2d(Double.NaN, 0.0, new Rotation2d()), new ChassisSpeeds()},
            {new Pose2d(0.0, Double.POSITIVE_INFINITY, new Rotation2d()), new ChassisSpeeds()},
            {new Pose2d(0.0, 0.0, Rotation2d.fromRadians(Double.NaN)), new ChassisSpeeds()},
            {new Pose2d(), new ChassisSpeeds(Double.NaN, 0.0, 0.0)},
            {new Pose2d(), new ChassisSpeeds(0.0, Double.NEGATIVE_INFINITY, 0.0)},
            {new Pose2d(), new ChassisSpeeds(0.0, 0.0, Double.NaN)}
        };

        for (Object[] invalidInput : invalidInputs) {
            ShotCalculator calculator = new ShotCalculator();
            calculator.calculate((Pose2d) invalidInput[0], (ChassisSpeeds) invalidInput[1]);

            assertFalse(calculator.isShotValid());
            assertEquals(500.0, calculator.getFlywheelRPM(), 1e-9);
            assertEquals(25.1, calculator.getHoodAzimuth(), 1e-9);
            assertTrue(Double.isFinite(calculator.getTurretAngle()));
            assertTrue(Double.isFinite(calculator.getTurretVelocityFF()));
            assertTrue(Double.isFinite(calculator.getDistanceToVirtualTarget()));
            assertTrue(Double.isFinite(calculator.getVirtualTargetPose().getX()));
            assertTrue(Double.isFinite(calculator.getVirtualTargetPose().getY()));
        }
    }

    @Test
    void missingAllianceResetsPreviouslyValidShotToSafeState() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();

        ShotCalculator calculator = new ShotCalculator();
        calculator.calculate(robotPoseAtBlueHubDistance(FIFTEEN_FEET_METERS), new ChassisSpeeds());
        assertTrue(calculator.isShotValid());

        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        calculator.calculate(robotPoseAtBlueHubDistance(FIFTEEN_FEET_METERS), new ChassisSpeeds());

        assertFalse(calculator.isShotValid());
        assertEquals(500.0, calculator.getFlywheelRPM(), 1e-9);
        assertEquals(25.1, calculator.getHoodAzimuth(), 1e-9);
        assertEquals(0.0, calculator.getDistanceToVirtualTarget(), 1e-9);
    }

    private static Pose2d robotPoseAtBlueHubDistance(double distanceMeters) {
        double robotX = FieldConstants.BLUE_HUB_CENTER.getX()
            - distanceMeters
            - TurretConstants.ROBOT_RELATIVE_TURRET.getX();
        double robotY = FieldConstants.BLUE_HUB_CENTER.getY()
            - TurretConstants.ROBOT_RELATIVE_TURRET.getY();
        return new Pose2d(robotX, robotY, new Rotation2d());
    }
}
