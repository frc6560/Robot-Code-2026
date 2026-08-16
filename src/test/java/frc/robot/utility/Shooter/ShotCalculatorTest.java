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
