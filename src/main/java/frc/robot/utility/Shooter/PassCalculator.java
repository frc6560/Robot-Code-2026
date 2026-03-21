package frc.robot.utility.Shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;

public class PassCalculator {

    public record TurretState(double positionRadians, double velocityRadiansPerSecond) {}

    private double hoodAzimuth;
    private double flywheelRPM;
    private double turretAngle;
    private double turretVelocityFF; // feedforward velocity in rad/s 

    public Translation2d virtualTargetPose;

    private static final InterpolatingDoubleTreeMap hoodAzimuthMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    public PassCalculator() {
        hoodAzimuth = 0.0;
        turretAngle = 0.0;
        turretVelocityFF = 0.0;
        virtualTargetPose = new Translation2d();

        hoodAzimuthMap.put(4.077, 30.0);
        hoodAzimuthMap.put(4.980, 36.0);
        hoodAzimuthMap.put(5.955, 40.0);
        hoodAzimuthMap.put(6.927, 44.0);
        hoodAzimuthMap.put(7.777, 44.0);
        hoodAzimuthMap.put(8.766, 44.0);

        flywheelRPMMap.put(4.077, 1900.0);
        flywheelRPMMap.put(4.980, 2100.0);
        flywheelRPMMap.put(5.955, 2200.0);
        flywheelRPMMap.put(6.927, 2400.0);
        flywheelRPMMap.put(7.777, 2500.0);
        flywheelRPMMap.put(8.766, 2600.0);

        timeOfFlightMap.put(4.077, 1.25);
        timeOfFlightMap.put(4.980, 1.26);
        timeOfFlightMap.put(5.955, 1.33);
        timeOfFlightMap.put(6.927, 1.33);
        timeOfFlightMap.put(7.777, 1.39);
        timeOfFlightMap.put(8.766, 1.47);
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getTurretVelocityFF() {
        return turretVelocityFF;
    }

    public double getHoodAzimuth(){
        return hoodAzimuth;
    }

    public double getFlywheelRPM(){
        return flywheelRPM;
    }

    /** Calculates the hood and turret angles based on the robot's pose */
    public void calculate(Pose2d robotPose, ChassisSpeeds fieldVelocity) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return;
        }

        // Check if robot is within DEAD_RAD of our alliance hub center
        Translation2d hubCenter = (alliance.get() == Alliance.Blue)
            ? FieldConstants.BLUE_HUB_CENTER
            : FieldConstants.RED_HUB_CENTER;
        double distanceToHub = robotPose.getTranslation().getDistance(hubCenter);
        boolean inHubDeadzone = distanceToHub < FieldConstants.DEAD_RAD;
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("PassCalc/InHubDeadzone", inHubDeadzone);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("PassCalc/DistanceToHub", distanceToHub);
        if (inHubDeadzone) {
            return; // if we're near our hub, we don't calculate a pass
        }

        // Check if robot is in opponent rectangular deadzone
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        boolean inOpponentDeadzone = false;
        if (alliance.get() == Alliance.Blue) {
            // Check Red side deadzone for Blue alliance
            inOpponentDeadzone = robotX >= FieldConstants.RED_DEADZONE_MIN_X && robotX <= FieldConstants.RED_DEADZONE_MAX_X &&
                robotY >= FieldConstants.PASS_DEADZONE_MIN_Y && robotY <= FieldConstants.PASS_DEADZONE_MAX_Y;
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("PassCalc/InRedDeadzone", inOpponentDeadzone);
            if (inOpponentDeadzone) {
                return; // if we're in opponent deadzone, we don't calculate a pass
            }
        } else {
            // Check Blue side deadzone for Red alliance
            inOpponentDeadzone = robotX >= FieldConstants.BLUE_DEADZONE_MIN_X && robotX <= FieldConstants.BLUE_DEADZONE_MAX_X &&
                robotY >= FieldConstants.PASS_DEADZONE_MIN_Y && robotY <= FieldConstants.PASS_DEADZONE_MAX_Y;
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("PassCalc/InBlueDeadzone", inOpponentDeadzone);
            if (inOpponentDeadzone) {
                return; // if we're in opponent deadzone, we don't calculate a pass
            }
        }

        // Calculates our target passing location.
        // Use the average of min/max Y as the threshold between top and bottom
        double yThreshold = (FieldConstants.PASS_DEADZONE_MIN_Y + FieldConstants.PASS_DEADZONE_MAX_Y) / 2.0; //too lazy 
        boolean useTopPass = robotPose.getY() > yThreshold;
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("PassCalc/UseTopPass", useTopPass);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("PassCalc/YThreshold", yThreshold);

        Translation2d targetPassLocation;
        if(useTopPass){
            targetPassLocation = (alliance.get() == Alliance.Blue)
                ? FieldConstants.BLUE_TOP_PASS_POS
                : FieldConstants.RED_TOP_PASS_POS;
        }
        else{
            targetPassLocation = (alliance.get() == Alliance.Blue)
                ? FieldConstants.BLUE_BOTTOM_PASS_POS
                : FieldConstants.RED_BOTTOM_PASS_POS;
        }

        // gets the turret's robot relative transform
        Transform2d turretTransform = new Transform2d(
            TurretConstants.ROBOT_RELATIVE_TURRET.getX(), 
            TurretConstants.ROBOT_RELATIVE_TURRET.getY(),
            new Rotation2d()
        );


        // gets the turret's field relative velocity
        Pose2d turretPose = robotPose.transformBy(turretTransform);
        double angleOffset = Math.atan2(turretTransform.getY(), turretTransform.getX());
        double r = Math.hypot(turretTransform.getX(), turretTransform.getY());

        double turretVx = fieldVelocity.vxMetersPerSecond
                        + (-r * fieldVelocity.omegaRadiansPerSecond * Math.sin(robotPose.getRotation().getRadians() + angleOffset));
        double turretVy = fieldVelocity.vyMetersPerSecond
                        + (r * fieldVelocity.omegaRadiansPerSecond * Math.cos(robotPose.getRotation().getRadians() + angleOffset));

        // calculates a virtual target iteratively based upon our parameters.
        virtualTargetPose = targetPassLocation;
        double timeOfFlight = 0;
        double distanceToTarget = turretPose.getTranslation().getDistance(targetPassLocation);

        // Convergence threshold for early exit (seconds)
        final double EPSILON = 0.01;
        double prevTimeOfFlight = 0;

        if(Math.hypot(turretVx, turretVy) > 0.3){
            for(int i = 0; i < 20; i++){
                timeOfFlight = timeOfFlightMap.get(distanceToTarget);

                // Early exit if time of flight has converged
                if (i > 0 && Math.abs(timeOfFlight - prevTimeOfFlight) < EPSILON) {
                    break;
                }

                prevTimeOfFlight = timeOfFlight;

                virtualTargetPose = targetPassLocation.minus(
                    new Translation2d(
                        turretVx * timeOfFlight,
                        turretVy * timeOfFlight
                    )
                );
                distanceToTarget = turretPose.getTranslation().getDistance(virtualTargetPose);
            }
        }
        
        double distanceToVirtualTarget = distanceToTarget;
        hoodAzimuth = hoodAzimuthMap.get(distanceToVirtualTarget);
        flywheelRPM = flywheelRPMMap.get(distanceToVirtualTarget);
        
        turretAngle = Math.atan2(virtualTargetPose.getY() - turretPose.getY(), virtualTargetPose.getX() - turretPose.getX())
                        - robotPose.getRotation().getRadians();
        

        double deltaX = virtualTargetPose.getX() - turretPose.getX();
        double deltaY = virtualTargetPose.getY() - turretPose.getY();
        double distSquared = distanceToVirtualTarget * distanceToVirtualTarget;

        if (distSquared > 0.01 && Math.hypot(turretVx, turretVy) > 0.3) { // avoid division by zero and ignore very small velocities
            double losRate = (turretVx * deltaY - turretVy * deltaX) / distSquared;
            turretVelocityFF = losRate - fieldVelocity.omegaRadiansPerSecond;
        } else {
            turretVelocityFF = -fieldVelocity.omegaRadiansPerSecond;
        }
    }
}