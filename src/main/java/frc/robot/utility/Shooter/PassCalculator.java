package frc.robot.utility.Shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class PassCalculator {

    // Note that the RPM 
    public record TurretState(double positionRadians, double velocityRadiansPerSecond) {}

    private double hoodAzimuth;

    private double turretAngle;
    private double turretVelocityFF; // feedforward velocity in rad/s 

    private static final InterpolatingDoubleTreeMap hoodAzimuthMap = new InterpolatingDoubleTreeMap();

    public PassCalculator() {
        hoodAzimuth = 0.0;
        turretAngle = 0.0;
        turretVelocityFF = 0.0;

        hoodAzimuthMap.put(1.593, 25.0);
        hoodAzimuthMap.put(1.885, 25.0);
        hoodAzimuthMap.put(2.500, 27.0);
        hoodAzimuthMap.put(3.098, 29.0);
        hoodAzimuthMap.put(3.700, 32.0);
        hoodAzimuthMap.put(4.273, 35.0);
        hoodAzimuthMap.put(4.987, 38.0);
        hoodAzimuthMap.put(5.602, 44.0);
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

    /** Calculates the hood and turret angles based on the robot's pose */
    public void calculate(Pose2d robotPose, ChassisSpeeds fieldVelocity) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return;
        }
        Translation2d targetPassLocation;

        // Calculates our target passing location.
        if(robotPose.getY() < FieldConstants.PASS_DEADZONE_MIN_Y){
            targetPassLocation = (alliance.get() == Alliance.Blue) 
                ? FieldConstants.BLUE_BOTTOM_PASS_POS 
                : FieldConstants.RED_BOTTOM_PASS_POS;
        }
        else if(robotPose.getY() > FieldConstants.PASS_DEADZONE_MAX_Y){
            targetPassLocation = (alliance.get() == Alliance.Blue) 
                ? FieldConstants.BLUE_TOP_PASS_POS 
                : FieldConstants.RED_TOP_PASS_POS;
        }
        else{
            return; // if we're in the deadzone, we don't calculate a pass
        }

        double distanceToTarget = robotPose.getTranslation().getDistance(targetPassLocation);
        hoodAzimuth = hoodAzimuthMap.get(distanceToTarget);
        turretAngle = Math.atan2(targetPassLocation.getY() - robotPose.getY(), targetPassLocation.getX() - robotPose.getX())
                        - robotPose.getRotation().getRadians();
        turretVelocityFF = - fieldVelocity.omegaRadiansPerSecond; // feedforward to help track the target as we move
    }
}