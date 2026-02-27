package frc.robot.utility.Shooter;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;

public class ShotCalculator {

    /** State container for turret position and velocity. */
    public record TurretState(double positionRadians, double velocityRadiansPerSecond) {}

    /** State container for hood position and velocity. */
    public record HoodState(double positionDegrees) {}

    /** Combined state for the entire shooter system. */
    public record ShooterState(
        TurretState turret,
        HoodState hood,
        double flywheelRPM,
        Translation2d virtualTargetPose
    ) {}

    private double flywheelRPM;
    private double hoodAzimuth; // in degrees
    private double turretAngle;
    private double turretVelocityFF; // feedforward velocity in rad/s 

    private static final double TIME_PARAMETER = 0.058; 

    private static final InterpolatingDoubleTreeMap hoodAzimuthMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    private static final double MIN_DISTANCE = 1.279;
    private static final double MAX_DISTANCE = 5.345;

    public Translation2d virtualTargetPose;
    private double distanceToVirtualTarget = 0.0; 

    /** A util class for outputting shooter state values, even while the robot is moving! */
    public ShotCalculator() {
        this.flywheelRPM = 0;
        this.hoodAzimuth = 0;
        this.turretAngle = 0;
        this.turretVelocityFF = 0;
        this.virtualTargetPose = new Translation2d();
        populateLUTs();
    }

    public void populateLUTs(){
        // Units are in meters/RPM/degrees/seconds.
        // shooter RPM
        flywheelRPMMap.put(1.593, 1550.0);
        flywheelRPMMap.put(1.885, 1650.0);
        flywheelRPMMap.put(2.500, 1750.0);
        flywheelRPMMap.put(3.098, 1850.0);
        flywheelRPMMap.put(3.700, 1900.0);
        flywheelRPMMap.put(4.273, 2050.0);
        flywheelRPMMap.put(4.987, 2150.0);
        flywheelRPMMap.put(5.602, 2300.0);

        // hood azimuth
        hoodAzimuthMap.put(1.593, 25.0);
        hoodAzimuthMap.put(1.885, 25.0);
        hoodAzimuthMap.put(2.500, 27.0);
        hoodAzimuthMap.put(3.098, 29.0);
        hoodAzimuthMap.put(3.700, 32.0);
        hoodAzimuthMap.put(4.273, 35.0);
        hoodAzimuthMap.put(4.987, 38.0);
        hoodAzimuthMap.put(5.602, 44.0);


        // time of flight 
        timeOfFlightMap.put(1.520, 0.80);
        timeOfFlightMap.put(1.885, 0.96);
        timeOfFlightMap.put(2.500, 1.06);
        timeOfFlightMap.put(3.098, 1.17);
        timeOfFlightMap.put(3.700, 1.19);
        timeOfFlightMap.put(4.273, 1.26);
        timeOfFlightMap.put(4.987, 1.33);
    }

    /** Returns the current hood azimuth in degrees. */
    public double getHoodAzimuth() {
        return hoodAzimuth;
    }

    /** Returns the current turret angle in radians. */
    public double getTurretAngle() {
        return turretAngle;
    }

    /** Returns the turret velocity feedforward in radians per second. */
    public double getTurretVelocityFF() {
        return turretVelocityFF;
    }

    /** Returns the current flywheel RPM. */
    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    public Translation2d getVirtualTargetPose() {
        return virtualTargetPose;
    }

    /** Returns true if the distance to the virtual target is within the LUT range. */
    public boolean isShotValid() {
        return distanceToVirtualTarget >= MIN_DISTANCE && distanceToVirtualTarget <= MAX_DISTANCE;
    }

    /** Returns the current distance to the virtual target. */
    public double getDistanceToVirtualTarget() {
        return distanceToVirtualTarget;
    }

    /** Returns the complete shooter state including positions and velocities. */
    public ShooterState getState() {
        return new ShooterState(
            new TurretState(turretAngle, turretVelocityFF),
            new HoodState(hoodAzimuth),
            flywheelRPM,
            virtualTargetPose
        );
    }

    /** Calculates the shot parameters based on current robot pose and field velocity. */
    public void calculate(Pose2d currentRobotPose, 
                            ChassisSpeeds fieldVelocity) {
        // Gets our target pose
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()){
            return;
        }
        Translation2d targetPose = (alliance.get() == Alliance.Blue) ? 
                                            FieldConstants.BLUE_HUB_CENTER : 
                                            FieldConstants.RED_HUB_CENTER;
        
        ChassisSpeeds robotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldVelocity.vxMetersPerSecond,
            fieldVelocity.vyMetersPerSecond,
            fieldVelocity.omegaRadiansPerSecond,
            currentRobotPose.getRotation()
        );
        
        // calculates projected position due to sensor lag
        Pose2d projectedPosition = currentRobotPose.exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * TIME_PARAMETER,
                robotVelocity.vyMetersPerSecond * TIME_PARAMETER,
                robotVelocity.omegaRadiansPerSecond * TIME_PARAMETER
            )
        );

       // gets the turret's robot relative transform
        Transform2d turretTransform = new Transform2d(
            TurretConstants.ROBOT_RELATIVE_TURRET.getX(), 
            TurretConstants.ROBOT_RELATIVE_TURRET.getY(),
            new Rotation2d()
        );


        // gets the turret's field relative velocity
        Pose2d turretPose = projectedPosition.transformBy(turretTransform);
        double angleOffset = Math.atan2(turretTransform.getY(), turretTransform.getX());
        double r = Math.hypot(turretTransform.getX(), turretTransform.getY());

        double turretVx = fieldVelocity.vxMetersPerSecond
                        + (-r * fieldVelocity.omegaRadiansPerSecond * Math.sin(projectedPosition.getRotation().getRadians() + angleOffset));
        double turretVy = fieldVelocity.vyMetersPerSecond
                        + (r * fieldVelocity.omegaRadiansPerSecond * Math.cos(projectedPosition.getRotation().getRadians() + angleOffset));
        
        // Log input velocities
        SmartDashboard.putNumber("SOTM/FieldVel/vX", fieldVelocity.vxMetersPerSecond);
        SmartDashboard.putNumber("SOTM/FieldVel/vY", fieldVelocity.vyMetersPerSecond);
        SmartDashboard.putNumber("SOTM/FieldVel/omega", fieldVelocity.omegaRadiansPerSecond);

        // Log turret field-relative velocity
        SmartDashboard.putNumber("SOTM/TurretVel/vX", turretVx);
        SmartDashboard.putNumber("SOTM/TurretVel/vY", turretVy);
        SmartDashboard.putNumber("SOTM/TurretVel/magnitude", Math.hypot(turretVx, turretVy));

        // calculates a virtual target iteratively based upon our parameters.
        virtualTargetPose = targetPose; 
        double timeOfFlight = 0;
        double distanceToTarget = turretPose.getTranslation().getDistance(targetPose);
        double staticDistance = distanceToTarget; // save for logging

        // Convergence threshold for early exit (seconds)
        final double EPSILON = 0.01;
        double prevTimeOfFlight = 0;
        int iterationsUsed = 0;

        if(Math.hypot(turretVx, turretVy) > 0.4){
            for(int i = 0; i < 20; i++){
                timeOfFlight = timeOfFlightMap.get(distanceToTarget);
                iterationsUsed = i + 1;

                // Early exit if time of flight has converged
                if (i > 0 && Math.abs(timeOfFlight - prevTimeOfFlight) < EPSILON) {
                    break;
                }

                prevTimeOfFlight = timeOfFlight;

                virtualTargetPose = targetPose.minus(
                    new Translation2d(
                        turretVx * timeOfFlight,
                        turretVy * timeOfFlight
                    )
                );
                distanceToTarget = turretPose.getTranslation().getDistance(virtualTargetPose);
            }
        }
        
        distanceToVirtualTarget = distanceToTarget;

        SmartDashboard.putNumber("SOTM/Iterations", iterationsUsed);
        SmartDashboard.putNumber("SOTM/TimeOfFlight", timeOfFlight);
        SmartDashboard.putNumber("SOTM/Distance/Static", staticDistance);
        SmartDashboard.putNumber("SOTM/Distance/Virtual", distanceToTarget);
        SmartDashboard.putBoolean("SOTM/ShotValid", isShotValid());

        Translation2d targetOffset = virtualTargetPose.minus(targetPose);
        SmartDashboard.putNumber("SOTM/VirtualOffset/X", targetOffset.getX());
        SmartDashboard.putNumber("SOTM/VirtualOffset/Y", targetOffset.getY());
        SmartDashboard.putNumber("SOTM/VirtualOffset/magnitude", targetOffset.getNorm());

        // calculates hood angle and flywheel RPM from virtual target
        hoodAzimuth = hoodAzimuthMap.get(distanceToTarget);
        flywheelRPM = flywheelRPMMap.get(distanceToTarget);
        turretAngle = MathUtil.angleModulus(Math.atan2(
            virtualTargetPose.getY() - turretPose.getY(),
            virtualTargetPose.getX() - turretPose.getX()
        ) - projectedPosition.getRotation().getRadians());

        double deltaX = virtualTargetPose.getX() - turretPose.getX();
        double deltaY = virtualTargetPose.getY() - turretPose.getY();
        double distSquared = distanceToTarget * distanceToTarget;

        if (distSquared > 0.01) { 
            double losRate = (turretVx * deltaY - turretVy * deltaX) / distSquared;
            turretVelocityFF = losRate - fieldVelocity.omegaRadiansPerSecond;
        } else {
            turretVelocityFF = -fieldVelocity.omegaRadiansPerSecond;
        }

        // Log final output values
        SmartDashboard.putNumber("SOTM/Output/TurretAngleDeg", Math.toDegrees(turretAngle));
        SmartDashboard.putNumber("SOTM/Output/HoodAngleDeg", hoodAzimuth);
        SmartDashboard.putNumber("SOTM/Output/FlywheelRPM", flywheelRPM);
        SmartDashboard.putNumber("SOTM/Output/TurretVelocityFF", Math.toDegrees(turretVelocityFF));
    }
}
