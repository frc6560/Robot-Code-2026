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
    public record TurretState(double positionRadians) {}

    /** State container for hood position and velocity. */
    public record HoodState(double positionRadians) {}

    /** Combined state for the entire shooter system. */
    public record ShooterState(
        TurretState turret,
        HoodState hood,
        double flywheelRPM,
        Translation2d virtualTargetPose
    ) {}

    private double flywheelRPM;
    private double hoodAzimuth; // in radians
    private double turretAngle; 

    private static final double TIME_PARAMETER = 0.058; 

    private static final InterpolatingDoubleTreeMap hoodAzimuthMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    public Translation2d virtualTargetPose; 

    /** A util class for outputting shooter state values, even while the robot is moving! */
    public ShotCalculator() {
        this.flywheelRPM = 0;
        this.hoodAzimuth = 0;
        this.turretAngle = 0;
        this.virtualTargetPose = new Translation2d();
        populateLUTs();
    }

    public void populateLUTs(){
        // shooter RPM
        flywheelRPMMap.put(1.279, 2500.0);
        flywheelRPMMap.put(1.853, 2700.0);
        flywheelRPMMap.put(2.459, 2800.0);
        flywheelRPMMap.put(3.062, 2900.0);
        flywheelRPMMap.put(3.676, 3000.0);
        flywheelRPMMap.put(4.234, 3200.0);
        flywheelRPMMap.put(4.822, 3300.0);
        flywheelRPMMap.put(5.345, 3550.0);


        // hood azimuth
        hoodAzimuthMap.put(1.279, 27.24);
        hoodAzimuthMap.put(1.853, 30.0);
        hoodAzimuthMap.put(2.459, 32.0);
        hoodAzimuthMap.put(3.062, 40.0);
        hoodAzimuthMap.put(3.676, 44.0);
        hoodAzimuthMap.put(4.234, 45.0);
        hoodAzimuthMap.put(4.822, 48.0);
        hoodAzimuthMap.put(5.345, 52.0);


        // time of flight (finish on main bot)
        timeOfFlightMap.put(3.93, 0.77);
        timeOfFlightMap.put(4.30, 0.81);
        timeOfFlightMap.put(4.83, 0.90);
        timeOfFlightMap.put(5.47, 1.008);
    }

    /** Returns the current hood azimuth in radians. */
    public double getHoodAzimuth() {
        return hoodAzimuth;
    }

    /** Returns the current turret angle in radians. */
    public double getTurretAngle() {
        return turretAngle;
    }

    /** Returns the current flywheel RPM. */
    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    public Translation2d getVirtualTargetPose() {
        return virtualTargetPose;
    }

    /** Returns the complete shooter state including positions and velocities. */
    public ShooterState getState() {
        return new ShooterState(
            new TurretState(turretAngle),
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
        
        SmartDashboard.putNumber("SOTM/Iterations", iterationsUsed);
        SmartDashboard.putNumber("SOTM/TimeOfFlight", timeOfFlight);
        SmartDashboard.putNumber("SOTM/Distance/Static", staticDistance);
        SmartDashboard.putNumber("SOTM/Distance/Virtual", distanceToTarget);

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


        // Log final output values
        SmartDashboard.putNumber("SOTM/Output/TurretAngleDeg", Math.toDegrees(turretAngle));
        SmartDashboard.putNumber("SOTM/Output/HoodAngleDeg", Math.toDegrees(hoodAzimuth));
        SmartDashboard.putNumber("SOTM/Output/FlywheelRPM", flywheelRPM);
    }
}
