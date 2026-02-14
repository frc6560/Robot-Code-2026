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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;

public class ShotCalculator {

    public record TurretState(double positionRadians, double velocityRadiansPerSecond) {}
    public record HoodState(double positionRadians, double velocityRadiansPerSecond) {}

    public record ShooterState(
        TurretState turret,
        HoodState hood,
        double flywheelRPM,
        Translation2d virtualTargetPose
    ) {}

    private double flywheelRPM;
    private double hoodAzimuth; 
    private double turretAngle; 
    private double hoodVelocity; 
    private double turretVelocity; 

    private static final double TIME_PARAMETER = 0.058; 
    private static final double VELOCITY_FILTER_ALPHA = 0.6;

    private static final InterpolatingDoubleTreeMap hoodAzimuthMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    public Translation2d virtualTargetPose; 

    // state for numerical differentiation
    private double prevHoodAzimuth = 0;
    private double prevTurretAngle = 0;
    private double prevTimestamp = 0;
    private double filteredHoodVelocity = 0;
    private double filteredTurretVelocity = 0;
    private boolean hasInitialized = false;

    public ShotCalculator() {
        this.flywheelRPM = 0;
        this.hoodAzimuth = 0;
        this.turretAngle = 0;
        this.hoodVelocity = 0;
        this.turretVelocity = 0;
        this.virtualTargetPose = new Translation2d();
        populateLUTs();
    }

    public void populateLUTs(){
        // FIXED: Changed negative RPMs to positive
       // flywheelRPMMap.put(1.1, 1000.0); 
        flywheelRPMMap.put(3.77, 1990.0);
        flywheelRPMMap.put(4.29, 2070.0);
        flywheelRPMMap.put(4.82, 2175.0); 
        flywheelRPMMap.put(5.47, 2285.0);

        hoodAzimuthMap.put(3.77, Math.toRadians(20));
        hoodAzimuthMap.put(4.29, Math.toRadians(25));
        hoodAzimuthMap.put(4.82, Math.toRadians(30));
        hoodAzimuthMap.put(5.47, Math.toRadians(35));

        timeOfFlightMap.put(3.93, 0.77);
        timeOfFlightMap.put(4.30, 0.81);
        timeOfFlightMap.put(4.83, 0.90);
        timeOfFlightMap.put(5.47, 1.008);
    }

    public double getHoodAzimuth() { return hoodAzimuth; }
    public double getTurretAngle() { return turretAngle; }
    public double getHoodVelocity() { return hoodVelocity; }
    public double getTurretVelocity() { return turretVelocity; }
    public double getFlywheelRPM() { return flywheelRPM; }
    public Translation2d getVirtualTargetPose() { return virtualTargetPose; }

    public ShooterState getState() {
        return new ShooterState(
            new TurretState(turretAngle, turretVelocity),
            new HoodState(hoodAzimuth, hoodVelocity),
            flywheelRPM,
            virtualTargetPose
        );
    }

    public void calculate(Pose2d currentRobotPose, ChassisSpeeds fieldVelocity) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()){ return; }
        Translation2d targetPose = (alliance.get() == Alliance.Blue) ? 
                                            FieldConstants.BLUE_HUB_CENTER : 
                                            FieldConstants.RED_HUB_CENTER;
        
        ChassisSpeeds robotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldVelocity.vxMetersPerSecond,
            fieldVelocity.vyMetersPerSecond,
            fieldVelocity.omegaRadiansPerSecond,
            currentRobotPose.getRotation()
        );
        
        Pose2d projectedPosition = currentRobotPose.exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * TIME_PARAMETER,
                robotVelocity.vyMetersPerSecond * TIME_PARAMETER,
                robotVelocity.omegaRadiansPerSecond * TIME_PARAMETER
            )
        );

        Transform2d turretTransform = new Transform2d(
            TurretConstants.ROBOT_RELATIVE_TURRET.getX(), 
            TurretConstants.ROBOT_RELATIVE_TURRET.getY(),
            new Rotation2d()
        );

        Pose2d turretPose = projectedPosition.transformBy(turretTransform);
        double angleOffset = Math.atan2(turretTransform.getY(), turretTransform.getX());
        double r = Math.hypot(turretTransform.getX(), turretTransform.getY());

        double turretVx = fieldVelocity.vxMetersPerSecond
                        + (-r * fieldVelocity.omegaRadiansPerSecond * Math.sin(projectedPosition.getRotation().getRadians() + angleOffset));
        double turretVy = fieldVelocity.vyMetersPerSecond
                        + (r * fieldVelocity.omegaRadiansPerSecond * Math.cos(projectedPosition.getRotation().getRadians() + angleOffset));
        
        SmartDashboard.putNumber("SOTM/FieldVel/vX", fieldVelocity.vxMetersPerSecond);
        SmartDashboard.putNumber("SOTM/FieldVel/vY", fieldVelocity.vyMetersPerSecond);

        virtualTargetPose = targetPose; 
        double timeOfFlight = 0;
        double distanceToTarget = turretPose.getTranslation().getDistance(targetPose);
        
        final double EPSILON = 0.01;
        double prevTimeOfFlight = 0;
        int iterationsUsed = 0;

        if(Math.hypot(turretVx, turretVy) > 0.1){
            // FIXED: Reduced iterations to 5 for CPU performance
            for(int i = 0; i < 5; i++){
                timeOfFlight = timeOfFlightMap.get(distanceToTarget);
                iterationsUsed = i + 1;

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
        SmartDashboard.putNumber("SOTM/Distance/Virtual", distanceToTarget);

        double newHoodAzimuth = hoodAzimuthMap.get(distanceToTarget);
        flywheelRPM = flywheelRPMMap.get(distanceToTarget);
        double newTurretAngle = MathUtil.angleModulus(Math.atan2(
            virtualTargetPose.getY() - turretPose.getY(),
            virtualTargetPose.getX() - turretPose.getX()
        ) - projectedPosition.getRotation().getRadians());

        // Velocity Calc
        double currentTime = Timer.getFPGATimestamp();
        if (hasInitialized) {
            double dt = currentTime - prevTimestamp;
            if (dt > 1e-6) {
                double rawHoodVelocity = MathUtil.angleModulus(newHoodAzimuth - prevHoodAzimuth) / dt;
                double rawTurretVelocity = MathUtil.angleModulus(newTurretAngle - prevTurretAngle) / dt;

                filteredHoodVelocity = VELOCITY_FILTER_ALPHA * filteredHoodVelocity
                                     + (1 - VELOCITY_FILTER_ALPHA) * rawHoodVelocity;
                filteredTurretVelocity = VELOCITY_FILTER_ALPHA * filteredTurretVelocity
                                       + (1 - VELOCITY_FILTER_ALPHA) * rawTurretVelocity;
            }
        } else {
            hasInitialized = true;
        }

        prevHoodAzimuth = newHoodAzimuth;
        prevTurretAngle = newTurretAngle;
        prevTimestamp = currentTime;

        hoodAzimuth = newHoodAzimuth;
        turretAngle = newTurretAngle;
        hoodVelocity = filteredHoodVelocity;
        turretVelocity = filteredTurretVelocity;

        SmartDashboard.putNumber("SOTM/Output/FlywheelRPM", flywheelRPM);
    }

    public void resetFilter() {
        hasInitialized = false;
        filteredHoodVelocity = 0;
        filteredTurretVelocity = 0;
    }
}