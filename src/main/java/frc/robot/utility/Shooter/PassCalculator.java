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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotModelConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.utility.Shooter.PhysicsShotSolver.PassSolution;

public class PassCalculator {

    public record TurretState(double positionRadians, double velocityRadiansPerSecond) {}

    private double flywheelRPM;
    private double turretAngle;
    private double turretVelocityFF; // feedforward velocity in rad/s 

    public Translation2d virtualTargetPose;

    private static final double PASS_TABLE_STEP_METERS = 0.25;

    /**
     * RPM and flight time to land a ball on the carpet at each distance, solved from the same
     * physics model as hub shots. Distances past the flywheel's reach clamp to the farthest pass.
     */
    private final InterpolatingDoubleTreeMap flywheelRPMMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    public PassCalculator() {
        flywheelRPM = ShooterConstants.FLYWHEEL_IDLE_RPM;
        turretAngle = 0.0;
        turretVelocityFF = 0.0;
        virtualTargetPose = new Translation2d();

        PhysicsShotSolver solver = new PhysicsShotSolver();
        for (double distance = ShotModelConstants.PASS_MIN_DISTANCE_METERS;
                distance <= ShotModelConstants.PASS_MAX_DISTANCE_METERS + 1e-9;
                distance += PASS_TABLE_STEP_METERS) {
            PassSolution pass = solver.solvePass(distance);
            if (!pass.valid()) {
                break; // out of reach; the map clamps to the last reachable distance
            }
            flywheelRPMMap.put(distance, pass.flywheelRPM());
            timeOfFlightMap.put(distance, pass.timeOfFlightSeconds());
        }
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getTurretVelocityFF() {
        return turretVelocityFF;
    }

    public double getFlywheelRPM(){
        return flywheelRPM;
    }

    /** Calculates the flywheel speed and turret angle based on the robot's pose */
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