package frc.robot.utility.Shooter;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotModelConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.utility.Shooter.PhysicsShotSolver.Solution;

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
    private double timeOfFlightSeconds;
    private boolean calculationValid;
    private Solution physicsSolution;

    private static final double TIME_PARAMETER = 0.051; 
    private static final double MOVING_VELOCITY_THRESHOLD_METERS_PER_SECOND = 0.3;
    private static final double TIME_OF_FLIGHT_EPSILON_SECONDS = 0.005;
    private static final int MAX_VIRTUAL_TARGET_ITERATIONS = 4;

    private final PhysicsShotSolver physicsSolver = new PhysicsShotSolver();

    public Translation2d virtualTargetPose;
    private double distanceToVirtualTarget = 0.0; 

    /** A util class for outputting shooter state values, even while the robot is moving! */
    public ShotCalculator() {
        resetToSafeState();
    }

    /** Returns the current hood azimuth in degrees. */
    public double getHoodAzimuth() {
        return hoodAzimuth;
    }

    public double getTurretTolerance (){
        double radius = FieldConstants.HUB_TOLERANCE / 2.0; // in meters, half the width of the hub
        return Math.toDegrees(Math.atan2(radius, distanceToVirtualTarget)); // in degrees
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

    /** Returns the physics model's flight time for the current virtual target. */
    public double getTimeOfFlightSeconds() {
        return timeOfFlightSeconds;
    }

    /** Returns the predicted downward entry angle at the upper HUB opening. */
    public double getEntryAngleDegrees() {
        return physicsSolution != null ? physicsSolution.entryAngleDegrees() : Double.NaN;
    }

    public Translation2d getVirtualTargetPose() {
        return virtualTargetPose;
    }

    /** Returns true when range and all modeled HUB scoring constraints are valid. */
    public boolean isShotValid() {
        return calculationValid;
    }

    /**
     * Returns whether the measured mechanism state still satisfies every modeled scoring
     * constraint at the current virtual-target range. This closes the release gate around the
     * physics-valid command window instead of relying only on broad subsystem tolerances.
     */
    public boolean measuredControlsScore(double measuredFlywheelRPM, double measuredHoodCommandDegrees) {
        if (!calculationValid) {
            return false;
        }
        return physicsSolver.evaluate(
            distanceToVirtualTarget,
            measuredFlywheelRPM,
            measuredHoodCommandDegrees
        ).valid();
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

    /** Calculates the shot parameters for a hub shot based on current robot pose and field velocity. */
    public void calculate(Pose2d currentRobotPose, 
                            ChassisSpeeds fieldVelocity) {
        calculationValid = false;
        SmartDashboard.putBoolean("SOTM/ShotValid", false);

        if (!hasFiniteInputs(currentRobotPose, fieldVelocity)) {
            resetToSafeState();
            DriverStation.reportWarning("Non-finite pose or velocity; shot calculation aborted.", false);
            return;
        }

        // Gets our target pose
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()){
            resetToSafeState();
            DriverStation.reportWarning("Alliance color not detected! Shot calculation aborted.", false);
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
        
        if(Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
                < MOVING_VELOCITY_THRESHOLD_METERS_PER_SECOND){
            robotVelocity = new ChassisSpeeds(0, 0, robotVelocity.omegaRadiansPerSecond);
        }

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

        // Calculates a virtual target from the flight time returned by the same physics
        // solution that supplies the final hood and RPM commands.
        virtualTargetPose = targetPose; 
        double distanceToTarget = turretPose.getTranslation().getDistance(targetPose);
        double staticDistance = distanceToTarget; // save for logging
        Solution solution = solveAtDistance(distanceToTarget);
        double timeOfFlight = solution.valid() ? solution.timeOfFlightSeconds() : 0.0;
        int iterationsUsed = 0;

        if(Math.hypot(turretVx, turretVy) > MOVING_VELOCITY_THRESHOLD_METERS_PER_SECOND){
            for(int i = 0; i < MAX_VIRTUAL_TARGET_ITERATIONS && solution.valid(); i++){
                double previousTimeOfFlight = timeOfFlight;
                iterationsUsed = i + 1;
                virtualTargetPose = targetPose.minus(
                    new Translation2d(
                        turretVx * timeOfFlight,
                        turretVy * timeOfFlight
                    )
                );
                distanceToTarget = turretPose.getTranslation().getDistance(virtualTargetPose);
                solution = solveAtDistance(distanceToTarget);
                if (!solution.valid()) {
                    break;
                }
                timeOfFlight = solution.timeOfFlightSeconds();

                if (Math.abs(timeOfFlight - previousTimeOfFlight)
                        < TIME_OF_FLIGHT_EPSILON_SECONDS) {
                    break;
                }
            }
        }
        
        distanceToVirtualTarget = distanceToTarget;
        physicsSolution = solution;
        timeOfFlightSeconds = timeOfFlight;
        boolean distanceInRange = distanceToTarget >= ShotModelConstants.MIN_DISTANCE_METERS
            && distanceToTarget <= ShotModelConstants.MAX_DISTANCE_METERS;
        calculationValid = distanceInRange && solution.valid();

        hoodAzimuth = solution.hoodCommandDegrees();
        flywheelRPM = solution.flywheelRPM();

        SmartDashboard.putNumber("SOTM/Iterations", iterationsUsed);
        SmartDashboard.putNumber("SOTM/TimeOfFlight", timeOfFlight);
        SmartDashboard.putNumber("SOTM/Distance/Static", staticDistance);
        SmartDashboard.putNumber("SOTM/Distance/Virtual", distanceToTarget);
        SmartDashboard.putBoolean("SOTM/ShotValid", isShotValid());

        Translation2d targetOffset = virtualTargetPose.minus(targetPose);
        SmartDashboard.putNumber("SOTM/VirtualOffset/X", targetOffset.getX());
        SmartDashboard.putNumber("SOTM/VirtualOffset/Y", targetOffset.getY());
        SmartDashboard.putNumber("SOTM/VirtualOffset/magnitude", targetOffset.getNorm());

        turretAngle = MathUtil.angleModulus(Math.atan2(
            virtualTargetPose.getY() - turretPose.getY(),
            virtualTargetPose.getX() - turretPose.getX()
        ) - projectedPosition.getRotation().getRadians());

        double deltaX = virtualTargetPose.getX() - turretPose.getX();
        double deltaY = virtualTargetPose.getY() - turretPose.getY();
        double distSquared = distanceToTarget * distanceToTarget;

        if (distSquared > 0.01
                && Math.hypot(turretVx, turretVy) > MOVING_VELOCITY_THRESHOLD_METERS_PER_SECOND) {
            double losRate = (turretVx * deltaY - turretVy * deltaX) / distSquared;
            turretVelocityFF = losRate - fieldVelocity.omegaRadiansPerSecond;
        } else {
            turretVelocityFF = -fieldVelocity.omegaRadiansPerSecond;
        }

        // Log final output values
        SmartDashboard.putNumber("SOTM/Output/TurretAngleDeg", Math.toDegrees(turretAngle));
        SmartDashboard.putNumber("SOTM/Output/HoodAngleDeg", hoodAzimuth);
        SmartDashboard.putNumber("SOTM/Output/LaunchElevationDeg", solution.launchElevationDegrees());
        SmartDashboard.putNumber("SOTM/Output/FlywheelRPM", flywheelRPM);
        SmartDashboard.putNumber("SOTM/Output/TurretVelocityFF", Math.toDegrees(turretVelocityFF));
        SmartDashboard.putNumber("SOTM/Model/EntryAngleDeg", solution.entryAngleDegrees());
        SmartDashboard.putNumber("SOTM/Model/OpeningClearanceMeters", solution.openingClearanceMeters());
        SmartDashboard.putNumber("SOTM/Model/NearRimClearanceMeters", solution.nearRimClearanceMeters());
        SmartDashboard.putBoolean("SOTM/Model/PolicyValidated", solution.valid());
    }

    private Solution solveAtDistance(double distanceMeters) {
        return physicsSolver.solveRuntime(distanceMeters);
    }

    private static boolean hasFiniteInputs(Pose2d pose, ChassisSpeeds velocity) {
        return pose != null
            && velocity != null
            && Double.isFinite(pose.getX())
            && Double.isFinite(pose.getY())
            && Double.isFinite(pose.getRotation().getRadians())
            && Double.isFinite(velocity.vxMetersPerSecond)
            && Double.isFinite(velocity.vyMetersPerSecond)
            && Double.isFinite(velocity.omegaRadiansPerSecond);
    }

    private void resetToSafeState() {
        flywheelRPM = ShooterConstants.FLYWHEEL_IDLE_RPM;
        hoodAzimuth = HoodConstants.HOOD_MIN_ANGLE;
        turretAngle = 0.0;
        turretVelocityFF = 0.0;
        timeOfFlightSeconds = 0.0;
        calculationValid = false;
        physicsSolution = null;
        virtualTargetPose = new Translation2d();
        distanceToVirtualTarget = 0.0;
    }
}
