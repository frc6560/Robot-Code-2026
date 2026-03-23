package frc.robot.commands.periodic;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utility.Shooter.PassCalculator;
import frc.robot.utility.Shooter.ShotCalculator;

public class SuperstructureCommand extends Command {

    enum SuperstructureState {
        IDLE,
        PASS,
        SHOOT
    }

    public interface PoseSupplier {
        Pose2d getPose();
    }

    public interface VelocitySupplier {
        ChassisSpeeds getFieldVelocity();
    }

    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;
    private final PoseSupplier poseSupplier;
    private final VelocitySupplier velocitySupplier;
    private final ShotCalculator shotCalculator;
    private final PassCalculator passCalculator;

    private SuperstructureState state = SuperstructureState.IDLE;

    // Trench detection fields
    private final double HOOD_DEACTUATION_TIME = 1.0; // in seconds
    private final double TRENCH_TOLERANCE = 0; // in meters, larger than trench boundary

    private int executeCounter = 0; // Debug counter to verify execute() is being called

    // Field2d for visualization
    private final Field2d field = new Field2d();
    private final Pose2d[] trajectoryPoses = new Pose2d[10]; // Pre-allocate for trajectory visualization

    public SuperstructureCommand(
            Hood hood,
            Shooter shooter,
            Turret turret,
            PoseSupplier poseSupplier,
            VelocitySupplier velocitySupplier,
            ShotCalculator shotCalculator,
            PassCalculator passCalculator) {
        this.hood = hood;
        this.shooter = shooter;
        this.turret = turret;
        this.poseSupplier = poseSupplier;
        this.velocitySupplier = velocitySupplier;
        this.passCalculator = passCalculator;
        this.shotCalculator = shotCalculator;
        addRequirements(hood, shooter, turret);
    }

    @Override
    public void initialize() {
        shooter.setIdle();

        // Log that the command has started
        executeCounter = 0; // Reset counter on initialize
        SmartDashboard.putBoolean("SuperstructureCommand/Running", true);
        SmartDashboard.putString("SuperstructureCommand/Status", "Initialized");
        SmartDashboard.putNumber("SuperstructureCommand/Initialize Time", System.currentTimeMillis() / 1000.0);
        System.out.println("SuperstructureCommand initialized at " + System.currentTimeMillis());

        // Add field visualization to SmartDashboard
        SmartDashboard.putData("Trajectory/Field", field);
    }

    @Override
    public void execute() {
        handleState();
        performTrenchDetection();
        updateBehavior();
    }

    private void handleState() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            state = SuperstructureState.IDLE;
            return;
        }

        Pose2d robotPose = poseSupplier.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        // Check if in our alliance zone -> SHOOT
        boolean inOurZone = (alliance.get() == Alliance.Blue && robotX < FieldConstants.BLUE_ZONE_X)
                         || (alliance.get() == Alliance.Red && robotX > FieldConstants.RED_ZONE_X);

        SmartDashboard.putBoolean("SuperstructureCmd/InPassZone", !inOurZone);

        if (inOurZone) {
            state = SuperstructureState.SHOOT;
        } else {
            // Pass zone is everywhere except our zone

            // Check hub deadzone
            Translation2d hubCenter = (alliance.get() == Alliance.Blue)
                ? FieldConstants.BLUE_HUB_CENTER
                : FieldConstants.RED_HUB_CENTER;
            double distanceToHub = robotPose.getTranslation().getDistance(hubCenter);
            boolean inHubDeadzone = distanceToHub < FieldConstants.DEAD_RAD;
            SmartDashboard.putBoolean("SuperstructureCmd/InHubDeadzone", inHubDeadzone);
            SmartDashboard.putNumber("SuperstructureCmd/DistanceToHub", distanceToHub);

            if (inHubDeadzone) {
                state = SuperstructureState.IDLE;
                SmartDashboard.putString("SuperstructureCmd/State", state.toString());
                return;
            }

            // Check opponent rectangular deadzone
            boolean inOpponentDeadzone = false;
            if (alliance.get() == Alliance.Blue) {
                inOpponentDeadzone = robotX >= FieldConstants.RED_DEADZONE_MIN_X &&
                                      robotX <= FieldConstants.RED_DEADZONE_MAX_X &&
                                      robotY >= FieldConstants.PASS_DEADZONE_MIN_Y &&
                                      robotY <= FieldConstants.PASS_DEADZONE_MAX_Y;
                SmartDashboard.putBoolean("SuperstructureCmd/InRedDeadzone", inOpponentDeadzone);
            } else {
                inOpponentDeadzone = robotX >= FieldConstants.BLUE_DEADZONE_MIN_X &&
                                      robotX <= FieldConstants.BLUE_DEADZONE_MAX_X &&
                                      robotY >= FieldConstants.PASS_DEADZONE_MIN_Y &&
                                      robotY <= FieldConstants.PASS_DEADZONE_MAX_Y;
                SmartDashboard.putBoolean("SuperstructureCmd/InBlueDeadzone", inOpponentDeadzone);
            }

            if (inOpponentDeadzone) {
                state = SuperstructureState.IDLE;
                SmartDashboard.putString("SuperstructureCmd/State", state.toString());
                return;
            }

            state = SuperstructureState.PASS;
        }

        SmartDashboard.putString("SuperstructureCmd/State", state.toString());
    }

    private void updateBehavior() {
        switch (state) {
            case IDLE:
                idleState();
                break;
            case PASS:
                trackPassingTarget();
                break;
            case SHOOT:
                trackHubTarget();
                break;
        }
    }

    private void idleState() {
        shooter.setIdle();
    }

    private void trackHubTarget() {
        Pose2d pose = poseSupplier.getPose();
        ChassisSpeeds velocity = velocitySupplier.getFieldVelocity();

        shotCalculator.calculate(pose, velocity);

        if(!DriverStation.isAutonomous()){
            shooter.setGoal(shotCalculator.getFlywheelRPM());
        }
        turret.setGoalWithVelocity(Units.radiansToDegrees(shotCalculator.getTurretAngle()), 
                                    Units.radiansToDegrees(shotCalculator.getTurretVelocityFF()));
        hood.setGoal(shotCalculator.getHoodAzimuth());
    }


    /**
     * Checks if robot trajectory intersects with a rectangular trench region.
     * Uses parametric motion: position(t) = robotPos + velocity * t
     * For each boundary (constant x or y), solve for crossing time t and check:
     *   1. Is 0 <= t <= HOOD_DEACTUATION_TIME?
     *   2. Is the perpendicular coordinate within bounds at time t?
     *
     * @param robotPos Current robot position
     * @param velocity Robot velocity vector
     * @param xMin Left boundary of trench
     * @param xMax Right boundary of trench
     * @param yMin Bottom boundary of trench
     * @param yMax Top boundary of trench
     * @return true if trajectory intersects trench
     */

    public void trackPassingTarget(){
        Pose2d pose = poseSupplier.getPose();
        ChassisSpeeds velocity = velocitySupplier.getFieldVelocity();

        passCalculator.calculate(pose, velocity);

        turret.setGoalWithVelocity(Units.radiansToDegrees(passCalculator.getTurretAngle()), 
                                    Units.radiansToDegrees(passCalculator.getTurretVelocityFF()));
        hood.setGoal(passCalculator.getHoodAzimuth());
        if(!DriverStation.isAutonomous()){
            shooter.setGoal(passCalculator.getFlywheelRPM());
        }
    }

    private boolean trajectoryIntersectsTrench(Translation2d robotPos, Translation2d velocity,
                                               double xMin, double xMax,
                                               double yMin, double yMax) {
        double x0 = robotPos.getX();
        double y0 = robotPos.getY();
        double vx = velocity.getX();
        double vy = velocity.getY();

        // Check if robot starts inside the trench
        if (x0 >= xMin && x0 <= xMax && y0 >= yMin && y0 <= yMax) {
            return true;
        }

        // Check crossing LEFT boundary (x = xMin)
        // Solve: x0 + vx * t = xMin  =>  t = (xMin - x0) / vx
        if (Math.abs(vx) > 1e-10) {
            double t = (xMin - x0) / vx;
            if (t >= 0 && t <= HOOD_DEACTUATION_TIME) {
                double y_at_crossing = y0 + vy * t;
                if (y_at_crossing >= yMin && y_at_crossing <= yMax) {
                    return true;
                }
            }
        }

        // Check crossing RIGHT boundary (x = xMax)
        // Solve: x0 + vx * t = xMax  =>  t = (xMax - x0) / vx
        if (Math.abs(vx) > 1e-10) {
            double t = (xMax - x0) / vx;
            if (t >= 0 && t <= HOOD_DEACTUATION_TIME) {
                double y_at_crossing = y0 + vy * t;
                if (y_at_crossing >= yMin && y_at_crossing <= yMax) {
                    return true;
                }
            }
        }

        // Check crossing BOTTOM boundary (y = yMin)
        // Solve: y0 + vy * t = yMin  =>  t = (yMin - y0) / vy
        if (Math.abs(vy) > 1e-10) {
            double t = (yMin - y0) / vy;
            if (t >= 0 && t <= HOOD_DEACTUATION_TIME) {
                double x_at_crossing = x0 + vx * t;
                if (x_at_crossing >= xMin && x_at_crossing <= xMax) {
                    return true;
                }
            }
        }

        // Check crossing TOP boundary (y = yMax)
        // Solve: y0 + vy * t = yMax  =>  t = (yMax - y0) / vy
        if (Math.abs(vy) > 1e-10) {
            double t = (yMax - y0) / vy;
            if (t >= 0 && t <= HOOD_DEACTUATION_TIME) {
                double x_at_crossing = x0 + vx * t;
                if (x_at_crossing >= xMin && x_at_crossing <= xMax) {
                    return true;
                }
            }
        }

        return false;
    }

    /** State machine to determine robot's (future) position and how to act correspondingly */
    private void performTrenchDetection() {
        // Debug: Increment counter and log to verify execute() is being called
        executeCounter++;
        // SmartDashboard.putNumber("SuperstructureCommand/Execute Counter", executeCounter);
        // SmartDashboard.putBoolean("SuperstructureCommand/Execute Running", true);

        // Get current position and velocity
        Translation2d robotPos = poseSupplier.getPose().getTranslation();
        ChassisSpeeds fieldVelocity = velocitySupplier.getFieldVelocity();
        Translation2d velocity = new Translation2d(
            fieldVelocity.vxMetersPerSecond,
            fieldVelocity.vyMetersPerSecond
        );

        // Calculate projected position at end of hood deactuation time
        Translation2d projectedPos = robotPos.plus(velocity.times(HOOD_DEACTUATION_TIME));

        // Check trajectory against all 4 trench regions (FRC 2026 field coordinates)
        // Blue Left Trench (narrower x range)
        boolean intersectsBlueLeft = trajectoryIntersectsTrench(
            robotPos, velocity,
            4.10, 5.25,
            6.75, 9.42
        );

        // Blue Right Trench
        boolean intersectsBlueRight = trajectoryIntersectsTrench(
            robotPos, velocity,
            4.10 - TRENCH_TOLERANCE, 5.25 + TRENCH_TOLERANCE,
            -0.70 - TRENCH_TOLERANCE, 1.27 + TRENCH_TOLERANCE
        );

        // Red Left Trench
        boolean intersectsRedLeft = trajectoryIntersectsTrench(
            robotPos, velocity,
            11.29 - TRENCH_TOLERANCE, 12.44 + TRENCH_TOLERANCE,
            -0.70 - TRENCH_TOLERANCE, 1.27 + TRENCH_TOLERANCE
        );

        // Red Right Trench
        boolean intersectsRedRight = trajectoryIntersectsTrench(
            robotPos, velocity,
            11.29 - TRENCH_TOLERANCE, 12.44 + TRENCH_TOLERANCE,
            6.75 - TRENCH_TOLERANCE, 9.42 + TRENCH_TOLERANCE
        );

        // Stop hood if trajectory intersects ANY trench
        boolean intersectsAnyTrench = intersectsBlueLeft || intersectsBlueRight ||
                                       intersectsRedLeft || intersectsRedRight;

        // Control hood based on trench intersection
        if (intersectsAnyTrench) {
            hood.setGoal(15); // Full rumble
            state = SuperstructureState.IDLE; // forces idle state
        }

        // ========== VISUALIZATION LOGGING FOR ADVANTAGESCOPE ==========

        // Get current robot pose with rotation
        Pose2d currentPose = poseSupplier.getPose();

        // Create projected pose (position at end of trajectory with current rotation)
        Pose2d projectedPose = new Pose2d(projectedPos, currentPose.getRotation());

        // Update Field2d visualization
        field.setRobotPose(currentPose); // Current robot position
        field.getObject("Projected Pose").setPose(projectedPose); // Projected position

        // Create trajectory line for visualization
        for (int i = 0; i < trajectoryPoses.length; i++) {
            double t = (HOOD_DEACTUATION_TIME / (trajectoryPoses.length - 1)) * i;
            Translation2d point = robotPos.plus(velocity.times(t));
            trajectoryPoses[i] = new Pose2d(point, currentPose.getRotation());
        }
        field.getObject("Trajectory Line").setPoses(trajectoryPoses);



        // Visualize trench boundaries on field as rectangles (corner poses)
        visualizeTrenchBoundary("Blue Left Trench",
            4.10 - TRENCH_TOLERANCE, 6.75 - TRENCH_TOLERANCE,
            5.25 + TRENCH_TOLERANCE, 9.42 + TRENCH_TOLERANCE,
            intersectsBlueLeft);
        visualizeTrenchBoundary("Blue Right Trench",
            4.10 - TRENCH_TOLERANCE, -0.70 - TRENCH_TOLERANCE,
            5.25 + TRENCH_TOLERANCE, 1.27 + TRENCH_TOLERANCE,
            intersectsBlueRight);
        visualizeTrenchBoundary("Red Left Trench",
            11.29 - TRENCH_TOLERANCE, -0.70 - TRENCH_TOLERANCE,
            12.44 + TRENCH_TOLERANCE, 1.27 + TRENCH_TOLERANCE,
            intersectsRedLeft);
        visualizeTrenchBoundary("Red Right Trench",
            11.29 - TRENCH_TOLERANCE, 6.75 - TRENCH_TOLERANCE,
            12.44 + TRENCH_TOLERANCE, 9.42 + TRENCH_TOLERANCE,
            intersectsRedRight);
    }

    /**
     * Helper method to visualize a trench boundary as a rectangle on the field
     */
    private void visualizeTrenchBoundary(String name, double xMin, double yMin, double xMax, double yMax, boolean isIntersecting) {
        // Create 4 corner poses to draw a rectangle
        Pose2d[] corners = new Pose2d[5]; // 5 points to close the rectangle
        corners[0] = new Pose2d(xMin, yMin, new Rotation2d());
        corners[1] = new Pose2d(xMax, yMin, new Rotation2d());
        corners[2] = new Pose2d(xMax, yMax, new Rotation2d());
        corners[3] = new Pose2d(xMin, yMax, new Rotation2d());
        corners[4] = new Pose2d(xMin, yMin, new Rotation2d()); // Close the loop

        field.getObject(name).setPoses(corners);

        // Also show center point with a marker
        double centerX = (xMin + xMax) / 2.0;
        double centerY = (yMin + yMax) / 2.0;
        field.getObject(name + " Center").setPose(new Pose2d(centerX, centerY, new Rotation2d()));
    }


    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        turret.stopMotor();
        hood.stop();
        SmartDashboard.putBoolean("SuperstructureCommand/Running", false);
        SmartDashboard.putString("SuperstructureCommand/Status", interrupted ? "INTERRUPTED" : "ENDED");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
