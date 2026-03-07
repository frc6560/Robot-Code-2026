package frc.robot.autonomous;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.ClimbCommand;

// YOU NEEDED THIS IMPORT:
import frc.robot.subsystems.climb.Climb; 

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// refactor code: add follow and shoot helper
// add a helper method to sequence commands: sequencePaths(Commands... commands)
public class AutoCommands {
    private SwerveSubsystem drivetrain;
    private Feeder feeder;
    private Intake intake;
    private Climb climb;

    private AutoFactory autoFactory;

    // ADDED 'Climb climb' TO THE ARGUMENTS HERE:
    public AutoCommands(SwerveSubsystem drivetrain,
        Feeder feeder,
        Intake intake,
        Climb climb 
    ) {
        this.drivetrain = drivetrain;
        this.feeder = feeder;
        this.intake = intake;
        this.climb = climb;
        
        autoFactory = new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetOdometry,
            drivetrain::followSegment,
            true,
            drivetrain
        );
    }

    private static final double DEFAULT_TOLERANCE_METERS = 0.1;

    /**
     * Wraps a trajectory command to end early when within tolerance of the final pose.
     * @param trajectory The AutoTrajectory to follow
     * @param toleranceMeters Distance tolerance in meters
     * @return Command that follows the trajectory but ends early if within tolerance
     */
    public Command cmdWithAccuracy(AutoTrajectory trajectory, double toleranceMeters) {
        return trajectory.cmd().until(() -> {
            Pose2d currentPose = drivetrain.getPose();
            Pose2d finalPose = trajectory.getFinalPose().orElse(null);
            if (finalPose == null) {
                return false;
            }
            return currentPose.getTranslation().getDistance(finalPose.getTranslation()) < toleranceMeters;
        });
    }

    /**
     * Wraps a trajectory command to end early when within 0.1m of the final pose.
     * @param trajectory The AutoTrajectory to follow
     * @return Command that follows the trajectory but ends early if within tolerance
     */
    public Command cmdWithAccuracy(AutoTrajectory trajectory) {
        return cmdWithAccuracy(trajectory, DEFAULT_TOLERANCE_METERS);
    }

    /** These are functions for returning different autonomous routines. See AutoNames.java for more information. */

    /** These literally do nothing. As in, nothing. */
    public AutoRoutine getNoAuto(){
        final AutoRoutine IDLE = autoFactory.newRoutine("idle");
        return IDLE;
    }

    public Command shoot(){
        return Commands.run(() -> feeder.requestFeed(), feeder)
            .withTimeout(3.0)
            .finallyDo((interrupted) -> {
                feeder.requestStop();
            });
    }

    public Command climb(){
        return new ClimbCommand(drivetrain, climb);
    }

    public Command intake(){
        return Commands.runOnce(() -> intake.setExtensionMode(), intake);
    }

    public Command retract(){
        return Commands.runOnce(() -> intake.setIdleMode(), intake);
    }

    /** Right side trench auto */
    public AutoRoutine getTestTrench(){
        AutoRoutine testRoutine = autoFactory.newRoutine("testTrench");
        
        AutoTrajectory trenchToCenter = testRoutine.trajectory("hpTrenchToCenter");
        AutoTrajectory trenchToShoot = testRoutine.trajectory("hpTrenchToShoot");
        AutoTrajectory trenchToClimb = testRoutine.trajectory("hpTrenchToClimb");
        AutoTrajectory trenchToCenterSecondSwipe = testRoutine.trajectory("hpTrenchToCenter2");

        trenchToCenter.atTime("intake")
            .onTrue(
                intake()
            );

        trenchToCenterSecondSwipe.atTime("intake")
            .onTrue(
                intake()
            );

        trenchToShoot.atTime("shoot")
            .onTrue(
                retract()
            );
        
        trenchToClimb.atTime("shoot")
            .onTrue(
                retract()
            );

        testRoutine
            .active()
                .onTrue(
                    Commands.sequence(
                        cmdWithAccuracy(trenchToCenter) 
                            .beforeStarting(trenchToCenter.resetOdometry()),
                        cmdWithAccuracy(trenchToShoot)
                            .andThen(Commands.waitSeconds(0.5))
                            .andThen(shoot()), 
                        cmdWithAccuracy(trenchToCenterSecondSwipe),
                        cmdWithAccuracy(trenchToClimb)
                            .andThen(shoot())
                            .andThen(climb())
                    )
        );

        return testRoutine;
    }

    /** Left side trench auto */
    public AutoRoutine getTestTrenchL(){
        AutoRoutine testRoutine = autoFactory.newRoutine("testTrenchL");
        
        AutoTrajectory trenchToCenter = testRoutine.trajectory("depotTrenchToCenter");
        AutoTrajectory trenchToShoot = testRoutine.trajectory("depotTrenchToShoot");
        AutoTrajectory trenchToClimb = testRoutine.trajectory("depotTrenchToClimb");
        AutoTrajectory trenchToCenterSecondSwipe = testRoutine.trajectory("depotTrenchToCenter2");

        trenchToCenter.atTime("intake")
            .onTrue(
                intake()
            );

        trenchToCenterSecondSwipe.atTime("intake")
            .onTrue(
                intake()
            );

        trenchToShoot.atTime("shoot")
            .onTrue(
                retract()
            );
        
        trenchToClimb.atTime("shoot")
            .onTrue(
                retract()
            );

        testRoutine
            .active()
                .onTrue(
                    Commands.sequence(
                        cmdWithAccuracy(trenchToCenter) 
                            .beforeStarting(trenchToCenter.resetOdometry()),
                        cmdWithAccuracy(trenchToShoot)
                            .andThen(Commands.waitSeconds(0.5))
                            .andThen(shoot()), 
                        cmdWithAccuracy(trenchToCenterSecondSwipe),
                        cmdWithAccuracy(trenchToClimb)
                            .andThen(shoot())
                            .andThen(climb())
                    )
        );

        return testRoutine;
    }
}