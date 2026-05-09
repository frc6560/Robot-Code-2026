package frc.robot.autonomous;


import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.utility.Shooter.ShotCalculator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


// refactor code: add follow and shoot helper
// add a helper method to sequence commands: sequencePaths(Commands... commands)
public class AutoCommands {
    private SwerveSubsystem drivetrain;
    private Feeder feeder;
    private Intake intake;
    private Shooter shooter;
    private ShotCalculator calculator = new ShotCalculator();

    private AutoFactory autoFactory;

    public AutoCommands(SwerveSubsystem drivetrain,
        Feeder feeder,
        Intake intake,
        Shooter shooter
    ) {
        this.drivetrain = drivetrain;
        this.feeder = feeder;
        this.intake = intake;
        this.shooter = shooter;

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
        return Commands.run(() -> {
            calculator.calculate(drivetrain.getPose(), drivetrain.getFieldVelocity());
            shooter.setGoal(calculator.getFlywheelRPM());
            if(shooter.atTarget()){
                feeder.setShooting(true);
                feeder.setIntaking(true);
                intake.activate();
            }
        }).withTimeout(3.0)
        .finallyDo((interrupted) -> {
            feeder.setShooting(false);
            feeder.setIntaking(false);
            intake.deactivate();
            shooter.setGoal(0);
        });
    }

    public Command intake(){
        return Commands.runOnce(() -> {
            intake.activate();
            feeder.setIntaking(true);
        }, intake, feeder);
    }

    public Command retract(){
        return Commands.runOnce(() -> {
            intake.deactivate();
            feeder.setIntaking(false);
        }, intake, feeder);
    }

    /** Right side trench auto */
    public AutoRoutine getRightAuto(){
        AutoRoutine testRoutine = autoFactory.newRoutine("testTrench");
        
        AutoTrajectory trenchToCenter = testRoutine.trajectory("hpTrenchToCenter");
        AutoTrajectory trenchToShoot = testRoutine.trajectory("hpTrenchToShoot");
        AutoTrajectory trenchToHp = testRoutine.trajectory("hpTrenchToHP");
        AutoTrajectory hpToCenter = testRoutine.trajectory("hpHpToCenter");

        trenchToCenter.atTime("intake")
            .onTrue(
                intake()
            );

        trenchToShoot.atTime("shoot")
            .onTrue(
                retract()
            );
        
        trenchToHp.atTime("intake")
            .onTrue(
                intake()
            );
        
        trenchToHp.atTime("shoot")
            .onTrue(
                retract()
            );
        
        hpToCenter.atTime("intake")
            .onTrue(
                intake()
            );

        testRoutine
            .active()
                .onTrue(
                    Commands.sequence(
                        cmdWithAccuracy(trenchToCenter)
                            .beforeStarting(trenchToCenter.resetOdometry()),
                        cmdWithAccuracy(trenchToShoot)
                            .andThen(shoot()),
                        cmdWithAccuracy(trenchToHp)
                            .andThen(shoot()),
                        cmdWithAccuracy(hpToCenter)
                    )
        );

        return testRoutine;
    }

    /** Left side trench auto */
    public AutoRoutine getLeftAuto(){
        AutoRoutine testRoutine = autoFactory.newRoutine("testTrenchL");
        
        AutoTrajectory trenchToCenter = testRoutine.trajectory("depotTrenchToCenter");
        AutoTrajectory trenchToShoot = testRoutine.trajectory("depotTrenchToShoot");
        AutoTrajectory trenchToDepot = testRoutine.trajectory("depotTrenchToDepot");
        AutoTrajectory trenchPullout = testRoutine.trajectory("depotPullout");
        AutoTrajectory trenchToCenterSecondSwipe = testRoutine.trajectory("depotDepotToCenter");

        trenchToCenter.atTime("intake")
            .onTrue(
                intake()
            );

        trenchToShoot.atTime("shoot")
            .onTrue(
                retract()
            );

        trenchToDepot.atTime("intake")
            .onTrue(
                intake()
            );
        
        trenchToDepot.atTime("shoot")
            .onTrue(
                retract()
            );

        trenchToCenterSecondSwipe.atTime("intake")
            .onTrue(
                intake()
            );
        
        testRoutine
            .active()
                .onTrue(
                    Commands.sequence(
                        cmdWithAccuracy(trenchToCenter)
                            .beforeStarting(trenchToCenter.resetOdometry()),
                        cmdWithAccuracy(trenchToShoot)
                            .andThen(shoot()),
                        cmdWithAccuracy(trenchToDepot)
                            .andThen(shoot()),
                        cmdWithAccuracy(trenchPullout),
                        cmdWithAccuracy(trenchToCenterSecondSwipe)
                    )
        );

        return testRoutine;
    }

    
    /** Two swipe HP side auto. Runs the first swipe as before, but then runs a loop for the second swipe */
    public AutoRoutine getRightTwoSwipe(){
        AutoRoutine testRoutine = autoFactory.newRoutine("twoSwipeHumanPlayer");

        AutoTrajectory trenchToCenter = testRoutine.trajectory("hpTrenchToCenter");
        AutoTrajectory trenchToShoot = testRoutine.trajectory("hpFirstSwipeBack");
        AutoTrajectory trenchToCenterSecondSwipe = testRoutine.trajectory("hpSecondSwipe");

        trenchToCenter.atTime("intake")
            .onTrue(
                intake()
            );
        
        trenchToShoot.atTime("shoot")
            .onTrue(
                retract()
            );
        
        trenchToCenterSecondSwipe.atTime("intake")
            .onTrue(
                intake()
            );
        
        trenchToCenterSecondSwipe.atTime("retract")
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
                            .andThen(shoot()),
                        cmdWithAccuracy(trenchToCenterSecondSwipe)
                            .andThen(shoot())
                    )
        );

        return testRoutine;
    }
}

