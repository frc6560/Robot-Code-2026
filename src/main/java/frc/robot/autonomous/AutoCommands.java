package frc.robot.autonomous;


import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


// refactor code: add follow and shoot helper
// add a helper method to sequence commands: sequencePaths(Commands... commands)
public class AutoCommands {
    private SwerveSubsystem drivetrain;
    private Feeder feeder;
    private Intake intake;

    private AutoFactory autoFactory;

    public AutoCommands(SwerveSubsystem drivetrain,
        Feeder feeder,
        Intake intake
    ) {
        this.drivetrain = drivetrain;
        this.feeder = feeder;
        this.intake = intake;
        
        autoFactory = new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetOdometry,
            drivetrain::followSegment,
            true,
            drivetrain
        );
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

    public Command intake(){
        return Commands.runOnce(() -> intake.setExtensionMode(), intake);
    }

    public Command retract(){
        return Commands.runOnce(() -> intake.setIdleMode(), intake);
    }

    /** Test auto on HP side. Should be comp level accuracy. */
    public AutoRoutine getTestBump(){
        AutoRoutine testRoutine = autoFactory.newRoutine("testBump");
        
        AutoTrajectory trenchToCenter = testRoutine.trajectory("hpTrenchToCenter");
        AutoTrajectory trenchToShoot = testRoutine.trajectory("hpTrenchToShoot");
        AutoTrajectory bumpToShoot = testRoutine.trajectory("hpBumpToShoot");
        AutoTrajectory climb = testRoutine.trajectory("hpClimb");

        trenchToCenter.atTime("intake")
            .onTrue(
                intake()
            );

        trenchToShoot.atTime("shoot")
            .onTrue(
                retract()
            );

        bumpToShoot.atTime("shoot")
            .onTrue(
                retract()
            );


        testRoutine
            .active()
                .onTrue(
                    Commands.sequence(
                        trenchToCenter.resetOdometry(),
                        trenchToCenter.cmd(), 
                        trenchToShoot.cmd()
                            .andThen(shoot()), 
                        trenchToCenter.cmd(),
                        bumpToShoot.cmd()
                            .andThen(shoot()),
                        climb.cmd()
                    )
        );

        return testRoutine;
    }

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
                        trenchToCenter.cmd() 
                            .beforeStarting(trenchToCenter.resetOdometry()),
                        trenchToShoot.cmd()
                            .andThen(shoot()), 
                        trenchToCenterSecondSwipe.cmd(),
                        trenchToClimb.cmd()
                            .andThen(shoot())
                    )
        );

        return testRoutine;
    }
}
