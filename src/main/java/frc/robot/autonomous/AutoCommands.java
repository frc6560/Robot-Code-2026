package frc.robot.autonomous;


import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.superstructure.Feeder;
// import frc.robot.subsystems.superstructure.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


// TODOs: delete all commented out files in robot.java, and this file
// refactor code: add follow and shoot helper
// add a helper method to sequence commands: sequencePaths(Commands... commands)
// helpers for markers
public class AutoCommands {
    private SwerveSubsystem drivetrain;
    // private Shooter shooter;
    // private Feeder feeder;

    private AutoFactory autoFactory;

    public AutoCommands(SwerveSubsystem drivetrain
        // , Shooter shooter, Feeder feeder
    ) {
        this.drivetrain = drivetrain;
        // this.shooter = shooter;
        // this.feeder = feeder;

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

    public Command spinUpShooter(){
    //     return Commands.runOnce(() -> shooter.setRPM(ShooterConstants.FLYWHEEL_RPM), shooter);
        return Commands.idle();
    }

    public Command shoot(){
        // return Commands.run(() -> feeder.setRPM(FeederConstants.FEEDER_RPM), feeder)
        //     .withTimeout(3.0)
        //     .finallyDo((interrupted) -> {
        //         shooter.setRPM(0);
        //         feeder.setRPM(0);
        //     });
        return Commands.idle();
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
                Commands.idle()
            );
        
        trenchToShoot.atTime("shoot")
            .onTrue(
                spinUpShooter()
            );
        
        bumpToShoot.atTime("shoot")
            .onTrue(
                spinUpShooter()
            );

        testRoutine
            .active()
                .onTrue(
                    Commands.sequence(
                        trenchToCenter.resetOdometry(),  // Only reset once at the start!
                        trenchToCenter.cmd(), // add an intake command after (or during) this.
                        trenchToShoot.cmd()
                            .andThen(shoot()), // to simulate shooting
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

        trenchToCenter.atTime("intake")
            .onTrue(
                Commands.idle()
            );
        
        trenchToShoot.atTime("shoot")
            .onTrue(
                spinUpShooter()
            );
        
        trenchToClimb.atTime("shoot")
            .onTrue(
                spinUpShooter()
            );

        testRoutine
            .active()
                .onTrue(
                    Commands.sequence(
                        trenchToCenter.resetOdometry(), 
                        trenchToCenter.cmd() // add an intake command after (or during) this.
                            .beforeStarting(trenchToCenter.resetOdometry()),
                        trenchToShoot.cmd()
                            .beforeStarting(trenchToShoot.resetOdometry())
                            .andThen(shoot()), // to simulate shooting
                        trenchToCenter.cmd()
                            .beforeStarting(trenchToCenter.resetOdometry()),
                        trenchToClimb.cmd()
                            .beforeStarting(trenchToClimb.resetOdometry())
                            .andThen(shoot())
                    )
        );

        return testRoutine;
    }
}
