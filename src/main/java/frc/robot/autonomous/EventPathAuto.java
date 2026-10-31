package frc.robot.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.scoring.FuelShootCommand;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;

/** One scheduler owner for B-Line events and the shot held through the end of autonomous. */
public final class EventPathAuto extends Command {
    private final FollowPath path;
    private final FuelShootCommand rev, shoot;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Intake intake;
    private final BooleanSupplier pitCoast;
    private final Runnable restoreEvents;
    private boolean pathActive, revActive, revRequested, shootRequested, shooting;

    EventPathAuto(FollowPath path, FuelShootCommand rev, FuelShootCommand shoot,
            Shooter shooter, Indexer indexer, Intake intake, BooleanSupplier pitCoast,
            Runnable restoreEvents) {
        this.path = path; this.rev = rev; this.shoot = shoot;
        this.shooter = shooter; this.indexer = indexer; this.intake = intake;
        this.pitCoast = pitCoast; this.restoreEvents = restoreEvents;
        addRequirements(path.getRequirements());
        addRequirements(shoot.getRequirements());
        addRequirements(rev.getRequirements());
        addRequirements(intake);
    }

    @Override public void initialize() {
        pathActive = true;
        revActive = revRequested = shootRequested = shooting = false;
        indexer.stop();
        shooter.setIdle();
        intake.stop();
        // Inline requests avoid scheduling a second command that would cancel this auto.
        FollowPath.registerEventTrigger("intake", (Runnable) () -> {
            intake.requestRackExtended();
            intake.setRollersRequested(true);
        });
        FollowPath.registerEventTrigger("retract", (Runnable) intake::requestRackRetracted);
        FollowPath.registerEventTrigger("rev", (Runnable) () -> revRequested = true);
        FollowPath.registerEventTrigger("shoot", (Runnable) () -> shootRequested = true);
        path.initialize();
    }

    public boolean isShooting() { return shooting; }
    public boolean isFeeding() { return shooting && shoot.isFeeding(); }

    @Override public void execute() {
        if (!DriverStation.isAutonomousEnabled() || pitCoast.getAsBoolean()) {
            stopMechanisms();
            return;
        }
        if (!shooting) {
            path.execute();
            // A marker at t=1 can be just beyond B-Line's endpoint tolerance. Reaching the
            // final waypoint also starts the final shot for these three shooting paths.
            if (shootRequested || path.isFinished()) {
                path.end(shootRequested);
                pathActive = false;
                if (revActive) { rev.end(true); revActive = false; }
                shoot.initialize();
                shooting = true;
            } else if (revRequested) {
                if (!revActive) { rev.initialize(); revActive = true; }
                rev.execute();
            } else {
                shooter.setIdle();
                indexer.stop();
            }
        }
        if (shooting) shoot.execute();
    }

    private void stopMechanisms() {
        if (pathActive) { path.end(true); pathActive = false; }
        if (revActive) { rev.end(true); revActive = false; }
        if (shooting) { shoot.end(true); shooting = false; }
        indexer.stop();
        shooter.stop();
        intake.stop();
    }

    @Override public boolean isFinished() {
        return !DriverStation.isAutonomousEnabled() || pitCoast.getAsBoolean();
    }

    @Override public void end(boolean interrupted) {
        stopMechanisms();
        restoreEvents.run();
        shooter.setIdle();
    }
}
