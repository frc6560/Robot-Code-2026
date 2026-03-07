package frc.robot.commands.scoring;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utility.Shooter.ShotCalculator;

public class ShotCommand extends Command {

    public interface PoseSupplier {
        Pose2d getPose();
    }

    private final Feeder feeder;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final ShotCalculator shotCalculator;
    private final PoseSupplier supplier;

    Debouncer debouncer = new Debouncer(0.25);

    public ShotCommand(
            Feeder feeder,
            Turret turret,
            Hood hood,
            Shooter shooter,
            ShotCalculator shotCalculator,
            PoseSupplier supplier) {
        this.feeder = feeder;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.shotCalculator = shotCalculator;
        this.supplier = supplier;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        boolean allAtTarget = debouncer.calculate(shotCalculator.isShotValid())
                && debouncer.calculate(turret.getAtTarget())
                && debouncer.calculate(hood.atTarget())
                && debouncer.calculate(shooter.atTarget());

        boolean notAtDeadzone = !(
            supplier.getPose().getX() > FieldConstants.BLUE_ZONE_X && supplier.getPose().getX() < FieldConstants.RED_ZONE_X
            && supplier.getPose().getY() > FieldConstants.PASS_DEADZONE_MIN_Y && supplier.getPose().getY() < FieldConstants.PASS_DEADZONE_MAX_Y
        );
        if (allAtTarget && notAtDeadzone) {
            feeder.requestFeed();
        } else {
            feeder.requestStop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.requestStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
