package frc.robot.commands.scoring;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.led.LED;
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
    private final LED led;

    private boolean isReady = false;

    Debouncer debouncer = new Debouncer(0.05);

    public ShotCommand(
            Feeder feeder,
            Turret turret,
            Hood hood,
            Shooter shooter,
            ShotCalculator shotCalculator,
            PoseSupplier supplier,
            LED led) {
        this.feeder = feeder;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.shotCalculator = shotCalculator;
        this.supplier = supplier;
        this.led = led;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        led.setShootIntent(true);
    }

    private static final double PASSING_TURRET_TOLERANCE_DEG = 15.0;

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()){
            return;
        }

        double poseX = supplier.getPose().getX();
        boolean inPassingZone = (alliance.get().equals(DriverStation.Alliance.Blue))
            ? poseX > FieldConstants.BLUE_ZONE_X : poseX < FieldConstants.RED_ZONE_X;

        boolean inOpponentZone = (alliance.get().equals(DriverStation.Alliance.Blue))
            ? poseX > FieldConstants.RED_ZONE_X : poseX < FieldConstants.BLUE_ZONE_X;

        double turretTolerance = inPassingZone ? PASSING_TURRET_TOLERANCE_DEG : shotCalculator.getTurretTolerance();
        boolean hoodAtTolerance = inPassingZone ? true : hood.atTarget();
        boolean shooterAtTolerance = inPassingZone ? true : shooter.atTarget();

        boolean raw = shotCalculator.isShotValid() && turret.getAtTarget(turretTolerance) && hoodAtTolerance && shooterAtTolerance;

        boolean allAtTarget = debouncer.calculate(raw);

        Translation2d hubCenter = (alliance.get() == Alliance.Blue)
                ? FieldConstants.BLUE_HUB_CENTER
                : FieldConstants.RED_HUB_CENTER;
        double distanceToHub = supplier.getPose().getTranslation().getDistance(hubCenter);

        boolean notAtDeadzone = !(
            (inOpponentZone
            && supplier.getPose().getY() > FieldConstants.PASS_DEADZONE_MIN_Y && supplier.getPose().getY() < FieldConstants.PASS_DEADZONE_MAX_Y)
            || inPassingZone && distanceToHub < 1.3
        );

        isReady = allAtTarget && notAtDeadzone;
        feeder.setShooting(isReady);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setShooting(false);
        led.setShootIntent(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
