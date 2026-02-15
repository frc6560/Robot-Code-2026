package frc.robot.commands.periodic;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.Shooter;
import frc.robot.subsystems.superstructure.Turret;
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

    private SuperstructureState state = SuperstructureState.IDLE;

    public SuperstructureCommand(
            Hood hood,
            Shooter shooter,
            Turret turret,
            PoseSupplier poseSupplier,
            VelocitySupplier velocitySupplier,
            ShotCalculator shotCalculator) {
        this.hood = hood;
        this.shooter = shooter;
        this.turret = turret;
        this.poseSupplier = poseSupplier;
        this.velocitySupplier = velocitySupplier;
        this.shotCalculator = shotCalculator;
        addRequirements(hood, shooter, turret);
    }

    @Override
    public void initialize() {
        shooter.setIdle();
    }

    @Override
    public void execute() {
        handleState();
        updateBehavior();
    }

    private void handleState() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            state = SuperstructureState.IDLE;
            return;
        }

        double robotX = poseSupplier.getPose().getX();

        if (robotX > FieldConstants.BLUE_ZONE_X && robotX < FieldConstants.RED_ZONE_X) {
            state = SuperstructureState.PASS;
        } else if ((alliance.get() == Alliance.Blue && robotX < FieldConstants.BLUE_ZONE_X)
                    || (alliance.get() == Alliance.Red && robotX > FieldConstants.RED_ZONE_X)) {
            state = SuperstructureState.SHOOT;
        } else {
            state = SuperstructureState.IDLE;
        }
    }

    private void updateBehavior() {
        switch (state) {
            case IDLE:
                idleState();
                break;
            case PASS:
                idleState();
                break;
            case SHOOT:
                trackHubTarget();
                break;
        }
    }

    private void idleState() {
        shooter.setIdle();
        turret.stopMotor();
        hood.stop();
    }

    private void trackHubTarget() {
        Pose2d pose = poseSupplier.getPose();
        ChassisSpeeds velocity = velocitySupplier.getFieldVelocity();

        shotCalculator.calculate(pose, velocity);

        shooter.setRPM(shotCalculator.getFlywheelRPM());
        turret.setGoal(Units.radiansToDegrees(shotCalculator.getTurretAngle()));
        hood.setGoal(Units.radiansToDegrees(shotCalculator.getHoodAzimuth()));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        turret.stopMotor();
        hood.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
