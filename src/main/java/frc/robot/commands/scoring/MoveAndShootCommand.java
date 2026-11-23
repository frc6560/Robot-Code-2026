package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.shooting.FuelShotModel;

import java.net.http.HttpRequest.BodyPublisher;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DrivebaseConstants;


public final class MoveAndShootCommand extends Command{

    private final SwerveSubsystem drive;
    private final Shooter shooter;
    private final Indexer indexer;
    private final BooleanSupplier pitCoast;
    private final Supplier<ChassisSpeeds> driverSpeeds;

    
    private Translation2d retreatDirection;
    private FollowPath retreat;
    private boolean retreatActive;
    private DriverStation.Alliance alliance;
    private double previousTime, feedSeconds;
    private double readySince;
    private final ProfiledPIDController  headingController = new ProfiledPIDController(
        DrivebaseConstants.HUB_AIM_KP,
        DrivebaseConstants.HUB_AIM_KI,
        DrivebaseConstants.HUB_AIM_KD,
        new TrapezoidProfile.Constraints(
            DrivebaseConstants.HUB_AIM_MAX_ANGULAR_SPEED_RAD_PER_SEC,
            DrivebaseConstants.HUB_AIM_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ));


    public MoveAndShootCommand(
            SwerveSubsystem drive,
            Shooter shooter,
            Supplier<ChassisSpeeds> driverSpeeds,
            Indexer indexer,
            BooleanSupplier pitCoast) {

        this.drive = drive;
        this.shooter = shooter;
        this.driverSpeeds = driverSpeeds;
        this.indexer=indexer;
        this.pitCoast=pitCoast;
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive, shooter, indexer);
    }




    @Override
    public void initialize() {
        indexer.stop();
        readySince = Double.NaN;
        headingController.reset(
            drive.getHeading().getRadians(),
            drive.getRobotVelocity().omegaRadiansPerSecond);
    }

    @Override public void execute() {
        
        super.execute();
    }








}
