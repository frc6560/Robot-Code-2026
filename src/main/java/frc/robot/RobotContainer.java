package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine; // IMPORTED SYSID

import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.subsystems.Flywheel;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FlywheelCommand;
import frc.robot.commands.HoodCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final XboxController firstXbox = new XboxController(0);
    private final XboxController secondXbox = new XboxController(1);
    private final ManualControls controls = new ManualControls(firstXbox, secondXbox);
    
    // Subsystems
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
    
    private final VisionSubsystem vision;
    private final ShotCalculator shotCalc = new ShotCalculator(() -> drivebase.getPose());

    private final Flywheel flywheel = new Flywheel(() -> drivebase.getPose()); 
    private final Hood hood = new Hood(() -> drivebase.getPose());

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

    public RobotContainer() {
        flywheel.setDefaultCommand(new FlywheelCommand(flywheel, shotCalc, controls));
        hood.setDefaultCommand(new HoodCommand(hood, controls, shotCalc));

        List<LimelightVision> limelights = new ArrayList<LimelightVision>();
        for(String name : LimelightConstants.LIMELIGHT_NAMES) {
            Pose3d cameraPose = LimelightConstants.getLimelightPose(name);
            limelights.add(new LimelightVision(drivebase, name, cameraPose));
        }

        vision = new VisionSubsystem(limelights);
        configureBindings();
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        // --- DRIVER CONTROLS (Standard) ---
        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));
        driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

        // --- COMMENTED OUT FOR SYSID TESTING ---
        
        driverXbox.a().onTrue(
          Commands.defer(() -> {
            return Commands.runOnce(() -> vision.hardReset("limelight"), vision);
          }, Set.of(vision))
        );
        driverXbox.y().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
        driverXbox.x().onTrue(Commands.defer(() -> drivebase.alignToTrenchCommand(), Set.of(drivebase)));
        driverXbox.b().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(drivebase.sysIdDriveMotorCommand()), drivebase));
        

        // // --- FLYWHEEL SYSID CONTROLS (ACTIVE) ---
        // // A Button: Slow Ramp Up (Quasistatic Forward)
        // driverXbox.a().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        
        // // B Button: Slow Ramp Down (Quasistatic Reverse)
        // driverXbox.b().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        
        // // X Button: Fast Step Up (Dynamic Forward)
        // driverXbox.x().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
        
        // // Y Button: Fast Step Down (Dynamic Reverse)
        // driverXbox.y().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
}