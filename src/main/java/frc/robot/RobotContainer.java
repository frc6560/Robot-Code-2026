package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine; 

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

    // --- CONTROLLERS ---
    // FIXED: Only define the controllers once!
    // Driver on Port 0, Operator on Port 1
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);

    // Pass the underlying HID (Hardware Interface) to your manual controls
    private final ManualControls controls = new ManualControls(driverXbox.getHID(), operatorXbox.getHID());
    
    // --- SUBSYSTEMS ---
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
    
    private final VisionSubsystem vision;
    
    // Pass the Pose Supplier directly (Method Reference)
    private final ShotCalculator shotCalc = new ShotCalculator(drivebase::getPose);
    private final Flywheel flywheel = new Flywheel(drivebase::getPose); 
    private final Hood hood = new Hood(drivebase::getPose);

    // Drive Input Stream
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

    public RobotContainer() {
        // Initialize Vision
        List<LimelightVision> limelights = new ArrayList<>();
        for(String name : LimelightConstants.LIMELIGHT_NAMES) {
            Pose3d cameraPose = LimelightConstants.getLimelightPose(name);
            limelights.add(new LimelightVision(drivebase, name, cameraPose));
        }
        vision = new VisionSubsystem(limelights);

        // Set Default Commands
        // These commands run logic 100% of the time (checking for button presses inside)
        flywheel.setDefaultCommand(new FlywheelCommand(flywheel, shotCalc, controls));
        hood.setDefaultCommand(new HoodCommand(hood, controls, shotCalc));
        
        // Configure Buttons
        configureBindings();
    }

    private void configureBindings() {
        // Default Drive Command
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        // --- DRIVER CONTROLS ---
        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));
        driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

        // --- TEST BINDINGS ---
        driverXbox.a().onTrue(
          Commands.defer(() -> {
            return Commands.runOnce(() -> vision.hardReset("limelight"), vision);
          }, Set.of(vision))
        );

        driverXbox.y().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
        
        driverXbox.x().onTrue(Commands.defer(() -> drivebase.alignToTrenchCommand(), Set.of(drivebase)));
        
        // SYSID (Keep commented out for matches)
        // driverXbox.b().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(drivebase.sysIdDriveMotorCommand()), drivebase));
    }
}