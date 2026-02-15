package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine; // Import for SysId
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.AutoModeChooser;
import frc.robot.autonomous.AutoCommands;
import frc.robot.commands.periodic.HoodCommand;
import frc.robot.commands.periodic.TurretCommand;

import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.Shooter;
import frc.robot.subsystems.superstructure.Turret;
import frc.robot.subsystems.superstructure.Feeder;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;

// ** NEW IMPORTS **
import frc.robot.utility.Shooter.ShotCalculator;
import frc.robot.ManualControls; 

public class RobotContainer {

    // --- CONTROLLERS ---
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);

    // ** NEW: Manual Controls Wrapper **
    // (Used by HoodCommand to read joystick buttons)
    private final ManualControls controls = new ManualControls(driverXbox.getHID(), operatorXbox.getHID());

    // --- SUBSYSTEMS ---
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
    
    // ** NEW: Shot Calculator **
    // (Passes drivebase so it can get Velocity for shooting on the move)
    private final ShotCalculator shotCalc = new ShotCalculator();

    private final VisionSubsystem vision;
    private final Hood hood = new Hood(drivebase::getPose);
    private final Shooter shooter = new Shooter(drivebase::getPose);
    private final Turret turret = new Turret();
    private final Feeder feeder = new Feeder();


    private final AutoCommands factory;
    private final AutoModeChooser autoChooser;

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);


    public RobotContainer() {
      factory = new AutoCommands(drivebase);
      autoChooser = new AutoModeChooser(factory);
      SmartDashboard.putData("Auto Chooser", autoChooser.getAutoChooser());

      List<LimelightVision> limelights = new ArrayList<LimelightVision>();
      for(String name : LimelightConstants.LIMELIGHT_NAMES) {
        Pose3d cameraPose = LimelightConstants.getLimelightPose(name);
        limelights.add(new LimelightVision(drivebase, name, cameraPose));
      }
      vision = new VisionSubsystem(limelights);

      // --- DEFAULT COMMANDS ---
      // This enables the Manual Jog and Auto-Aim logic we wrote in HoodCommand
      //hood.setDefaultCommand(new HoodCommand(hood, controls, shotCalc));
      
      configureBindings();
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        // --- DRIVER CONTROLS ---

        // 1. SysId Tests (Hood Characterization)
        // WARNING: Be ready to release these buttons immediately!
        
        // Quasistatic (Slow Ramp) -> Finds kS and kG
        driverXbox.a().whileTrue(hood.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driverXbox.b().whileTrue(hood.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // Dynamic (Fast Step) -> Finds kV and kA
        // Only run if Soft Limits are working!
        driverXbox.x().whileTrue(hood.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driverXbox.y().whileTrue(hood.sysIdDynamic(SysIdRoutine.Direction.kReverse));


        // 2. Drive Utilities
        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));
        
        // Turret Test (Driver Bumpers)
        driverXbox.leftBumper()
          .onTrue(Commands.runOnce(() -> turret.setGoal(92)))
          .onFalse(Commands.runOnce(() -> turret.setGoal(0)));
        driverXbox.rightBumper()
          .onTrue(Commands.runOnce(() -> turret.setGoal(-92)))
          .onFalse(Commands.runOnce(() -> turret.setGoal(0)));


        // --- OPERATOR CONTROLS ---

        // 1. Hood Test Buttons (Overrides Default Command)
        operatorXbox.b().onTrue(Commands.runOnce(() -> hood.setGoal(20.0), hood));
        operatorXbox.x().onTrue(Commands.runOnce(() -> hood.setGoal(0.0), hood));

        // 2. Feeder
        operatorXbox.y()
          .onTrue(Commands.runOnce(feeder::requestFeed, feeder))
          .onFalse(Commands.runOnce(feeder::requestStop, feeder));
        operatorXbox.a()
          .onTrue(Commands.runOnce(feeder::requestStop, feeder));

        // // 3. Shooter Test
        // operatorXbox.leftBumper()
        //   .onTrue(Commands.run(() -> shooter.setGoal(2000.0), shooter)) // Fixed method name (setRPM -> setGoal)
        //   .onFalse(Commands.run(() -> shooter.stop(), shooter));
        
        // 4. Manual Hood Jog / Auto Aim
        // These are handled by 'hood.setDefaultCommand(new HoodCommand...)'
        // But if you want a specific override:
        operatorXbox.rightBumper()
          .onTrue(Commands.runOnce(() -> hood.setGoal(25.0), hood))
          .onFalse(Commands.runOnce(() -> hood.setGoal(0.0), hood));
    }

    public Command getAutonomousCommand() {
      return autoChooser.getAutoChooser().selectedCommand();
    }
}