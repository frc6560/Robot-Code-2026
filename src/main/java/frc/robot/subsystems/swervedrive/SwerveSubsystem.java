// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.LimelightHelpers.PoseEstimate;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;

import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;
  private final StringSubscriber autoChooserSubscriber;
  private boolean initialPoseSet = false;
  private boolean pitCoastMode = false;

  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(DrivebaseConstants.kS, 
                                                                            DrivebaseConstants.kV, 
                                                                            DrivebaseConstants.kA);


  PIDController m_pidControllerTheta = new PIDController(DrivebaseConstants.ALIGN_ROTATION_KP,
                                                          DrivebaseConstants.ALIGN_ROTATION_KI,
                                                          DrivebaseConstants.ALIGN_ROTATION_KD);

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    // Subscribe to the auto chooser's selected value from NetworkTables
    autoChooserSubscriber = NetworkTableInstance.getDefault()
        .getStringTopic("/SmartDashboard/Auto Chooser/selected")
        .subscribe("Idle");

    Pose2d startingPose = FieldConstants.BLUE_TESTING_START;
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); 
    swerveDrive.setCosineCompensator(true);
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1);
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); 
    swerveDrive.replaceSwerveModuleFeedforward(driveFF);
    setMotorBrake(true);
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    // Subscribe to the auto chooser's selected value from NetworkTables
    autoChooserSubscriber = NetworkTableInstance.getDefault()
        .getStringTopic("/SmartDashboard/Auto Chooser/selected")
        .subscribe("Idle");

    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  Constants.MAX_SPEED,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                  Rotation2d.fromDegrees(0)));
    setMotorBrake(true);
  }

  @Override
  public void periodic() {
    // Pose2d is published as structured AdvantageKit data. In AdvantageScope, add
    // Swerve/Pose to a 2D Field tab and BLine/FollowPath/pathTranslations as the path.
    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput("Swerve/RobotVelocity", getRobotVelocity());
    Logger.recordOutput("Swerve/FieldVelocity", getFieldVelocity());
    Logger.recordOutput("Swerve/PitCoastMode", pitCoastMode);

    // Set initial pose once based on auto selection (only while disabled)
    if (!initialPoseSet && DriverStation.isDisabled()) {
      System.out.println("Resetting pose!");
      updateInitialPose();
    }

    // Reset flag when entering autonomous so pose can be set again next match
    if (DriverStation.isAutonomousEnabled()) {
      initialPoseSet = true;  // Lock in the pose once auto starts
    }

    // Update distance to hub on SmartDashboard for debugging
    Transform2d turretTransform = new Transform2d(
            TurretConstants.ROBOT_RELATIVE_TURRET.getX(), 
            TurretConstants.ROBOT_RELATIVE_TURRET.getY(),
            new Rotation2d()
        );
    Pose2d robotRelativeTurret = getPose().transformBy( turretTransform );
    swerveDrive.field.getObject("TurretPose").setPose(robotRelativeTurret);
    swerveDrive.field.getObject("BlueHub").setPose(new Pose2d(FieldConstants.BLUE_HUB_CENTER, new Rotation2d()));
    SmartDashboard.getEntry("DistToBlueHub").setDouble(
      robotRelativeTurret.getTranslation().getDistance(FieldConstants.BLUE_HUB_CENTER));
  }

  /** Rotates to a specified angle while inheriting the chassis's original translational velocity */
  public void rotateToAngle(double targetInRadians){
    if (pitCoastMode) {
      stopAllMotors();
      return;
    }
    m_pidControllerTheta.enableContinuousInput(-Math.PI, Math.PI);

    double currentHeading = getPose().getRotation().getRadians();
    double headingError = MathUtil.angleModulus(targetInRadians - currentHeading);
    double commandedOmega = m_pidControllerTheta.calculate(currentHeading, targetInRadians);

    SmartDashboard.getEntry("Yaw error").setDouble(headingError);
    SmartDashboard.getEntry("Pose in radians").setDouble(currentHeading);
    Logger.recordOutput("HeadingAlign/TargetHeadingDeg", Math.toDegrees(targetInRadians));
    Logger.recordOutput("HeadingAlign/CurrentHeadingDeg", Math.toDegrees(currentHeading));
    Logger.recordOutput("HeadingAlign/HeadingErrorDeg", Math.toDegrees(headingError));
    Logger.recordOutput("HeadingAlign/CommandedOmegaRadPerSec", commandedOmega);
    Logger.recordOutput(
        "HeadingAlign/MeasuredOmegaRadPerSec", getRobotVelocity().omegaRadiansPerSecond);

    ChassisSpeeds targetSpeeds = new ChassisSpeeds(
      getFieldVelocity().vxMetersPerSecond,
      getFieldVelocity().vyMetersPerSecond,
      commandedOmega
    );

    swerveDrive.driveFieldOriented(targetSpeeds);
  }

    /** Aligns the robot to face the trench while driving */
    public Command alignToTrenchCommand(){
      m_pidControllerTheta.enableContinuousInput(-Math.PI, Math.PI);

      final double ROT_TOLERANCE = Units.degreesToRadians(2.0);
      double target = MathUtil.angleModulus(getPose().getRotation().getRadians() - (MathUtil.inputModulus(getPose().getRotation().getRadians(), - Math.PI/2 , Math.PI/2)));
      SmartDashboard.getEntry("Target pose").setDouble(target);

      Command alignToTrenchCommand = new FunctionalCommand(
        () -> {},
        () -> {
          rotateToAngle(target);
        }, 
        (interrupted) -> {
        },
        () -> Math.abs(MathUtil.angleModulus(getPose().getRotation().getRadians() - target)) < ROT_TOLERANCE
      );
      return alignToTrenchCommand;
    }

    

  @Override
  public void simulationPeriodic(){
  }

  /**
   * Updates the initial pose based on the auto chooser selection and alliance.
   * Should only be called once before auto starts.
   */
  private void updateInitialPose() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return;  // Don't set pose yet - alliance unknown
    }

    String selectedAuto = autoChooserSubscriber.get();
    boolean isRed = alliance.get() == Alliance.Red;

    Pose2d startPose;
    switch (selectedAuto) {
      case "HP Turkish Delight":
        startPose = isRed ? FieldConstants.RED_RIGHT_START : FieldConstants.BLUE_RIGHT_START;
        break;
      case "Depot Turkish Delight":
        startPose = isRed ? FieldConstants.RED_LEFT_START : FieldConstants.BLUE_LEFT_START;
        break;
      case "Idle":
      default:
        startPose = isRed ? FieldConstants.RED_TESTING_START : FieldConstants.BLUE_TESTING_START;
        break;
    }

    resetOdometry(startPose);
    initialPoseSet = true;
  }

  /**
   * Resets the initial pose flag, allowing the pose to be set again.
   * Call this when preparing for a new match.
   */
  public void resetInitialPoseFlag() {
    initialPoseSet = false;
  }

  

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    System.out.println("Running SysID Command!");
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  // a hack method to reset the MT2 gyro
  public void resetOdometryToLimelight() {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    Pose2d pose = poseEstimate.pose;
    if(pose != null){
      resetOdometry(pose);
      System.out.println("Resetting odometry to Limelight pose: " + pose);
    } else {
      System.out.println("Limelight pose is null, cannot reset odometry.");
    }
  }
  
  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                     distanceInMeters);
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(SimpleMotorFeedforward feedforward)
  {
    swerveDrive.replaceSwerveModuleFeedforward(feedforward);
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      drive(SwerveMath.scaleTranslation(new Translation2d(
                translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
            Math.pow(angularRotationX.getAsDouble(), 3)
                * -swerveDrive.getMaximumChassisAngularVelocity(),
            true);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    if (pitCoastMode) {
      stopAllMotors();
      return;
    }
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    if (pitCoastMode) {
      stopAllMotors();
      return;
    }
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> driveFieldOriented(velocity.get()));
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    if (pitCoastMode) {
      stopAllMotors();
      return;
    }
    swerveDrive.drive(velocity);
  }


  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
    System.out.println("Resetting odometry to: " + initialHolonomicPose);
  }


  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    if (pitCoastMode) {
      stopAllMotors();
      return;
    }
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  public void zeroNoAprilTagsGyro() {
    // 1. Zero the gyro sensor itself
    zeroGyro();
    Pose2d currentPose = getPose();
    resetOdometry(new Pose2d(
        currentPose.getTranslation(),
        Rotation2d.fromDegrees(0)
    ));
  }


  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Stops all drive and steering outputs and toggles every swerve motor between coast and its
   * normal brake mode. Unlike disabling the robot, this mode can be toggled while teleop remains
   * enabled so the modules and wheels can be moved by hand.
   */
  public void setPitCoastMode(boolean enabled)
  {
    pitCoastMode = enabled;
    stopAllMotors();
    Arrays.asList(swerveDrive.getModules()).forEach(module -> {
      module.getDriveMotor().setMotorBrake(!enabled);
      module.getAngleMotor().setMotorBrake(!enabled);
    });
  }

  private void stopAllMotors()
  {
    Arrays.asList(swerveDrive.getModules()).forEach(module -> {
      module.getDriveMotor().set(0.0);
      module.getAngleMotor().set(0.0);
    });
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }
}
