// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.shooter.FlywheelVisualizer;
import frc.robot.utility.LimelightHelpers;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /** Simulation-only spinning-wheel view of the physics-calculated flywheel schedule. */
  private FlywheelVisualizer flywheelVisualizer;

  private Timer disabledTimer;

  private final boolean runHeadlessAutoDiagnostic =
      isSimulation() && "1".equals(System.getenv("BLINE_HEADLESS_DIAGNOSTIC"));
  private double headlessDiagnosticStartSeconds;
  private double headlessAutoFinishedSeconds = -1.0;
  private Pose2d headlessAutoFinishedPose;
  private double headlessMaxPostAutoDisplacementMeters;
  private double headlessMaxPostAutoSpeedMetersPerSecond;
  private double headlessFinalPostAutoSpeedMetersPerSecond;

  public Robot()
  {
    Logger.recordMetadata("Robot", "2026 Alpha");
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    } else if (Constants.currentMode == Constants.Mode.REPLAY) {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); 
      Logger.setReplaySource(new WPILOGReader(logPath)); 
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); 
    } else {
      // Publish a live desktop simulation so AdvantageScope can connect over NT4.
      // Keep normal 20 ms timing enabled so Driver Station mode changes and path motion
      // can be watched in real time.
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      disabledTimer.stop();
      disabledTimer.reset();
    }
    for(String limelightName : LimelightConstants.LIMELIGHT_NAMES){
      LimelightHelpers.SetIMUMode(limelightName, 1);
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.disablePitCoastMode();

    for(String limelightName : LimelightConstants.LIMELIGHT_NAMES){
      LimelightHelpers.SetIMUMode(limelightName, 4);
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }

    for(String limelightName : LimelightConstants.LIMELIGHT_NAMES){
      LimelightHelpers.SetIMUMode(limelightName, 4);
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
    flywheelVisualizer = new FlywheelVisualizer();

    if (runHeadlessAutoDiagnostic) {
      headlessDiagnosticStartSeconds = Timer.getFPGATimestamp();
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAutonomous(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      System.out.println("BLINE_HEADLESS: Enabled Zigzag autonomous on Blue alliance");
    }
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
    if (flywheelVisualizer != null) {
      flywheelVisualizer.update(getPeriod());
    }

    if (!runHeadlessAutoDiagnostic) {
      return;
    }

    double now = Timer.getFPGATimestamp();
    if (m_autonomousCommand != null
        && !CommandScheduler.getInstance().isScheduled(m_autonomousCommand)) {
      if (headlessAutoFinishedPose == null) {
        headlessAutoFinishedPose = m_robotContainer.getDrivebase().getPose();
        headlessAutoFinishedSeconds = now;
        System.out.printf(
            "BLINE_HEADLESS: Auto command finished at x=%.3f y=%.3f heading=%.1fdeg%n",
            headlessAutoFinishedPose.getX(),
            headlessAutoFinishedPose.getY(),
            headlessAutoFinishedPose.getRotation().getDegrees());
      }

      Pose2d currentPose = m_robotContainer.getDrivebase().getPose();
      double displacement =
          currentPose.getTranslation().getDistance(headlessAutoFinishedPose.getTranslation());
      double speed = Math.hypot(
          m_robotContainer.getDrivebase().getRobotVelocity().vxMetersPerSecond,
          m_robotContainer.getDrivebase().getRobotVelocity().vyMetersPerSecond);
      headlessMaxPostAutoDisplacementMeters =
          Math.max(headlessMaxPostAutoDisplacementMeters, displacement);
      headlessMaxPostAutoSpeedMetersPerSecond =
          Math.max(headlessMaxPostAutoSpeedMetersPerSecond, speed);
      headlessFinalPostAutoSpeedMetersPerSecond = speed;

      if (now - headlessAutoFinishedSeconds >= 3.0) {
        boolean stayedStopped =
            headlessMaxPostAutoDisplacementMeters < 0.05
                && headlessFinalPostAutoSpeedMetersPerSecond < 0.05;
        System.out.printf(
            "BLINE_HEADLESS_RESULT: %s postAutoDisplacement=%.4fm "
                + "initialCoastPeak=%.4fmps finalSpeed=%.4fmps%n",
            stayedStopped ? "PASS" : "FAIL",
            headlessMaxPostAutoDisplacementMeters,
            headlessMaxPostAutoSpeedMetersPerSecond,
            headlessFinalPostAutoSpeedMetersPerSecond);
        stopHeadlessDiagnostic();
      }
    } else if (now - headlessDiagnosticStartSeconds >= 30.0) {
      System.out.printf(
          "BLINE_HEADLESS_RESULT: TIMEOUT pose=(%.3f, %.3f, %.1fdeg)%n",
          m_robotContainer.getDrivebase().getPose().getX(),
          m_robotContainer.getDrivebase().getPose().getY(),
          m_robotContainer.getDrivebase().getPose().getRotation().getDegrees());
      stopHeadlessDiagnostic();
    }
  }

  private void stopHeadlessDiagnostic() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    endCompetition();
  }
}
