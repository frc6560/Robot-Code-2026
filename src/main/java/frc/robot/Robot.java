package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private static Robot instance;
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private Timer disabledTimer;

    public Robot() {
        instance = this;
    }

    public static Robot getInstance() {
        return instance;
    }

    @Override
    public void robotInit() {
        // Instantiate RobotContainer (subsystems, commands, bindings)
        m_robotContainer = new RobotContainer();

        // Timer for disabled motor handling
        disabledTimer = new Timer();

        if (isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
    }

    @Override
    public void robotPeriodic() {
        // Run the CommandScheduler
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        disabledTimer.reset();
        disabledTimer.start();
    }

    @Override
    public void disabledPeriodic() {
        if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
            disabledTimer.stop();
            disabledTimer.reset();
        }
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        } else {
            CommandScheduler.getInstance().cancelAll();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
