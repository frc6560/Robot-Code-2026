package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.StateSpaceSwerve;

/** Driver bindings for a field-relative, state-space regulated swerve drivetrain. */
public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);
  private final StateSpaceSwerve swerve = new StateSpaceSwerve();

  public RobotContainer() {
    swerve.setDefaultCommand(Commands.run(() -> swerve.drive(
        new ChassisSpeeds(
            -MathUtil.applyDeadband(driver.getLeftY(), 0.08) * Constants.Drive.MAX_SPEED_METERS_PER_SECOND,
            -MathUtil.applyDeadband(driver.getLeftX(), 0.08) * Constants.Drive.MAX_SPEED_METERS_PER_SECOND,
            -MathUtil.applyDeadband(driver.getRightX(), 0.08) * Constants.Drive.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND),
        true), swerve));
    driver.start().onTrue(Commands.runOnce(swerve::zeroGyro, swerve));
    driver.x().whileTrue(Commands.run(swerve::lockX, swerve));
  }
}
