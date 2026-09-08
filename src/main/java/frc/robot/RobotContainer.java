package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.StateSpaceSwerve;

/** Driver bindings for a field-relative, state-space regulated swerve drivetrain. */
public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);
  private final StateSpaceSwerve swerve = new StateSpaceSwerve();
  private final SlewRateLimiter omegaLimiter =
      new SlewRateLimiter(Constants.Drive.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED);

  public RobotContainer() {
    swerve.setDefaultCommand(Commands.run(this::driveWithTelemetry, swerve));
    driver.start().onTrue(Commands.runOnce(swerve::zeroGyro, swerve));
    driver.x().whileTrue(Commands.run(swerve::lockX, swerve));
  }

  private void driveWithTelemetry() {
    double leftX = driver.getLeftX();
    double leftY = driver.getLeftY();
    double rightX = driver.getRightX();
    double translationX = MathUtil.applyDeadband(
        leftX, Constants.Drive.CONTROLLER_TRANSLATION_DEADBAND);
    double translationY = MathUtil.applyDeadband(
        leftY, Constants.Drive.CONTROLLER_TRANSLATION_DEADBAND);
    double requestedOmega = -MathUtil.applyDeadband(
        rightX, Constants.Drive.CONTROLLER_ROTATION_DEADBAND)
        * Constants.Drive.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    double limitedOmega;
    if (requestedOmega == 0.0) {
      // A slew limiter normally ramps its previous output back to zero. For swerve this leaks a
      // rotational component into pure translation and toes the modules inward/outward.
      omegaLimiter.reset(0.0);
      limitedOmega = 0.0;
    } else {
      limitedOmega = omegaLimiter.calculate(requestedOmega);
    }
    ChassisSpeeds requestedSpeeds = new ChassisSpeeds(
        -translationY * Constants.Drive.MAX_SPEED_METERS_PER_SECOND,
        -translationX * Constants.Drive.MAX_SPEED_METERS_PER_SECOND,
        limitedOmega);

    // Driver translation is robot-relative: pushing sideways must always command a 90-degree
    // module direction relative to the chassis. Passing true here rotates that direction by the
    // current gyro heading, which presented as the repeatable 20-30 degree strafe error.
    swerve.driveWithControllerCapture(requestedSpeeds, false, leftX, leftY, rightX);
  }
}
