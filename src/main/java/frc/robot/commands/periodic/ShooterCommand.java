// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.periodic;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Shooter; // Check this import path!
import frc.robot.utility.Shooter.ShotCalculator;
import frc.robot.ManualControls;

public class ShooterCommand extends Command {

  enum ShooterState {
    IDLE, 
    SHOOTING_TARGET, 
    SHOOTING_PASS
  }

  private final Shooter shooter; 
  private final ShotCalculator shotCalculator;

  /** * Creates a new ShooterCommand. 
   * Updated to accept ShotCalculator to match RobotContainer.
   */
  public ShooterCommand(Shooter shooter, ShotCalculator shotCalculator, ManualControls controls) {
    this.shooter = shooter;
    this.shotCalculator = shotCalculator;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // Start at Idle speed immediately
    shooter.setIdle();
  }

  @Override
  public void execute() {
    shooter.setIdle(); // Default to idle speed
    // later we can add more important stuff
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}