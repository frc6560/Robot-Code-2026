// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.periodic;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ManualControls;
import frc.robot.subsystems.superstructure.Shooter;
import frc.robot.utility.Shooter.ShotCalculator;

public class ShooterCommand extends Command {

  private final Shooter shooter;
  private final ShotCalculator shotCalculator;
  private final ManualControls controls;
  
  // Suppliers to feed the calculator
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;

  public ShooterCommand(
      Shooter shooter, 
      ShotCalculator shotCalculator, 
      ManualControls controls,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier) {
      
    this.shooter = shooter;
    this.shotCalculator = shotCalculator;
    this.controls = controls;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setIdle();
    shotCalculator.resetFilter(); 
  }

  @Override
  public void execute() {
    // 1. Update the Calculator with where the robot is NOW
    shotCalculator.calculate(poseSupplier.get(), speedsSupplier.get());

    // 2. Check Driver Input
    if (controls.getShooterRev()) {
      // 3. Set RPM based on the calculated virtual target
      shooter.setRPMFromCalculator(shotCalculator);
    } else {
      // 4. Idle
      shooter.setIdle();
    }
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