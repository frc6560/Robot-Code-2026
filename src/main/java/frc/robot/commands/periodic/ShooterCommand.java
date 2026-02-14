// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.periodic;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Shooter;
import frc.robot.utility.Shooter.ShotCalculator;

public class ShooterCommand extends Command {

  private final Shooter shooter;
  private final ShotCalculator shotCalculator;
  
  // CHANGED: Replaced ManualControls with a generic BooleanSupplier
  private final BooleanSupplier isShootingSupplier; 
  
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;

  public ShooterCommand(
      Shooter shooter, 
      ShotCalculator shotCalculator, 
      BooleanSupplier isShootingSupplier, // Pass the trigger check here
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier) {
      
    this.shooter = shooter;
    this.shotCalculator = shotCalculator;
    this.isShootingSupplier = isShootingSupplier;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setIdle();
  }

  @Override
  public void execute() {
    // 1. Constantly update the math
    shotCalculator.calculate(poseSupplier.get(), speedsSupplier.get());

    // 2. Check the BooleanSupplier (The binding from RobotContainer)
    if (isShootingSupplier.getAsBoolean()) {
      shooter.setRPMFromCalculator(shotCalculator);
    } else {
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