// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.superstructure.Flywheel;

/**
 * Dead simple flywheel test - just spins motors at a fixed RPM.
 * Perfect for initial motor validation.
 */
public class SimpleFlywheelTest extends Command {
  private final Flywheel flywheel;
  private final double testRPM;

  /**
   * @param flywheel The flywheel subsystem
   * @param testRPM The RPM to test at (start low, like 500-1000)
   */
  public SimpleFlywheelTest(Flywheel flywheel, double testRPM) {
    this.flywheel = flywheel;
    this.testRPM = testRPM;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {
    System.out.println("=== FLYWHEEL TEST START ===");
    System.out.println("Target RPM: " + testRPM);
    System.out.println("Watch SmartDashboard for telemetry");
    flywheel.setRPM(testRPM);
  }

  @Override
  public void execute() {
    // Periodic in Flywheel subsystem handles everything
    // Just print status every 50 cycles (~1 second)
    if (CommandScheduler.getInstance().isScheduled(this)) {
      // Optional: Could add periodic status printing here
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("=== FLYWHEEL TEST END ===");
    System.out.println("Final RPM: " + flywheel.getCurrentRPM());
    System.out.println("Stopping motors...");
    flywheel.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }
}
