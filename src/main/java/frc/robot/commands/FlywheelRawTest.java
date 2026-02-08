// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;

/**
 * Raw motor test - bypasses PID and directly outputs percentage power.
 * Use this to verify motors are wired correctly and can physically spin.
 */
public class FlywheelRawTest extends Command {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final double power;

  private final DutyCycleOut dutyCycleControl;

  /**
   * @param power Percent output from -1.0 to 1.0 (start with 0.1 or 0.2)
   */
  public FlywheelRawTest(double power) {
    this.power = power;
    this.leftMotor = new TalonFX(FlywheelConstants.LEFT_FLYWHEEL_ID, "rio");
    this.rightMotor = new TalonFX(FlywheelConstants.RIGHT_FLYWHEEL_ID, "rio");
    this.dutyCycleControl = new DutyCycleOut(0);
  }

  @Override
  public void initialize() {
    System.out.println("=== RAW MOTOR TEST ===");
    System.out.println("Power: " + (power * 100) + "%");
    System.out.println("WATCH THE MOTORS - they should spin!");
  }

  @Override
  public void execute() {
    leftMotor.setControl(dutyCycleControl.withOutput(power));
    rightMotor.setControl(dutyCycleControl.withOutput(power));

    // Log telemetry
    SmartDashboard.putNumber("RawTest/Power", power);
    SmartDashboard.putNumber("RawTest/Left RPS", leftMotor.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber("RawTest/Right RPS", rightMotor.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber("RawTest/Left Voltage", leftMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("RawTest/Right Voltage", rightMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("RawTest/Left Current", leftMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("RawTest/Right Current", rightMotor.getSupplyCurrent().getValueAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("=== RAW TEST END ===");
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
