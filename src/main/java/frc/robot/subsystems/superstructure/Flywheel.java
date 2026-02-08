// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;



public class Flywheel extends SubsystemBase {

  private final TalonFX leftFlywheelMotor;
  private final TalonFX rightFlywheelMotor;

  //control 
  private final VelocityVoltage velocityControl;

  private final NetworkTable limelightTable; 

  //pose supplier

    // get pose from swerve subsystem

  //state
  private double targetRPM = 0.0;



  /** Creates a new Flywheel. */
  public Flywheel() {

    // initialize limelight network table
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    //initialize motor
    leftFlywheelMotor = new TalonFX(FlywheelConstants.LEFT_FLYWHEEL_ID, "rio");
    rightFlywheelMotor = new TalonFX(FlywheelConstants.RIGHT_FLYWHEEL_ID, "rio");

    configureMotor(leftFlywheelMotor, true); 
    configureMotor (rightFlywheelMotor, false);

    //initialize control
    velocityControl = new VelocityVoltage(0.0).withSlot(0);

  }

    private void configureMotor(TalonFX flywheelMotor, boolean inverted){ 
    //motor configuration
    TalonFXConfiguration config = new TalonFXConfiguration();

    //pid config
    //config.Slot0 = FlywheelConstants.FLYWHEEL_PID_CONFIG;
    config.Slot0.kP = FlywheelConstants.kP;
    config.Slot0.kI = FlywheelConstants.kI;
    config.Slot0.kD = FlywheelConstants.kD;
    config.Slot0.kV = FlywheelConstants.kV;
    //config.Slot0 = config.Slot0;

    //motor output 
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Apply inversion
    if (inverted) {
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }


    //current limits
    config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheelMotor.getConfigurator().apply(config);
    }

    /**
   * Gets goal position based on alliance 
   * @return Goal position on field (meters)
   */

  public void setIdle(){
    setRPM(FlywheelConstants.FLYWHEEL_IDLE_RPM);
  }

  public void setRPM(double rpm){
    targetRPM = rpm;
    // actual motor output will be computed in periodic using PID
  }

  public void adjustRPM(double deltaRPM) {
    targetRPM += deltaRPM;
    // Clamp to reasonable values
    targetRPM = Math.max(0, Math.min(targetRPM, FlywheelConstants.FLYWHEEL_MAX_RPM));
    SmartDashboard.putNumber("Flywheel/Manual Target RPM", targetRPM);
  }
  
  public void stop(){
    targetRPM = 0.0;
      leftFlywheelMotor.stopMotor();
      rightFlywheelMotor.stopMotor();
  }

  public double getCurrentRPM(){
    // Average both motors for robustness
    double leftVel = leftFlywheelMotor.getRotorVelocity().getValueAsDouble();
    double rightVel = rightFlywheelMotor.getRotorVelocity().getValueAsDouble();

    // Check if motors are spinning in opposite directions (wiring issue)
    if (Math.abs(leftVel) > 1.0 && Math.abs(rightVel) > 1.0) {
      if (Math.signum(leftVel) != Math.signum(rightVel)) {
        SmartDashboard.putBoolean("Flywheel/MOTOR DIRECTION ERROR", true);
        System.err.println("WARNING: Flywheel motors spinning in opposite directions!");
      } else {
        SmartDashboard.putBoolean("Flywheel/MOTOR DIRECTION ERROR", false);
      }
    }

    double avgMotorRPS = (Math.abs(leftVel) + Math.abs(rightVel)) / 2.0;

    // Convert motor RPS to flywheel RPM
    return (avgMotorRPS * 60.0) / FlywheelConstants.FLYWHEEL_GEAR_RATIO;
  }

  public double getTargetRPM(){
    return targetRPM;
  }

  public boolean atTargetRPM(){
    double currentRPM = getCurrentRPM();
    return Math.abs(currentRPM - targetRPM) < FlywheelConstants.FLYWHEEL_RPM_TOLERANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentRPM = getCurrentRPM();

    // Get individual motor velocities for detailed monitoring
    double leftVelRPS = leftFlywheelMotor.getRotorVelocity().getValueAsDouble();
    double rightVelRPS = rightFlywheelMotor.getRotorVelocity().getValueAsDouble();

    // Safety: stop motors if target is zero
    if (Math.abs(targetRPM) < 1e-3) {
      leftFlywheelMotor.stopMotor();
      rightFlywheelMotor.stopMotor();
    } else {
      // Convert RPM to motor shaft rotations per second (accounting for gear ratio)
      double targetMotorRPS = (targetRPM * FlywheelConstants.FLYWHEEL_GEAR_RATIO) / 60.0;

      // Use hardware velocity control (runs at 1kHz onboard)
      leftFlywheelMotor.setControl(velocityControl.withVelocity(targetMotorRPS));
      rightFlywheelMotor.setControl(velocityControl.withVelocity(targetMotorRPS));

      // Log motor setpoint for verification
      SmartDashboard.putNumber("Flywheel/Target Motor RPS", targetMotorRPS);
    }

    // Calculate error for easy plotting
    double rpmError = targetRPM - currentRPM;

    // Core telemetry
    SmartDashboard.putNumber("Flywheel/Current RPM", currentRPM);
    SmartDashboard.putNumber("Flywheel/Target RPM", targetRPM);
    SmartDashboard.putNumber("Flywheel/RPM Error", rpmError);
    SmartDashboard.putBoolean("Flywheel/At Target", atTargetRPM());

    // Individual motor monitoring
    SmartDashboard.putNumber("Flywheel/Left Motor RPS", leftVelRPS);
    SmartDashboard.putNumber("Flywheel/Right Motor RPS", rightVelRPS);
    SmartDashboard.putNumber("Flywheel/Motor Velocity Diff", Math.abs(leftVelRPS - rightVelRPS));

    // Electrical monitoring
    SmartDashboard.putNumber("Flywheel/Voltage", leftFlywheelMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Flywheel/Left Current", leftFlywheelMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Flywheel/Right Current", rightFlywheelMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Flywheel/Left Temp (C)", leftFlywheelMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Flywheel/Right Temp (C)", rightFlywheelMotor.getDeviceTemp().getValueAsDouble());

}
}