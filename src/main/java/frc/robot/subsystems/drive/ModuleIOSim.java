package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), constants.DriveInertia, constants.DriveMotorGearRatio),
            DCMotor.getKrakenX60Foc(1));
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), constants.SteerInertia, constants.SteerMotorGearRatio),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.setInputVoltage(driveAppliedVolts);
    turnSim.setInputVoltage(turnAppliedVolts);
    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = Rotation2d.fromRadians(turnSim.getAngularPositionRad());
    inputs.turnPosition = Rotation2d.fromRadians(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveAppliedVolts = MathUtil.clamp(output, -12.0, 12.0);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnAppliedVolts = MathUtil.clamp(output, -12.0, 12.0);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    // Simple P controller for sim
    double error = velocityRadPerSec - driveSim.getAngularVelocityRadPerSec();
    driveAppliedVolts = MathUtil.clamp(error * 2.0, -12.0, 12.0);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double error = rotation.minus(Rotation2d.fromRadians(turnSim.getAngularPositionRad())).getRadians();
    turnAppliedVolts = MathUtil.clamp(error * 12.0, -12.0, 12.0);
  }
}
