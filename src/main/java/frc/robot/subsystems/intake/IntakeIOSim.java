package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim deploySim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.01, 1.0), DCMotor.getKrakenX44(1));
    private final DCMotorSim rollerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1.0), DCMotor.getKrakenX60(1));
    private double deployTargetRotations = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
    private double deployAppliedVolts;
    private double rollerAppliedVolts;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double positionError = deployTargetRotations - deploySim.getAngularPositionRotations();
        deployAppliedVolts = MathUtil.clamp(positionError * 4.0, -12.0, 12.0);
        deploySim.setInputVoltage(deployAppliedVolts);
        rollerSim.setInputVoltage(rollerAppliedVolts);
        deploySim.update(0.02);
        rollerSim.update(0.02);
        inputs.deployPositionRotations = deploySim.getAngularPositionRotations();
        inputs.deployVelocityRPS = deploySim.getAngularVelocityRPM() / 60.0;
        inputs.deployAppliedVolts = deployAppliedVolts;
        inputs.deployCurrentAmps = Math.abs(deploySim.getCurrentDrawAmps());
        inputs.deployTempCelsius = 25.0;
        inputs.rollerVelocityRPS = rollerSim.getAngularVelocityRPM() / 60.0;
        inputs.rollerAppliedVolts = rollerAppliedVolts;
        inputs.rollerCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
        inputs.rollerTempCelsius = 25.0;
    }

    @Override
    public void setRollerRPM(double rpm) {
        rollerAppliedVolts = MathUtil.clamp(
            rpm * IntakeConstants.ROLLER_GEARING / 6000.0 * 12.0, -12.0, 12.0);
    }

    @Override
    public void setDeployPosition(double motorRotations) {
        deployTargetRotations = MathUtil.clamp(motorRotations,
            IntakeConstants.RETRACTED_POSITION_ROTATIONS, IntakeConstants.EXTENDED_POSITION_ROTATIONS);
    }

    @Override
    public void stop() {
        deployAppliedVolts = 0.0;
        rollerAppliedVolts = 0.0;
        deploySim.setInputVoltage(0.0);
        rollerSim.setInputVoltage(0.0);
    }
}
