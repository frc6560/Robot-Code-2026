package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim leftSim;
    private final DCMotorSim rightSim;

    private double appliedVolts = 0.0;

    private static final double GEARING = 1.0;
    private static final double MOI = 0.001;

    public IntakeIOSim() {
        leftSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                MOI,
                GEARING
            ),
            DCMotor.getKrakenX60(1)
        );
        rightSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                MOI,
                GEARING
            ),
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if (inputs instanceof IntakeIOInputsAutoLogged loggedInputs) {
            updateInputs(loggedInputs);
            return;
        }

        updateMotorInputs(inputs);
        inputs.currentAmps = new edu.wpi.first.wpilibj.Joystick(1).getRawButton(1) ? 25.0 : 0.0;
    }

    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        updateMotorInputs(inputs);
        inputs.currentAmps = new edu.wpi.first.wpilibj.Joystick(1).getRawButton(1) ? 25.0 : 0.0;
    }

    private void updateMotorInputs(IntakeIOInputs inputs) {
        leftSim.setInputVoltage(appliedVolts);
        rightSim.setInputVoltage(appliedVolts);

        leftSim.update(0.02);
        rightSim.update(0.02);

        inputs.leftVelocityRPS = leftSim.getAngularVelocityRPM() / 60.0;
        inputs.leftAppliedVolts = appliedVolts;
        inputs.leftCurrentAmps = Math.abs(leftSim.getCurrentDrawAmps());
        inputs.leftTempCelsius = 25.0;

        inputs.rightVelocityRPS = rightSim.getAngularVelocityRPM() / 60.0;
        inputs.rightAppliedVolts = appliedVolts;
        inputs.rightCurrentAmps = Math.abs(rightSim.getCurrentDrawAmps());
        inputs.rightTempCelsius = 25.0;
    }

    @Override
    public void setRollerRPM(double rpm) {
        // Simple feedforward approximation for sim: target velocity -> voltage
        double targetRPS = rpm / 60.0;
        appliedVolts = targetRPS * 0.12 * 12.0; // kV * nominal voltage
    }

    @Override
    public void stop() {
        appliedVolts = 0.0;
    }
}
