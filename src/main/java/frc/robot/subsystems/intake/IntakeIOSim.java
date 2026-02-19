package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim extendSim;
    private final DCMotorSim spinSim;

    private double extendAppliedVolts = 0.0;
    private double spinAppliedVolts = 0.0;
    private double extendPosition = 0.0;
    private boolean simulatedLimitSwitch = true;

    private static final double EXTEND_GEARING = 1.0;
    private static final double SPIN_GEARING = 1.0;
    private static final double EXTEND_MOI = 0.001;
    private static final double SPIN_MOI = 0.001;

    public IntakeIOSim() {
        extendSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                EXTEND_MOI,
                EXTEND_GEARING
            ),
            DCMotor.getKrakenX60(1)
        );
        spinSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                SPIN_MOI,
                SPIN_GEARING
            ),
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        extendSim.setInputVoltage(extendAppliedVolts);
        spinSim.setInputVoltage(spinAppliedVolts);

        extendSim.update(0.02);
        spinSim.update(0.02);

        extendPosition += extendSim.getAngularVelocityRPM() / 60.0 * 0.02;

        inputs.extendPositionRotations = extendPosition;
        inputs.extendVelocityRPS = extendSim.getAngularVelocityRPM() / 60.0;
        inputs.extendAppliedVolts = extendAppliedVolts;
        inputs.extendCurrentAmps = Math.abs(extendSim.getCurrentDrawAmps());
        inputs.extendTempCelsius = 25.0;

        inputs.spinVelocityRPS = spinSim.getAngularVelocityRPM() / 60.0;
        inputs.spinAppliedVolts = spinAppliedVolts;
        inputs.spinCurrentAmps = Math.abs(spinSim.getCurrentDrawAmps());
        inputs.spinTempCelsius = 25.0;

        // Simulate limit switch - triggered when position is near zero
        simulatedLimitSwitch = extendPosition <= 0.1;
        inputs.retractLimitSwitch = simulatedLimitSwitch;
    }

    @Override
    public void setExtendPercent(double percent) {
        extendAppliedVolts = percent * 12.0;
    }

    @Override
    public void setSpinPercent(double percent) {
        spinAppliedVolts = percent * 12.0;
    }

    @Override
    public void resetExtendPosition() {
        extendPosition = 0.0;
    }

    @Override
    public void setSpringyCurrentLimits(boolean springy) {
        // No-op in simulation
    }
}
