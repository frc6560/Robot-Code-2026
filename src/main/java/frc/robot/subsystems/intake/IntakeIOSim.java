package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim extendIntakeSim;
    private final DCMotorSim spinSim;

    private double extendIntakeAppliedVolts = 0.0;
    private double spinAppliedVolts = 0.0;
    private double extendIntakePosition = 0.0;
    private double extendIntakeTargetRotations = 0.0;
    private boolean extendIntakePositionControlEnabled = false;

    private static final double EXTEND_INTAKE_GEARING = 64.0 / 14.0;
    private static final double SPIN_GEARING = 1.0;
    private static final double EXTEND_INTAKE_MOI = 0.001;
    private static final double SPIN_MOI = 0.001;

    public IntakeIOSim() {
        extendIntakeSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                EXTEND_INTAKE_MOI,
                EXTEND_INTAKE_GEARING
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
        if (extendIntakePositionControlEnabled) {
            // A simple proportional position loop is enough to exercise the one-shot release in sim.
            double error = extendIntakeTargetRotations - extendIntakePosition;
            extendIntakeAppliedVolts = MathUtil.clamp(error * 24.0, -12.0, 12.0);
        }

        extendIntakeSim.setInputVoltage(extendIntakeAppliedVolts);
        spinSim.setInputVoltage(spinAppliedVolts);

        extendIntakeSim.update(0.02);
        spinSim.update(0.02);

        extendIntakePosition += extendIntakeSim.getAngularVelocityRPM() / 60.0 * 0.02;

        inputs.extendIntakePositionRotations = extendIntakePosition;
        inputs.extendIntakeVelocityRPS = extendIntakeSim.getAngularVelocityRPM() / 60.0;
        inputs.extendIntakeAppliedVolts = extendIntakeAppliedVolts;
        inputs.extendIntakeCurrentAmps = Math.abs(extendIntakeSim.getCurrentDrawAmps());
        inputs.extendIntakeTempCelsius = 25.0;

        inputs.spinVelocityRPS = spinSim.getAngularVelocityRPM() / 60.0;
        inputs.spinAppliedVolts = spinAppliedVolts;
        inputs.spinCurrentAmps = Math.abs(spinSim.getCurrentDrawAmps());
        inputs.spinTempCelsius = 25.0;
    }

    @Override
    public void setExtendIntakePercent(double percent) {
        extendIntakePositionControlEnabled = false;
        extendIntakeAppliedVolts = percent * 12.0;
    }

    @Override
    public void setExtendIntakePosition(double rotations) {
        extendIntakeTargetRotations = rotations;
        extendIntakePositionControlEnabled = true;
    }

    @Override
    public void stopExtendIntake() {
        extendIntakePositionControlEnabled = false;
        extendIntakeAppliedVolts = 0.0;
    }

    @Override
    public void setSpinPercent(double percent) {
        spinAppliedVolts = percent * 12.0;
    }

    @Override
    public void resetExtendIntakePosition() {
        // Mirror the real hardware behavior where the pre-release latched position is zeroed.
        extendIntakePosition = 0.0;
        extendIntakeTargetRotations = 0.0;
        extendIntakeAppliedVolts = 0.0;
        extendIntakePositionControlEnabled = false;
    }
}
