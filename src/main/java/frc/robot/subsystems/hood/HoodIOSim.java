package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.HoodConstants;

public class HoodIOSim implements HoodIO {
    private final SingleJointedArmSim hoodSim;
    private final PIDController pidController;

    private double targetAngleDegrees =
        HoodConstants.HOOD_MIN_ANGLE - HoodConstants.HOOD_HORIZONTAL_OFFSET;
    private double appliedVolts = 0.0;

    private static final double HOOD_LENGTH_METERS = 0.2;
    private static final double HOOD_MASS_KG = 1.0;
    private static final double SIM_KP_VOLTS_PER_DEGREE = 0.08;
    private static final double SIM_KD_VOLTS_PER_DEGREE_PER_SECOND = 0.002;

    public HoodIOSim() {
        hoodSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            HoodConstants.HOOD_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(HOOD_LENGTH_METERS, HOOD_MASS_KG),
            HOOD_LENGTH_METERS,
            Math.toRadians(HoodConstants.HOOD_MIN_ANGLE - HoodConstants.HOOD_HORIZONTAL_OFFSET),
            Math.toRadians(HoodConstants.HOOD_MAX_ANGLE - HoodConstants.HOOD_HORIZONTAL_OFFSET),
            true,
            Math.toRadians(HoodConstants.HOOD_MIN_ANGLE - HoodConstants.HOOD_HORIZONTAL_OFFSET)
        );

        // The real Talon gains operate in motor rotations. The simulation controller works in
        // mechanism degrees, so it needs gains expressed in volts per degree.
        pidController = new PIDController(
            SIM_KP_VOLTS_PER_DEGREE,
            0.0,
            SIM_KD_VOLTS_PER_DEGREE_PER_SECOND);
        pidController.setTolerance(0.5);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        // Calculate control effort
        double currentAngle = Math.toDegrees(hoodSim.getAngleRads());
        appliedVolts = pidController.calculate(currentAngle, targetAngleDegrees);
        appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));

        hoodSim.setInputVoltage(appliedVolts);
        hoodSim.update(0.02);

        double motorRotations = Math.toDegrees(hoodSim.getAngleRads()) * HoodConstants.HOOD_GEAR_RATIO / 360.0;
        double motorVelocityRPS = Math.toDegrees(hoodSim.getVelocityRadPerSec()) * HoodConstants.HOOD_GEAR_RATIO / 360.0;

        inputs.motorPositionRotations = motorRotations;
        inputs.motorVelocityRPS = motorVelocityRPS;
        inputs.motorAppliedVolts = appliedVolts;
        inputs.motorCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
        inputs.motorTempCelsius = 25.0;

        inputs.hoodAngleDegrees = Math.toDegrees(hoodSim.getAngleRads());
        inputs.absoluteEncoderPositionRotations = inputs.hoodAngleDegrees / 360.0 * HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO;
    }

    @Override
    public void setTargetAngle(double angleDegrees) {
        targetAngleDegrees = angleDegrees;
    }

    @Override
    public void stop() {
        appliedVolts = 0.0;
    }
}
