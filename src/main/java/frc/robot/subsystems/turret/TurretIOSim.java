package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.TurretConstants;

public class TurretIOSim implements TurretIO {
    private final SingleJointedArmSim turretSim;
    private final PIDController pidController;

    private double targetAngleDegrees = 0.0;
    private double appliedVolts = 0.0;

    private static final double TURRET_LENGTH_METERS = 0.3;
    private static final double TURRET_MASS_KG = 3.0;

    public TurretIOSim() {
        turretSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            TurretConstants.MOTOR_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(TURRET_LENGTH_METERS, TURRET_MASS_KG),
            TURRET_LENGTH_METERS,
            Math.toRadians(TurretConstants.LOWER_SOFT_LIMIT),
            Math.toRadians(TurretConstants.UPPER_SOFT_LIMIT),
            false,
            0.0
        );

        pidController = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        pidController.setTolerance(1.0);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // Calculate control effort
        double currentAngle = Math.toDegrees(turretSim.getAngleRads());
        appliedVolts = pidController.calculate(currentAngle, targetAngleDegrees);
        appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));

        turretSim.setInputVoltage(appliedVolts);
        turretSim.update(0.02);

        double turretAngle = Math.toDegrees(turretSim.getAngleRads());
        double motorRotations = turretAngle * TurretConstants.MOTOR_GEAR_RATIO / 360.0;
        double motorVelocityRPS = Math.toDegrees(turretSim.getVelocityRadPerSec()) * TurretConstants.MOTOR_GEAR_RATIO / 360.0;

        inputs.motorPositionRotations = motorRotations;
        inputs.motorVelocityRPS = motorVelocityRPS;
        inputs.motorAppliedVolts = appliedVolts;
        inputs.motorCurrentAmps = Math.abs(turretSim.getCurrentDrawAmps());
        inputs.motorTempCelsius = 25.0;

        inputs.turretAngleDegrees = turretAngle;
        inputs.turretVelocityDegreesPerSec = Math.toDegrees(turretSim.getVelocityRadPerSec());
        inputs.absoluteEncoderPositionRotations = turretAngle / 360.0 * TurretConstants.ENCODER_GEAR_RATIO;
    }

    @Override
    public void setTargetAngle(double angleDegrees) {
        targetAngleDegrees = angleDegrees;
    }

    @Override
    public void stop() {
        appliedVolts = 0.0;
    }

    @Override
    public void seedMotorEncoder() {
        // No-op in simulation
    }
}
