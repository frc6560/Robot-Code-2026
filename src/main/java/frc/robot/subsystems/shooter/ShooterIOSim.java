package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim flywheelSim;

    private double appliedVolts = 0.0;
    private double positionRotations = 0.0;

    public ShooterIOSim() {
        // FLYWHEEL_GEARBOX already carries the reduction, so the plant is built at 1:1.
        flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                ShooterConstants.FLYWHEEL_GEARBOX,
                ShooterConstants.FLYWHEEL_MOI,
                1.0
            ),
            ShooterConstants.FLYWHEEL_GEARBOX
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelSim.setInputVoltage(appliedVolts);
        flywheelSim.update(0.02);

        double velocityRadsPerSec = flywheelSim.getAngularVelocityRadPerSec();
        double motorRPS = velocityRadsPerSec / (2.0 * Math.PI) / ShooterConstants.FLYWHEEL_GEAR_RATIO;
        positionRotations += motorRPS * 0.02;
        double currentPerMotor =
            Math.abs(flywheelSim.getCurrentDrawAmps()) / ShooterConstants.FLYWHEEL_MOTOR_COUNT;

        inputs.leaderConnected = true;
        inputs.followerConnected = true;
        inputs.velocityRadsPerSec = velocityRadsPerSec;

        inputs.leaderPositionRotations = positionRotations;
        inputs.leaderVelocityRPS = motorRPS;
        inputs.leaderAppliedVolts = appliedVolts;
        inputs.leaderCurrentAmps = currentPerMotor;
        inputs.leaderTempCelsius = 25.0;

        inputs.followerVelocityRPS = motorRPS;
        inputs.followerAppliedVolts = appliedVolts;
        inputs.followerCurrentAmps = currentPerMotor;
        inputs.followerTempCelsius = 25.0;
    }

    @Override
    public void applyOutputs(ShooterIOOutputs outputs) {
        double velocity = flywheelSim.getAngularVelocityRadPerSec();
        double backEmf = velocity / ShooterConstants.FLYWHEEL_GEARBOX.KvRadPerSecPerVolt;
        double volts = switch (outputs.mode) {
            case VELOCITY -> {
                double errorMotorRPS = (outputs.velocityRadsPerSec - velocity)
                    / (2.0 * Math.PI) / ShooterConstants.FLYWHEEL_GEAR_RATIO;
                yield outputs.feedforwardVolts + outputs.kP * errorMotorRPS;
            }
            case VOLTAGE -> outputs.voltage;
            // Coast is an open circuit: no current, so match back-EMF instead of shorting at 0 V.
            case COAST -> backEmf;
        };

        // Mirror the Talon's stator limit so the sim can't spin up faster than the real thing.
        double currentHeadroom = ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT
            * ShooterConstants.FLYWHEEL_MOTOR_COUNT * ShooterConstants.FLYWHEEL_GEARBOX.rOhms;
        volts = MathUtil.clamp(volts, backEmf - currentHeadroom, backEmf + currentHeadroom);
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }
}
