package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim flywheelSim;

    private double appliedVolts = 0.0;
    private double positionRotations = 0.0;

    private static final double FLYWHEEL_MOI = 0.01; // kg*m^2

    public ShooterIOSim() {
        flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                FLYWHEEL_MOI,
                ShooterConstants.FLYWHEEL_GEAR_RATIO
            ),
            DCMotor.getKrakenX60(2)
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelSim.setInputVoltage(appliedVolts);
        flywheelSim.update(0.02);

        double velocityRPS = flywheelSim.getAngularVelocityRPM() / 60.0;
        positionRotations += velocityRPS * 0.02;

        inputs.leaderPositionRotations = positionRotations;
        inputs.leaderVelocityRPS = velocityRPS;
        inputs.leaderAppliedVolts = appliedVolts;
        inputs.leaderCurrentAmps = Math.abs(flywheelSim.getCurrentDrawAmps() / 2.0);
        inputs.leaderTempCelsius = 25.0;

        inputs.followerVelocityRPS = velocityRPS;
        inputs.followerAppliedVolts = appliedVolts;
        inputs.followerCurrentAmps = Math.abs(flywheelSim.getCurrentDrawAmps() / 2.0);
        inputs.followerTempCelsius = 25.0;
    }

    @Override
    public void setVelocityRPS(double rps) {
        // Simple feedforward approximation for simulation
        double motorRPS = rps / ShooterConstants.FLYWHEEL_GEAR_RATIO;
        appliedVolts = ShooterConstants.kV * motorRPS + ShooterConstants.kS * Math.signum(rps);
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
    }

    @Override
    public void stop() {
        appliedVolts = 0.0;
    }
}
