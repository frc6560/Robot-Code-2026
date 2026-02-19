package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FeederIOSim implements FeederIO {
    private final FlywheelSim panSim;
    private final FlywheelSim pusherSim;

    private static final double PAN_GEAR_RATIO = 1.0 / 33.14;
    private static final double PUSHER_GEAR_RATIO = 1.0 / 2.5;
    private static final double MOI = 0.001;

    private double panAppliedVolts = 0.0;
    private double pusherAppliedVolts = 0.0;
    private double panPositionRotations = 0.0;
    private double pusherPositionRotations = 0.0;

    public FeederIOSim() {
        panSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                MOI,
                1.0 / PAN_GEAR_RATIO
            ),
            DCMotor.getKrakenX60(1)
        );
        pusherSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                MOI,
                1.0 / PUSHER_GEAR_RATIO
            ),
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        panSim.setInputVoltage(panAppliedVolts);
        pusherSim.setInputVoltage(pusherAppliedVolts);

        panSim.update(0.02);
        pusherSim.update(0.02);

        double panVelocityRPS = panSim.getAngularVelocityRPM() / 60.0;
        double pusherVelocityRPS = pusherSim.getAngularVelocityRPM() / 60.0;

        panPositionRotations += panVelocityRPS * 0.02;
        pusherPositionRotations += pusherVelocityRPS * 0.02;

        inputs.panPositionRotations = panPositionRotations;
        inputs.panVelocityRPS = panVelocityRPS;
        inputs.panAppliedVolts = panAppliedVolts;
        inputs.panCurrentAmps = Math.abs(panSim.getCurrentDrawAmps());
        inputs.panTempCelsius = 25.0;

        inputs.pusherPositionRotations = pusherPositionRotations;
        inputs.pusherVelocityRPS = pusherVelocityRPS;
        inputs.pusherAppliedVolts = pusherAppliedVolts;
        inputs.pusherCurrentAmps = Math.abs(pusherSim.getCurrentDrawAmps());
        inputs.pusherTempCelsius = 25.0;
    }

    @Override
    public void setPanRPM(double rpm) {
        // Simple feedforward for sim
        double targetRPS = (rpm / 60.0) / PAN_GEAR_RATIO;
        panAppliedVolts = 0.12 * targetRPS;
    }

    @Override
    public void setPusherRPM(double rpm) {
        // Simple feedforward for sim
        double targetRPS = (rpm / 60.0) / PUSHER_GEAR_RATIO;
        pusherAppliedVolts = 0.12 * targetRPS;
    }

    @Override
    public void stop() {
        panAppliedVolts = 0.0;
        pusherAppliedVolts = 0.0;
    }
}
