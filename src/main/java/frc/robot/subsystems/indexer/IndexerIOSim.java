package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
    private static final double LOOP_PERIOD_SECONDS = 0.02;
    private static final double GEAR_RATIO = 2.5;
    private static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED = 0.001;

    private final DCMotorSim motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1),
                    MOMENT_OF_INERTIA_KG_METERS_SQUARED,
                    GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        motorSim.setInputVoltage(appliedVolts);
        motorSim.update(LOOP_PERIOD_SECONDS);

        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
        inputs.sensorTriggered = new edu.wpi.first.wpilibj.Joystick(1).getRawButton(2);
    }

    @Override
    public void setSpeed(double speed) {
        appliedVolts = MathUtil.clamp(speed, -1.0, 1.0) * 12.0;
    }

    @Override
    public void stop() {
        setSpeed(0.0);
    }
}
