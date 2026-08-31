package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {
    private static final double LOOP_PERIOD_SECONDS = 0.02;
    private static final double FLOOR_GEAR_RATIO = 2.5;
    private static final double TOWER_GEAR_RATIO = 5.0 / 3.0;
    private static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED = 0.001;

    private final FlywheelSim floorSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60(2),
                    MOMENT_OF_INERTIA_KG_METERS_SQUARED,
                    FLOOR_GEAR_RATIO),
            DCMotor.getKrakenX60(2));
    private final FlywheelSim towerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60(1),
                    MOMENT_OF_INERTIA_KG_METERS_SQUARED,
                    TOWER_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    private double floorAppliedVolts = 0.0;
    private double towerAppliedVolts = 0.0;
    private double floorPositionRotations = 0.0;
    private double towerPositionRotations = 0.0;
    private double simulatedDistance = 1.0;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        floorSim.update(LOOP_PERIOD_SECONDS);
        towerSim.update(LOOP_PERIOD_SECONDS);

        double floorVelocityRPS = floorSim.getAngularVelocityRPM() / 60.0;
        double towerVelocityRPS = towerSim.getAngularVelocityRPM() / 60.0;
        floorPositionRotations += floorVelocityRPS * LOOP_PERIOD_SECONDS;
        towerPositionRotations += towerVelocityRPS * LOOP_PERIOD_SECONDS;

        if (floorAppliedVolts > 0.0) {
            simulatedDistance = Math.max(0.0, simulatedDistance - 0.02);
        } else if (floorAppliedVolts < 0.0) {
            simulatedDistance = 1.0;
        }

        inputs.floorPositionRotations = floorPositionRotations;
        inputs.floorVelocityRPS = floorVelocityRPS;
        inputs.floorAppliedVolts = floorAppliedVolts;
        inputs.floorLeaderCurrentAmps = Math.abs(floorSim.getCurrentDrawAmps()) / 2.0;
        inputs.floorFollowerCurrentAmps = Math.abs(floorSim.getCurrentDrawAmps()) / 2.0;

        inputs.towerPositionRotations = towerPositionRotations;
        inputs.towerVelocityRPS = towerVelocityRPS;
        inputs.towerAppliedVolts = towerAppliedVolts;
        inputs.towerCurrentAmps = Math.abs(towerSim.getCurrentDrawAmps());
        inputs.towerSensorDistanceMeters = simulatedDistance;
        inputs.hasGamePiece = false;
    }

    @Override
    public void setFloorDutyCycle(double dutyCycle) {
        floorAppliedVolts = dutyCycle * 12.0;
        floorSim.setInputVoltage(floorAppliedVolts);
    }

    @Override
    public void setTowerDutyCycle(double dutyCycle) {
        towerAppliedVolts = dutyCycle * 12.0;
        towerSim.setInputVoltage(towerAppliedVolts);
    }

    @Override
    public void stop() {
        setFloorDutyCycle(0.0);
        setTowerDutyCycle(0.0);
    }
}
