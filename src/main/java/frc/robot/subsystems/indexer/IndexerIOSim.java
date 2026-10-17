package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IndexerConstants;

/** Independent floor and tunnel velocity loops, using mechanism-side RPM like the real IO. */
public class IndexerIOSim implements IndexerIO {
    private final DCMotorSim floor = createMotor(IndexerConstants.FLOOR_GEAR_RATIO);
    private final DCMotorSim tunnel = createMotor(IndexerConstants.TUNNEL_GEAR_RATIO);
    private double floorRPM = 0.0;
    private double tunnelRPM = 0.0;

    private static DCMotorSim createMotor(double reduction) {
        return new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.001, reduction), DCMotor.getKrakenX60(1));
    }

    private static double updateMotor(DCMotorSim motor, double rpm, double reduction) {
        double volts = DriverStation.isDisabled() || rpm == 0.0 ? 0.0 : MathUtil.clamp(
            IndexerConstants.MOTOR_KV * reduction * rpm / 60.0
                + IndexerConstants.VELOCITY_KP * (rpm - motor.getAngularVelocityRPM()) / 60.0,
            -12.0, 12.0);
        motor.setInputVoltage(volts);
        motor.update(0.02);
        return volts;
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.floorAppliedVolts = updateMotor(floor, floorRPM, IndexerConstants.FLOOR_GEAR_RATIO);
        inputs.appliedVolts = updateMotor(tunnel, tunnelRPM, IndexerConstants.TUNNEL_GEAR_RATIO);
        inputs.floorRPM = floor.getAngularVelocityRPM();
        inputs.tunnelRPM = tunnel.getAngularVelocityRPM();
        inputs.currentAmps = Math.abs(floor.getCurrentDrawAmps()) + Math.abs(tunnel.getCurrentDrawAmps());
        inputs.floorCurrentAmps = Math.abs(floor.getCurrentDrawAmps());
        inputs.floorConnected = true;
        inputs.floorConfigurationStatus = "Simulation";
        inputs.floorControlStatus = "Simulation";
        inputs.floorTelemetryStatus = "Simulation";
        inputs.tunnelConnected = true;
        inputs.tunnelAppliedVolts = inputs.appliedVolts;
        inputs.tunnelCurrentAmps = Math.abs(tunnel.getCurrentDrawAmps());
        inputs.tunnelConfigurationStatus = "Simulation";
        inputs.tunnelControlStatus = "Simulation";
        inputs.tunnelTelemetryStatus = "Simulation";
    }

    @Override
    public void setFloorRPM(double rpm) {
        floorRPM = rpm;
    }

    @Override
    public void setTunnelRPM(double rpm) {
        tunnelRPM = rpm;
    }

    @Override
    public void stop() {
        floorRPM = 0.0;
        tunnelRPM = 0.0;
    }
}
