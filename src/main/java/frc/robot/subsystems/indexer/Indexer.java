package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IndexerConstants;
import frc.robot.utility.shooting.FuelShotConfig;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIO.IndexerIOInputs inputs = new IndexerIO.IndexerIOInputs();
    private double nextTelemetry;
    private double floorTargetRPM = 0.0;
    private double tunnelTargetRPM = 0.0;

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (edu.wpi.first.wpilibj.RobotBase.isReal() && now < nextTelemetry) return;
        nextTelemetry = now + 0.5;
        SmartDashboard.putNumber("Indexer/FloorPositionRotations", inputs.floorPositionRotations);
        SmartDashboard.putNumber("Indexer/FloorMotorRPM", inputs.floorRPM * IndexerConstants.FLOOR_GEAR_RATIO);
        SmartDashboard.putNumber("Indexer/FloorMotorTargetRPM", floorTargetRPM * IndexerConstants.FLOOR_GEAR_RATIO);
        SmartDashboard.putNumber("Indexer/FloorVolts", inputs.floorAppliedVolts);
        SmartDashboard.putNumber("Indexer/FloorAmps", inputs.floorCurrentAmps);
        SmartDashboard.putNumber("Indexer/FloorSecondaryPositionRotations", inputs.floorSecondaryPositionRotations);
        SmartDashboard.putNumber("Indexer/FloorSecondaryMotorRPM", inputs.floorSecondaryRPM * IndexerConstants.FLOOR_GEAR_RATIO);
        SmartDashboard.putNumber("Indexer/FloorSecondaryMotorTargetRPM", floorTargetRPM * IndexerConstants.FLOOR_GEAR_RATIO);
        SmartDashboard.putNumber("Indexer/FloorSecondaryVolts", inputs.floorSecondaryAppliedVolts);
        SmartDashboard.putNumber("Indexer/FloorSecondaryAmps", inputs.floorSecondaryCurrentAmps);
        SmartDashboard.putNumber("Indexer/TunnelPositionRotations", inputs.tunnelPositionRotations);
        SmartDashboard.putNumber("Indexer/TunnelMotorRPM", inputs.tunnelRPM * IndexerConstants.TUNNEL_GEAR_RATIO);
        SmartDashboard.putNumber("Indexer/TunnelMotorTargetRPM", tunnelTargetRPM * IndexerConstants.TUNNEL_GEAR_RATIO);
        SmartDashboard.putNumber("Indexer/TunnelVolts", inputs.appliedVolts);
        SmartDashboard.putNumber("Indexer/TunnelStatorAmps", inputs.tunnelCurrentAmps);
    }

    public void setFloorRPM(double rpm) {
        floorTargetRPM = FuelShotConfig.FLOOR_ENABLED && Double.isFinite(rpm) ? rpm : 0.0;
        io.setFloorRPM(floorTargetRPM);
    }

    public void setTunnelRPM(double rpm) {
        tunnelTargetRPM = rpm;
        io.setTunnelRPM(rpm);
    }

    public void stop() {
        floorTargetRPM = 0.0;
        tunnelTargetRPM = 0.0;
        io.stop();
    }

    public boolean isTunnelReady() { return inputs.tunnelConnected && inputs.tunnelConfigured; }
    public double getTunnelRPM() { return inputs.tunnelRPM; }

    public double getFloorTargetRPM() {
        return floorTargetRPM;
    }

    public double getTunnelTargetRPM() {
        return tunnelTargetRPM;
    }
}
