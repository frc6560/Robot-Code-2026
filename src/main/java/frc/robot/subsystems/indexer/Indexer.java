package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.telemetry.RateLimitedTelemetry;
import frc.robot.utility.shooting.FuelShotConfig;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private double floorTargetRPM = 0.0;
    private double tunnelTargetRPM = 0.0;

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Clear legacy telemetry: automatic stall cutoffs are disabled by operator request.
        RateLimitedTelemetry.putBoolean("Indexer/FloorStallFault", false);
        RateLimitedTelemetry.putString("Indexer/FloorBlockedReason", "None");
        RateLimitedTelemetry.recordOutput("Indexer/FloorStallFault", false);
        RateLimitedTelemetry.processInputs("Indexer", inputs);
        RateLimitedTelemetry.recordOutput("Indexer/FloorTargetRPM", floorTargetRPM);
        RateLimitedTelemetry.putNumber("Indexer/FloorStatorLimitAmps", inputs.floorStatorLimitAmps);
        RateLimitedTelemetry.putNumber("Indexer/TunnelStatorLimitAmps", inputs.tunnelStatorLimitAmps);
        RateLimitedTelemetry.putBoolean("Indexer/FloorCurrentLimitOK", inputs.floorCurrentLimitOK);
        RateLimitedTelemetry.putBoolean("Indexer/TunnelCurrentLimitOK", inputs.tunnelCurrentLimitOK);
        RateLimitedTelemetry.putBoolean("Indexer/FloorEnabled", FuelShotConfig.FLOOR_ENABLED);
        RateLimitedTelemetry.recordOutput("Indexer/TunnelTargetRPM", tunnelTargetRPM);
        RateLimitedTelemetry.putNumber("Indexer/TunnelRPM", inputs.tunnelRPM);
        RateLimitedTelemetry.putNumber("Indexer/TunnelTargetRPM", tunnelTargetRPM);
        RateLimitedTelemetry.putNumber("Indexer/TunnelVolts", inputs.appliedVolts);
        RateLimitedTelemetry.putNumber("Indexer/TunnelStatorAmps", inputs.tunnelCurrentAmps);
        RateLimitedTelemetry.putString("Indexer/TunnelConfigurationStatus", inputs.tunnelConfigurationStatus);
        RateLimitedTelemetry.putString("Indexer/TunnelControlStatus", inputs.tunnelControlStatus);
        RateLimitedTelemetry.putBoolean("Indexer/TunnelReady", isTunnelReady());
        // Keep the floor diagnostics separate from the tunnel's legacy appliedVolts field.
        RateLimitedTelemetry.putNumber("Indexer/FloorTargetRPM", floorTargetRPM);
        RateLimitedTelemetry.putNumber("Indexer/FloorRPM", inputs.floorRPM);
        RateLimitedTelemetry.putBoolean("Indexer/FloorConnected", inputs.floorConnected);
        RateLimitedTelemetry.putNumber("Indexer/FloorVolts", inputs.floorAppliedVolts);
        RateLimitedTelemetry.putNumber("Indexer/FloorAmps", inputs.floorCurrentAmps);
        RateLimitedTelemetry.putString("Indexer/FloorConfigurationStatus", inputs.floorConfigurationStatus);
        RateLimitedTelemetry.putString("Indexer/FloorControlStatus", inputs.floorControlStatus);
        RateLimitedTelemetry.putBoolean("Indexer/FloorSecondaryConnected", inputs.floorSecondaryConnected);
        RateLimitedTelemetry.putNumber("Indexer/FloorSecondaryRPM", inputs.floorSecondaryRPM);
        RateLimitedTelemetry.putNumber("Indexer/FloorSecondaryVolts", inputs.floorSecondaryAppliedVolts);
        RateLimitedTelemetry.putNumber("Indexer/FloorSecondaryAmps", inputs.floorSecondaryCurrentAmps);


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
