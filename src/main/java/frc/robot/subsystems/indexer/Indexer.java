package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import frc.robot.utility.shooting.FuelShotConfig;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Indexer extends SubsystemBase {
    private static final double LOOP_PERIOD_SECONDS = 0.02;
    private static final double MAX_VISUALIZER_SPEED_DEGREES_PER_SECOND = 720.0;

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final LoggedMechanism2d mech2d = new LoggedMechanism2d(2.0, 2.0);
    private final LoggedMechanismRoot2d rollerRoot = mech2d.getRoot("RollerRoot", 1.0, 1.0);
    private final LoggedMechanismLigament2d roller = rollerRoot.append(
            new LoggedMechanismLigament2d("Roller", 0.75, 0.0));

    private double rollerAngleDegrees = 0.0;
    private double floorTargetRPM = 0.0;
    private double tunnelTargetRPM = 0.0;

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Clear legacy telemetry: automatic stall cutoffs are disabled by operator request.
        SmartDashboard.putBoolean("Indexer/FloorStallFault", false);
        SmartDashboard.putString("Indexer/FloorBlockedReason", "None");
        Logger.recordOutput("Indexer/FloorStallFault", false);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("Indexer/FloorTargetRPM", floorTargetRPM);
        SmartDashboard.putNumber("Indexer/FloorStatorLimitAmps", inputs.floorStatorLimitAmps);
        SmartDashboard.putNumber("Indexer/TunnelStatorLimitAmps", inputs.tunnelStatorLimitAmps);
        SmartDashboard.putBoolean("Indexer/FloorCurrentLimitOK", inputs.floorCurrentLimitOK);
        SmartDashboard.putBoolean("Indexer/TunnelCurrentLimitOK", inputs.tunnelCurrentLimitOK);
        SmartDashboard.putBoolean("Indexer/FloorEnabled", FuelShotConfig.FLOOR_ENABLED);
        Logger.recordOutput("Indexer/TunnelTargetRPM", tunnelTargetRPM);
        SmartDashboard.putNumber("Indexer/TunnelRPM", inputs.tunnelRPM);
        SmartDashboard.putNumber("Indexer/TunnelTargetRPM", tunnelTargetRPM);
        SmartDashboard.putNumber("Indexer/TunnelVolts", inputs.appliedVolts);
        SmartDashboard.putNumber("Indexer/TunnelStatorAmps", inputs.tunnelCurrentAmps);
        SmartDashboard.putString("Indexer/TunnelConfigurationStatus", inputs.tunnelConfigurationStatus);
        SmartDashboard.putString("Indexer/TunnelControlStatus", inputs.tunnelControlStatus);
        SmartDashboard.putBoolean("Indexer/TunnelReady", isTunnelReady());
        // Keep the floor diagnostics separate from the tunnel's legacy appliedVolts field.
        SmartDashboard.putNumber("Indexer/FloorTargetRPM", floorTargetRPM);
        SmartDashboard.putNumber("Indexer/FloorRPM", inputs.floorRPM);
        SmartDashboard.putBoolean("Indexer/FloorConnected", inputs.floorConnected);
        SmartDashboard.putNumber("Indexer/FloorVolts", inputs.floorAppliedVolts);
        SmartDashboard.putNumber("Indexer/FloorAmps", inputs.floorCurrentAmps);
        SmartDashboard.putString("Indexer/FloorConfigurationStatus", inputs.floorConfigurationStatus);
        SmartDashboard.putString("Indexer/FloorControlStatus", inputs.floorControlStatus);
        SmartDashboard.putBoolean("Indexer/FloorSecondaryConnected", inputs.floorSecondaryConnected);
        SmartDashboard.putNumber("Indexer/FloorSecondaryRPM", inputs.floorSecondaryRPM);
        SmartDashboard.putNumber("Indexer/FloorSecondaryVolts", inputs.floorSecondaryAppliedVolts);
        SmartDashboard.putNumber("Indexer/FloorSecondaryAmps", inputs.floorSecondaryCurrentAmps);

        rollerAngleDegrees += inputs.appliedVolts / 12.0
                * MAX_VISUALIZER_SPEED_DEGREES_PER_SECOND
                * LOOP_PERIOD_SECONDS;
        roller.setAngle(rollerAngleDegrees);
        Logger.recordOutput("Indexer/Mechanism", mech2d);
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
