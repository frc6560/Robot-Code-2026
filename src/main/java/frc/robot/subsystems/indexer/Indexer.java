package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
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
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("Indexer/FloorTargetRPM", floorTargetRPM);
        Logger.recordOutput("Indexer/TunnelTargetRPM", tunnelTargetRPM);
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
        floorTargetRPM = rpm;
        io.setFloorRPM(rpm);
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

    public double getFloorTargetRPM() {
        return floorTargetRPM;
    }

    public double getTunnelTargetRPM() {
        return tunnelTargetRPM;
    }
}
