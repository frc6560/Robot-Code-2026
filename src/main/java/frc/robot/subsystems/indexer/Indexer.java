package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private final LoggedMechanismRoot2d sensorRoot = mech2d.getRoot("SensorRoot", 1.65, 0.25);
    private final LoggedMechanismLigament2d sensorIndicator = sensorRoot.append(
            new LoggedMechanismLigament2d("SensorTriggered", 0.1, 90.0));

    private double rollerAngleDegrees = 0.0;

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        rollerAngleDegrees += inputs.appliedVolts / 12.0
                * MAX_VISUALIZER_SPEED_DEGREES_PER_SECOND
                * LOOP_PERIOD_SECONDS;
        roller.setAngle(rollerAngleDegrees);
        sensorIndicator.setLength(inputs.sensorTriggered ? 0.5 : 0.1);
        Logger.recordOutput("Indexer/Mechanism", mech2d);
    }

    public void runIndexer(double speed) {
        io.setSpeed(speed);
    }

    public void stop() {
        io.stop();
    }

    public boolean hasGamePiece() {
        return inputs.sensorTriggered;
    }
}
