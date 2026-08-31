package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Indexer extends SubsystemBase {
    private static final double MECHANISM_WIDTH_METERS = 3.0;
    private static final double MECHANISM_HEIGHT_METERS = 3.0;
    private static final double TOWER_SENSOR_X_METERS = 2.25;
    private static final double TOWER_SENSOR_Y_METERS = 2.5;

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private final LoggedMechanism2d mechanism =
            new LoggedMechanism2d(MECHANISM_WIDTH_METERS, MECHANISM_HEIGHT_METERS);

    private final LoggedMechanismRoot2d floorRoot = mechanism.getRoot("FloorRoot", 0.75, 0.5);
    private final LoggedMechanismLigament2d floorLigament = floorRoot.append(
            new LoggedMechanismLigament2d("FloorRollers", 0.75, 0.0));

    private final LoggedMechanismRoot2d towerRoot = mechanism.getRoot("TowerRoot", 2.25, 1.25);
    private final LoggedMechanismLigament2d towerLigament = towerRoot.append(
            new LoggedMechanismLigament2d("TowerRollers", 0.75, 0.0));

    private final LoggedMechanismRoot2d gamePieceRoot =
            mechanism.getRoot("GamePieceRoot", TOWER_SENSOR_X_METERS, 0.5);
    private final LoggedMechanismLigament2d gamePieceLigament = gamePieceRoot.append(
            new LoggedMechanismLigament2d("GamePiece", 0.25, 90.0));

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        floorLigament.setAngle(inputs.floorPositionRotations * 360.0);
        towerLigament.setAngle(inputs.towerPositionRotations * 360.0);

        double gamePieceY = MathUtil.clamp(
                TOWER_SENSOR_Y_METERS - inputs.towerSensorDistanceMeters,
                0.0,
                MECHANISM_HEIGHT_METERS - gamePieceLigament.getLength());
        gamePieceRoot.setPosition(TOWER_SENSOR_X_METERS, gamePieceY);

        Logger.recordOutput("Indexer/Mechanism", mechanism);
    }

    public void runFloor(double speed) {
        io.setFloorDutyCycle(speed);
    }

    public void runTower(double speed) {
        io.setTowerDutyCycle(speed);
    }

    public void runAll(double speed) {
        runFloor(speed);
        runTower(speed);
    }

    public void stop() {
        io.stop();
    }

    public boolean hasGamePiece(double threshold) {
        return inputs.towerSensorDistanceMeters <= threshold;
    }
}
