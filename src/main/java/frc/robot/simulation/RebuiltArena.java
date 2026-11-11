package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/** Flat REBUILT collision model with perimeter walls and approximate hub footprints. */
public final class RebuiltArena extends SimulatedArena {
    public RebuiltArena() {
        super(new RebuiltFieldMap());
    }

    @Override
    public void placeGamePiecesOnField() {}

    private static final class RebuiltFieldMap extends FieldMap {
        RebuiltFieldMap() {
            var blueBottom = new Translation2d();
            var redBottom = new Translation2d(FieldConstants.FIELD_LENGTH, 0);
            var redTop = new Translation2d(FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);
            var blueTop = new Translation2d(0, FieldConstants.FIELD_WIDTH);
            addBorderLine(blueBottom, redBottom);
            addBorderLine(redBottom, redTop);
            addBorderLine(redTop, blueTop);
            addBorderLine(blueTop, blueBottom);

            double hubWidth = 47 * 0.0254;
            addRectangularObstacle(hubWidth, hubWidth,
                new Pose2d(FieldConstants.BLUE_HUB_CENTER, Rotation2d.kZero));
            addRectangularObstacle(hubWidth, hubWidth,
                new Pose2d(FieldConstants.RED_HUB_CENTER, Rotation2d.kZero));
        }
    }
}
