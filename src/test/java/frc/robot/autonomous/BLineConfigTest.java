package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.BLineConstants;
import frc.robot.lib.BLine.JsonUtils;
import frc.robot.lib.BLine.Path;
import org.junit.jupiter.api.Test;

class BLineConfigTest {
    @Test
    void javaDefaultsAndExportedPathsUseTheSameConfig() {
        var config = JsonUtils.loadGlobalConstraints(JsonUtils.PROJECT_ROOT);
        assertEquals(config.getMaxVelocityMetersPerSec(), BLineConstants.GLOBAL_MAX_VELOCITY_MPS);
        assertEquals(config.getMaxAccelerationMetersPerSec2(), BLineConstants.GLOBAL_MAX_ACCELERATION_MPS2);
        assertEquals(config.getMaxVelocityDegPerSec(), BLineConstants.GLOBAL_MAX_ANGULAR_VELOCITY_DEG_PER_SEC);
        assertEquals(config.getMaxAccelerationDegPerSec2(), BLineConstants.GLOBAL_MAX_ANGULAR_ACCELERATION_DEG_PER_SEC2);
        assertEquals(config.getEndTranslationToleranceMeters(), BLineConstants.END_TRANSLATION_TOLERANCE_METERS);
        assertEquals(config.getEndRotationToleranceDeg(), BLineConstants.END_ROTATION_TOLERANCE_DEGREES);
        assertEquals(config.getIntermediateHandoffRadiusMeters(), BLineConstants.INTERMEDIATE_HANDOFF_RADIUS_METERS);
        var exported = new Path("zigzag").getDefaultGlobalConstraints();
        assertEquals(config.getMaxVelocityDegPerSec(), exported.getMaxVelocityDegPerSec());
        assertEquals(config.getMaxAccelerationDegPerSec2(), exported.getMaxAccelerationDegPerSec2());
    }

    @Test
    void explicitSegmentLimitsStillOverrideSharedDefaults() {
        var path = new Path(
            new Path.PathConstraints().setMaxVelocityMetersPerSec(1.25),
            new Path.Waypoint(0, 0, new Rotation2d()),
            new Path.Waypoint(1, 0, new Rotation2d()));
        var constraint = (Path.WaypointConstraint)
            path.getPathElementsWithConstraints().get(1).getSecond();
        assertEquals(1.25, constraint.maxVelocityMetersPerSec());
        assertEquals(BLineConstants.GLOBAL_MAX_ANGULAR_VELOCITY_DEG_PER_SEC,
            constraint.maxVelocityDegPerSec());
    }
}
