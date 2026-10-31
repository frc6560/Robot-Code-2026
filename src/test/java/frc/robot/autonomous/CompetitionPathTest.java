package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.lib.BLine.Path;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class CompetitionPathTest {
    @BeforeAll static void initialize() { assertTrue(HAL.initialize(500, 0)); }

    @Test void allRedTargetsAreHalfTurnRotationsWithEventsAndConstraintsPreserved() {
        for (String name : new String[] {"hub", "human", "non-human"}) {
            Path blue = AutoCommands.competitionPath(name, false);
            Path red = AutoCommands.competitionPath(name, true);
            assertEquals(blue.getPathElements().size(), red.getPathElements().size());
            for (int i = 0; i < blue.getPathElements().size(); i++) {
                assertRotated(blue.getElement(i), red.getElement(i));
            }
            var blueResolved = blue.getPathElementsWithConstraints();
            var redResolved = red.getPathElementsWithConstraints();
            assertEquals(blueResolved.size(), redResolved.size());
            for (int i = 0; i < blueResolved.size(); i++) {
                assertEquals(blueResolved.get(i).getSecond(), redResolved.get(i).getSecond());
            }
            // Loading red must never mutate the source blue geometry.
            assertEquals(new Path(name).getPathElements(), blue.getPathElements());
        }
    }

    @Test void competitionPathsDisableProfiledRotationAndUsePointThreeHandoff() {
        for (String name : new String[] {"hub", "human", "non-human"}) {
            for (boolean red : new boolean[] {false, true}) {
                for (var element : AutoCommands.competitionPath(name, red).getPathElements()) {
                    if (element instanceof Path.Waypoint waypoint) {
                        assertFalse(waypoint.rotationTarget().profiledRotation());
                        assertEquals(0.3, waypoint.translationTarget().intermediateHandoffRadiusMeters().orElseThrow());
                    } else if (element instanceof Path.RotationTarget target) {
                        assertFalse(target.profiledRotation());
                    } else if (element instanceof Path.TranslationTarget target) {
                        assertEquals(0.3, target.intermediateHandoffRadiusMeters().orElseThrow());
                    }
                }
            }
        }
    }

    private void assertRotated(Path.PathElement blue, Path.PathElement red) {
        if (blue instanceof Path.Waypoint b) {
            assertInstanceOf(Path.Waypoint.class, red);
            var r = (Path.Waypoint) red;
            assertRotated(b.translationTarget(), r.translationTarget());
            assertRotated(b.rotationTarget(), r.rotationTarget());
        } else if (blue instanceof Path.TranslationTarget b) {
            assertInstanceOf(Path.TranslationTarget.class, red);
            var r = (Path.TranslationTarget) red;
            assertEquals(FieldConstants.FIELD_LENGTH - b.translation().getX(), r.translation().getX(), 1e-9);
            assertEquals(FieldConstants.FIELD_WIDTH - b.translation().getY(), r.translation().getY(), 1e-9);
            assertEquals(b.intermediateHandoffRadiusMeters(), r.intermediateHandoffRadiusMeters());
        } else if (blue instanceof Path.RotationTarget b) {
            assertInstanceOf(Path.RotationTarget.class, red);
            var r = (Path.RotationTarget) red;
            assertEquals(0, MathUtil.angleModulus(r.rotation().getRadians() - b.rotation().getRadians() - Math.PI), 1e-9);
            assertEquals(b.t_ratio(), r.t_ratio());
            assertEquals(b.profiledRotation(), r.profiledRotation());
        } else {
            assertEquals(blue, red); // Events retain command, segment position and order.
        }
    }
}
