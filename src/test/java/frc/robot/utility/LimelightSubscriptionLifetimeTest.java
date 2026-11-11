package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.Test;

/** Native-backed regression for accidentally subscribing on every vision update. */
class LimelightSubscriptionLifetimeTest {
    @Test void repeatedPoseAndHeartbeatLookupsReuseObjectsAndNativeHandles() {
        String[] names = {"lifetime-test-front", "lifetime-test-left", "lifetime-test-right"};
        String[] topics = {"botpose_orb_wpiblue", "botpose_wpiblue"};
        DoubleArrayEntry[][] original = new DoubleArrayEntry[names.length][topics.length];
        int[][] handles = new int[names.length][topics.length];
        var heartbeat = new edu.wpi.first.networktables.NetworkTableEntry[names.length];
        for (int i = 0; i < names.length; i++) {
            heartbeat[i] = LimelightHelpers.getLimelightNTTableEntry(names[i], "hb");
            for (int j = 0; j < topics.length; j++) {
                original[i][j] = LimelightHelpers.getLimelightDoubleArrayEntry(names[i], topics[j]);
                handles[i][j] = original[i][j].getHandle();
                assertNotEquals(0, handles[i][j]);
            }
        }
        for (int iteration = 0; iteration < 10_000; iteration++) {
            for (int i = 0; i < names.length; i++) {
                assertSame(heartbeat[i], LimelightHelpers.getLimelightNTTableEntry(names[i], "hb"));
                for (int j = 0; j < topics.length; j++) {
                    var entry = LimelightHelpers.getLimelightDoubleArrayEntry(names[i], topics[j]);
                    assertSame(original[i][j], entry);
                    assertEquals(handles[i][j], entry.getHandle());
                }
            }
        }
        // Helpers own their cached entries for process lifetime; closing them here would
        // leave closed handles in that shared cache. No per-iteration resource is created.
    }

    @Test void explicitlySubscribingTwiceCreatesDistinctNativeHandles() {
        var topic = NetworkTableInstance.getDefault().getTable("lifetime-test").getDoubleTopic("tx");
        try (var first = topic.subscribe(0.0); var second = topic.subscribe(0.0)) {
            assertNotEquals(first.getHandle(), second.getHandle());
        }
    }
}
