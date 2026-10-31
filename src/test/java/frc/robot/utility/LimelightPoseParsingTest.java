package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.Test;

class LimelightPoseParsingTest {
    private double[] packet(double count, int length) {
        double[] data = new double[length];
        if (length >= 11) {
            data[0] = 3; data[1] = 4; data[5] = 90;
            data[6] = 20; data[7] = count; data[8] = 1; data[9] = 2; data[10] = 3;
        }
        return data;
    }
    @Test void rejectsMissingOrShortHeaders() {
        assertNull(LimelightHelpers.parsePoseEstimate(null, 0, true));
        for (int length = 0; length < 11; length++)
            assertNull(LimelightHelpers.parsePoseEstimate(new double[length], 0, true));
    }
    @Test void rejectsInvalidCountsBeforeAllocating() {
        for (double count : new double[] {-1, -0.5, 1.5, Double.NaN,
                Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.MAX_VALUE,
                Integer.MAX_VALUE, 100_000_000, LimelightHelpers.MAX_POSE_FIDUCIALS + 1}) {
            assertNull(LimelightHelpers.parsePoseEstimate(packet(count, 11), 0, true), "Count " + count);
        }
    }
    @Test void rejectsPartialExcessOrUnexpectedRawDetails() {
        for (int length : new int[] {12, 17, 18, 24, 26, 32})
            assertNull(LimelightHelpers.parsePoseEstimate(packet(2, length), 0, true), "Length " + length);
        assertNull(LimelightHelpers.parsePoseEstimate(packet(0, 18), 0, true));
    }
    @Test void preservesHeaderOnlyCountAndSharesEmptyDetails() {
        var first = LimelightHelpers.parsePoseEstimate(packet(2, 11), 1_000_000, true);
        var zero = LimelightHelpers.parsePoseEstimate(packet(0, 11), 1_000_000, false);
        assertEquals(2, first.tagCount);
        assertEquals(0, first.rawFiducials.length);
        assertSame(first.rawFiducials, zero.rawFiducials);
        assertEquals(0, zero.tagCount);
        assertTrue(first.isMegaTag2);
        assertFalse(zero.isMegaTag2);
        assertEquals(0.98, first.timestampSeconds, 1e-9);
        assertEquals(3, first.pose.getX());
        assertEquals(4, first.pose.getY());
        assertEquals(90, first.pose.getRotation().getDegrees(), 1e-9);
        assertEquals(20, first.latency);
        assertEquals(1, first.tagSpan);
        assertEquals(2, first.avgTagDist);
        assertEquals(3, first.avgTagArea);
    }
    @Test void preservesCompleteTagDetailsAndAcceptsUpperBoundary() {
        for (int count : new int[] {1, 2, LimelightHelpers.MAX_POSE_FIDUCIALS}) {
            double[] data = packet(count, 11 + count * 7);
            for (int i = 0; i < count; i++) {
                int base = 11 + i * 7;
                for (int j = 0; j < 7; j++) data[base + j] = i + j + 1;
            }
            var estimate = LimelightHelpers.parsePoseEstimate(data, 0, true);
            assertEquals(count, estimate.tagCount);
            assertEquals(count, estimate.rawFiducials.length);
            for (int i = 0; i < count; i++) {
                var tag = estimate.rawFiducials[i];
                assertEquals(i + 1, tag.id);
                assertEquals(i + 2, tag.txnc);
                assertEquals(i + 3, tag.tync);
                assertEquals(i + 4, tag.ta);
                assertEquals(i + 5, tag.distToCamera);
                assertEquals(i + 6, tag.distToRobot);
                assertEquals(i + 7, tag.ambiguity);
            }
        }
        assertNotNull(LimelightHelpers.parsePoseEstimate(packet(LimelightHelpers.MAX_POSE_FIDUCIALS, 11), 0, true));
    }
    @Test void publicMt1AndMt2ReadersRejectThePreviouslyReproducedOomPacket() {
        for (String topic : new String[] {"botpose_wpiblue", "botpose_orb_wpiblue"}) {
            var entry = NetworkTableInstance.getDefault().getTable("bounded-pose-parser").getEntry(topic);
            entry.setDoubleArray(packet(Integer.MAX_VALUE, 11));
            try {
                var estimate = topic.equals("botpose_wpiblue")
                    ? LimelightHelpers.getBotPoseEstimate_wpiBlue("bounded-pose-parser")
                    : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("bounded-pose-parser");
                assertNull(estimate);
            } finally { entry.unpublish(); entry.close(); }
        }
    }
}
