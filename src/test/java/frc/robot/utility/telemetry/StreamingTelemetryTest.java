package frc.robot.utility.telemetry;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.networktables.NT4Publisher;

class StreamingTelemetryTest {
    @Test void livePublisherDeliversSnapshotsWithoutAFileReceiverAtTwoHertz() throws Exception {
        var receiver = new RateLimitedReceiver(new NT4Publisher());
        var entry = NetworkTableInstance.getDefault().getTable("AdvantageKit")
            .getEntry("RealOutputs/StreamingPolicyProbe");
        receiver.start();
        try {
            // NT timestamp 0 means "now", so use explicit increasing timestamps throughout.
            long base = edu.wpi.first.util.WPIUtilJNI.now() + 1_000_000;
            var first = new LogTable(base);
            first.getSubtable("RealOutputs").put("StreamingPolicyProbe", 1.0);
            receiver.putTable(first);
            assertEquals(1.0, entry.getDouble(-1));
            var suppressed = new LogTable(base + 20_000);
            suppressed.getSubtable("RealOutputs").put("StreamingPolicyProbe", 2.0);
            receiver.putTable(suppressed);
            assertEquals(1.0, entry.getDouble(-1));
            var next = new LogTable(base + 500_000);
            next.getSubtable("RealOutputs").put("StreamingPolicyProbe", 3.0);
            receiver.putTable(next);
            assertEquals(3.0, entry.getDouble(-1));
        } finally { receiver.end(); entry.close(); }
    }
}
