import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Locale;

/** Extracts saved AdvantageKit Swerve/Pose samples, including active-auto status. */
class ExtractBLineTrace {
  public static void main(String[] args) throws Exception {
    var reader = new DataLogReader(args[0]);
    if (!reader.isValid()) throw new IllegalArgumentException("Invalid WPILOG");
    var entries = new HashMap<Integer, String>();
    var csv = new StringBuilder("time_seconds,x_meters,y_meters,heading_radians,auto_active,target_heading_degrees,requested_omega_radps,measured_omega_radps,translation_element_index\n");
    double[] pose = null;
    long timestamp = -1;
    boolean active = false;
    double targetHeading = Double.NaN;
    double requestedOmega = Double.NaN;
    double measuredOmega = Double.NaN;
    double translationIndex = Double.NaN;
    int count = 0;
    for (DataLogRecord record : reader) {
      if (record.isStart()) {
        var start = record.getStartData();
        entries.put(start.entry, start.name);
      } else if (!record.isControl()) {
        String name = entries.getOrDefault(record.getEntry(), "");
        long currentTimestamp = record.getTimestamp();
        if (timestamp != currentTimestamp) {
          if (pose != null) {
            csv.append(String.format(Locale.ROOT, "%.6f,%.9f,%.9f,%.9f,%s,%.9f,%.9f,%.9f,%.0f%n", timestamp / 1e6, pose[0], pose[1], pose[2], active, targetHeading, requestedOmega, measuredOmega, translationIndex));
            count++;
          }
          timestamp = currentTimestamp;
          pose = null;
        }
        if (name.endsWith("/Swerve/Pose")) {
          var bytes = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
          if (bytes.remaining() != 24) throw new IllegalArgumentException("Unexpected Pose2d struct size");
          pose = new double[] { bytes.getDouble(), bytes.getDouble(), bytes.getDouble() };
        } else if (name.endsWith("/BLine/Diagnostic/AutoActive")) {
          active = record.getBoolean();
        } else if (name.endsWith("/BLine/FollowPath/targetRotationDeg")) {
          targetHeading = record.getDouble();
        } else if (name.endsWith("/BLine/Drive/RequestedOmegaRadPerSec")) {
          requestedOmega = record.getDouble();
        } else if (name.endsWith("/Swerve/RobotVelocity")) {
          var bytes = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
          if (bytes.remaining() == 24) measuredOmega = bytes.getDouble(16);
        } else if (name.endsWith("/BLine/FollowPath/translationElementIndex")) {
          translationIndex = record.getDouble();
        } else if (name.contains("/BLine/Tuning/")) {
          System.out.println(name + " = " + record.getDouble());
        }
      }
    }
    if (pose != null) {
      csv.append(String.format(Locale.ROOT, "%.6f,%.9f,%.9f,%.9f,%s,%.9f,%.9f,%.9f,%.0f%n", timestamp / 1e6, pose[0], pose[1], pose[2], active, targetHeading, requestedOmega, measuredOmega, translationIndex));
      count++;
    }
    Files.writeString(Path.of(args[1]), csv.toString());
    System.out.println("Extracted " + count + " saved pose updates");
  }
}
