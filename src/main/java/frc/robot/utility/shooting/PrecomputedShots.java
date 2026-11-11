package frc.robot.utility.shooting;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.ShotModelConstants;
import frc.robot.utility.shooting.FuelShotModel.Flight;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.lang.reflect.Modifier;

/** Offline-generated 5 RPM grid; the 10 RPM solver's band midpoints also lie on this grid. */
public final class PrecomputedShots {
    private PrecomputedShots() {}

    static String signature() {
        StringBuilder result = new StringBuilder("rk4-v1");
        for (Class<?> type : new Class<?>[] {ShotModelConstants.class, FuelShotConfig.class, LaunchSpeedTuning.class}) {
            Arrays.stream(type.getDeclaredFields()).filter(f -> Modifier.isStatic(f.getModifiers())
                && Modifier.isFinal(f.getModifiers()) && f.getType() == double.class)
                .sorted(java.util.Comparator.comparing(java.lang.reflect.Field::getName)).forEach(f -> {
                    try { result.append('|').append(f.getName()).append('=').append(f.getDouble(null)); }
                    catch (IllegalAccessException e) { throw new IllegalStateException(e); }
                });
        }
        var calibration = LaunchCalibration.ideal();
        result.append('|').append(calibration.minRPM()).append('|').append(calibration.maxRPM());
        return result.toString();
    }

    public static FuelShotModel load() {
        return load(Filesystem.getDeployDirectory().toPath().resolve("shooting/trajectories.csv"));
    }

    public static FuelShotModel load(Path path) {
        try (var reader = Files.newBufferedReader(path)) {
            if (!signature().equals(reader.readLine()))
                throw new IllegalStateException("Shot table is stale; run generateShotTable before deployment");
            var calibration = LaunchCalibration.ideal();
            int count = (int) ((calibration.maxRPM() - calibration.minRPM()) / 5) + 1;
            Flight[] flights = new Flight[count];
            for (int i = 0; i < count; i++) {
                String line = reader.readLine();
                if (line == null) throw new IllegalArgumentException("Truncated shot table");
                String[] cells = line.split(",");
                if (cells.length != 6 || Double.parseDouble(cells[0]) != calibration.minRPM() + i * 5)
                    throw new IllegalArgumentException("Invalid shot grid");
                flights[i] = new Flight(Double.parseDouble(cells[1]), Double.parseDouble(cells[2]),
                    Double.parseDouble(cells[3]), Double.parseDouble(cells[4]), Boolean.parseBoolean(cells[5]));
            }
            if (reader.readLine() != null) throw new IllegalArgumentException("Extra shot rows");
            return new FuelShotModel(calibration, flights);
        } catch (java.io.IOException e) {
            throw new IllegalStateException("Cannot load precomputed shot table", e);
        }
    }

    /** Desktop only: invoke with Gradle generateShotTable after changing physics constants. */
    public static void main(String[] args) throws Exception {
        Path path = Path.of("src/main/deploy/shooting/trajectories.csv");
        Files.createDirectories(path.getParent());
        var model = new FuelShotModel(LaunchCalibration.ideal());
        try (var writer = Files.newBufferedWriter(path)) {
            writer.write(signature());
            writer.newLine();
            for (double rpm = model.calibration().minRPM(); rpm <= model.calibration().maxRPM(); rpm += 5) {
                Flight f = model.flight(rpm);
                writer.write(rpm + "," + f.crossingX() + "," + f.entryDegrees() + "," + f.time()
                    + "," + f.risingX() + "," + f.reached());
                writer.newLine();
            }
        }
    }
}
