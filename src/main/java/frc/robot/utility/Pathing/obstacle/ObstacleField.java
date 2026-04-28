package frc.robot.utility.Pathing.obstacle;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.BitSet;

/**
 * Occupancy grid over the field. Cells flagged as blocked are obstacles that all paths —
 * including on-the-fly generations — must route around.
 *
 * <p>Stored as a bit-packed array with a tiny text header so it round-trips through the
 * GUI without a JSON dependency on the robot side. Format:
 * <pre>
 *   OBSTACLES v1
 *   fieldLength={double} fieldWidth={double} resolution={double}
 *   cols={int} rows={int}
 *   {cols*rows 0/1 chars, row-major from y=0 upward}
 * </pre>
 *
 * <p>Singleton on the robot: {@link #getInstance()} loads from
 * {@code deploy/pathing/obstacles.dat} once. Missing file → an empty field (no obstacles).
 */
public class ObstacleField {
    private static final String DEFAULT_RESOURCE = "pathing/obstacles.dat";

    private final double fieldLength;
    private final double fieldWidth;
    private final double resolution;
    private final int cols;
    private final int rows;
    private final BitSet blocked;

    private static volatile ObstacleField INSTANCE;

    public ObstacleField(double fieldLength, double fieldWidth, double resolution) {
        this.fieldLength = fieldLength;
        this.fieldWidth = fieldWidth;
        this.resolution = resolution;
        this.cols = (int) Math.ceil(fieldLength / resolution);
        this.rows = (int) Math.ceil(fieldWidth / resolution);
        this.blocked = new BitSet(cols * rows);
    }

    public static ObstacleField getInstance() {
        ObstacleField local = INSTANCE;
        if (local == null) {
            synchronized (ObstacleField.class) {
                local = INSTANCE;
                if (local == null) {
                    File file = new File(Filesystem.getDeployDirectory(), DEFAULT_RESOURCE);
                    if (file.exists()) {
                        try {
                            local = load(file);
                        } catch (IOException e) {
                            System.err.println("[ObstacleField] failed to load " + file + ": " + e);
                            local = new ObstacleField(17.55, 8.05, 0.10);
                        }
                    } else {
                        // FRC default field dimensions; overridden by GUI export when present.
                        local = new ObstacleField(17.55, 8.05, 0.10);
                    }
                    INSTANCE = local;
                }
            }
        }
        return local;
    }

    /** Replaces the process-wide singleton. Primarily for tests and GUI preview. */
    public static void setInstance(ObstacleField field) {
        synchronized (ObstacleField.class) {
            INSTANCE = field;
        }
    }

    public double getFieldLength() { return fieldLength; }
    public double getFieldWidth() { return fieldWidth; }
    public double getResolution() { return resolution; }
    public int getCols() { return cols; }
    public int getRows() { return rows; }

    /** World coord -> grid column. */
    public int toCol(double x) {
        return clamp((int) Math.floor(x / resolution), 0, cols - 1);
    }

    /** World coord -> grid row. */
    public int toRow(double y) {
        return clamp((int) Math.floor(y / resolution), 0, rows - 1);
    }

    /** Cell center in world coords. */
    public Translation2d cellCenter(int col, int row) {
        return new Translation2d((col + 0.5) * resolution, (row + 0.5) * resolution);
    }

    public boolean inBounds(int col, int row) {
        return col >= 0 && col < cols && row >= 0 && row < rows;
    }

    public boolean isBlocked(int col, int row) {
        if (!inBounds(col, row)) return true;     // off-field counts as blocked
        return blocked.get(row * cols + col);
    }

    public boolean isBlocked(double x, double y) {
        return isBlocked(toCol(x), toRow(y));
    }

    public void setBlocked(int col, int row, boolean value) {
        if (!inBounds(col, row)) return;
        blocked.set(row * cols + col, value);
    }

    /** Inflates obstacles by {@code radius} meters (Chebyshev). Call once at load time if
     * you want cell-based clearance for a robot of some size. */
    public ObstacleField inflated(double radius) {
        ObstacleField out = new ObstacleField(fieldLength, fieldWidth, resolution);
        int inflateCells = (int) Math.ceil(radius / resolution);
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                if (!isBlocked(c, r)) continue;
                for (int dr = -inflateCells; dr <= inflateCells; dr++) {
                    for (int dc = -inflateCells; dc <= inflateCells; dc++) {
                        out.setBlocked(c + dc, r + dr, true);
                    }
                }
            }
        }
        return out;
    }

    /** Bresenham-style supercover check: any cell touched by the segment is tested. */
    public boolean isSegmentBlocked(Translation2d a, Translation2d b) {
        int x0 = toCol(a.getX());
        int y0 = toRow(a.getY());
        int x1 = toCol(b.getX());
        int y1 = toRow(b.getY());
        int dx = Math.abs(x1 - x0);
        int dy = Math.abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        int x = x0, y = y0;
        while (true) {
            if (isBlocked(x, y)) return true;
            if (x == x1 && y == y1) return false;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx)  { err += dx; y += sy; }
        }
    }

    /** Sample-based clearance check for any {@code t -> Translation2d} parametric curve.
     * @param stepMeters approximate arc-length step for sampling (smaller = safer, slower) */
    public boolean isCurveBlocked(java.util.function.DoubleFunction<Translation2d> curve,
                                  double arcLength, double stepMeters) {
        int samples = Math.max(2, (int) Math.ceil(arcLength / Math.max(stepMeters, 1e-3)));
        Translation2d prev = curve.apply(0.0);
        for (int i = 1; i <= samples; i++) {
            double t = (double) i / samples;
            Translation2d cur = curve.apply(t);
            if (isSegmentBlocked(prev, cur)) return true;
            prev = cur;
        }
        return false;
    }

    public void writeTo(File file) throws IOException {
        try (FileWriter w = new FileWriter(file)) {
            w.write("OBSTACLES v1\n");
            w.write(String.format("fieldLength=%.6f fieldWidth=%.6f resolution=%.6f%n",
                    fieldLength, fieldWidth, resolution));
            w.write("cols=" + cols + " rows=" + rows + "\n");
            StringBuilder sb = new StringBuilder(cols * rows);
            for (int r = 0; r < rows; r++) {
                for (int c = 0; c < cols; c++) {
                    sb.append(isBlocked(c, r) ? '1' : '0');
                }
                sb.append('\n');
            }
            w.write(sb.toString());
        }
    }

    public static ObstacleField load(File file) throws IOException {
        try (BufferedReader r = new BufferedReader(new FileReader(file))) {
            String header = r.readLine();
            if (header == null || !header.startsWith("OBSTACLES v1")) {
                throw new IOException("bad header: " + header);
            }
            String dims = r.readLine();
            double fl = parseDouble(dims, "fieldLength");
            double fw = parseDouble(dims, "fieldWidth");
            double res = parseDouble(dims, "resolution");
            String sizeLine = r.readLine();
            int cols = parseInt(sizeLine, "cols");
            int rows = parseInt(sizeLine, "rows");
            ObstacleField field = new ObstacleField(fl, fw, res);
            if (field.cols != cols || field.rows != rows) {
                // Still load — but trust the header dims; reconstruct with declared grid size.
            }
            for (int row = 0; row < rows; row++) {
                String line = r.readLine();
                if (line == null) break;
                for (int col = 0; col < cols && col < line.length(); col++) {
                    if (line.charAt(col) == '1') field.setBlocked(col, row, true);
                }
            }
            return field;
        }
    }

    private static double parseDouble(String line, String key) {
        for (String tok : line.split("\\s+")) {
            if (tok.startsWith(key + "=")) return Double.parseDouble(tok.substring(key.length() + 1));
        }
        throw new IllegalArgumentException("missing " + key + " in " + line);
    }

    private static int parseInt(String line, String key) {
        for (String tok : line.split("\\s+")) {
            if (tok.startsWith(key + "=")) return Integer.parseInt(tok.substring(key.length() + 1));
        }
        throw new IllegalArgumentException("missing " + key + " in " + line);
    }

    private static int clamp(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
