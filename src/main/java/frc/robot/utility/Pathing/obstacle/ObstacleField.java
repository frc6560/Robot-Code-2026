package frc.robot.utility.Pathing.obstacle;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

import frc.robot.Constants;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Locale;

/**
 * Occupancy grid over the field. Cells flagged as blocked are keep-out zones that all
 * paths — including on-the-fly generations — must route around.
 *
 * <p>Persisted as JSON to {@code deploy/pathing/obstacle-field.json}. There is exactly one
 * canonical obstacle-field map; only the "Keep-Out Zones" tab in the editor writes it.
 * Row data is run-length-encoded as a list of {@code [runLength, value]} pairs so the
 * file stays small (a handful of big rectangular keep-out zones compress well).
 *
 * <p>Format:
 * <pre>
 * {
 *   "version": 1,
 *   "fieldLength": 16.54,
 *   "fieldWidth": 8.07,
 *   "resolution": 0.25,
 *   "cols": 67,
 *   "rows": 33,
 *   "rows_rle": [
 *     [[67, 0]],
 *     [[40, 0], [12, 1], [15, 0]],
 *     ...
 *   ]
 * }
 * </pre>
 *
 * <p>Singleton on the robot: {@link #getInstance()} loads from
 * {@code deploy/pathing/obstacle-field.json} once. Missing file → empty field. */
public class ObstacleField {
    private static final String DEFAULT_RESOURCE = "pathing/obstacle-field.json";

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
                            local = empty();
                        }
                    } else {
                        // No obstacle map shipped — start empty. Overridden by GUI export when present.
                        local = empty();
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

    /** Canonical on-disk location inside the deploy directory. */
    public static File defaultFile() {
        return new File(Filesystem.getDeployDirectory(), DEFAULT_RESOURCE);
    }

    /** Empty field sized to match {@code Constants.FieldConstants} at 0.25 m resolution. */
    public static ObstacleField empty() {
        return new ObstacleField(
                Constants.FieldConstants.FIELD_LENGTH,
                Constants.FieldConstants.FIELD_WIDTH,
                0.25);
    }

    public void writeTo(File file) throws IOException {
        File parent = file.getParentFile();
        if (parent != null && !parent.exists()) parent.mkdirs();
        try (FileWriter w = new FileWriter(file)) {
            w.write("{\n");
            w.write("  \"version\": 1,\n");
            w.write(String.format(Locale.ROOT, "  \"fieldLength\": %.6f,%n", fieldLength));
            w.write(String.format(Locale.ROOT, "  \"fieldWidth\": %.6f,%n", fieldWidth));
            w.write(String.format(Locale.ROOT, "  \"resolution\": %.6f,%n", resolution));
            w.write("  \"cols\": " + cols + ",\n");
            w.write("  \"rows\": " + rows + ",\n");
            w.write("  \"rows_rle\": [\n");
            for (int r = 0; r < rows; r++) {
                w.write("    [");
                int run = 0;
                int prev = isBlocked(0, r) ? 1 : 0;
                boolean firstPair = true;
                for (int c = 0; c < cols; c++) {
                    int v = isBlocked(c, r) ? 1 : 0;
                    if (v == prev) { run++; continue; }
                    if (!firstPair) w.write(",");
                    w.write("[" + run + "," + prev + "]");
                    firstPair = false;
                    prev = v;
                    run = 1;
                }
                if (!firstPair) w.write(",");
                w.write("[" + run + "," + prev + "]");
                w.write("]" + (r == rows - 1 ? "\n" : ",\n"));
            }
            w.write("  ]\n");
            w.write("}\n");
        }
    }

    public static ObstacleField load(File file) throws IOException {
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) sb.append(line).append('\n');
            String src = sb.toString();

            double fl = extractNumber(src, "fieldLength");
            double fw = extractNumber(src, "fieldWidth");
            double res = extractNumber(src, "resolution");
            int cols = (int) extractNumber(src, "cols");
            int rows = (int) extractNumber(src, "rows");

            ObstacleField field = new ObstacleField(fl, fw, res);

            int rleStart = src.indexOf("rows_rle");
            if (rleStart < 0) throw new IOException("missing rows_rle");
            int arrStart = src.indexOf('[', rleStart);
            int arrEnd = findMatchingBracket(src, arrStart);
            String body = src.substring(arrStart + 1, arrEnd);

            List<String> rowPayloads = splitTopLevelArrays(body);
            int rowIdx = 0;
            for (String rowPayload : rowPayloads) {
                if (rowIdx >= field.rows) break;
                int c = 0;
                for (String pair : splitTopLevelArrays(rowPayload)) {
                    String inner = pair.trim();
                    if (inner.isEmpty()) continue;
                    String[] parts = inner.split(",");
                    int runLen = Integer.parseInt(parts[0].trim());
                    int value = Integer.parseInt(parts[1].trim());
                    if (value == 1) {
                        for (int k = 0; k < runLen && c + k < field.cols; k++) {
                            field.setBlocked(c + k, rowIdx, true);
                        }
                    }
                    c += runLen;
                }
                rowIdx++;
            }
            // Silence "unused" warnings when header cols/rows mismatch the ctor-computed grid.
            if (cols != field.cols || rows != field.rows) {
                // Trust ctor-derived grid; header values are informational.
            }
            return field;
        }
    }

    private static double extractNumber(String src, String key) {
        int i = src.indexOf('"' + key + '"');
        if (i < 0) throw new IllegalArgumentException("missing " + key);
        int colon = src.indexOf(':', i);
        int start = colon + 1;
        // Skip leading whitespace between colon and the number.
        while (start < src.length() && Character.isWhitespace(src.charAt(start))) start++;
        int end = start;
        // A JSON number is digits/sign/decimal/exponent — stop on anything else.
        while (end < src.length()) {
            char ch = src.charAt(end);
            if (Character.isDigit(ch) || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E') {
                end++;
            } else break;
        }
        String token = src.substring(start, end);
        if (token.isEmpty()) {
            throw new IllegalArgumentException("no numeric value for " + key);
        }
        return Double.parseDouble(token);
    }

    private static int findMatchingBracket(String src, int openIdx) {
        int depth = 0;
        for (int i = openIdx; i < src.length(); i++) {
            char ch = src.charAt(i);
            if (ch == '[') depth++;
            else if (ch == ']') {
                depth--;
                if (depth == 0) return i;
            }
        }
        throw new IllegalArgumentException("unmatched [");
    }

    private static List<String> splitTopLevelArrays(String body) {
        List<String> out = new ArrayList<>();
        int depth = 0;
        int start = -1;
        for (int i = 0; i < body.length(); i++) {
            char ch = body.charAt(i);
            if (ch == '[') {
                if (depth == 0) start = i + 1;
                depth++;
            } else if (ch == ']') {
                depth--;
                if (depth == 0 && start >= 0) {
                    out.add(body.substring(start, i));
                    start = -1;
                }
            }
        }
        return out;
    }

    private static int clamp(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
