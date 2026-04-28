package frc.robot.utility.Pathing.obstacle;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.PriorityQueue;

/**
 * Grid A* over an {@link ObstacleField}. Returns a thinned list of world-space waypoints
 * that {@link frc.robot.utility.Pathing.PathCalculator} can stitch into Bezier segments.
 *
 * <p>8-connected, octile distance heuristic (admissible and consistent on 8-grids).
 * Corner-cutting through a diagonal between two blocked cells is disallowed.
 */
public class AStarPlanner {
    private static final int[] DX = { 1, -1,  0,  0,  1,  1, -1, -1};
    private static final int[] DY = { 0,  0,  1, -1,  1, -1,  1, -1};
    // cost per step; diagonals = sqrt(2)
    private static final double[] STEP = { 1, 1, 1, 1, Math.sqrt(2), Math.sqrt(2), Math.sqrt(2), Math.sqrt(2)};

    private final ObstacleField field;

    public AStarPlanner(ObstacleField field) {
        this.field = field;
    }

    /** Plan from {@code start} to {@code goal}. Returns an empty list if unreachable; returns
     * just [start, goal] if the straight line is already clear. */
    public List<Translation2d> plan(Translation2d start, Translation2d goal) {
        if (!field.isSegmentBlocked(start, goal)) {
            return Arrays.asList(start, goal);
        }

        int sc = field.toCol(start.getX());
        int sr = field.toRow(start.getY());
        int gc = field.toCol(goal.getX());
        int gr = field.toRow(goal.getY());

        // If the goal cell itself is blocked, nudge to the nearest free cell — otherwise
        // there's nothing A* can do.
        if (field.isBlocked(gc, gr)) {
            int[] nudged = nearestFree(gc, gr);
            if (nudged == null) return List.of();
            gc = nudged[0]; gr = nudged[1];
        }
        if (field.isBlocked(sc, sr)) {
            int[] nudged = nearestFree(sc, sr);
            if (nudged == null) return List.of();
            sc = nudged[0]; sr = nudged[1];
        }

        int cols = field.getCols();
        int rows = field.getRows();
        double[] g = new double[cols * rows];
        int[] parent = new int[cols * rows];
        Arrays.fill(g, Double.POSITIVE_INFINITY);
        Arrays.fill(parent, -1);

        int startIdx = sr * cols + sc;
        int goalIdx = gr * cols + gc;
        g[startIdx] = 0.0;

        PriorityQueue<long[]> open = new PriorityQueue<>((a, b) -> Double.compare(
                Double.longBitsToDouble(a[0]), Double.longBitsToDouble(b[0])));
        open.add(new long[]{Double.doubleToLongBits(octile(sc, sr, gc, gr)), startIdx});

        while (!open.isEmpty()) {
            long[] top = open.poll();
            int idx = (int) top[1];
            if (idx == goalIdx) break;

            int cx = idx % cols;
            int cy = idx / cols;
            double gCur = g[idx];

            for (int i = 0; i < 8; i++) {
                int nx = cx + DX[i];
                int ny = cy + DY[i];
                if (!field.inBounds(nx, ny) || field.isBlocked(nx, ny)) continue;
                // Prevent corner-cutting between two blocked orthogonal neighbors.
                if (i >= 4 && (field.isBlocked(cx + DX[i], cy) || field.isBlocked(cx, cy + DY[i]))) continue;

                int nIdx = ny * cols + nx;
                double tentative = gCur + STEP[i] * field.getResolution();
                if (tentative < g[nIdx]) {
                    g[nIdx] = tentative;
                    parent[nIdx] = idx;
                    double f = tentative + octile(nx, ny, gc, gr) * field.getResolution();
                    open.add(new long[]{Double.doubleToLongBits(f), nIdx});
                }
            }
        }

        if (parent[goalIdx] == -1 && startIdx != goalIdx) return List.of();

        // Reconstruct
        ArrayList<Integer> cellPath = new ArrayList<>();
        for (int cur = goalIdx; cur != -1; cur = parent[cur]) {
            cellPath.add(cur);
            if (cur == startIdx) break;
        }
        java.util.Collections.reverse(cellPath);

        ArrayList<Translation2d> world = new ArrayList<>();
        world.add(start);
        // Thin via line-of-sight: keep the last reachable waypoint in world space.
        int anchor = 0;
        for (int i = 2; i < cellPath.size(); i++) {
            Translation2d a = cellOf(cellPath.get(anchor));
            Translation2d b = cellOf(cellPath.get(i));
            if (field.isSegmentBlocked(a, b)) {
                world.add(cellOf(cellPath.get(i - 1)));
                anchor = i - 1;
            }
        }
        world.add(goal);
        return world;
    }

    private Translation2d cellOf(int idx) {
        int c = idx % field.getCols();
        int r = idx / field.getCols();
        return field.cellCenter(c, r);
    }

    private int[] nearestFree(int c, int r) {
        int maxR = Math.max(field.getCols(), field.getRows());
        for (int radius = 1; radius < maxR; radius++) {
            for (int dr = -radius; dr <= radius; dr++) {
                for (int dc = -radius; dc <= radius; dc++) {
                    if (Math.abs(dr) != radius && Math.abs(dc) != radius) continue;
                    int nc = c + dc, nr = r + dr;
                    if (field.inBounds(nc, nr) && !field.isBlocked(nc, nr)) {
                        return new int[]{nc, nr};
                    }
                }
            }
        }
        return null;
    }

    private static double octile(int ax, int ay, int bx, int by) {
        int dx = Math.abs(ax - bx);
        int dy = Math.abs(ay - by);
        return (dx + dy) + (Math.sqrt(2) - 2) * Math.min(dx, dy);
    }
}
