package frc.robot.utility.Pathing.gui;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.Path;
import frc.robot.utility.Pathing.obstacle.ObstacleField;
import frc.robot.utility.Pathing.serialization.PathIO;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.File;

/**
 * Standalone Swing editor. Two tabs:
 * <ol>
 *   <li><b>Path Editor</b> — drag Bezier handles, save/load .path files. Shows the current
 *       obstacle map as a translucent overlay (read-only).</li>
 *   <li><b>Keep-Out Zones</b> — paint/erase occupancy cells with a grid visualization. This
 *       tab is the <i>only</i> writer of the canonical {@code obstacle-field.json}.</li>
 * </ol>
 *
 * <p>A background image of the 2026 FRC field is loaded from
 * {@code src/main/deploy/pathing/field-2026.png} when present. World coordinates assume
 * (0,0) at the bottom-left corner of the image, x along field length, y along field width.
 *
 * <p>Launch from an IDE run config targeting this main, or
 * {@code ./gradlew run -PmainClass=frc.robot.utility.Pathing.gui.PathGuiMain}. */
public class PathGuiMain extends JFrame {
    static final double FIELD_LENGTH = Constants.FieldConstants.FIELD_LENGTH;
    static final double FIELD_WIDTH = Constants.FieldConstants.FIELD_WIDTH;
    static final double OBSTACLE_RES = 0.25; // m per grid cell

    static final int MARGIN = 20;
    static final double PX_PER_M = 60.0;

    /** Single source of truth for the keep-out map within this JVM. The Path tab reads it;
     * the Keep-Out tab mutates it. Saved exactly once to the canonical JSON. */
    private ObstacleField obstacles = new ObstacleField(FIELD_LENGTH, FIELD_WIDTH, OBSTACLE_RES);

    private final BufferedImage fieldImage = loadFieldImage();

    private final PathTab pathTab = new PathTab();
    private final KeepOutTab keepOutTab = new KeepOutTab();

    public PathGuiMain() {
        super("FRC Path Editor");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        attemptAutoLoadObstacles();

        JTabbedPane tabs = new JTabbedPane();
        tabs.addTab("Path Editor", pathTab);
        tabs.addTab("Keep-Out Zones", keepOutTab);
        setContentPane(tabs);

        // Refresh whichever tab just became visible (obstacle map may have changed).
        tabs.addChangeListener(e -> {
            pathTab.repaint();
            keepOutTab.repaint();
        });

        pack();
        setLocationRelativeTo(null);
    }

    private void attemptAutoLoadObstacles() {
        File f = new File(defaultPathingDir(), "obstacle-field.json");
        if (f.exists()) {
            try { obstacles = ObstacleField.load(f); }
            catch (Exception ex) { System.err.println("[PathGui] load obstacle-field.json failed: " + ex); }
        }
    }

    private BufferedImage loadFieldImage() {
        File f = new File(defaultPathingDir(), "field-2026.png");
        if (!f.exists()) return null;
        try { return ImageIO.read(f); }
        catch (Exception ex) { System.err.println("[PathGui] load field image failed: " + ex); return null; }
    }

    static File defaultPathingDir() {
        File d = new File("src/main/deploy/pathing");
        if (!d.exists()) d.mkdirs();
        return d;
    }

    static Point worldToScreen(Translation2d w) {
        int x = MARGIN + (int)(w.getX() * PX_PER_M);
        int y = MARGIN + (int)((FIELD_WIDTH - w.getY()) * PX_PER_M);
        return new Point(x, y);
    }

    static Translation2d screenToWorld(int sx, int sy) {
        double x = (sx - MARGIN) / PX_PER_M;
        double y = FIELD_WIDTH - (sy - MARGIN) / PX_PER_M;
        return new Translation2d(
                Math.max(0, Math.min(FIELD_LENGTH, x)),
                Math.max(0, Math.min(FIELD_WIDTH, y)));
    }

    static void drawFieldBackground(Graphics2D g, BufferedImage fieldImage) {
        Point tl = worldToScreen(new Translation2d(0, FIELD_WIDTH));
        Point br = worldToScreen(new Translation2d(FIELD_LENGTH, 0));
        int w = br.x - tl.x, h = br.y - tl.y;
        if (fieldImage != null) {
            g.drawImage(fieldImage, tl.x, tl.y, w, h, null);
        } else {
            g.setColor(new Color(45, 50, 58));
            g.fillRect(tl.x, tl.y, w, h);
        }
        g.setColor(new Color(90, 95, 105));
        g.drawRect(tl.x, tl.y, w, h);
    }

    /** Draw obstacles. Call with small alpha for read-only preview on the path tab. */
    static void drawObstacles(Graphics2D g, ObstacleField field, int alpha) {
        g.setColor(new Color(200, 60, 60, alpha));
        double res = field.getResolution();
        for (int r = 0; r < field.getRows(); r++) {
            for (int c = 0; c < field.getCols(); c++) {
                if (!field.isBlocked(c, r)) continue;
                Point p = worldToScreen(new Translation2d(c * res, (r + 1) * res));
                int w = (int) Math.ceil(res * PX_PER_M);
                int h = (int) Math.ceil(res * PX_PER_M);
                g.fillRect(p.x, p.y, w, h);
            }
        }
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new PathGuiMain().setVisible(true));
    }

    // -- Path Tab -----------------------------------------------------------------

    private final class PathTab extends JPanel {
        private static final int HANDLE_PX = 10;
        private static final int PLOT_HEIGHT = 160;

        private Translation2d startPt   = new Translation2d(2.0, 4.0);
        private Translation2d endPt     = new Translation2d(10.0, 4.0);
        private Translation2d startCtrl = new Translation2d(4.0, 4.0);
        private Translation2d endCtrl   = new Translation2d(8.0, 4.0);

        private final double maxVel = 5.0;
        private final double maxAt = 4.0;
        private final double maxOmega = Math.PI;
        private final double maxAlpha = 2 * Math.PI;
        private final double maxCentripetal = 3.0;

        private Translation2d dragging = null;
        private final FieldCanvas canvas = new FieldCanvas();

        PathTab() {
            setLayout(new BorderLayout());
            JToolBar bar = new JToolBar();
            bar.setFloatable(false);
            bar.add(new AbstractAction("Save Path…") {
                public void actionPerformed(ActionEvent e) { savePath(); }
            });
            bar.add(new AbstractAction("Load Path…") {
                public void actionPerformed(ActionEvent e) { loadPath(); }
            });
            add(bar, BorderLayout.NORTH);
            add(canvas, BorderLayout.CENTER);
        }

        private void savePath() {
            JFileChooser chooser = new JFileChooser(defaultPathingDir());
            if (chooser.showSaveDialog(this) != JFileChooser.APPROVE_OPTION) return;
            try {
                PathIO.write(buildPath(), chooser.getSelectedFile(),
                        maxVel, maxAt, maxOmega, maxAlpha, maxCentripetal);
            } catch (Exception ex) { error("save path", ex); }
        }

        private void loadPath() {
            JFileChooser chooser = new JFileChooser(defaultPathingDir());
            if (chooser.showOpenDialog(this) != JFileChooser.APPROVE_OPTION) return;
            try {
                Path p = PathIO.read(chooser.getSelectedFile());
                startPt = p.getStartPose().getTranslation();
                endPt = p.getEndPose().getTranslation();
                startCtrl = p.getStartControlHeading().getTranslation();
                endCtrl = p.getEndControlHeading().getTranslation();
                repaint();
            } catch (Exception ex) { error("load path", ex); }
        }

        private Path buildPath() {
            Setpoint start = new Setpoint(startPt.getX(), startPt.getY(), 0, 0, 0, 0);
            Setpoint end = new Setpoint(endPt.getX(), endPt.getY(), 0, 0, 0, 0);
            Pose2d startCtrlPose = new Pose2d(startCtrl, new Rotation2d(0));
            Pose2d endCtrlPose = new Pose2d(endCtrl, new Rotation2d(0));
            return new Path(start, end, startCtrlPose, endCtrlPose,
                    maxVel, maxAt, maxOmega, maxAlpha, maxCentripetal);
        }

        private final class FieldCanvas extends JPanel {
            FieldCanvas() {
                setPreferredSize(new Dimension(
                        (int)(FIELD_LENGTH * PX_PER_M) + 2 * MARGIN,
                        (int)(FIELD_WIDTH  * PX_PER_M) + 2 * MARGIN + PLOT_HEIGHT));
                setBackground(new Color(25, 25, 28));

                MouseAdapter h = new MouseAdapter() {
                    @Override public void mousePressed(MouseEvent e) {
                        dragging = nearestHandle(e.getX(), e.getY());
                    }
                    @Override public void mouseReleased(MouseEvent e) { dragging = null; }
                    @Override public void mouseDragged(MouseEvent e) {
                        if (dragging == null) return;
                        Translation2d w = screenToWorld(e.getX(), e.getY());
                        if (dragging == startPt)        startPt = w;
                        else if (dragging == endPt)     endPt = w;
                        else if (dragging == startCtrl) startCtrl = w;
                        else if (dragging == endCtrl)   endCtrl = w;
                        dragging = w;
                        repaint();
                    }
                };
                addMouseListener(h); addMouseMotionListener(h);
            }

            private Translation2d nearestHandle(int sx, int sy) {
                Translation2d best = null; double bestD = 15 * 15;
                Translation2d[] all = { startPt, endPt, startCtrl, endCtrl };
                for (Translation2d pt : all) {
                    Point p = worldToScreen(pt);
                    double d = (p.x - sx) * (p.x - sx) + (p.y - sy) * (p.y - sy);
                    if (d < bestD) { bestD = d; best = pt; }
                }
                return best;
            }

            @Override protected void paintComponent(Graphics g0) {
                super.paintComponent(g0);
                Graphics2D g = (Graphics2D) g0;
                g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

                drawFieldBackground(g, fieldImage);
                drawObstacles(g, obstacles, 90); // translucent preview only

                Path path;
                try { path = buildPath(); }
                catch (Exception ex) { return; }
                double arc = path.getArcLength();

                // Bezier (curvature-colored)
                int N = 200;
                Point prev = null;
                for (int i = 0; i <= N; i++) {
                    double t = (double) i / N;
                    Translation2d pos = path.calculatePosition(t);
                    double k = Math.abs(path.getCurvature(t));
                    float hue = (float) Math.min(0.65, k / 4.0);
                    g.setColor(Color.getHSBColor(0.33f - hue * 0.33f, 0.9f, 1.0f));
                    Point cur = worldToScreen(pos);
                    if (prev != null) g.drawLine(prev.x, prev.y, cur.x, cur.y);
                    prev = cur;
                }

                // Handles
                drawHandle(g, startPt,   new Color(60, 220, 110));
                drawHandle(g, endPt,     new Color(230, 80, 80));
                drawHandle(g, startCtrl, new Color(90, 180, 255));
                drawHandle(g, endCtrl,   new Color(255, 150, 220));
                g.setColor(new Color(180, 180, 180, 120));
                drawLine(g, startPt, startCtrl);
                drawLine(g, endPt, endCtrl);

                // v(s) plot
                int plotY0 = getHeight() - PLOT_HEIGHT + 10;
                g.setColor(new Color(35, 35, 40));
                g.fillRect(MARGIN, plotY0, getWidth() - 2 * MARGIN, PLOT_HEIGHT - 30);
                g.setColor(new Color(200, 200, 210));
                g.drawString(String.format("v(s)  arc=%.2f m   max=%.1f m/s", arc, maxVel),
                        MARGIN + 6, plotY0 + 14);
                int plotW = getWidth() - 2 * MARGIN;
                int plotH = PLOT_HEIGHT - 40;
                Point prevV = null;
                int samples = 200;
                for (int i = 0; i <= samples; i++) {
                    double s = arc * i / samples;
                    double v = path.getVelocityProfile().velocityAt(s);
                    int px = MARGIN + (int)(plotW * (double) i / samples);
                    int py = plotY0 + plotH - (int)(plotH * v / maxVel);
                    g.setColor(new Color(120, 220, 160));
                    if (prevV != null) g.drawLine(prevV.x, prevV.y, px, py);
                    prevV = new Point(px, py);
                }
            }

            private void drawHandle(Graphics2D g, Translation2d w, Color c) {
                Point p = worldToScreen(w);
                g.setColor(c);
                g.fillOval(p.x - HANDLE_PX / 2, p.y - HANDLE_PX / 2, HANDLE_PX, HANDLE_PX);
            }

            private void drawLine(Graphics2D g, Translation2d a, Translation2d b) {
                Point pa = worldToScreen(a); Point pb = worldToScreen(b);
                g.drawLine(pa.x, pa.y, pb.x, pb.y);
            }
        }
    }

    // -- Keep-Out Tab -------------------------------------------------------------

    private final class KeepOutTab extends JPanel {
        private boolean painting = false;
        private boolean erasing = false;
        private final FieldCanvas canvas = new FieldCanvas();

        KeepOutTab() {
            setLayout(new BorderLayout());
            JToolBar bar = new JToolBar();
            bar.setFloatable(false);
            bar.add(new JLabel("  Left-drag: paint cell   |   Right-drag: erase cell   "));
            bar.addSeparator();
            bar.add(new AbstractAction("Save obstacle-field.json") {
                public void actionPerformed(ActionEvent e) { saveCanonical(); }
            });
            bar.add(new AbstractAction("Reload from disk") {
                public void actionPerformed(ActionEvent e) { reloadCanonical(); }
            });
            bar.add(new AbstractAction("Clear") {
                public void actionPerformed(ActionEvent e) {
                    obstacles = new ObstacleField(FIELD_LENGTH, FIELD_WIDTH, OBSTACLE_RES);
                    repaint();
                }
            });
            add(bar, BorderLayout.NORTH);
            add(canvas, BorderLayout.CENTER);
        }

        private void saveCanonical() {
            File target = new File(defaultPathingDir(), "obstacle-field.json");
            try {
                obstacles.writeTo(target);
                JOptionPane.showMessageDialog(this, "Wrote " + target.getAbsolutePath());
            } catch (Exception ex) { error("save obstacle-field.json", ex); }
        }

        private void reloadCanonical() {
            File f = new File(defaultPathingDir(), "obstacle-field.json");
            if (!f.exists()) { error("reload", new RuntimeException("no obstacle-field.json")); return; }
            try { obstacles = ObstacleField.load(f); repaint(); }
            catch (Exception ex) { error("reload", ex); }
        }

        private final class FieldCanvas extends JPanel {
            FieldCanvas() {
                setPreferredSize(new Dimension(
                        (int)(FIELD_LENGTH * PX_PER_M) + 2 * MARGIN,
                        (int)(FIELD_WIDTH  * PX_PER_M) + 2 * MARGIN));
                setBackground(new Color(25, 25, 28));

                MouseAdapter h = new MouseAdapter() {
                    @Override public void mousePressed(MouseEvent e) {
                        painting = SwingUtilities.isLeftMouseButton(e);
                        erasing = SwingUtilities.isRightMouseButton(e);
                        toggle(e);
                    }
                    @Override public void mouseReleased(MouseEvent e) {
                        painting = false; erasing = false;
                    }
                    @Override public void mouseDragged(MouseEvent e) {
                        if (painting || erasing) toggle(e);
                    }
                };
                addMouseListener(h); addMouseMotionListener(h);
            }

            private void toggle(MouseEvent e) {
                Translation2d w = screenToWorld(e.getX(), e.getY());
                int c = obstacles.toCol(w.getX());
                int r = obstacles.toRow(w.getY());
                obstacles.setBlocked(c, r, painting && !erasing);
                repaint();
            }

            @Override protected void paintComponent(Graphics g0) {
                super.paintComponent(g0);
                Graphics2D g = (Graphics2D) g0;
                g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

                drawFieldBackground(g, fieldImage);

                // Grid overlay — the defining feature of this tab, so make it visible.
                g.setColor(new Color(255, 255, 255, 40));
                double res = obstacles.getResolution();
                for (int c = 0; c <= obstacles.getCols(); c++) {
                    Point a = worldToScreen(new Translation2d(c * res, 0));
                    Point b = worldToScreen(new Translation2d(c * res, FIELD_WIDTH));
                    g.drawLine(a.x, a.y, b.x, b.y);
                }
                for (int r = 0; r <= obstacles.getRows(); r++) {
                    Point a = worldToScreen(new Translation2d(0, r * res));
                    Point b = worldToScreen(new Translation2d(FIELD_LENGTH, r * res));
                    g.drawLine(a.x, a.y, b.x, b.y);
                }

                // Opaque obstacles — this is the authoritative editor view.
                drawObstacles(g, obstacles, 200);

                // Status
                g.setColor(new Color(220, 220, 230));
                int blockedCount = 0;
                for (int r = 0; r < obstacles.getRows(); r++) {
                    for (int c = 0; c < obstacles.getCols(); c++) {
                        if (obstacles.isBlocked(c, r)) blockedCount++;
                    }
                }
                g.drawString(String.format("grid: %dx%d  res=%.2f m  blocked cells=%d",
                        obstacles.getCols(), obstacles.getRows(), obstacles.getResolution(), blockedCount),
                        MARGIN, getHeight() - 8);
            }
        }
    }

    private void error(String what, Exception ex) {
        JOptionPane.showMessageDialog(this, "Failed to " + what + ": " + ex.getMessage(),
                "Error", JOptionPane.ERROR_MESSAGE);
    }
}
