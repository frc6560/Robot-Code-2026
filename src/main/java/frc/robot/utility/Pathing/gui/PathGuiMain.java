package frc.robot.utility.Pathing.gui;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.Path;
import frc.robot.utility.Pathing.obstacle.ObstacleField;
import frc.robot.utility.Pathing.serialization.PathIO;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.io.File;

/**
 * Standalone Swing editor for paths and the obstacle field. Runs on a laptop, writes
 * into {@code src/main/deploy/pathing/}. Does NOT run on the robot.
 *
 * <p>Controls:
 * <ul>
 *   <li>Left-drag a green/red/cyan/magenta handle to move it.</li>
 *   <li>Shift+Left-drag on the field to paint obstacle cells.</li>
 *   <li>Shift+Right-drag to erase obstacle cells.</li>
 *   <li>File menu to save/load the path and the obstacle field.</li>
 * </ul>
 *
 * <p>Launch via {@code ./gradlew run -PmainClass=frc.robot.utility.Pathing.gui.PathGuiMain},
 * or from an IDE run config pointing at this main.
 */
public class PathGuiMain extends JFrame {
    private static final double FIELD_LENGTH = Constants.FieldConstants.FIELD_LENGTH;
    private static final double FIELD_WIDTH = Constants.FieldConstants.FIELD_WIDTH;
    private static final double OBSTACLE_RES = 0.25; // m per grid cell in the obstacle field

    private Translation2d startPt   = new Translation2d(2.0, 4.0);
    private Translation2d endPt     = new Translation2d(10.0, 4.0);
    private Translation2d startCtrl = new Translation2d(4.0, 4.0);
    private Translation2d endCtrl   = new Translation2d(8.0, 4.0);

    private final double maxVel = 5.0;
    private final double maxAt = 4.0;
    private final double maxOmega = Math.PI;
    private final double maxAlpha = 2 * Math.PI;
    private final double maxCentripetal = 3.0;

    private ObstacleField obstacles = new ObstacleField(FIELD_LENGTH, FIELD_WIDTH, OBSTACLE_RES);

    private final FieldPanel canvas = new FieldPanel();

    public PathGuiMain() {
        super("FRC Path Editor");
        setDefaultCloseOperation(EXIT_ON_CLOSE);

        JMenuBar bar = new JMenuBar();
        JMenu file = new JMenu("File");
        file.add(action("Save Path…", this::savePath));
        file.add(action("Load Path…", this::loadPath));
        file.addSeparator();
        file.add(action("Save Obstacles…", this::saveObstacles));
        file.add(action("Load Obstacles…", this::loadObstacles));
        file.add(action("Clear Obstacles", e -> {
            obstacles = new ObstacleField(FIELD_LENGTH, FIELD_WIDTH, OBSTACLE_RES);
            canvas.repaint();
        }));
        bar.add(file);
        setJMenuBar(bar);

        setContentPane(canvas);
        pack();
        setLocationRelativeTo(null);
    }

    private static Action action(String name, ActionListener l) {
        return new AbstractAction(name) {
            public void actionPerformed(ActionEvent e) { l.actionPerformed(e); }
        };
    }

    private void savePath(ActionEvent e) {
        JFileChooser chooser = new JFileChooser(defaultPathingDir());
        if (chooser.showSaveDialog(this) != JFileChooser.APPROVE_OPTION) return;
        try {
            Path p = buildPath();
            PathIO.write(p, chooser.getSelectedFile(),
                    maxVel, maxAt, maxOmega, maxAlpha, maxCentripetal);
        } catch (Exception ex) { error("save path", ex); }
    }

    private void loadPath(ActionEvent e) {
        JFileChooser chooser = new JFileChooser(defaultPathingDir());
        if (chooser.showOpenDialog(this) != JFileChooser.APPROVE_OPTION) return;
        try {
            Path p = PathIO.read(chooser.getSelectedFile());
            startPt = p.getStartPose().getTranslation();
            endPt = p.getEndPose().getTranslation();
            startCtrl = p.getStartControlHeading().getTranslation();
            endCtrl = p.getEndControlHeading().getTranslation();
            canvas.repaint();
        } catch (Exception ex) { error("load path", ex); }
    }

    private void saveObstacles(ActionEvent e) {
        JFileChooser chooser = new JFileChooser(defaultPathingDir());
        chooser.setSelectedFile(new File(defaultPathingDir(), "obstacles.dat"));
        if (chooser.showSaveDialog(this) != JFileChooser.APPROVE_OPTION) return;
        try { obstacles.writeTo(chooser.getSelectedFile()); }
        catch (Exception ex) { error("save obstacles", ex); }
    }

    private void loadObstacles(ActionEvent e) {
        JFileChooser chooser = new JFileChooser(defaultPathingDir());
        if (chooser.showOpenDialog(this) != JFileChooser.APPROVE_OPTION) return;
        try { obstacles = ObstacleField.load(chooser.getSelectedFile()); canvas.repaint(); }
        catch (Exception ex) { error("load obstacles", ex); }
    }

    private static File defaultPathingDir() {
        File d = new File("src/main/deploy/pathing");
        if (!d.exists()) d.mkdirs();
        return d;
    }

    private void error(String what, Exception ex) {
        JOptionPane.showMessageDialog(this, "Failed to " + what + ": " + ex.getMessage(),
                "Error", JOptionPane.ERROR_MESSAGE);
    }

    private Path buildPath() {
        Setpoint start = new Setpoint(startPt.getX(), startPt.getY(), 0, 0, 0, 0);
        Setpoint end = new Setpoint(endPt.getX(), endPt.getY(), 0, 0, 0, 0);
        Pose2d startCtrlPose = new Pose2d(startCtrl, new Rotation2d(0));
        Pose2d endCtrlPose = new Pose2d(endCtrl, new Rotation2d(0));
        return new Path(start, end, startCtrlPose, endCtrlPose,
                maxVel, maxAt, maxOmega, maxAlpha, maxCentripetal);
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new PathGuiMain().setVisible(true));
    }

    // -- Canvas ---------------------------------------------------------------

    private final class FieldPanel extends JPanel {
        private static final int MARGIN = 20;
        private static final double PX_PER_M = 60.0;
        private static final int HANDLE_PX = 10;
        private static final int PLOT_HEIGHT = 160;

        private Translation2d dragging = null;
        private boolean paintingObstacle = false;
        private boolean erasingObstacle = false;

        FieldPanel() {
            setPreferredSize(new Dimension(
                    (int)(FIELD_LENGTH * PX_PER_M) + 2 * MARGIN,
                    (int)(FIELD_WIDTH  * PX_PER_M) + 2 * MARGIN + PLOT_HEIGHT));
            setBackground(new Color(25, 25, 28));

            MouseAdapter h = new MouseAdapter() {
                @Override public void mousePressed(MouseEvent e) {
                    if (e.isShiftDown()) {
                        paintingObstacle = SwingUtilities.isLeftMouseButton(e);
                        erasingObstacle = SwingUtilities.isRightMouseButton(e);
                        toggleCell(e);
                        return;
                    }
                    dragging = nearestHandle(e.getX(), e.getY());
                }
                @Override public void mouseReleased(MouseEvent e) {
                    dragging = null; paintingObstacle = false; erasingObstacle = false;
                }
                @Override public void mouseDragged(MouseEvent e) {
                    if (paintingObstacle || erasingObstacle) { toggleCell(e); return; }
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

        private void toggleCell(MouseEvent e) {
            Translation2d w = screenToWorld(e.getX(), e.getY());
            int c = obstacles.toCol(w.getX());
            int r = obstacles.toRow(w.getY());
            obstacles.setBlocked(c, r, paintingObstacle);
            repaint();
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

        private Point worldToScreen(Translation2d w) {
            int x = MARGIN + (int)(w.getX() * PX_PER_M);
            int y = MARGIN + (int)((FIELD_WIDTH - w.getY()) * PX_PER_M);
            return new Point(x, y);
        }

        private Translation2d screenToWorld(int sx, int sy) {
            double x = (sx - MARGIN) / PX_PER_M;
            double y = FIELD_WIDTH - (sy - MARGIN) / PX_PER_M;
            return new Translation2d(
                    Math.max(0, Math.min(FIELD_LENGTH, x)),
                    Math.max(0, Math.min(FIELD_WIDTH, y)));
        }

        @Override protected void paintComponent(Graphics g0) {
            super.paintComponent(g0);
            Graphics2D g = (Graphics2D) g0;
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            // Field outline
            g.setColor(new Color(45, 50, 58));
            Point tl = worldToScreen(new Translation2d(0, FIELD_WIDTH));
            Point br = worldToScreen(new Translation2d(FIELD_LENGTH, 0));
            g.fillRect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);

            // Obstacles
            g.setColor(new Color(200, 60, 60, 180));
            for (int r = 0; r < obstacles.getRows(); r++) {
                for (int c = 0; c < obstacles.getCols(); c++) {
                    if (!obstacles.isBlocked(c, r)) continue;
                    double x = c * OBSTACLE_RES;
                    double y = r * OBSTACLE_RES;
                    Point p = worldToScreen(new Translation2d(x, y + OBSTACLE_RES));
                    int w = (int) Math.ceil(OBSTACLE_RES * PX_PER_M);
                    int h = (int) Math.ceil(OBSTACLE_RES * PX_PER_M);
                    g.fillRect(p.x, p.y, w, h);
                }
            }

            // Bezier (curvature-colored)
            Path path;
            try { path = buildPath(); }
            catch (Exception ex) { return; }
            double arc = path.getArcLength();
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
