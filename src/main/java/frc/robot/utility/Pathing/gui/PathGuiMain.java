package frc.robot.utility.Pathing.gui;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.Path;
import frc.robot.utility.Pathing.obstacle.ObstacleField;
import frc.robot.utility.Pathing.serialization.PathIO;
import frc.robot.utility.Pathing.serialization.RobotProfile;

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
    private final RobotProfileTab profileTab = new RobotProfileTab();

    public PathGuiMain() {
        super("FRC Path Editor");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        attemptAutoLoadObstacles();
        attemptAutoLoadProfile();

        JTabbedPane tabs = new JTabbedPane();
        tabs.addTab("Path Editor", pathTab);
        tabs.addTab("Keep-Out Zones", keepOutTab);
        tabs.addTab("Robot Profile", profileTab);
        setContentPane(tabs);

        // Refresh on tab change — obstacle map or profile may have changed.
        tabs.addChangeListener(e -> {
            pathTab.repaint();
            keepOutTab.repaint();
            profileTab.repaint();
        });

        pack();
        setLocationRelativeTo(null);
    }

    private void attemptAutoLoadProfile() {
        File f = new File(defaultPathingDir(), "robot-profile.json");
        if (f.exists()) {
            try { RobotProfile.setInstance(RobotProfile.read(f)); }
            catch (Exception ex) { System.err.println("[PathGui] load robot-profile.json failed: " + ex); }
        }
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

    /** Draw obstacles. Call with small alpha for read-only preview on the path tab.
     *  Hard cells render red; soft (auto-derived Manhattan halo) render amber. */
    static void drawObstacles(Graphics2D g, ObstacleField field, int alpha) {
        double res = field.getResolution();
        int w = (int) Math.ceil(res * PX_PER_M);
        int h = (int) Math.ceil(res * PX_PER_M);
        // Soft first so hard overpaints if someone ever edits both.
        int softAlpha = Math.max(30, alpha / 2);
        g.setColor(new Color(230, 170, 60, softAlpha));
        for (int r = 0; r < field.getRows(); r++) {
            for (int c = 0; c < field.getCols(); c++) {
                if (!field.isSoft(c, r)) continue;
                Point p = worldToScreen(new Translation2d(c * res, (r + 1) * res));
                g.fillRect(p.x, p.y, w, h);
            }
        }
        g.setColor(new Color(200, 60, 60, alpha));
        for (int r = 0; r < field.getRows(); r++) {
            for (int c = 0; c < field.getCols(); c++) {
                if (!field.isHard(c, r)) continue;
                Point p = worldToScreen(new Translation2d(c * res, (r + 1) * res));
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
        private static final double DT_ANIM = 0.02;   // 50 Hz playback tick
        private static final int ANIM_TIMER_MS = 33;  // ~30 fps repaint

        private Translation2d startPt   = new Translation2d(2.0, 4.0);
        private Translation2d endPt     = new Translation2d(10.0, 4.0);
        private Translation2d startCtrl = new Translation2d(4.0, 4.0);
        private Translation2d endCtrl   = new Translation2d(8.0, 4.0);
        private double startHeadingDeg = 0.0;
        private double endHeadingDeg = 0.0;

        private Translation2d dragging = null;
        private boolean draggingStartHeading = false;
        private boolean draggingEndHeading = false;
        private static final double HEADING_ARM = 0.7;
        private JSpinner startHSpinner, endHSpinner;
        private final FieldCanvas canvas = new FieldCanvas();

        // Playback state: a simulated robot pose is advanced by ticking the real Path
        // object's calculate() using a stub current-rotation feed.
        private Path animPath = null;
        private double animSimX, animSimY, animSimTheta;
        private double animSpeed;
        private boolean animPlaying = false;
        private double animElapsed = 0.0;
        private final Timer animTimer;

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
            bar.addSeparator();
            bar.add(new JLabel(" start θ° "));
            startHSpinner = new JSpinner(new SpinnerNumberModel(0.0, -180.0, 180.0, 5.0));
            startHSpinner.addChangeListener(e -> { startHeadingDeg = (Double) startHSpinner.getValue(); repaint(); });
            bar.add(startHSpinner);
            bar.add(new JLabel("  end θ° "));
            endHSpinner = new JSpinner(new SpinnerNumberModel(0.0, -180.0, 180.0, 5.0));
            endHSpinner.addChangeListener(e -> { endHeadingDeg = (Double) endHSpinner.getValue(); repaint(); });
            bar.add(endHSpinner);
            bar.addSeparator();
            bar.add(new AbstractAction("▶ Play") {
                public void actionPerformed(ActionEvent e) { playAnim(); }
            });
            bar.add(new AbstractAction("⏸ Pause") {
                public void actionPerformed(ActionEvent e) { animPlaying = false; }
            });
            bar.add(new AbstractAction("⏮ Reset") {
                public void actionPerformed(ActionEvent e) { resetAnim(); repaint(); }
            });
            add(bar, BorderLayout.NORTH);
            add(canvas, BorderLayout.CENTER);

            animTimer = new Timer(ANIM_TIMER_MS, e -> tickAnim());
        }

        private void savePath() {
            JFileChooser chooser = new JFileChooser(defaultPathingDir());
            if (chooser.showSaveDialog(this) != JFileChooser.APPROVE_OPTION) return;
            try {
                RobotProfile prof = RobotProfile.getInstance();
                PathIO.write(buildPath(), chooser.getSelectedFile(),
                        prof.maxVelocity, prof.maxAccel, prof.maxOmega, prof.maxAlpha, prof.maxCentripetal);
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
                startHeadingDeg = Math.toDegrees(p.getStartPose().getRotation().getRadians());
                endHeadingDeg = Math.toDegrees(p.getEndPose().getRotation().getRadians());
                resetAnim();
                repaint();
            } catch (Exception ex) { error("load path", ex); }
        }

        private Path buildPath() {
            RobotProfile prof = RobotProfile.getInstance();
            double sTheta = Math.toRadians(startHeadingDeg);
            double eTheta = Math.toRadians(endHeadingDeg);
            Setpoint start = new Setpoint(startPt.getX(), startPt.getY(), sTheta, 0, 0, 0);
            Setpoint end = new Setpoint(endPt.getX(), endPt.getY(), eTheta, 0, 0, 0);
            Pose2d startCtrlPose = new Pose2d(startCtrl, new Rotation2d(sTheta));
            Pose2d endCtrlPose = new Pose2d(endCtrl, new Rotation2d(eTheta));
            return new Path(start, end, startCtrlPose, endCtrlPose,
                    prof.maxVelocity, prof.maxAccel, prof.maxOmega, prof.maxAlpha, prof.maxCentripetal);
        }

        private void playAnim() {
            if (animPath == null || animDistanceRemaining() < 1e-3) {
                // (Re)start from beginning with a freshly built Path so profile changes are picked up.
                try { animPath = buildPath(); }
                catch (Exception ex) { return; }
                animSimX = startPt.getX();
                animSimY = startPt.getY();
                animSimTheta = Math.toRadians(startHeadingDeg);
                animElapsed = 0.0;
            }
            animPlaying = true;
            animTimer.start();
        }

        private void resetAnim() {
            animPlaying = false;
            animTimer.stop();
            animPath = null;
            animElapsed = 0.0;
            animSimX = startPt.getX();
            animSimY = startPt.getY();
            animSimTheta = Math.toRadians(startHeadingDeg);
            animSpeed = 0.0;
        }

        private double animDistanceRemaining() {
            if (animPath == null) return 0.0;
            // Rough proxy — when linear velocity goes near zero *and* we're near the end,
            // treat animation as complete.
            double toEnd = Math.hypot(endPt.getX() - animSimX, endPt.getY() - animSimY);
            return toEnd;
        }

        private void tickAnim() {
            if (!animPlaying) return;
            if (animPath == null) { playAnim(); if (animPath == null) return; }
            // Step the Path's internal profile forward. Use the current simulated heading
            // as the "robot rotation" so the rotation profile's angle-unwrap works sanely.
            Setpoint next = animPath.calculate(animSimTheta, DT_ANIM);
            // Treat returned setpoint as the commanded pose — simulate perfect tracking.
            animSimX = next.x;
            animSimY = next.y;
            animSimTheta = next.theta;
            animSpeed = Math.hypot(next.vx, next.vy);
            animElapsed += DT_ANIM;
            // Stop when commanded velocity is ~0 and we're at the end.
            if (animSpeed < 0.02 && animDistanceRemaining() < 0.05 && animElapsed > 0.2) {
                animPlaying = false;
                animTimer.stop();
            }
            canvas.repaint();
        }

        private final class FieldCanvas extends JPanel {
            FieldCanvas() {
                setPreferredSize(new Dimension(
                        (int)(FIELD_LENGTH * PX_PER_M) + 2 * MARGIN,
                        (int)(FIELD_WIDTH  * PX_PER_M) + 2 * MARGIN + PLOT_HEIGHT));
                setBackground(new Color(25, 25, 28));

                MouseAdapter h = new MouseAdapter() {
                    @Override public void mousePressed(MouseEvent e) {
                        int sx = e.getX(), sy = e.getY();
                        if (nearHeadingHandle(sx, sy, startPt, startHeadingDeg)) {
                            draggingStartHeading = true;
                        } else if (nearHeadingHandle(sx, sy, endPt, endHeadingDeg)) {
                            draggingEndHeading = true;
                        } else {
                            dragging = nearestHandle(sx, sy);
                        }
                    }
                    @Override public void mouseReleased(MouseEvent e) {
                        dragging = null;
                        draggingStartHeading = false;
                        draggingEndHeading = false;
                    }
                    @Override public void mouseDragged(MouseEvent e) {
                        if (draggingStartHeading) {
                            Translation2d w = screenToWorld(e.getX(), e.getY());
                            startHeadingDeg = Math.toDegrees(Math.atan2(
                                    w.getY() - startPt.getY(), w.getX() - startPt.getX()));
                            startHSpinner.setValue(startHeadingDeg);
                            repaint();
                        } else if (draggingEndHeading) {
                            Translation2d w = screenToWorld(e.getX(), e.getY());
                            endHeadingDeg = Math.toDegrees(Math.atan2(
                                    w.getY() - endPt.getY(), w.getX() - endPt.getX()));
                            endHSpinner.setValue(endHeadingDeg);
                            repaint();
                        } else if (dragging != null) {
                            Translation2d w = screenToWorld(e.getX(), e.getY());
                            if (dragging == startPt)        startPt = w;
                            else if (dragging == endPt)     endPt = w;
                            else if (dragging == startCtrl) startCtrl = w;
                            else if (dragging == endCtrl)   endCtrl = w;
                            dragging = w;
                            repaint();
                        }
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

            private boolean nearHeadingHandle(int sx, int sy, Translation2d anchor, double headingDeg) {
                double rad = Math.toRadians(headingDeg);
                Translation2d tip = new Translation2d(
                        anchor.getX() + HEADING_ARM * Math.cos(rad),
                        anchor.getY() + HEADING_ARM * Math.sin(rad));
                Point p = worldToScreen(tip);
                double d = (p.x - sx) * (p.x - sx) + (p.y - sy) * (p.y - sy);
                return d < 15 * 15;
            }

            @Override protected void paintComponent(Graphics g0) {
                super.paintComponent(g0);
                Graphics2D g = (Graphics2D) g0;
                g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

                drawFieldBackground(g, fieldImage);
                drawObstacles(g, obstacles, 90); // translucent preview only

                RobotProfile prof = RobotProfile.getInstance();

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

                // Footprint preview at start + end, translucent so the handles stay visible.
                drawFootprint(g, startPt.getX(), startPt.getY(), Math.toRadians(startHeadingDeg),
                        prof, new Color(60, 220, 110, 70), new Color(60, 220, 110, 180));
                drawFootprint(g, endPt.getX(), endPt.getY(), Math.toRadians(endHeadingDeg),
                        prof, new Color(230, 80, 80, 70), new Color(230, 80, 80, 180));

                // Animated robot, if playing or paused mid-run
                if (animPath != null) {
                    drawFootprint(g, animSimX, animSimY, animSimTheta, prof,
                            new Color(255, 220, 100, 140), new Color(255, 220, 100, 230));
                }

                // Handles
                drawHandle(g, startPt,   new Color(60, 220, 110));
                drawHandle(g, endPt,     new Color(230, 80, 80));
                drawHandle(g, startCtrl, new Color(90, 180, 255));
                drawHandle(g, endCtrl,   new Color(255, 150, 220));
                g.setColor(new Color(180, 180, 180, 120));
                drawLine(g, startPt, startCtrl);
                drawLine(g, endPt, endCtrl);

                // Heading drag handles — diamond at the tip of a heading arm
                drawHeadingHandle(g, startPt, startHeadingDeg, new Color(60, 220, 110));
                drawHeadingHandle(g, endPt, endHeadingDeg, new Color(230, 80, 80));

                // v(s) plot
                int plotY0 = getHeight() - PLOT_HEIGHT + 10;
                g.setColor(new Color(35, 35, 40));
                g.fillRect(MARGIN, plotY0, getWidth() - 2 * MARGIN, PLOT_HEIGHT - 30);
                g.setColor(new Color(200, 200, 210));
                g.drawString(String.format("v(s)  arc=%.2f m   max=%.1f m/s   t=%.2f s",
                        arc, prof.maxVelocity, animElapsed),
                        MARGIN + 6, plotY0 + 14);
                int plotW = getWidth() - 2 * MARGIN;
                int plotH = PLOT_HEIGHT - 40;
                Point prevV = null;
                int samples = 200;
                for (int i = 0; i <= samples; i++) {
                    double s = arc * i / samples;
                    double v = path.getVelocityProfile().velocityAt(s);
                    int px = MARGIN + (int)(plotW * (double) i / samples);
                    int py = plotY0 + plotH - (int)(plotH * v / Math.max(prof.maxVelocity, 1e-3));
                    g.setColor(new Color(120, 220, 160));
                    if (prevV != null) g.drawLine(prevV.x, prevV.y, px, py);
                    prevV = new Point(px, py);
                }
            }

            /** Draw a filled rotated rectangle for the robot footprint plus a short
             *  heading arrow out the front. Colors: fill and outline. */
            private void drawFootprint(Graphics2D g, double x, double y, double thetaRad,
                                       RobotProfile prof, Color fill, Color outline) {
                double halfL = prof.footprintLength / 2.0;
                double halfW = prof.footprintWidth / 2.0;
                // Corners in robot frame (X forward, Y left).
                double[][] local = {
                        { halfL,  halfW}, { halfL, -halfW},
                        {-halfL, -halfW}, {-halfL,  halfW}
                };
                int[] px = new int[4], py = new int[4];
                double cos = Math.cos(thetaRad), sin = Math.sin(thetaRad);
                for (int i = 0; i < 4; i++) {
                    double wx = x + local[i][0] * cos - local[i][1] * sin;
                    double wy = y + local[i][0] * sin + local[i][1] * cos;
                    Point p = worldToScreen(new Translation2d(wx, wy));
                    px[i] = p.x; py[i] = p.y;
                }
                g.setColor(fill);
                g.fillPolygon(px, py, 4);
                g.setColor(outline);
                g.drawPolygon(px, py, 4);
                // Heading arrow.
                Point center = worldToScreen(new Translation2d(x, y));
                Point nose = worldToScreen(new Translation2d(
                        x + halfL * cos * 1.2, y + halfL * sin * 1.2));
                g.drawLine(center.x, center.y, nose.x, nose.y);
                g.fillOval(nose.x - 3, nose.y - 3, 6, 6);
            }

            private void drawHandle(Graphics2D g, Translation2d w, Color c) {
                Point p = worldToScreen(w);
                g.setColor(c);
                g.fillOval(p.x - HANDLE_PX / 2, p.y - HANDLE_PX / 2, HANDLE_PX, HANDLE_PX);
            }

            private void drawHeadingHandle(Graphics2D g, Translation2d anchor, double headingDeg, Color c) {
                double rad = Math.toRadians(headingDeg);
                Translation2d tip = new Translation2d(
                        anchor.getX() + HEADING_ARM * Math.cos(rad),
                        anchor.getY() + HEADING_ARM * Math.sin(rad));
                Point anchorPx = worldToScreen(anchor);
                Point tipPx = worldToScreen(tip);
                g.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(), 150));
                g.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND,
                        0, new float[]{6, 4}, 0));
                g.drawLine(anchorPx.x, anchorPx.y, tipPx.x, tipPx.y);
                g.setStroke(new BasicStroke(1f));
                int d = 8;
                int[] dx = {tipPx.x, tipPx.x + d, tipPx.x, tipPx.x - d};
                int[] dy = {tipPx.y - d, tipPx.y, tipPx.y + d, tipPx.y};
                g.setColor(c);
                g.fillPolygon(dx, dy, 4);
                g.setColor(Color.WHITE);
                g.drawPolygon(dx, dy, 4);
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
                int hardCount = 0, softCount = 0;
                for (int r = 0; r < obstacles.getRows(); r++) {
                    for (int c = 0; c < obstacles.getCols(); c++) {
                        if (obstacles.isHard(c, r)) hardCount++;
                        else if (obstacles.isSoft(c, r)) softCount++;
                    }
                }
                g.drawString(String.format("grid: %dx%d  res=%.2f m  hard=%d  soft(auto)=%d",
                        obstacles.getCols(), obstacles.getRows(), obstacles.getResolution(),
                        hardCount, softCount),
                        MARGIN, getHeight() - 8);
            }
        }
    }

    private void error(String what, Exception ex) {
        JOptionPane.showMessageDialog(this, "Failed to " + what + ": " + ex.getMessage(),
                "Error", JOptionPane.ERROR_MESSAGE);
    }

    // -- Robot Profile Tab --------------------------------------------------------

    /** Editor for the kinematic limits + footprint. Explicitly NOT for PID or feedforward
     *  — those belong with the drivetrain subsystem. */
    private final class RobotProfileTab extends JPanel {
        private final JTextField fMaxVel = new JTextField(8);
        private final JTextField fMaxAccel = new JTextField(8);
        private final JTextField fMaxOmega = new JTextField(8);
        private final JTextField fMaxAlpha = new JTextField(8);
        private final JTextField fMaxCent = new JTextField(8);
        private final JTextField fFootLen = new JTextField(8);
        private final JTextField fFootWid = new JTextField(8);
        private final JLabel status = new JLabel(" ");

        RobotProfileTab() {
            setLayout(new BorderLayout());
            JPanel form = new JPanel(new GridBagLayout());
            GridBagConstraints gc = new GridBagConstraints();
            gc.insets = new Insets(4, 6, 4, 6);
            gc.anchor = GridBagConstraints.WEST;

            int row = 0;
            row = addRow(form, gc, row, "Max velocity (m/s)", fMaxVel,
                    "Top tangential speed the robot can actually hold. From SysID quasistatic.");
            row = addRow(form, gc, row, "Max accel (m/s^2)", fMaxAccel,
                    "Max tangential acceleration. From SysID dynamic.");
            row = addRow(form, gc, row, "Max omega (rad/s)", fMaxOmega,
                    "Max rotational velocity of the chassis.");
            row = addRow(form, gc, row, "Max alpha (rad/s^2)", fMaxAlpha,
                    "Max rotational acceleration of the chassis.");
            row = addRow(form, gc, row, "Max centripetal (m/s^2)", fMaxCent,
                    "Sideways accel cap before the robot slides. Typically 2–4. Drives curvature-based slowdown.");
            row = addRow(form, gc, row, "Footprint length (m)", fFootLen,
                    "Bumper-to-bumper along robot X. Used for GUI preview only.");
            row = addRow(form, gc, row, "Footprint width (m)", fFootWid,
                    "Bumper-to-bumper along robot Y. Used for GUI preview only.");

            JPanel toolbar = new JPanel(new FlowLayout(FlowLayout.LEFT));
            toolbar.add(new JButton(new AbstractAction("Save") {
                public void actionPerformed(ActionEvent e) { saveProfile(); }
            }));
            toolbar.add(new JButton(new AbstractAction("Reload from disk") {
                public void actionPerformed(ActionEvent e) { reloadProfile(); }
            }));
            toolbar.add(new JButton(new AbstractAction("Reset to defaults") {
                public void actionPerformed(ActionEvent e) { populate(RobotProfile.DEFAULT); apply(); }
            }));
            toolbar.add(Box.createHorizontalStrut(20));
            toolbar.add(status);

            add(form, BorderLayout.NORTH);
            add(toolbar, BorderLayout.SOUTH);

            populate(RobotProfile.getInstance());
        }

        private int addRow(JPanel form, GridBagConstraints gc, int row, String label,
                           JTextField field, String helpText) {
            gc.gridx = 0; gc.gridy = row; gc.weightx = 0;
            form.add(new JLabel(label), gc);
            gc.gridx = 1; gc.weightx = 0;
            form.add(field, gc);
            gc.gridx = 2; gc.weightx = 1; gc.fill = GridBagConstraints.HORIZONTAL;
            JLabel help = new JLabel(helpText);
            help.setForeground(new Color(140, 140, 150));
            form.add(help, gc);
            gc.fill = GridBagConstraints.NONE;
            // Apply on change so the path tab's v(s) preview updates live.
            field.addActionListener(e -> apply());
            field.addFocusListener(new FocusAdapter() {
                @Override public void focusLost(FocusEvent e) { apply(); }
            });
            return row + 1;
        }

        private void populate(RobotProfile p) {
            fMaxVel.setText(Double.toString(p.maxVelocity));
            fMaxAccel.setText(Double.toString(p.maxAccel));
            fMaxOmega.setText(Double.toString(p.maxOmega));
            fMaxAlpha.setText(Double.toString(p.maxAlpha));
            fMaxCent.setText(Double.toString(p.maxCentripetal));
            fFootLen.setText(Double.toString(p.footprintLength));
            fFootWid.setText(Double.toString(p.footprintWidth));
        }

        private RobotProfile harvest() {
            try {
                return new RobotProfile(
                        Double.parseDouble(fMaxVel.getText().trim()),
                        Double.parseDouble(fMaxAccel.getText().trim()),
                        Double.parseDouble(fMaxOmega.getText().trim()),
                        Double.parseDouble(fMaxAlpha.getText().trim()),
                        Double.parseDouble(fMaxCent.getText().trim()),
                        Double.parseDouble(fFootLen.getText().trim()),
                        Double.parseDouble(fFootWid.getText().trim()));
            } catch (NumberFormatException ex) {
                status.setText("Invalid number in form — not applied");
                return null;
            }
        }

        private void apply() {
            RobotProfile p = harvest();
            if (p == null) return;
            RobotProfile.setInstance(p);
            status.setText("Applied (not saved)");
            pathTab.repaint();
        }

        private void saveProfile() {
            RobotProfile p = harvest();
            if (p == null) return;
            RobotProfile.setInstance(p);
            try {
                p.write(new File(defaultPathingDir(), "robot-profile.json"));
                status.setText("Saved to " + defaultPathingDir() + "/robot-profile.json");
                pathTab.repaint();
            } catch (Exception ex) { error("save robot-profile.json", ex); }
        }

        private void reloadProfile() {
            File f = new File(defaultPathingDir(), "robot-profile.json");
            if (!f.exists()) { status.setText("No robot-profile.json on disk"); return; }
            try {
                RobotProfile p = RobotProfile.read(f);
                RobotProfile.setInstance(p);
                populate(p);
                status.setText("Reloaded");
                pathTab.repaint();
            } catch (Exception ex) { error("reload robot-profile.json", ex); }
        }
    }
}
