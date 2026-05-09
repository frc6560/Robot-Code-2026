package frc.robot.utility.Pathing.gui;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.Path;
import frc.robot.utility.Pathing.PathCalculator;
import frc.robot.utility.Pathing.PathChain;
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
            try {
                obstacles = ObstacleField.load(f);
                ObstacleField.setInstance(obstacles);
            }
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
        private int draggingWaypointIdx = -1;
        private static final double HEADING_ARM = 0.7;
        private JSpinner startHSpinner, endHSpinner;
        private static final int MODE_BEZIER = 0, MODE_ASTAR = 1, MODE_GENERATE = 2;
        private int pathMode = MODE_BEZIER;
        private final java.util.ArrayList<Translation2d> waypoints = new java.util.ArrayList<>();
        private JPanel waypointListPanel;
        private PathChain generatedChain = null;
        private final FieldCanvas canvas = new FieldCanvas();

        // Sidebar fields
        private final JSpinner startXField = new JSpinner(new SpinnerNumberModel(2.0, 0.0, FIELD_LENGTH, 0.05));
        private final JSpinner startYField = new JSpinner(new SpinnerNumberModel(4.0, 0.0, FIELD_WIDTH, 0.05));
        private final JSpinner startThetaField = new JSpinner(new SpinnerNumberModel(0.0, -180.0, 180.0, 1.0));
        private final JSpinner endXField = new JSpinner(new SpinnerNumberModel(10.0, 0.0, FIELD_LENGTH, 0.05));
        private final JSpinner endYField = new JSpinner(new SpinnerNumberModel(4.0, 0.0, FIELD_WIDTH, 0.05));
        private final JSpinner endThetaField = new JSpinner(new SpinnerNumberModel(0.0, -180.0, 180.0, 1.0));
        private boolean updatingFromCanvas = false;

        // Playback state
        private PathChain animChain = null;
        private double animSimX, animSimY, animSimTheta;
        private double animSpeed;
        private boolean animPlaying = false;
        private double animElapsed = 0.0;
        private final Timer animTimer;

        PathTab() {
            setLayout(new BorderLayout());

            // -- Toolbar (top) --
            JToolBar bar = new JToolBar();
            bar.setFloatable(false);
            bar.add(new AbstractAction("Save Path…") {
                public void actionPerformed(ActionEvent e) { savePath(); }
            });
            bar.add(new AbstractAction("Load Path…") {
                public void actionPerformed(ActionEvent e) { loadPath(); }
            });
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

            // -- Sidebar (left) --
            startHSpinner = startThetaField;
            endHSpinner = endThetaField;

            JPanel sidebar = new JPanel();
            sidebar.setLayout(new BoxLayout(sidebar, BoxLayout.Y_AXIS));
            sidebar.setBackground(new Color(35, 35, 40));
            sidebar.setPreferredSize(new Dimension(200, 800));
            sidebar.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));

            sidebar.add(buildWaypointPanel("Start Pose", new Color(60, 220, 110),
                    startXField, startYField, startThetaField));
            sidebar.add(Box.createVerticalStrut(12));
            sidebar.add(buildWaypointPanel("End Pose", new Color(230, 80, 80),
                    endXField, endYField, endThetaField));
            sidebar.add(Box.createVerticalStrut(16));

            // Mode toggle
            JPanel modePanel = new JPanel();
            modePanel.setLayout(new BoxLayout(modePanel, BoxLayout.Y_AXIS));
            modePanel.setBackground(new Color(45, 45, 52));
            modePanel.setBorder(BorderFactory.createCompoundBorder(
                    BorderFactory.createMatteBorder(0, 3, 0, 0, new Color(180, 150, 255)),
                    BorderFactory.createEmptyBorder(6, 8, 6, 8)));
            modePanel.setMaximumSize(new Dimension(Integer.MAX_VALUE, 80));
            modePanel.setAlignmentX(Component.LEFT_ALIGNMENT);

            JLabel modeTitle = new JLabel("Path Mode");
            modeTitle.setForeground(new Color(180, 150, 255));
            modeTitle.setFont(modeTitle.getFont().deriveFont(Font.BOLD, 12f));
            modeTitle.setAlignmentX(Component.LEFT_ALIGNMENT);
            modePanel.add(modeTitle);
            modePanel.add(Box.createVerticalStrut(4));

            String[] modes = {"Bezier (path planning)", "A* (on-the-fly)", "Path Generation"};
            JComboBox<String> modeCombo = new JComboBox<>(modes);
            modeCombo.setAlignmentX(Component.LEFT_ALIGNMENT);
            modeCombo.setMaximumSize(new Dimension(Integer.MAX_VALUE, 26));
            JButton generateBtn = new JButton("Generate A* Path");
            generateBtn.setAlignmentX(Component.LEFT_ALIGNMENT);
            generateBtn.setMaximumSize(new Dimension(Integer.MAX_VALUE, 26));
            generateBtn.setVisible(false);
            generateBtn.addActionListener(e -> {
                try {
                    generatedChain = buildGeneratedChain();
                    resetAnim();
                    canvas.repaint();
                } catch (Exception ex) {
                    error("generate path", ex);
                }
            });

            modeCombo.addActionListener(e -> {
                pathMode = modeCombo.getSelectedIndex();
                generatedChain = null;
                generateBtn.setVisible(pathMode == MODE_GENERATE);
                resetAnim();
                canvas.repaint();
            });
            modePanel.add(modeCombo);
            modePanel.add(Box.createVerticalStrut(4));
            modePanel.add(generateBtn);
            sidebar.add(modePanel);
            sidebar.add(Box.createVerticalStrut(12));

            // Waypoints section
            JPanel wpPanel = new JPanel();
            wpPanel.setLayout(new BoxLayout(wpPanel, BoxLayout.Y_AXIS));
            wpPanel.setBackground(new Color(45, 45, 52));
            wpPanel.setBorder(BorderFactory.createCompoundBorder(
                    BorderFactory.createMatteBorder(0, 3, 0, 0, new Color(255, 180, 60)),
                    BorderFactory.createEmptyBorder(6, 8, 6, 8)));
            wpPanel.setMaximumSize(new Dimension(Integer.MAX_VALUE, Integer.MAX_VALUE));
            wpPanel.setAlignmentX(Component.LEFT_ALIGNMENT);

            JLabel wpTitle = new JLabel("Waypoints");
            wpTitle.setForeground(new Color(255, 180, 60));
            wpTitle.setFont(wpTitle.getFont().deriveFont(Font.BOLD, 12f));
            wpTitle.setAlignmentX(Component.LEFT_ALIGNMENT);
            wpPanel.add(wpTitle);
            wpPanel.add(Box.createVerticalStrut(4));

            waypointListPanel = new JPanel();
            waypointListPanel.setLayout(new BoxLayout(waypointListPanel, BoxLayout.Y_AXIS));
            waypointListPanel.setOpaque(false);
            waypointListPanel.setAlignmentX(Component.LEFT_ALIGNMENT);
            wpPanel.add(waypointListPanel);
            wpPanel.add(Box.createVerticalStrut(4));

            JPanel wpButtons = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 0));
            wpButtons.setOpaque(false);
            wpButtons.setAlignmentX(Component.LEFT_ALIGNMENT);
            wpButtons.setMaximumSize(new Dimension(Integer.MAX_VALUE, 26));
            JButton addWpBtn = new JButton("+ Add");
            addWpBtn.addActionListener(e -> {
                Translation2d mid = startPt.plus(endPt).div(2.0);
                if (!waypoints.isEmpty()) {
                    Translation2d last = waypoints.get(waypoints.size() - 1);
                    mid = last.plus(endPt).div(2.0);
                }
                waypoints.add(mid);
                rebuildWaypointList();
                resetAnim();
                canvas.repaint();
            });
            JButton clearWpBtn = new JButton("Clear");
            clearWpBtn.addActionListener(e -> {
                waypoints.clear();
                rebuildWaypointList();
                resetAnim();
                canvas.repaint();
            });
            wpButtons.add(addWpBtn);
            wpButtons.add(clearWpBtn);
            wpPanel.add(wpButtons);

            sidebar.add(wpPanel);

            sidebar.add(Box.createVerticalGlue());

            // Wire sidebar fields → canvas state
            startXField.addChangeListener(e -> { if (!updatingFromCanvas) { startPt = new Translation2d((Double) startXField.getValue(), startPt.getY()); canvas.repaint(); }});
            startYField.addChangeListener(e -> { if (!updatingFromCanvas) { startPt = new Translation2d(startPt.getX(), (Double) startYField.getValue()); canvas.repaint(); }});
            startThetaField.addChangeListener(e -> { if (!updatingFromCanvas) { startHeadingDeg = (Double) startThetaField.getValue(); canvas.repaint(); }});
            endXField.addChangeListener(e -> { if (!updatingFromCanvas) { endPt = new Translation2d((Double) endXField.getValue(), endPt.getY()); canvas.repaint(); }});
            endYField.addChangeListener(e -> { if (!updatingFromCanvas) { endPt = new Translation2d(endPt.getX(), (Double) endYField.getValue()); canvas.repaint(); }});
            endThetaField.addChangeListener(e -> { if (!updatingFromCanvas) { endHeadingDeg = (Double) endThetaField.getValue(); canvas.repaint(); }});

            JScrollPane sidebarScroll = new JScrollPane(sidebar);
            sidebarScroll.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
            sidebarScroll.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED);
            sidebarScroll.getVerticalScrollBar().setUnitIncrement(16);
            sidebarScroll.setBorder(null);
            sidebarScroll.setPreferredSize(new Dimension(210, 0));
            add(sidebarScroll, BorderLayout.WEST);
            add(canvas, BorderLayout.CENTER);

            animTimer = new Timer(ANIM_TIMER_MS, e -> tickAnim());
        }

        private JPanel buildWaypointPanel(String title, Color accent,
                                           JSpinner xField, JSpinner yField, JSpinner thetaField) {
            JPanel panel = new JPanel();
            panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
            panel.setBackground(new Color(45, 45, 52));
            panel.setBorder(BorderFactory.createCompoundBorder(
                    BorderFactory.createMatteBorder(0, 3, 0, 0, accent),
                    BorderFactory.createEmptyBorder(6, 8, 6, 8)));
            panel.setMaximumSize(new Dimension(Integer.MAX_VALUE, 120));
            panel.setAlignmentX(Component.LEFT_ALIGNMENT);

            JLabel titleLabel = new JLabel(title);
            titleLabel.setForeground(accent);
            titleLabel.setFont(titleLabel.getFont().deriveFont(Font.BOLD, 12f));
            titleLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
            panel.add(titleLabel);
            panel.add(Box.createVerticalStrut(4));

            panel.add(buildFieldRow("X", xField, "m"));
            panel.add(Box.createVerticalStrut(2));
            panel.add(buildFieldRow("Y", yField, "m"));
            panel.add(Box.createVerticalStrut(2));
            panel.add(buildFieldRow("θ", thetaField, "°"));

            return panel;
        }

        private JPanel buildFieldRow(String label, JSpinner field, String unit) {
            JPanel row = new JPanel(new BorderLayout(4, 0));
            row.setOpaque(false);
            row.setMaximumSize(new Dimension(Integer.MAX_VALUE, 24));
            row.setAlignmentX(Component.LEFT_ALIGNMENT);

            JLabel lbl = new JLabel(label);
            lbl.setForeground(new Color(180, 180, 190));
            lbl.setPreferredSize(new Dimension(16, 20));
            row.add(lbl, BorderLayout.WEST);

            field.setPreferredSize(new Dimension(80, 20));
            row.add(field, BorderLayout.CENTER);

            JLabel unitLbl = new JLabel(unit);
            unitLbl.setForeground(new Color(120, 120, 130));
            unitLbl.setPreferredSize(new Dimension(20, 20));
            row.add(unitLbl, BorderLayout.EAST);

            return row;
        }

        private void syncSidebarFromCanvas() {
            updatingFromCanvas = true;
            startXField.setValue(Math.round(startPt.getX() * 100.0) / 100.0);
            startYField.setValue(Math.round(startPt.getY() * 100.0) / 100.0);
            startThetaField.setValue(Math.round(startHeadingDeg * 10.0) / 10.0);
            endXField.setValue(Math.round(endPt.getX() * 100.0) / 100.0);
            endYField.setValue(Math.round(endPt.getY() * 100.0) / 100.0);
            endThetaField.setValue(Math.round(endHeadingDeg * 10.0) / 10.0);
            updatingFromCanvas = false;
        }

        private void rebuildWaypointList() {
            waypointListPanel.removeAll();
            for (int i = 0; i < waypoints.size(); i++) {
                final int idx = i;
                Translation2d wp = waypoints.get(i);
                JPanel row = new JPanel(new BorderLayout(2, 0));
                row.setOpaque(false);
                row.setMaximumSize(new Dimension(Integer.MAX_VALUE, 22));
                row.setAlignmentX(Component.LEFT_ALIGNMENT);

                JLabel label = new JLabel(String.format("  %d  (%.1f, %.1f)", i + 1, wp.getX(), wp.getY()));
                label.setForeground(new Color(255, 180, 60));
                label.setFont(label.getFont().deriveFont(11f));
                row.add(label, BorderLayout.CENTER);

                JButton del = new JButton("×");
                del.setMargin(new Insets(0, 4, 0, 4));
                del.addActionListener(e -> {
                    waypoints.remove(idx);
                    rebuildWaypointList();
                    resetAnim();
                    canvas.repaint();
                });
                row.add(del, BorderLayout.EAST);

                waypointListPanel.add(row);
            }
            waypointListPanel.revalidate();
            waypointListPanel.repaint();
            waypointListPanel.getParent().revalidate();
            waypointListPanel.getParent().repaint();
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
                syncSidebarFromCanvas();
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

        private PathChain buildBezierChain() {
            if (waypoints.isEmpty()) {
                return new PathChain(java.util.List.of(buildPath()),
                        RobotProfile.getInstance().maxVelocity, RobotProfile.getInstance().maxAccel,
                        RobotProfile.getInstance().maxOmega, RobotProfile.getInstance().maxAlpha,
                        RobotProfile.getInstance().maxCentripetal);
            }
            RobotProfile prof = RobotProfile.getInstance();
            double sTheta = Math.toRadians(startHeadingDeg);
            double eTheta = Math.toRadians(endHeadingDeg);

            // Build ordered list: start, wp0, wp1, ..., end
            java.util.List<Translation2d> pts = new java.util.ArrayList<>();
            pts.add(startPt);
            pts.addAll(waypoints);
            pts.add(endPt);

            java.util.List<Path> segments = new java.util.ArrayList<>();
            for (int i = 0; i < pts.size() - 1; i++) {
                Translation2d a = pts.get(i);
                Translation2d b = pts.get(i + 1);

                double headingAtA = (i == 0) ? sTheta
                        : Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
                double headingAtB = (i == pts.size() - 2) ? eTheta
                        : Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
                if (i > 0) {
                    Translation2d prev = pts.get(i - 1);
                    double hIn = Math.atan2(a.getY() - prev.getY(), a.getX() - prev.getX());
                    double hOut = Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
                    headingAtA = Math.atan2(Math.sin(hIn) + Math.sin(hOut), Math.cos(hIn) + Math.cos(hOut));
                }
                if (i < pts.size() - 2) {
                    Translation2d next = pts.get(i + 2);
                    double hIn = Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
                    double hOut = Math.atan2(next.getY() - b.getY(), next.getX() - b.getX());
                    headingAtB = Math.atan2(Math.sin(hIn) + Math.sin(hOut), Math.cos(hIn) + Math.cos(hOut));
                }

                double dist = a.getDistance(b);
                double len = Math.max(0.5, Math.min(3.0, dist / 3.0));

                Pose2d aCtrl = new Pose2d(
                        a.getX() + len * Math.cos(headingAtA),
                        a.getY() + len * Math.sin(headingAtA),
                        new Rotation2d(headingAtA));
                Pose2d bCtrl = new Pose2d(
                        b.getX() - len * Math.cos(headingAtB),
                        b.getY() - len * Math.sin(headingAtB),
                        new Rotation2d(headingAtB));

                double segTheta = sTheta + (eTheta - sTheta) * i / (pts.size() - 1);
                double segThetaEnd = sTheta + (eTheta - sTheta) * (i + 1) / (pts.size() - 1);
                Setpoint aSet = new Setpoint(a.getX(), a.getY(), segTheta, 0, 0, 0);
                Setpoint bSet = new Setpoint(b.getX(), b.getY(), segThetaEnd, 0, 0, 0);

                segments.add(new Path(aSet, bSet, aCtrl, bCtrl,
                        prof.maxVelocity, prof.maxAccel, prof.maxOmega, prof.maxAlpha, prof.maxCentripetal));
            }
            return new PathChain(segments,
                    prof.maxVelocity, prof.maxAccel, prof.maxOmega, prof.maxAlpha, prof.maxCentripetal);
        }

        private PathChain buildGeneratedChain() {
            RobotProfile prof = RobotProfile.getInstance();
            double sTheta = Math.toRadians(startHeadingDeg);
            double eTheta = Math.toRadians(endHeadingDeg);
            ObstacleField.setInstance(obstacles);

            // Ordered list of must-hit points: start, waypoints, end
            java.util.List<Translation2d> userPts = new java.util.ArrayList<>();
            userPts.add(startPt);
            userPts.addAll(waypoints);
            userPts.add(endPt);

            // Run A* between each consecutive user pair, merge all route points
            // into one flat list (de-duplicating shared endpoints).
            java.util.List<Translation2d> allPts = new java.util.ArrayList<>();
            allPts.add(userPts.get(0));
            for (int i = 0; i < userPts.size() - 1; i++) {
                Translation2d a = userPts.get(i);
                Translation2d b = userPts.get(i + 1);
                ObstacleField field = ObstacleField.getInstance();
                if (field.isSegmentBlocked(a, b)) {
                    java.util.List<Translation2d> route = new frc.robot.utility.Pathing.obstacle.AStarPlanner(field).plan(a, b);
                    if (route.size() < 2) return null;
                    for (int j = 1; j < route.size(); j++) {
                        allPts.add(route.get(j));
                    }
                } else {
                    allPts.add(b);
                }
            }

            // Build smooth Bezier segments with bisector-averaged headings at junctions
            java.util.List<Path> segments = new java.util.ArrayList<>();
            for (int i = 0; i < allPts.size() - 1; i++) {
                Translation2d a = allPts.get(i);
                Translation2d b = allPts.get(i + 1);

                double headingAtA = Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
                double headingAtB = Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
                if (i > 0) {
                    Translation2d prev = allPts.get(i - 1);
                    double hIn = Math.atan2(a.getY() - prev.getY(), a.getX() - prev.getX());
                    double hOut = Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
                    headingAtA = Math.atan2(Math.sin(hIn) + Math.sin(hOut), Math.cos(hIn) + Math.cos(hOut));
                }
                if (i < allPts.size() - 2) {
                    Translation2d next = allPts.get(i + 2);
                    double hIn = Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
                    double hOut = Math.atan2(next.getY() - b.getY(), next.getX() - b.getX());
                    headingAtB = Math.atan2(Math.sin(hIn) + Math.sin(hOut), Math.cos(hIn) + Math.cos(hOut));
                }

                double dist = a.getDistance(b);
                double len = Math.max(0.5, Math.min(3.0, dist / 3.0));

                Pose2d aCtrl = new Pose2d(
                        a.getX() + len * Math.cos(headingAtA),
                        a.getY() + len * Math.sin(headingAtA),
                        new Rotation2d(headingAtA));
                Pose2d bCtrl = new Pose2d(
                        b.getX() - len * Math.cos(headingAtB),
                        b.getY() - len * Math.sin(headingAtB),
                        new Rotation2d(headingAtB));

                double segTheta = sTheta + (eTheta - sTheta) * i / (allPts.size() - 1);
                double segThetaEnd = sTheta + (eTheta - sTheta) * (i + 1) / (allPts.size() - 1);
                Setpoint aSet = new Setpoint(a.getX(), a.getY(), segTheta, 0, 0, 0);
                Setpoint bSet = new Setpoint(b.getX(), b.getY(), segThetaEnd, 0, 0, 0);

                segments.add(new Path(aSet, bSet, aCtrl, bCtrl,
                        prof.maxVelocity, prof.maxAccel, prof.maxOmega, prof.maxAlpha, prof.maxCentripetal));
            }
            if (segments.isEmpty()) return null;
            return new PathChain(segments,
                    prof.maxVelocity, prof.maxAccel, prof.maxOmega, prof.maxAlpha, prof.maxCentripetal);
        }

        private PathChain buildChain() {
            double sTheta = Math.toRadians(startHeadingDeg);
            double eTheta = Math.toRadians(endHeadingDeg);
            Setpoint start = new Setpoint(startPt.getX(), startPt.getY(), sTheta, 0, 0, 0);
            Setpoint end = new Setpoint(endPt.getX(), endPt.getY(), eTheta, 0, 0, 0);
            ObstacleField.setInstance(obstacles);
            PathCalculator calc = new PathCalculator(start, end);
            PathChain chain = calc.calculatePathChain();
            return chain;
        }

        private void playAnim() {
            if (animChain == null || animDistanceRemaining() < 1e-3) {
                try {
                    if (pathMode == MODE_GENERATE && generatedChain != null) {
                        animChain = generatedChain;
                    } else if (pathMode == MODE_ASTAR) {
                        animChain = buildChain();
                    } else {
                        animChain = buildBezierChain();
                    }
                } catch (Exception ex) { return; }
                if (animChain == null) return;
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
            animChain = null;
            animElapsed = 0.0;
            animSimX = startPt.getX();
            animSimY = startPt.getY();
            animSimTheta = Math.toRadians(startHeadingDeg);
            animSpeed = 0.0;
        }

        private double animDistanceRemaining() {
            if (animChain == null) return 0.0;
            double toEnd = Math.hypot(endPt.getX() - animSimX, endPt.getY() - animSimY);
            return toEnd;
        }

        private void tickAnim() {
            if (!animPlaying) return;
            if (animChain == null) { playAnim(); if (animChain == null) return; }
            Setpoint next = animChain.calculate(animSimTheta, DT_ANIM);
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
                            draggingWaypointIdx = nearestWaypoint(sx, sy);
                            if (draggingWaypointIdx < 0) {
                                dragging = nearestHandle(sx, sy);
                            }
                        }
                    }
                    @Override public void mouseReleased(MouseEvent e) {
                        dragging = null;
                        draggingStartHeading = false;
                        draggingEndHeading = false;
                        if (draggingWaypointIdx >= 0) {
                            rebuildWaypointList();
                            draggingWaypointIdx = -1;
                        }
                    }
                    @Override public void mouseDragged(MouseEvent e) {
                        if (draggingStartHeading) {
                            Translation2d w = screenToWorld(e.getX(), e.getY());
                            startHeadingDeg = Math.toDegrees(Math.atan2(
                                    w.getY() - startPt.getY(), w.getX() - startPt.getX()));
                            syncSidebarFromCanvas();
                            repaint();
                        } else if (draggingEndHeading) {
                            Translation2d w = screenToWorld(e.getX(), e.getY());
                            endHeadingDeg = Math.toDegrees(Math.atan2(
                                    w.getY() - endPt.getY(), w.getX() - endPt.getX()));
                            syncSidebarFromCanvas();
                            repaint();
                        } else if (draggingWaypointIdx >= 0) {
                            Translation2d w = screenToWorld(e.getX(), e.getY());
                            waypoints.set(draggingWaypointIdx, w);
                            repaint();
                        } else if (dragging != null) {
                            Translation2d w = screenToWorld(e.getX(), e.getY());
                            if (dragging == startPt)        startPt = w;
                            else if (dragging == endPt)     endPt = w;
                            else if (dragging == startCtrl) startCtrl = w;
                            else if (dragging == endCtrl)   endCtrl = w;
                            dragging = w;
                            syncSidebarFromCanvas();
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

            private int nearestWaypoint(int sx, int sy) {
                int best = -1; double bestD = 15 * 15;
                for (int i = 0; i < waypoints.size(); i++) {
                    Point p = worldToScreen(waypoints.get(i));
                    double d = (p.x - sx) * (p.x - sx) + (p.y - sy) * (p.y - sy);
                    if (d < bestD) { bestD = d; best = i; }
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

                // Grey connecting lines: start → wp1 → wp2 → ... → end
                {
                    if (pathMode == MODE_GENERATE) {
                        g.setColor(new Color(200, 200, 200, 220));
                        g.setStroke(new BasicStroke(2.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND,
                                0, new float[]{8, 6}, 0));
                    } else {
                        g.setColor(new Color(140, 140, 140, 120));
                        g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND,
                                0, new float[]{8, 6}, 0));
                    }
                    java.util.List<Translation2d> linePoints = new java.util.ArrayList<>();
                    linePoints.add(startPt);
                    linePoints.addAll(waypoints);
                    linePoints.add(endPt);

                    for (int i = 0; i < linePoints.size() - 1; i++) {
                        Point a = worldToScreen(linePoints.get(i));
                        Point b = worldToScreen(linePoints.get(i + 1));
                        g.drawLine(a.x, a.y, b.x, b.y);
                    }
                    g.setStroke(new BasicStroke(1f));
                }

                // Draw the colored path for the current mode
                PathChain drawChain = null;
                float hueBase = 0.33f, hueRange = 0.33f;
                if (pathMode == MODE_ASTAR) {
                    try { drawChain = buildChain(); } catch (Exception ex) { /* skip */ }
                    hueBase = 0.55f; hueRange = 0.55f;
                } else if (pathMode == MODE_GENERATE && generatedChain != null) {
                    drawChain = generatedChain;
                    hueBase = 0.75f; hueRange = 0.25f;
                } else if (pathMode == MODE_BEZIER) {
                    try { drawChain = buildBezierChain(); } catch (Exception ex) { /* skip */ }
                }
                if (drawChain != null) {
                    int N = 200;
                    Point prev = null;
                    for (int s = 0; s < drawChain.size(); s++) {
                        Path seg = drawChain.getSegment(s);
                        int samplesPerSeg = Math.max(N / drawChain.size(), 40);
                        for (int i = 0; i <= samplesPerSeg; i++) {
                            double t = (double) i / samplesPerSeg;
                            Translation2d pos = seg.calculatePosition(t);
                            double k = Math.abs(seg.getCurvature(t));
                            float hue = (float) Math.min(0.65, k / 4.0);
                            g.setColor(Color.getHSBColor(hueBase - hue * hueRange, 0.9f, 1.0f));
                            Point cur = worldToScreen(pos);
                            if (prev != null) g.drawLine(prev.x, prev.y, cur.x, cur.y);
                            prev = cur;
                        }
                    }
                }

                // Footprint preview at start + end, translucent so the handles stay visible.
                drawFootprint(g, startPt.getX(), startPt.getY(), Math.toRadians(startHeadingDeg),
                        prof, new Color(60, 220, 110, 70), new Color(60, 220, 110, 180));
                drawFootprint(g, endPt.getX(), endPt.getY(), Math.toRadians(endHeadingDeg),
                        prof, new Color(230, 80, 80, 70), new Color(230, 80, 80, 180));

                // Animated robot, if playing or paused mid-run
                if (animChain != null) {
                    drawFootprint(g, animSimX, animSimY, animSimTheta, prof,
                            new Color(255, 220, 100, 140), new Color(255, 220, 100, 230));
                }

                // Handles
                drawHandle(g, startPt,   new Color(60, 220, 110));
                drawHandle(g, endPt,     new Color(230, 80, 80));
                if (pathMode == MODE_BEZIER) {
                    drawHandle(g, startCtrl, new Color(90, 180, 255));
                    drawHandle(g, endCtrl,   new Color(255, 150, 220));
                    g.setColor(new Color(180, 180, 180, 120));
                    drawLine(g, startPt, startCtrl);
                    drawLine(g, endPt, endCtrl);
                }

                // Waypoint handles
                Color wpColor = new Color(255, 180, 60);
                for (int i = 0; i < waypoints.size(); i++) {
                    Translation2d wp = waypoints.get(i);
                    drawHandle(g, wp, wpColor);
                    g.setColor(new Color(255, 180, 60, 160));
                    Point p = worldToScreen(wp);
                    g.drawString(String.valueOf(i + 1), p.x + 8, p.y - 4);
                }

                // Heading drag handles — diamond at the tip of a heading arm
                drawHeadingHandle(g, startPt, startHeadingDeg, new Color(60, 220, 110));
                drawHeadingHandle(g, endPt, endHeadingDeg, new Color(230, 80, 80));

                // v(s) plot — use the chain's joint profile when available
                // so multi-segment paths show one smooth curve instead of per-segment dips.
                double plotArc = (drawChain != null) ? drawChain.getTotalLength() : arc;
                var plotProfile = (drawChain != null) ? drawChain.getJointProfile() : path.getVelocityProfile();
                int plotY0 = getHeight() - PLOT_HEIGHT + 10;
                g.setColor(new Color(35, 35, 40));
                g.fillRect(MARGIN, plotY0, getWidth() - 2 * MARGIN, PLOT_HEIGHT - 30);
                g.setColor(new Color(200, 200, 210));
                g.drawString(String.format("v(s)  arc=%.2f m   max=%.1f m/s   t=%.2f s",
                        plotArc, prof.maxVelocity, animElapsed),
                        MARGIN + 6, plotY0 + 14);
                int plotW = getWidth() - 2 * MARGIN;
                int plotH = PLOT_HEIGHT - 40;
                Point prevV = null;
                int samples = 200;
                for (int i = 0; i <= samples; i++) {
                    double s = plotArc * i / samples;
                    double v = plotProfile.velocityAt(s);
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
                    ObstacleField.setInstance(obstacles);
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
            try {
                obstacles = ObstacleField.load(f);
                ObstacleField.setInstance(obstacles);
                repaint();
            } catch (Exception ex) { error("reload", ex); }
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
