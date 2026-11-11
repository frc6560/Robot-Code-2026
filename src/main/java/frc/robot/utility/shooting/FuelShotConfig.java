package frc.robot.utility.shooting;

/** Stationary auto-turn shooting. Aerodynamic values require field validation. */
public final class FuelShotConfig {
    private FuelShotConfig() {}
    /** Multiply ideal exit speed; increase if actual balls launch faster than predicted. */
    public static final double EXIT_SPEED_MULTIPLIER = 1.0;
    /** Multiply ideal rolling backspin independently of exit speed. */
    public static final double BACKSPIN_MULTIPLIER = 1.0;
    public static final double TUNNEL_RPM = 1500.0;
    /** Global floor enable, applied at the subsystem output boundary. */
    public static final boolean FLOOR_ENABLED = true;
    public static final double RPM_TOLERANCE = 60.0;
    public static final double READY_DWELL_SECONDS = 0.10;
    public static final double MAX_TRANSLATION_MPS = 0.15;
    public static final double MIN_ENTRY_DEGREES = 10.0;
    public static final double LAUNCH_ANGLE_OFFSET_DEGREES = 0.0;
    public static final double DRAG_COEFFICIENT = 0.47;
    public static final double LIFT_SLOPE = 0.75;
    public static final double SPIN_DECAY_PER_SECOND = 0.10;
    // Measure from the chassis pose origin to ball release, positive forward/left.
    public static final double RELEASE_FORWARD_METERS = 0.0;
    public static final double RELEASE_LEFT_METERS = 0.0;
}
