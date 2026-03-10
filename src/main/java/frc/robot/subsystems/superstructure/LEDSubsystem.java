package frc.robot.subsystems.superstructure;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShotTimingConstants;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LEDSubsystem — self-contained LED state machine for FRC 2026.
 *
 * What this version does:
 *  - PREGAME (Disabled): shows READY / NOT STOWED, plus directional "shift" swipe cues
 *    (shiftLeft/Right/Forward/Back are your driver alignment flags).
 *
 *  - INGAME (Auto/Teleop): determines PASSING vs SHOOTING based on:
 *      1) Your alliance (DriverStation.getAlliance())
 *      2) 2026 game-specific message ('R' or 'B') indicating which alliance's hub is inactive first
 *      3) Current teleop "shift segment" based on match time remaining.
 *
 *    Semantics:
 *      - SHOOTING = your hub is ACTIVE
 *      - PASSING  = your hub is INACTIVE
 *
 *  - Transition visualization:
 *      - Replaces all pulse behavior with a LOADING BAR for the final LOADING_BAR_SECONDS
 *        before a boundary where YOUR hub state flips.
 *
 *  - AdvantageScope telemetry:
 *      Publishes keys under "LED/..." via SmartDashboard (NetworkTables).
 */
public class LEDSubsystem extends SubsystemBase {

    // ═══════════════════════════════════════════════════════════════════════════
    // CONFIG — adjust these to match your robot
    // ═══════════════════════════════════════════════════════════════════════════

    private static final int LED_PORT   = 9;   // PWM port
    private static final int LED_LENGTH = 60;  // Total LED count

    /** Seconds before a hub-state flip to show loading bar (change this freely) */
    private static final double LOADING_BAR_SECONDS = 2.0;

    /** SOTM "OK to shoot" pulse: pulse when timeToBoundary <= TOF (+pad) until hub becomes active. */
    private static final double SOTM_TOF_PAD_SEC = 0.05;
    private static final double SOTM_OK_PULSE_PERIOD_SEC = 0.25;

    // Swipe animation (pregame directional shifts)
    private static final double SWIPE_STEP_SEC = 0.03; // seconds per LED step
    private static final int    SWIPE_TRAIL    = 12;   // LEDs in the trail

    // ═══════════════════════════════════════════════════════════════════════════
    // REBUILT TELEOP "ALLIANCE SHIFT" TIMING (seconds remaining shown on DS)
    //
    // Teleop time typically counts down from ~135 to 0.
    //
    // TRANSITION: 2:20–2:10  => 140..130 (but teleop starts around 135; we treat >130 as transition)
    // SHIFT 1:    2:10–1:45  => 130..105
    // SHIFT 2:    1:45–1:20  => 105..80
    // SHIFT 3:    1:20–0:55  => 80..55
    // SHIFT 4:    0:55–0:30  => 55..30
    // ENDGAME:    0:30–0:00  => 30..0
    //
    // NOTE: If your event uses different breakpoints, only change these constants.
    // ═══════════════════════════════════════════════════════════════════════════

    private static final double TELEOP_TRANSITION_END = 130.0;
    private static final double SHIFT_1_END           = 105.0;
    private static final double SHIFT_2_END           =  80.0;
    private static final double SHIFT_3_END           =  55.0;
    private static final double SHIFT_4_END           =  30.0;
    private static final double ENDGAME_END           =   0.0;

    // ═══════════════════════════════════════════════════════════════════════════
    // ENUMS
    // ═══════════════════════════════════════════════════════════════════════════

    /** Top-level robot phase, driven by DriverStation */
    private enum RobotPhase { PREGAME, INGAME, OFF }

    /** Teleop segments for 2026 alliance shifts */
    private enum TeleopSegment {
        UNKNOWN,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        ENDGAME
    }

    /** All possible LED output states */
    private enum LEDState {
        // PreGame
        PREGAME_NOT_STOWED,
        PREGAME_READY,
        PREGAME_SHIFT_LEFT,
        PREGAME_SHIFT_RIGHT,
        PREGAME_SHIFT_FORWARD,
        PREGAME_SHIFT_BACK,

        // InGame (mechanism overrides)
        CLIMB_ACTUATING,
        CLIMB_AUTO_ALIGN,
        AUTO_STOW_ACTIVATED,

        // InGame (gameplay)
        HUB_ACTIVE_SHOOT,
        HUB_INACTIVE_PASS,
        SOTM_OK_TO_SHOOT_PULSE,
        LOADING_TO_ACTIVE,
        LOADING_TO_INACTIVE,
        GAME_DATA_UNKNOWN,

        OFF
    }

    /** Swipe animation directions */
    private enum SwipeDirection { LEFT, RIGHT, INWARD, OUTWARD }

    // ═══════════════════════════════════════════════════════════════════════════
    // HARDWARE
    // ═══════════════════════════════════════════════════════════════════════════

    private final AddressableLED       led;
    private final AddressableLEDBuffer buffer;
    private final Timer                animTimer = new Timer();

    // Odometry supplier (used for SOTM timing LEDs)
    private final Supplier<Pose2d>      poseSupplier;

    // ═══════════════════════════════════════════════════════════════════════════
    // ROBOT STATE — set these via the public setters below
    // ═══════════════════════════════════════════════════════════════════════════

    private boolean mechanismsStowed = false;
    private boolean autoStowActive   = false;
    private boolean climbActuating   = false;
    private boolean autoAlignActive  = false;

    // Driver “directional shift” cues (NOT the 2026 alliance shifts)
    private boolean shiftLeft    = false;
    private boolean shiftRight   = false;
    private boolean shiftForward = false;
    private boolean shiftBack    = false;

    // ═══════════════════════════════════════════════════════════════════════════
    // INTERNAL STATE MACHINE
    // ═══════════════════════════════════════════════════════════════════════════

    private LEDState currentState  = LEDState.OFF;
    private LEDState previousState = null;

    // ═══════════════════════════════════════════════════════════════════════════
    // AdvantageScope / NT telemetry snapshot
    // ═══════════════════════════════════════════════════════════════════════════
    private String  t_alliance               = "Unknown";
    private String  t_gameData               = "";
    private String  t_inactiveFirstAlliance  = "Unknown";
    private String  t_teleopSegment          = "Unknown";
    private boolean t_myHubActive            = false;

    private boolean t_loadingBarActive       = false;
    private boolean t_loadingToActive        = false;
    private double  t_loadingProgress        = 0.0;

    private double  t_timeToNextBoundary     = -1.0;

    // SOTM "OK to shoot" timing (inactive -> active boundary)
    private boolean t_sotmPulseActive      = false;
    private double  t_sotmDistanceToHubM   = -1.0;
    private double  t_sotmTimeOfFlightSec  = -1.0;
    private double  t_sotmPulseStartTime   = -1.0; // match time remaining when pulse should begin
    private double  t_sotmActiveBoundary   = -1.0; // match time remaining at the hub activation boundary

    // ═══════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════════

    
    public LEDSubsystem() {
        this(() -> new Pose2d());
    }

    public LEDSubsystem(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = (poseSupplier != null) ? poseSupplier : (() -> new Pose2d());

        led = new AddressableLED(LED_PORT);
        buffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(LED_LENGTH);
        led.setData(buffer);
        led.start();

        animTimer.start();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // PUBLIC SETTERS — call these from your subsystems / commands
    // ═══════════════════════════════════════════════════════════════════════════

    public void setMechanismsStowed(boolean stowed)   { this.mechanismsStowed = stowed; }
    public void setAutoStowActive(boolean active)     { this.autoStowActive   = active; }
    public void setClimbActuating(boolean actuating)  { this.climbActuating   = actuating; }
    public void setAutoAlignActive(boolean active)    { this.autoAlignActive  = active; }

    public void setShiftLeft(boolean active)          { this.shiftLeft        = active; }
    public void setShiftRight(boolean active)         { this.shiftRight       = active; }
    public void setShiftForward(boolean active)       { this.shiftForward     = active; }
    public void setShiftBack(boolean active)          { this.shiftBack        = active; }

    // ═══════════════════════════════════════════════════════════════════════════
    // PERIODIC — state machine runs here
    // ═══════════════════════════════════════════════════════════════════════════

    @Override
    public void periodic() {
        LEDState nextState = resolveState();
        transitionIfNeeded(nextState);
        renderState();
        led.setData(buffer);

        publishTelemetry();
    }

    // ── State resolution ──────────────────────────────────────────────────────

    private LEDState resolveState() {
        RobotPhase phase = resolvePhase();

        return switch (phase) {
            case PREGAME -> resolvePreGameState();
            case INGAME  -> resolveInGameState();
            case OFF     -> LEDState.OFF;
        };
    }

    private RobotPhase resolvePhase() {
        if (DriverStation.isDisabled())                               return RobotPhase.PREGAME;
        if (DriverStation.isTeleop() || DriverStation.isAutonomous()) return RobotPhase.INGAME;
        return RobotPhase.OFF;
    }

    private LEDState resolvePreGameState() {
        // Directional shifts take highest priority (driver alignment cues)
        if (shiftLeft)    return LEDState.PREGAME_SHIFT_LEFT;
        if (shiftRight)   return LEDState.PREGAME_SHIFT_RIGHT;
        if (shiftForward) return LEDState.PREGAME_SHIFT_FORWARD;
        if (shiftBack)    return LEDState.PREGAME_SHIFT_BACK;

        // Ready vs not stowed
        return mechanismsStowed ? LEDState.PREGAME_READY : LEDState.PREGAME_NOT_STOWED;
    }

    private LEDState resolveInGameState() {
        // Auto stow overrides everything
        if (autoStowActive) return LEDState.AUTO_STOW_ACTIVATED;

        // Climb states override gameplay
        if (climbActuating) {
            return autoAlignActive ? LEDState.CLIMB_AUTO_ALIGN : LEDState.CLIMB_ACTUATING;
        }

        // Gameplay-based state
        return resolveHubGameplayState();
    }

    /**
     * Determines PASSING vs SHOOTING based on:
     *  - DriverStation alliance
     *  - game-specific message ('R'/'B' = alliance whose hub is inactive first)
     *  - current teleop shift segment based on match time
     *
     * Also determines when to show loading bar for flips.
     */
    private LEDState resolveHubGameplayState() {
        // Reset telemetry defaults each loop (so stale values don't persist)
        t_gameData              = safeString(DriverStation.getGameSpecificMessage());
        t_alliance              = "Unknown";
        t_inactiveFirstAlliance = "Unknown";
        t_teleopSegment         = "Unknown";
        t_myHubActive           = false;
        t_loadingBarActive      = false;
        t_loadingToActive       = false;
        t_loadingProgress       = 0.0;
        t_timeToNextBoundary    = -1.0;

        t_sotmPulseActive     = false;
        t_sotmDistanceToHubM  = -1.0;
        t_sotmTimeOfFlightSec = -1.0;
        t_sotmPulseStartTime  = -1.0;
        t_sotmActiveBoundary  = -1.0;

        // AUTO: treat as active (both hubs active)
        if (DriverStation.isAutonomous()) {
            t_myHubActive = true;
            return LEDState.HUB_ACTIVE_SHOOT;
        }

        // Only compute segments in teleop
        if (!DriverStation.isTeleop()) {
            return LEDState.GAME_DATA_UNKNOWN;
        }

        Optional<Alliance> myAllianceOpt = DriverStation.getAlliance();
        if (myAllianceOpt.isEmpty()) {
            return LEDState.GAME_DATA_UNKNOWN;
        }
        Alliance myAlliance = myAllianceOpt.get();
        t_alliance = (myAlliance == Alliance.Blue) ? "Blue" : "Red";

        double t = DriverStation.getMatchTime();
        TeleopSegment seg = getTeleopSegment(t);
        t_teleopSegment = seg.name();

        // TRANSITION + ENDGAME: both hubs active
        if (seg == TeleopSegment.TRANSITION || seg == TeleopSegment.ENDGAME) {
            t_myHubActive = true;
            return LEDState.HUB_ACTIVE_SHOOT;
        }

        // Unknown time
        if (seg == TeleopSegment.UNKNOWN) {
            return LEDState.GAME_DATA_UNKNOWN;
        }

        Optional<Alliance> inactiveFirstOpt = parseInactiveFirstAlliance(t_gameData);
        if (inactiveFirstOpt.isEmpty()) {
            return LEDState.GAME_DATA_UNKNOWN;
        }
        Alliance inactiveFirst = inactiveFirstOpt.get();
        t_inactiveFirstAlliance = (inactiveFirst == Alliance.Blue) ? "Blue" : "Red";

        boolean activeNow = isMyHubActiveDuringShift(myAlliance, inactiveFirst, seg);
        t_myHubActive = activeNow;

        
        // Determine time to next boundary and whether we should show loading bar / SOTM timing pulse
        double timeToBoundary = secondsUntilNextBoundary(t, seg);
        t_timeToNextBoundary = timeToBoundary;

        TeleopSegment nextSeg = nextTeleopSegment(seg);

        boolean activeAfter;
        if (nextSeg == TeleopSegment.TRANSITION || nextSeg == TeleopSegment.ENDGAME) {
            activeAfter = true; // both active
        } else if (nextSeg == TeleopSegment.SHIFT_1 || nextSeg == TeleopSegment.SHIFT_2
                || nextSeg == TeleopSegment.SHIFT_3 || nextSeg == TeleopSegment.SHIFT_4) {
            activeAfter = isMyHubActiveDuringShift(myAlliance, inactiveFirst, nextSeg);
        } else {
            activeAfter = activeNow;
        }

        // ── SOTM "OK to shoot" pulse:
        // If we're about to flip from INACTIVE -> ACTIVE, pulse once it's late enough that a shot
        // fired now would arrive after the hub is ACTIVE (dt <= TOF).
        if (timeToBoundary >= 0.0 && activeAfter && !activeNow) {
            Pose2d pose = safePose(poseSupplier.get());
            double tofSec = estimateTimeOfFlightSecondsToMyHub(pose, myAlliance);
            double boundary = nextBoundaryMatchTimeRemaining(seg);

            t_sotmTimeOfFlightSec = tofSec;
            t_sotmActiveBoundary  = boundary;
            t_sotmPulseStartTime  = boundary + tofSec;

            Translation2d hub = (myAlliance == Alliance.Blue) ? FieldConstants.BLUE_HUB_CENTER : FieldConstants.RED_HUB_CENTER;
            t_sotmDistanceToHubM = pose.getTranslation().getDistance(hub);

            if (timeToBoundary <= (tofSec + SOTM_TOF_PAD_SEC)) {
                t_sotmPulseActive = true;
                return LEDState.SOTM_OK_TO_SHOOT_PULSE;
            }
        }

        // ── Existing loading bar behavior (kept for other flips and as a "soon" indicator) ──
        if (timeToBoundary >= 0.0 && timeToBoundary <= LOADING_BAR_SECONDS) {
            // Only show loading bar if our hub state actually flips at this boundary
            if (activeAfter != activeNow) {
                t_loadingBarActive = true;
                t_loadingToActive  = (activeAfter && !activeNow);
                t_loadingProgress  = clamp(1.0 - (timeToBoundary / LOADING_BAR_SECONDS), 0.0, 1.0);

                return t_loadingToActive ? LEDState.LOADING_TO_ACTIVE : LEDState.LOADING_TO_INACTIVE;
            }
        }

        return activeNow ? LEDState.HUB_ACTIVE_SHOOT : LEDState.HUB_INACTIVE_PASS;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // 2026 SHIFT HELPERS
    // ═══════════════════════════════════════════════════════════════════════════

    public static Optional<Double> getUpcomingAllianceActivationBoundary(
            Alliance myAlliance,
            String gameData,
            double matchTimeRemainingSeconds) {
        TeleopSegment seg = getTeleopSegment(matchTimeRemainingSeconds);
        if (seg == TeleopSegment.UNKNOWN || seg == TeleopSegment.ENDGAME) {
            return Optional.empty();
        }

        Optional<Alliance> inactiveFirstOpt = parseInactiveFirstAlliance(gameData);
        if (inactiveFirstOpt.isEmpty()) {
            return Optional.empty();
        }

        Alliance inactiveFirst = inactiveFirstOpt.get();
        boolean activeNow = (seg == TeleopSegment.TRANSITION)
                || isMyHubActiveDuringShift(myAlliance, inactiveFirst, seg);

        TeleopSegment nextSeg = nextTeleopSegment(seg);
        boolean activeAfter = (nextSeg == TeleopSegment.TRANSITION || nextSeg == TeleopSegment.ENDGAME)
                ? true
                : isMyHubActiveDuringShift(myAlliance, inactiveFirst, nextSeg);

        if (activeAfter && !activeNow) {
            return Optional.of(nextBoundaryMatchTimeRemaining(seg));
        }

        return Optional.empty();
    }

    private static TeleopSegment getTeleopSegment(double matchTimeRemainingSeconds) {
        // DS can return <0 when not synced/valid
        if (matchTimeRemainingSeconds < 0) return TeleopSegment.UNKNOWN;

        // Treat >130 as transition (teleop begins around ~135)
        if (matchTimeRemainingSeconds > TELEOP_TRANSITION_END) return TeleopSegment.TRANSITION;
        if (matchTimeRemainingSeconds > SHIFT_1_END)           return TeleopSegment.SHIFT_1;
        if (matchTimeRemainingSeconds > SHIFT_2_END)           return TeleopSegment.SHIFT_2;
        if (matchTimeRemainingSeconds > SHIFT_3_END)           return TeleopSegment.SHIFT_3;
        if (matchTimeRemainingSeconds > SHIFT_4_END)           return TeleopSegment.SHIFT_4;
        if (matchTimeRemainingSeconds >= ENDGAME_END)          return TeleopSegment.ENDGAME;

        return TeleopSegment.UNKNOWN;
    }

    private static TeleopSegment nextTeleopSegment(TeleopSegment seg) {
        return switch (seg) {
            case TRANSITION -> TeleopSegment.SHIFT_1;
            case SHIFT_1    -> TeleopSegment.SHIFT_2;
            case SHIFT_2    -> TeleopSegment.SHIFT_3;
            case SHIFT_3    -> TeleopSegment.SHIFT_4;
            case SHIFT_4    -> TeleopSegment.ENDGAME;
            case ENDGAME    -> TeleopSegment.ENDGAME;
            default         -> TeleopSegment.UNKNOWN;
        };
    }

    /**
     * Seconds until the next segment boundary (when match time crosses the next "*_END" value).
     * Returns -1 if unknown.
     */
    private static double secondsUntilNextBoundary(double matchTimeRemainingSeconds, TeleopSegment seg) {
        if (matchTimeRemainingSeconds < 0) return -1.0;

        double boundary = switch (seg) {
            case TRANSITION -> TELEOP_TRANSITION_END;
            case SHIFT_1    -> SHIFT_1_END;
            case SHIFT_2    -> SHIFT_2_END;
            case SHIFT_3    -> SHIFT_3_END;
            case SHIFT_4    -> SHIFT_4_END;
            case ENDGAME    -> ENDGAME_END;
            default         -> -1.0;
        };

        if (boundary < 0) return -1.0;
        return matchTimeRemainingSeconds - boundary;
    }


    /** Returns the match-time-remaining value of the next boundary for a given segment (seconds remaining). */
    private static double nextBoundaryMatchTimeRemaining(TeleopSegment seg) {
        return switch (seg) {
            case TRANSITION -> TELEOP_TRANSITION_END;
            case SHIFT_1    -> SHIFT_1_END;
            case SHIFT_2    -> SHIFT_2_END;
            case SHIFT_3    -> SHIFT_3_END;
            case SHIFT_4    -> SHIFT_4_END;
            case ENDGAME    -> ENDGAME_END;
            default         -> -1.0;
        };
    }

    /**
     * Game Data parsing:
     * Expects first char of message to be:
     *  - 'R' => Red hub inactive first (SHIFT 1)
     *  - 'B' => Blue hub inactive first (SHIFT 1)
     */
    private static Optional<Alliance> parseInactiveFirstAlliance(String gameData) {
        if (gameData == null || gameData.isEmpty()) return Optional.empty();

        char c = Character.toUpperCase(gameData.charAt(0));
        if (c == 'R') return Optional.of(Alliance.Red);
        if (c == 'B') return Optional.of(Alliance.Blue);
        return Optional.empty();
    }

    /**
     * Hub activity during SHIFT 1–4.
     *
     * This encodes the pattern:
     *  - inactiveFirst alliance hub is INACTIVE in SHIFT 1 and SHIFT 3, ACTIVE in SHIFT 2 and SHIFT 4
     *  - the other alliance is the opposite during SHIFT 1–4
     *
     * Outside SHIFT 1–4, we treat both hubs active (handled elsewhere).
     */
    private static boolean isMyHubActiveDuringShift(Alliance myAlliance, Alliance inactiveFirst, TeleopSegment seg) {
        boolean inactiveFirstActive = switch (seg) {
            case SHIFT_1 -> false;
            case SHIFT_2 -> true;
            case SHIFT_3 -> false;
            case SHIFT_4 -> true;
            default      -> true;
        };

        boolean myIsInactiveFirst = (myAlliance == inactiveFirst);
        return myIsInactiveFirst ? inactiveFirstActive : !inactiveFirstActive;
    }

    // ── State transition ──────────────────────────────────────────────────────

    /** Resets animation timer whenever the state changes */
    private void transitionIfNeeded(LEDState nextState) {
        if (nextState != currentState) {
            previousState = currentState;
            currentState  = nextState;
            animTimer.reset();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // RENDERING
    // ═══════════════════════════════════════════════════════════════════════════

    private void renderState() {
        switch (currentState) {
            // ── PreGame ──────────────────────────────────────────────────────
            case PREGAME_NOT_STOWED    -> setSolid(255, 0, 0);
            case PREGAME_READY         -> setSolid(0, 0, 255);
            case PREGAME_SHIFT_LEFT    -> animateSwipe(SwipeDirection.LEFT,    255, 255, 255);
            case PREGAME_SHIFT_RIGHT   -> animateSwipe(SwipeDirection.RIGHT,   255, 255, 255);
            case PREGAME_SHIFT_FORWARD -> animateSwipe(SwipeDirection.INWARD,  255, 255, 255);
            case PREGAME_SHIFT_BACK    -> animateSwipe(SwipeDirection.OUTWARD, 255, 255, 255);

            // ── Mechanism overrides ─────────────────────────────────────────
            case AUTO_STOW_ACTIVATED -> setSolid(255, 0, 0);
            case CLIMB_ACTUATING     -> setSolid(0, 0, 255);
            case CLIMB_AUTO_ALIGN    -> setSolid(0, 0, 255); // (no pulse; keep simple)

            // ── Gameplay (no pulses; only solids + loading bar) ─────────────
            case HUB_ACTIVE_SHOOT   -> setSolid(0, 255, 255);   // cyan
            case HUB_INACTIVE_PASS  -> setSolid(255, 255, 255); // white
            case SOTM_OK_TO_SHOOT_PULSE -> renderPulse(0, 255, 255, SOTM_OK_PULSE_PERIOD_SEC);

            case LOADING_TO_ACTIVE   -> renderLoadingBar(/*toActive=*/true);
            case LOADING_TO_INACTIVE -> renderLoadingBar(/*toActive=*/false);

            case GAME_DATA_UNKNOWN  -> setSolid(255, 165, 0);   // orange
            case OFF                -> setSolid(0, 0, 0);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LOADING BAR (transition animation)
    // ═══════════════════════════════════════════════════════════════════════════

    private void renderLoadingBar(boolean toActive) {
        // Choose bar color based on where we're going:
        //  - toActive => cyan (shoot)
        //  - toInactive => white (pass)
        int r = toActive ? 0   : 255;
        int g = toActive ? 255 : 255;
        int b = toActive ? 255 : 255;

        // Use the telemetry progress if we computed it; otherwise compute locally
        double progress = t_loadingProgress;

        if (!t_loadingBarActive) {
            // Fallback compute
            double t = DriverStation.getMatchTime();
            TeleopSegment seg = getTeleopSegment(t);
            double dt = secondsUntilNextBoundary(t, seg);
            if (dt >= 0.0) {
                progress = clamp(1.0 - (dt / LOADING_BAR_SECONDS), 0.0, 1.0);
            } else {
                progress = 1.0;
            }
        }

        int lit = (int)Math.round(progress * LED_LENGTH);

        for (int i = 0; i < LED_LENGTH; i++) {
            if (i < lit) buffer.setRGB(i, r, g, b);
            else         buffer.setRGB(i, 0, 0, 0);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // ANIMATION PRIMITIVES
    // ═══════════════════════════════════════════════════════════════════════════

    private void setSolid(int r, int g, int b) {
        for (int i = 0; i < LED_LENGTH; i++) {
            buffer.setRGB(i, r, g, b);
        }
    }


    /** Simple 50% duty-cycle pulse (on/off) at the requested period. */
    private void renderPulse(int r, int g, int b, double periodSec) {
        double phase = (animTimer.get() % periodSec) / periodSec;
        boolean on = phase < 0.5;

        if (on) setSolid(r, g, b);
        else    setSolid(0, 0, 0);
    }

    private void animateSwipe(SwipeDirection dir, int r, int g, int b) {
        int head = (int)(animTimer.get() / SWIPE_STEP_SEC) % LED_LENGTH;

        for (int i = 0; i < LED_LENGTH; i++) {
            int distance = (head - i + LED_LENGTH) % LED_LENGTH;
            int physical = mapSwipeIndex(i, dir);

            if (distance < SWIPE_TRAIL) {
                double fade = 1.0 - ((double) distance / SWIPE_TRAIL);
                buffer.setRGB(physical, (int)(r * fade), (int)(g * fade), (int)(b * fade));
            } else {
                buffer.setRGB(physical, 0, 0, 0);
            }
        }
    }

    /**
     * Modify this to match your wiring layout.
     * Assumes: first half = left side, second half = right side.
     */
    private int mapSwipeIndex(int i, SwipeDirection dir) {
        int half = LED_LENGTH / 2;
        return switch (dir) {
            case LEFT    -> i;
            case RIGHT   -> (LED_LENGTH - 1) - i;
            case INWARD  -> (i < half) ? i : (LED_LENGTH - 1) - (i - half);
            case OUTWARD -> (i < half) ? (half - 1) - i : half + (LED_LENGTH - 1 - i);
        };
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // TELEMETRY (AdvantageScope via NetworkTables)
    // ═══════════════════════════════════════════════════════════════════════════

    private void publishTelemetry() {
        SmartDashboard.putString("LED/State", currentState.name());
        SmartDashboard.putString("LED/PrevState", previousState == null ? "null" : previousState.name());

        SmartDashboard.putString("LED/Alliance", t_alliance);
        SmartDashboard.putString("LED/GameData", t_gameData);
        SmartDashboard.putString("LED/InactiveFirstAlliance", t_inactiveFirstAlliance);
        SmartDashboard.putString("LED/TeleopSegment", t_teleopSegment);

        SmartDashboard.putBoolean("LED/MyHubActive", t_myHubActive);

        SmartDashboard.putBoolean("LED/IsLoadingBar", t_loadingBarActive);
        SmartDashboard.putBoolean("LED/LoadingToActive", t_loadingToActive);
        SmartDashboard.putNumber("LED/LoadingProgress", t_loadingProgress);
        SmartDashboard.putNumber("LED/TimeToNextBoundary", t_timeToNextBoundary);


        SmartDashboard.putBoolean("LED/SOTM/PulseActive", t_sotmPulseActive);
        SmartDashboard.putNumber("LED/SOTM/DistanceToHubM", t_sotmDistanceToHubM);
        SmartDashboard.putNumber("LED/SOTM/TimeOfFlightSec", t_sotmTimeOfFlightSec);
        SmartDashboard.putNumber("LED/SOTM/PulseStartMatchTime", t_sotmPulseStartTime);
        SmartDashboard.putNumber("LED/SOTM/ActiveBoundaryMatchTime", t_sotmActiveBoundary);

        SmartDashboard.putNumber("LED/MatchTime", DriverStation.getMatchTime());
        SmartDashboard.putBoolean("LED/Teleop", DriverStation.isTeleop());
        SmartDashboard.putBoolean("LED/Auto", DriverStation.isAutonomous());
        SmartDashboard.putBoolean("LED/Disabled", DriverStation.isDisabled());
        SmartDashboard.putBoolean("LED/FMSAttached", DriverStation.isFMSAttached());
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SMALL UTILS
    // ═══════════════════════════════════════════════════════════════════════════

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static String safeString(String s) {
        return (s == null) ? "" : s;
    }

    private static Pose2d safePose(Pose2d pose) {
        return (pose != null) ? pose : new Pose2d();
    }

    /** Distance-based TOF estimate for LED timing (SOTM). */
    private static double estimateTimeOfFlightSecondsToMyHub(Pose2d robotPose, Alliance myAlliance) {
        Translation2d hub = (myAlliance == Alliance.Blue)
                ? FieldConstants.BLUE_HUB_CENTER
                : FieldConstants.RED_HUB_CENTER;

        double distance = robotPose.getTranslation().getDistance(hub);
        return ShotTimingConstants.getTimeOfFlightSeconds(distance);
    }

}
