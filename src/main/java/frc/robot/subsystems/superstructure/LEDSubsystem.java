package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LEDSubsystem — self-contained LED state machine for FRC 2026.
 *
 * Call the setters below from your other subsystems or RobotContainer
 * to update robot state. This subsystem handles all display logic internally.
 *
 * Setup in RobotContainer:
 *   private final LEDSubsystem m_leds = new LEDSubsystem();
 *   // Then in other subsystems/commands, inject m_leds and call its setters.
 */
public class LEDSubsystem extends SubsystemBase {

    // ═══════════════════════════════════════════════════════════════════════════
    // CONFIG — adjust these to match your robot
    // ═══════════════════════════════════════════════════════════════════════════

    private static final int    LED_PORT          = 9;    // PWM port
    private static final int    LED_LENGTH        = 60;   // Total LED count

    // Match time windows (seconds remaining in match)
    private static final double PASSING_START     = 135.0;
    private static final double PASSING_END       = 105.0;
    private static final double SHOOTING_END      =  30.0;
    // Climb period = everything below SHOOTING_END

    // Pulse ramp starts this many seconds before a period transition
    private static final double PULSE_THRESHOLD   = 5.0;

    // Swipe animation
    private static final double SWIPE_STEP_SEC    = 0.03; // seconds per LED step
    private static final int    SWIPE_TRAIL       = 12;   // LEDs in the trail

    // Pulse animation
    private static final double PULSE_BASE_PERIOD = 1.0;  // seconds per cycle at 1x speed

    // ═══════════════════════════════════════════════════════════════════════════
    // ENUMS
    // ═══════════════════════════════════════════════════════════════════════════

    /** Top-level robot phase, driven by DriverStation */
    private enum RobotPhase { PREGAME, INGAME, OFF }

    /** All possible LED output states */
    private enum LEDState {
        // PreGame
        PREGAME_NOT_STOWED,
        PREGAME_READY,
        PREGAME_SHIFT_LEFT,
        PREGAME_SHIFT_RIGHT,
        PREGAME_SHIFT_FORWARD,
        PREGAME_SHIFT_BACK,

        // InGame
        PASSING_PERIOD,
        SHOOTING_PERIOD,
        CLIMB_ACTUATING,
        CLIMB_AUTO_ALIGN,
        AUTO_STOW_ACTIVATED,

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

    // ═══════════════════════════════════════════════════════════════════════════
    // ROBOT STATE — set these via the public setters below
    // ═══════════════════════════════════════════════════════════════════════════

    private boolean mechanismsStowed = false;
    private boolean autoStowActive   = false;
    private boolean climbActuating   = false;
    private boolean autoAlignActive  = false;
    private boolean shiftLeft        = false;
    private boolean shiftRight       = false;
    private boolean shiftForward     = false;
    private boolean shiftBack        = false;

    // ═══════════════════════════════════════════════════════════════════════════
    // INTERNAL STATE MACHINE
    // ═══════════════════════════════════════════════════════════════════════════

    private LEDState currentState    = LEDState.OFF;
    private LEDState previousState   = null;

    // ═══════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════════

    public LEDSubsystem() {
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
    }

    // ── State resolution ──────────────────────────────────────────────────────

    /** Determines what state we should be in given current robot + match conditions */
    private LEDState resolveState() {
        RobotPhase phase = resolvePhase();

        return switch (phase) {
            case PREGAME -> resolvePreGameState();
            case INGAME  -> resolveInGameState();
            case OFF     -> LEDState.OFF;
        };
    }

    private RobotPhase resolvePhase() {
        if (DriverStation.isDisabled())                                  return RobotPhase.PREGAME;
        if (DriverStation.isTeleop() || DriverStation.isAutonomous())    return RobotPhase.INGAME;
        return RobotPhase.OFF;
    }

    private LEDState resolvePreGameState() {
        // Directional shifts take highest priority
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

        // Climb states
        if (climbActuating) {
            return autoAlignActive ? LEDState.CLIMB_AUTO_ALIGN : LEDState.CLIMB_ACTUATING;
        }

        // Period-based state
        return resolveMatchPeriodState();
    }

    private LEDState resolveMatchPeriodState() {
        double t = DriverStation.getMatchTime();

        if (t < 0)              return LEDState.PASSING_PERIOD; // timer not yet valid
        if (t >= PASSING_START) return LEDState.PASSING_PERIOD;
        if (t >= PASSING_END)   return LEDState.PASSING_PERIOD;
        if (t >= SHOOTING_END)  return LEDState.SHOOTING_PERIOD;
        return LEDState.CLIMB_ACTUATING;
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

            // ── InGame ───────────────────────────────────────────────────────
            case PASSING_PERIOD -> {
                if (isNearTransition()) animatePulse(255, 255, 255, pulseMultiplier());
                else                    setSolid(255, 255, 255);
            }
            case SHOOTING_PERIOD -> {
                if (isNearTransition()) animatePulse(0, 255, 255, pulseMultiplier());
                else                    setSolid(0, 255, 255);
            }
            case CLIMB_ACTUATING  -> setSolid(0, 0, 255);
            case CLIMB_AUTO_ALIGN -> animatePulse(0, 0, 255, 2.0); // fixed 2x speed pulse
            case AUTO_STOW_ACTIVATED -> setSolid(255, 0, 0);

            case OFF -> setSolid(0, 0, 0);
        }
    }

    // ── Pulse helpers ─────────────────────────────────────────────────────────

    /** True when we're within PULSE_THRESHOLD seconds of the next period transition */
    private boolean isNearTransition() {
        double t = DriverStation.getMatchTime();
        if (t < 0) return false;
        double nearest = nearestUpcomingBoundary(t);
        double timeToTransition = t - nearest;
        return timeToTransition >= 0 && timeToTransition <= PULSE_THRESHOLD;
    }

    /**
     * Ramps from 1x → (1 + PULSE_THRESHOLD)x as we approach the boundary.
     * e.g. with PULSE_THRESHOLD = 5: 1x at 5s out, 6x at 0s.
     */
    private double pulseMultiplier() {
        double t = DriverStation.getMatchTime();
        if (t < 0) return 1.0;
        double nearest = nearestUpcomingBoundary(t);
        double timeToTransition = Math.max(0, t - nearest);
        return 1.0 + (PULSE_THRESHOLD - timeToTransition);
    }

    /** Returns the next period-end time boundary that match time will cross */
    private double nearestUpcomingBoundary(double t) {
        if (t > PASSING_END)  return PASSING_END;
        if (t > SHOOTING_END) return SHOOTING_END;
        return 0.0;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // ANIMATION PRIMITIVES
    // ═══════════════════════════════════════════════════════════════════════════

    /** Solid fill */
    private void setSolid(int r, int g, int b) {
        for (int i = 0; i < LED_LENGTH; i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    /** Sine-wave brightness pulse. multiplier speeds up the cycle. */
    private void animatePulse(int r, int g, int b, double multiplier) {
        double period     = PULSE_BASE_PERIOD / Math.max(1.0, multiplier);
        double brightness = (Math.sin(2 * Math.PI * animTimer.get() / period) + 1.0) / 2.0;
        for (int i = 0; i < LED_LENGTH; i++) {
            buffer.setRGB(i,
                (int)(r * brightness),
                (int)(g * brightness),
                (int)(b * brightness));
        }
    }

    /**
     * Moving swipe with a fading trail. Loops continuously.
     *
     * LED layout assumption:
     *   Index 0 → (LED_LENGTH/2 - 1) : left side  (front → back)
     *   Index LED_LENGTH/2 → LED_LENGTH-1 : right side (front → back)
     *
     * Modify mapSwipeIndex() to match your robot's actual wiring.
     */
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
     * Maps a logical index to a physical LED index based on swipe direction.
     * Assumes strip is split: first half = left side, second half = right side.
     *
     * ⚠️ Modify this to match your robot's LED wiring layout.
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
}