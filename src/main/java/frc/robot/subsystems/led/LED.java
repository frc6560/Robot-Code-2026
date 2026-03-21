package frc.robot.subsystems.led;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    // Config
    private static final double LOADING_BAR_SECONDS = 2.0;
    private static final double SWIPE_STEP_SEC = 0.03;
    private static final int SWIPE_TRAIL = 12;

    // Teleop shift timing (seconds remaining)
    private static final double TELEOP_TRANSITION_END = 130.0;
    private static final double SHIFT_1_END = 105.0;
    private static final double SHIFT_2_END = 80.0;
    private static final double SHIFT_3_END = 55.0;
    private static final double SHIFT_4_END = 30.0;
    private static final double ENDGAME_END = 0.0;

    private enum RobotPhase { PREGAME, INGAME, OFF }

    private enum TeleopSegment {
        UNKNOWN, TRANSITION, SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4, ENDGAME
    }

    public enum LEDState {
        PREGAME_NOT_STOWED,
        PREGAME_READY,
        PREGAME_SHIFT_LEFT,
        PREGAME_SHIFT_RIGHT,
        PREGAME_SHIFT_FORWARD,
        PREGAME_SHIFT_BACK,
        AUTO_STOW_ACTIVATED,
        HUB_ACTIVE_SHOOT,
        HUB_INACTIVE_PASS,
        LOADING_TO_ACTIVE,
        LOADING_TO_INACTIVE,
        GAME_DATA_UNKNOWN,
        OFF
    }

    private enum SwipeDirection { LEFT, RIGHT, INWARD, OUTWARD }

    private final LEDIO io;
    private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();
    private final Timer animTimer = new Timer();

    // Robot state
    private boolean mechanismsStowed = false;
    private boolean autoStowActive = false;
    private boolean shiftLeft = false;
    private boolean shiftRight = false;
    private boolean shiftForward = false;
    private boolean shiftBack = false;

    // State machine
    private LEDState currentState = LEDState.OFF;
    private LEDState previousState = null;

    // Telemetry cache
    private String t_alliance = "Unknown";
    private String t_gameData = "";
    private String t_inactiveFirstAlliance = "Unknown";
    private String t_teleopSegment = "Unknown";
    private boolean t_myHubActive = false;
    private boolean t_loadingBarActive = false;
    private boolean t_loadingToActive = false;
    private double t_loadingProgress = 0.0;
    private double t_timeToNextBoundary = -1.0;

    public LED(LEDIO io) {
        this.io = io;
        animTimer.start();
    }

    // Public setters
    public void setMechanismsStowed(boolean stowed) { this.mechanismsStowed = stowed; }
    public void setAutoStowActive(boolean active) { this.autoStowActive = active; }
    public void setShiftLeft(boolean active) { this.shiftLeft = active; }
    public void setShiftRight(boolean active) { this.shiftRight = active; }
    public void setShiftForward(boolean active) { this.shiftForward = active; }
    public void setShiftBack(boolean active) { this.shiftBack = active; }

    public LEDState getCurrentState() { return currentState; }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LED", inputs);

        LEDState nextState = resolveState();
        transitionIfNeeded(nextState);
        renderState();
        io.setData();

        publishTelemetry();
    }

    private LEDState resolveState() {
        RobotPhase phase = resolvePhase();
        return switch (phase) {
            case PREGAME -> resolvePreGameState();
            case INGAME -> resolveInGameState();
            case OFF -> LEDState.OFF;
        };
    }

    private RobotPhase resolvePhase() {
        if (DriverStation.isDisabled()) return RobotPhase.PREGAME;
        if (DriverStation.isTeleop() || DriverStation.isAutonomous()) return RobotPhase.INGAME;
        return RobotPhase.OFF;
    }

    private LEDState resolvePreGameState() {
        if (shiftLeft) return LEDState.PREGAME_SHIFT_LEFT;
        if (shiftRight) return LEDState.PREGAME_SHIFT_RIGHT;
        if (shiftForward) return LEDState.PREGAME_SHIFT_FORWARD;
        if (shiftBack) return LEDState.PREGAME_SHIFT_BACK;
        return mechanismsStowed ? LEDState.PREGAME_READY : LEDState.PREGAME_NOT_STOWED;
    }

    private LEDState resolveInGameState() {
        if (autoStowActive) return LEDState.AUTO_STOW_ACTIVATED;
        return resolveHubGameplayState();
    }

    private LEDState resolveHubGameplayState() {
        t_gameData = safeString(DriverStation.getGameSpecificMessage());
        t_alliance = "Unknown";
        t_inactiveFirstAlliance = "Unknown";
        t_teleopSegment = "Unknown";
        t_myHubActive = false;
        t_loadingBarActive = false;
        t_loadingToActive = false;
        t_loadingProgress = 0.0;
        t_timeToNextBoundary = -1.0;

        if (DriverStation.isAutonomous()) {
            t_myHubActive = true;
            return LEDState.HUB_ACTIVE_SHOOT;
        }

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

        if (seg == TeleopSegment.TRANSITION || seg == TeleopSegment.ENDGAME) {
            t_myHubActive = true;
            return LEDState.HUB_ACTIVE_SHOOT;
        }

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

        double timeToBoundary = secondsUntilNextBoundary(t, seg);
        t_timeToNextBoundary = timeToBoundary;

        if (timeToBoundary >= 0.0 && timeToBoundary <= LOADING_BAR_SECONDS) {
            TeleopSegment nextSeg = nextTeleopSegment(seg);
            boolean activeAfter;
            if (nextSeg == TeleopSegment.TRANSITION || nextSeg == TeleopSegment.ENDGAME) {
                activeAfter = true;
            } else if (nextSeg == TeleopSegment.SHIFT_1 || nextSeg == TeleopSegment.SHIFT_2
                    || nextSeg == TeleopSegment.SHIFT_3 || nextSeg == TeleopSegment.SHIFT_4) {
                activeAfter = isMyHubActiveDuringShift(myAlliance, inactiveFirst, nextSeg);
            } else {
                activeAfter = activeNow;
            }

            if (activeAfter != activeNow) {
                t_loadingBarActive = true;
                t_loadingToActive = (activeAfter && !activeNow);
                t_loadingProgress = clamp(1.0 - (timeToBoundary / LOADING_BAR_SECONDS), 0.0, 1.0);
                return t_loadingToActive ? LEDState.LOADING_TO_ACTIVE : LEDState.LOADING_TO_INACTIVE;
            }
        }

        return activeNow ? LEDState.HUB_ACTIVE_SHOOT : LEDState.HUB_INACTIVE_PASS;
    }

    private TeleopSegment getTeleopSegment(double matchTimeRemainingSeconds) {
        if (matchTimeRemainingSeconds < 0) return TeleopSegment.UNKNOWN;
        if (matchTimeRemainingSeconds > TELEOP_TRANSITION_END) return TeleopSegment.TRANSITION;
        if (matchTimeRemainingSeconds > SHIFT_1_END) return TeleopSegment.SHIFT_1;
        if (matchTimeRemainingSeconds > SHIFT_2_END) return TeleopSegment.SHIFT_2;
        if (matchTimeRemainingSeconds > SHIFT_3_END) return TeleopSegment.SHIFT_3;
        if (matchTimeRemainingSeconds > SHIFT_4_END) return TeleopSegment.SHIFT_4;
        if (matchTimeRemainingSeconds >= ENDGAME_END) return TeleopSegment.ENDGAME;
        return TeleopSegment.UNKNOWN;
    }

    private TeleopSegment nextTeleopSegment(TeleopSegment seg) {
        return switch (seg) {
            case TRANSITION -> TeleopSegment.SHIFT_1;
            case SHIFT_1 -> TeleopSegment.SHIFT_2;
            case SHIFT_2 -> TeleopSegment.SHIFT_3;
            case SHIFT_3 -> TeleopSegment.SHIFT_4;
            case SHIFT_4 -> TeleopSegment.ENDGAME;
            case ENDGAME -> TeleopSegment.ENDGAME;
            default -> TeleopSegment.UNKNOWN;
        };
    }

    private double secondsUntilNextBoundary(double matchTimeRemainingSeconds, TeleopSegment seg) {
        if (matchTimeRemainingSeconds < 0) return -1.0;
        double boundary = switch (seg) {
            case TRANSITION -> TELEOP_TRANSITION_END;
            case SHIFT_1 -> SHIFT_1_END;
            case SHIFT_2 -> SHIFT_2_END;
            case SHIFT_3 -> SHIFT_3_END;
            case SHIFT_4 -> SHIFT_4_END;
            case ENDGAME -> ENDGAME_END;
            default -> -1.0;
        };
        if (boundary < 0) return -1.0;
        return matchTimeRemainingSeconds - boundary;
    }

    private Optional<Alliance> parseInactiveFirstAlliance(String gameData) {
        if (gameData == null || gameData.isEmpty()) return Optional.empty();
        char c = Character.toUpperCase(gameData.charAt(0));
        if (c == 'R') return Optional.of(Alliance.Red);
        if (c == 'B') return Optional.of(Alliance.Blue);
        return Optional.empty();
    }

    private boolean isMyHubActiveDuringShift(Alliance myAlliance, Alliance inactiveFirst, TeleopSegment seg) {
        boolean inactiveFirstActive = switch (seg) {
            case SHIFT_1 -> false;
            case SHIFT_2 -> true;
            case SHIFT_3 -> false;
            case SHIFT_4 -> true;
            default -> true;
        };
        boolean myIsInactiveFirst = (myAlliance == inactiveFirst);
        return myIsInactiveFirst ? inactiveFirstActive : !inactiveFirstActive;
    }

    private void transitionIfNeeded(LEDState nextState) {
        if (nextState != currentState) {
            previousState = currentState;
            currentState = nextState;
            animTimer.reset();
        }
    }

    private void renderState() {
        switch (currentState) {
            case PREGAME_NOT_STOWED -> setSolid(255, 0, 0);
            case PREGAME_READY -> setSolid(0, 0, 255);
            case PREGAME_SHIFT_LEFT -> animateSwipe(SwipeDirection.LEFT, 255, 255, 255);
            case PREGAME_SHIFT_RIGHT -> animateSwipe(SwipeDirection.RIGHT, 255, 255, 255);
            case PREGAME_SHIFT_FORWARD -> animateSwipe(SwipeDirection.INWARD, 255, 255, 255);
            case PREGAME_SHIFT_BACK -> animateSwipe(SwipeDirection.OUTWARD, 255, 255, 255);
            case AUTO_STOW_ACTIVATED -> setSolid(255, 0, 0);
            case HUB_ACTIVE_SHOOT -> setSolid(0, 255, 255);
            case HUB_INACTIVE_PASS -> setSolid(255, 255, 255);
            case LOADING_TO_ACTIVE -> renderLoadingBar(true);
            case LOADING_TO_INACTIVE -> renderLoadingBar(false);
            case GAME_DATA_UNKNOWN -> setSolid(255, 165, 0);
            case OFF -> setSolid(0, 0, 0);
        }
    }

    private void renderLoadingBar(boolean toActive) {
        int r = toActive ? 0 : 255;
        int g = toActive ? 255 : 255;
        int b = toActive ? 255 : 255;

        double progress = t_loadingProgress;
        if (!t_loadingBarActive) {
            double t = DriverStation.getMatchTime();
            TeleopSegment seg = getTeleopSegment(t);
            double dt = secondsUntilNextBoundary(t, seg);
            if (dt >= 0.0) {
                progress = clamp(1.0 - (dt / LOADING_BAR_SECONDS), 0.0, 1.0);
            } else {
                progress = 1.0;
            }
        }

        int length = io.getLength();
        int lit = (int) Math.round(progress * length);

        for (int i = 0; i < length; i++) {
            if (i < lit) {
                io.setRGB(i, r, g, b);
            } else {
                io.setRGB(i, 0, 0, 0);
            }
        }
    }

    private void setSolid(int r, int g, int b) {
        io.setAllRGB(r, g, b);
    }

    private void animateSwipe(SwipeDirection dir, int r, int g, int b) {
        int length = io.getLength();
        int head = (int) (animTimer.get() / SWIPE_STEP_SEC) % length;

        for (int i = 0; i < length; i++) {
            int distance = (head - i + length) % length;
            int physical = mapSwipeIndex(i, dir, length);

            if (distance < SWIPE_TRAIL) {
                double fade = 1.0 - ((double) distance / SWIPE_TRAIL);
                io.setRGB(physical, (int) (r * fade), (int) (g * fade), (int) (b * fade));
            } else {
                io.setRGB(physical, 0, 0, 0);
            }
        }
    }

    private int mapSwipeIndex(int i, SwipeDirection dir, int length) {
        int half = length / 2;
        return switch (dir) {
            case LEFT -> i;
            case RIGHT -> (length - 1) - i;
            case INWARD -> (i < half) ? i : (length - 1) - (i - half);
            case OUTWARD -> (i < half) ? (half - 1) - i : half + (length - 1 - i);
        };
    }

    private void publishTelemetry() {
        Logger.recordOutput("LED/State", currentState.name());
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static String safeString(String s) {
        return (s == null) ? "" : s;
    }
}
