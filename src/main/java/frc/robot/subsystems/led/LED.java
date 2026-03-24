package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private static final double SWIPE_STEP_SEC = 0.03;
    private static final int SWIPE_TRAIL = 12;
    private static final double BLINK_PERIOD_SEC = 0.1;

    // Colors: 00b6ae = ready, ffffff = not ready
    private static final int READY_R = 0x00;
    private static final int READY_G = 0xb6;
    private static final int READY_B = 0xae;

    private static final int NOT_READY_R = 0xff;
    private static final int NOT_READY_G = 0xff;
    private static final int NOT_READY_B = 0xff;

    private enum RobotPhase { PREGAME, INGAME, OFF }

    public enum LEDState {
        PREGAME_NOT_STOWED,
        PREGAME_READY,
        PREGAME_SHIFT_LEFT,
        PREGAME_SHIFT_RIGHT,
        PREGAME_SHIFT_FORWARD,
        PREGAME_SHIFT_BACK,
        SHOOT_READY,
        SHOOT_NOT_READY,
        OFF
    }

    private enum SwipeDirection { LEFT, RIGHT, INWARD, OUTWARD }

    private final LEDIO io;
    private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();
    private final Timer animTimer = new Timer();

    // Robot state
    private boolean mechanismsStowed = false;
    private boolean shiftLeft = false;
    private boolean shiftRight = false;
    private boolean shiftForward = false;
    private boolean shiftBack = false;

    // Shoot state
    private boolean shootIntent = false;
    private boolean readyToShoot = false;

    // State machine
    private LEDState currentState = LEDState.OFF;

    public LED(LEDIO io) {
        this.io = io;
        animTimer.start();
    }

    // Public setters
    public void setMechanismsStowed(boolean stowed) { this.mechanismsStowed = stowed; }
    public void setShiftLeft(boolean active) { this.shiftLeft = active; }
    public void setShiftRight(boolean active) { this.shiftRight = active; }
    public void setShiftForward(boolean active) { this.shiftForward = active; }
    public void setShiftBack(boolean active) { this.shiftBack = active; }
    public void setShootIntent(boolean intent) { this.shootIntent = intent; }
    public void setReadyToShoot(boolean ready) { this.readyToShoot = ready; }

    public LEDState getCurrentState() { return currentState; }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LED", inputs);

        LEDState nextState = resolveState();
        if (nextState != currentState) {
            currentState = nextState;
            animTimer.reset();
        }
        renderState();
        io.setData();

        Logger.recordOutput("LED/State", currentState.name());
        Logger.recordOutput("LED/ShootIntent", shootIntent);
        Logger.recordOutput("LED/ReadyToShoot", readyToShoot);
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
        return readyToShoot ? LEDState.SHOOT_READY : LEDState.SHOOT_NOT_READY;
    }

    private void renderState() {
        switch (currentState) {
            case PREGAME_NOT_STOWED -> setSolid(255, 0, 0);
            case PREGAME_READY -> setSolid(0, 0, 255);
            case PREGAME_SHIFT_LEFT -> animateSwipe(SwipeDirection.LEFT, 255, 255, 255);
            case PREGAME_SHIFT_RIGHT -> animateSwipe(SwipeDirection.RIGHT, 255, 255, 255);
            case PREGAME_SHIFT_FORWARD -> animateSwipe(SwipeDirection.INWARD, 255, 255, 255);
            case PREGAME_SHIFT_BACK -> animateSwipe(SwipeDirection.OUTWARD, 255, 255, 255);
            case SHOOT_READY -> renderShootState(READY_R, READY_G, READY_B);
            case SHOOT_NOT_READY -> renderShootState(NOT_READY_R, NOT_READY_G, NOT_READY_B);
            case OFF -> setSolid(0, 0, 0);
        }
    }

    private void renderShootState(int r, int g, int b) {
        if (shootIntent) {
            // Blink when shoot intent is active
            boolean on = ((int) (Timer.getFPGATimestamp() / BLINK_PERIOD_SEC)) % 2 == 0;
            if (on) {
                setSolid(r, g, b);
            } else {
                setSolid(0, 0, 0);
            }
        } else {
            setSolid(r, g, b);
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
}
