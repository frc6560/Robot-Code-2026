package frc.robot.subsystems.led;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utility.Shooter.ShotCalculator;

public class LED extends SubsystemBase {

    private static final double SWIPE_STEP_SEC = 0.03;
    private static final int SWIPE_TRAIL = 12;
    private static final double BLINK_PERIOD_SEC = 0.1;

    // Hub shift timing (seconds remaining in teleop)
    private static final double SHIFT_1_END = 105.0;
    private static final double SHIFT_2_END = 80.0;
    private static final double SHIFT_3_END = 55.0;
    private static final double SHIFT_4_END = 30.0;

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

    // Subsystems for tolerance checking
    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;
    private final ShotCalculator shotCalculator;

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

    public LED(LEDIO io, Hood hood, Shooter shooter, Turret turret, ShotCalculator shotCalculator) {
        this.io = io;
        this.hood = hood;
        this.shooter = shooter;
        this.turret = turret;
        this.shotCalculator = shotCalculator;
        animTimer.start();
    }

    // Public setters
    public void setMechanismsStowed(boolean stowed) { this.mechanismsStowed = stowed; }
    public void setShiftLeft(boolean active) { this.shiftLeft = active; }
    public void setShiftRight(boolean active) { this.shiftRight = active; }
    public void setShiftForward(boolean active) { this.shiftForward = active; }
    public void setShiftBack(boolean active) { this.shiftBack = active; }
    public void setShootIntent(boolean intent) { this.shootIntent = intent; }

    public LEDState getCurrentState() { return currentState; }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LED", inputs);

        // Always calculate ready to shoot based on tolerances
        boolean shotValid = shotCalculator.isShotValid();
        double turretTolerance = shotCalculator.getTurretTolerance();
        boolean turretReady = turret.getAtTarget(turretTolerance);
        boolean hoodReady = hood.atTarget();
        boolean shooterReady = shooter.atTarget();

        readyToShoot = shotValid && turretReady && hoodReady && shooterReady;

        Logger.recordOutput("LED/ShotValid", shotValid);
        Logger.recordOutput("LED/TurretReady", turretReady);
        Logger.recordOutput("LED/HoodReady", hoodReady);
        Logger.recordOutput("LED/ShooterReady", shooterReady);

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

        publishHubShiftInfo();
    }

    private void publishHubShiftInfo() {
        double matchTime = DriverStation.getMatchTime();
        Logger.recordOutput("HubShift/MatchTimeRemaining", matchTime);

        // Determine current shift (1-4) based on match time
        int currentShift = 0;
        if (matchTime > SHIFT_1_END) {
            currentShift = 0; // Transition period
        } else if (matchTime > SHIFT_2_END) {
            currentShift = 1;
        } else if (matchTime > SHIFT_3_END) {
            currentShift = 2;
        } else if (matchTime > SHIFT_4_END) {
            currentShift = 3;
        } else {
            currentShift = 4; // Endgame
        }
        Logger.recordOutput("HubShift/CurrentShift", currentShift);

        // Parse game data to determine which alliance is inactive first
        String gameData = DriverStation.getGameSpecificMessage();
        Optional<Alliance> myAllianceOpt = DriverStation.getAlliance();

        boolean myHubActive = true; // Default to active
        String inactiveFirstAlliance = "Unknown";

        if (gameData != null && !gameData.isEmpty() && myAllianceOpt.isPresent()) {
            char firstChar = Character.toUpperCase(gameData.charAt(0));
            Alliance inactiveFirst = (firstChar == 'R') ? Alliance.Red : (firstChar == 'B') ? Alliance.Blue : null;

            if (inactiveFirst != null) {
                inactiveFirstAlliance = (inactiveFirst == Alliance.Red) ? "Red" : "Blue";
                Alliance myAlliance = myAllianceOpt.get();
                boolean iAmInactiveFirst = (myAlliance == inactiveFirst);

                // Shifts 1 and 3: inactive-first alliance is inactive
                // Shifts 2 and 4: inactive-first alliance is active
                boolean inactiveFirstIsActive = (currentShift == 2 || currentShift == 4);
                myHubActive = iAmInactiveFirst ? inactiveFirstIsActive : !inactiveFirstIsActive;

                // During transition (shift 0) or endgame (shift 4+), both are active
                if (currentShift == 0 || currentShift >= 4) {
                    myHubActive = true;
                }
            }
        }

        Logger.recordOutput("HubShift/InactiveFirstAlliance", inactiveFirstAlliance);
        Logger.recordOutput("HubShift/MyHubActive", myHubActive);
        Logger.recordOutput("HubShift/Shift1EndTime", SHIFT_1_END);
        Logger.recordOutput("HubShift/Shift2EndTime", SHIFT_2_END);
        Logger.recordOutput("HubShift/Shift3EndTime", SHIFT_3_END);
        Logger.recordOutput("HubShift/Shift4EndTime", SHIFT_4_END);
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
        if (!shootIntent) {
            return LEDState.OFF; // No shoot intent, LEDs off
        }
        return readyToShoot ? LEDState.SHOOT_READY : LEDState.SHOOT_NOT_READY;
    }

    private void renderState() {
        switch (currentState) {
            case PREGAME_NOT_STOWED -> {
                setSolid(255, 0, 0);
                Logger.recordOutput("LED/Color", "Red (255,0,0)");
            }
            case PREGAME_READY -> {
                setSolid(0, 0, 255);
                Logger.recordOutput("LED/Color", "Blue (0,0,255)");
            }
            case PREGAME_SHIFT_LEFT -> {
                animateSwipe(SwipeDirection.LEFT, 255, 255, 255);
                Logger.recordOutput("LED/Color", "White Swipe Left");
            }
            case PREGAME_SHIFT_RIGHT -> {
                animateSwipe(SwipeDirection.RIGHT, 255, 255, 255);
                Logger.recordOutput("LED/Color", "White Swipe Right");
            }
            case PREGAME_SHIFT_FORWARD -> {
                animateSwipe(SwipeDirection.INWARD, 255, 255, 255);
                Logger.recordOutput("LED/Color", "White Swipe Inward");
            }
            case PREGAME_SHIFT_BACK -> {
                animateSwipe(SwipeDirection.OUTWARD, 255, 255, 255);
                Logger.recordOutput("LED/Color", "White Swipe Outward");
            }
            case SHOOT_READY -> {
                renderShootState(READY_R, READY_G, READY_B);
                Logger.recordOutput("LED/Color", "Cyan/Ready (0,182,174)");
            }
            case SHOOT_NOT_READY -> {
                renderShootState(NOT_READY_R, NOT_READY_G, NOT_READY_B);
                Logger.recordOutput("LED/Color", "White/Not Ready (255,255,255)");
            }
            case OFF -> {
                setSolid(0, 0, 0);
                Logger.recordOutput("LED/Color", "Off (0,0,0)");
            }
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
