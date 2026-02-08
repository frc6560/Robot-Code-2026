package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ManualControls {

    private final XboxController secondXbox;
    private final XboxController firstXbox;

    /* ================= UTIL ================= */

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
        }
        return 0.0;
    }

    private static double modifyAxis(double value) {
        value = deadband(value, 0.01);
        return Math.copySign(value * value, value);
    }

    /* ================= CONSTRUCTOR ================= */

    public ManualControls(XboxController firstXbox, XboxController secondXbox) {
        this.firstXbox = firstXbox;
        this.secondXbox = secondXbox;
    }

    /* ================= CLIMB ================= */

    public boolean getClimbDown() {
        return secondXbox.getRightY() > 0.7;
    }

    public boolean getClimbUp() {
        return secondXbox.getRightY() < -0.7;
    }

    /* ================= ELEVATOR ================= */

    public boolean goToStow() {
        return secondXbox.getPOV() == 180;
    }

    public boolean goToL2Ball() {
        return secondXbox.getXButton();
    }

    public boolean goToL3Ball() {
        return secondXbox.getBButton();
    }

    public boolean goToShootBall() {
        return secondXbox.getYButton();
    }

    public boolean goToGroundBall() {
        return secondXbox.getAButton();
    }

    public boolean shiftedControls() {
        return secondXbox.getRightBumperButton();
    }

    /* ================= INTAKE ================= */

    public boolean runIntake() {
        return secondXbox.getLeftBumperButton();
    }

    public boolean runOuttake() {
        return secondXbox.getRightBumperButton();
    }

    /* ================= MISC ================= */

    public boolean zeroNoAprilTagsGyro() {
        return secondXbox.getStartButton();
    }

    /* ===================================================== */
    /* ================= REVOLVER CONTROLS ================= */
    /* ===================================================== */

    /** Hold to run revolver feed (state machine handles rest) */
    public Trigger revolverFeed() {
        return new Trigger(secondXbox::getYButton);
    }

    /** Emergency stop */
    public Trigger revolverStop() {
        return new Trigger(secondXbox::getAButton);
    }

    /** Beam-break override */
    public Trigger revolverOverride() {
        return new Trigger(secondXbox::getLeftBumperButton);
    }
}
