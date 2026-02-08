package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class ManualControls {

    private final XboxController secondXbox;
    private final XboxController firstXbox;

    public ManualControls(XboxController firstXbox, XboxController secondXbox) {
        this.secondXbox = secondXbox;
        this.firstXbox = firstXbox;
    }

    /* ================= UTILS ================= */

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            return (value > 0.0)
                ? (value - deadband) / (1.0 - deadband)
                : (value + deadband) / (1.0 - deadband);
        }
        return 0.0;
    }

    private static double modifyAxis(double value) {
        value = deadband(value, 0.01);
        return Math.copySign(value * value, value);
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

    public boolean shiftedControls() {
        return secondXbox.getRightBumperButton();
    }

    /* ================= INTAKE ================= */

    public boolean runIntake() {
        return secondXbox.getLeftBumperButton();
    }

    /* ================= FLYWHEEL ================= */

    public boolean shootWithLimelight() {
        return secondXbox.getRightBumperButton();
    }

    /* ================= REVOLVER ================= */
    // These are what RobotContainer should bind to commands

    /** Y button: start feeding (state machine handles rest) */
    public boolean revolverFeed() {
        return secondXbox.getYButton();
    }

    /** A button: emergency stop */
    public boolean revolverStop() {
        return secondXbox.getAButton();
    }

    /** Left bumper: beam break override */
    public boolean revolverOverride() {
        return secondXbox.getLeftBumperButton();
    }

    /* ================= MISC ================= */

    public boolean zeroNoAprilTagsGyro() {
        return secondXbox.getStartButton();
    }
}
