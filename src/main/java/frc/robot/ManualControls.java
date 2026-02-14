package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class ManualControls {

    private final XboxController secondXbox;
    private final XboxController firstXbox;

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }

    public ManualControls(XboxController firstXbox, XboxController secondXbox) {
        this.secondXbox = secondXbox;
        this.firstXbox = firstXbox;
    }

    // --- SHOOTER CONTROLS ---

    /** Right Trigger Revs the Flywheels */
    public boolean getShooterRev() {
        return secondXbox.getRightTriggerAxis() > 0.5; 
    }

    /** Left Trigger Feeds the Note (Intake) */
    public boolean getShooterFeed() {
        return secondXbox.getLeftTriggerAxis() > 0.5;
    }

    // --- MECHANISMS ---

    public boolean getClimbDown() {
      return secondXbox.getRightY() > 0.7; 
    }

    public boolean getClimbUp() {
      return secondXbox.getRightY() < -0.7;
    }

    public boolean goToStow(){
      return secondXbox.getPOV() == 180;
    }
    
    public boolean goToL2Ball(){
        return secondXbox.getXButton();
    }

    public boolean goToL3Ball(){
        return secondXbox.getBButton();
    }

    public boolean goToShootBall(){
        return secondXbox.getYButton();
    }

    public boolean shiftedControls(){
      return secondXbox.getRightBumperButton();
    }

    public boolean goToGroundBall() {
      return secondXbox.getAButton();
    }

    public boolean runIntake(){
      return secondXbox.getLeftBumperButton();
    }

    public boolean runOuttake() {
      // FIXED: Moved to Back Button to avoid conflict with Shift
      return secondXbox.getBackButton(); 
    }

    public boolean zeroNoAprilTagsGyro() {
      return secondXbox.getStartButton();
    }
}