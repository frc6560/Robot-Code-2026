package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ManualControls {

    private final Joystick m_buttonBoard;

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

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.01);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        return value;
    }

    public ManualControls(int USB_ID){
        m_buttonBoard = new Joystick(USB_ID);
    }

    // THE FIX: This now reads the continuous physical state (Held down)
    public boolean getButton(int buttonNumber){
        return m_buttonBoard.getRawButton(buttonNumber);
    }

    // This reads the exact moment you let go
    public boolean getButtonRelease(int buttonNumber){
        return m_buttonBoard.getRawButtonReleased(buttonNumber);
    }

    public boolean getShootTrigger(){
      return getButton(12);
    }

    public boolean getShootReleaseTrigger(){
      return getButtonRelease(12);
    }

    public boolean getIntakeReleaseTrigger(){
      return getButton(1);
    }

    public boolean getIntakeTrigger(){
      return getButtonRelease(1) || getButtonRelease(2);
    }

    public boolean getIntakeRollingTrigger(){
      return getButton(2);
    }

    // CLIMB SWITCH - UP POSITION
    public boolean getClimbTrigger(){
      return getButton(9); 
    }

    // CLIMB SWITCH - LETTING GO OF UP
    public boolean getClimbReleaseTrigger(){
      return getButtonRelease(9);
    }

    // CLIMB SWITCH - DOWN POSITION
    public boolean getAutoAlignTrigger(){
      return getButton(10); // Change this 10 to the ID you found in the Driver Station!
    }

    public boolean getVisionResetTrigger(){
      return getButton(5);
    }

    public boolean getIntakeResetTrigger(){
      return getButton(6);
    }

    public boolean getClimbResetTrigger(){
      return getButton(7);
    }
}