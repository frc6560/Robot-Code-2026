package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

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

    public boolean getButton(int buttonNumber){
        return m_buttonBoard.getRawButtonPressed(buttonNumber);
    }

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
        return getButtonRelease(1);
      }
  
      public boolean getClimbTrigger(){
        return getButton(3);
      }
  
      public boolean getClimbReleaseTrigger(){
        return getButtonRelease(3);
      }
  
      public boolean getAutoAlignTrigger(){
        return getButton(10);
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
