package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/** Button-board controls still used alongside the driver Xbox controller. */
public class ManualControls {
    private final Joystick buttonBoard;

    public ManualControls(int port) {
        buttonBoard = new Joystick(port);
    }

    public boolean getButtonActivated(int button) {
        return buttonBoard.getRawButton(button);
    }

    public boolean getVisionResetTrigger() {
        return buttonBoard.getRawButtonPressed(5);
    }
}
