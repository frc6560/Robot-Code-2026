package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
    @AutoLog
    public static class LEDIOInputs {
        public boolean connected = false;
        public int ledCount = 0;
    }

    default void updateInputs(LEDIOInputs inputs) {}

    default void setRGB(int index, int r, int g, int b) {}

    default void setAllRGB(int r, int g, int b) {}

    default void setData() {}

    default int getLength() { return 0; }
}
