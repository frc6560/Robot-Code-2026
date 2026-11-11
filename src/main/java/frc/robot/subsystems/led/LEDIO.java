package frc.robot.subsystems.led;


public interface LEDIO {
    default void setRGB(int index, int r, int g, int b) {}

    default void setAllRGB(int r, int g, int b) {}

    default void setData() {}

    default int getLength() { return 0; }
}
