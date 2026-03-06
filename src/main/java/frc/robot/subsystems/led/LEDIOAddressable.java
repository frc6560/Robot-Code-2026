package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDIOAddressable implements LEDIO {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final int length;
    private boolean connected = true;

    public LEDIOAddressable(int port, int length) {
        this.length = length;
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.setData(buffer);
        led.start();
    }

    @Override
    public void updateInputs(LEDIOInputs inputs) {
        inputs.connected = connected;
        inputs.ledCount = length;
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        if (index >= 0 && index < length) {
            buffer.setRGB(index, r, g, b);
        }
    }

    @Override
    public void setAllRGB(int r, int g, int b) {
        for (int i = 0; i < length; i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    @Override
    public void setData() {
        led.setData(buffer);
    }

    @Override
    public int getLength() {
        return length;
    }
}
