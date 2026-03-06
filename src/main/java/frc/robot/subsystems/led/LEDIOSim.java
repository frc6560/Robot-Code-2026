package frc.robot.subsystems.led;

public class LEDIOSim implements LEDIO {
    private final int length;
    private final int[][] colors;

    public LEDIOSim(int length) {
        this.length = length;
        this.colors = new int[length][3];
    }

    @Override
    public void updateInputs(LEDIOInputs inputs) {
        inputs.connected = true;
        inputs.ledCount = length;
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        if (index >= 0 && index < length) {
            colors[index][0] = r;
            colors[index][1] = g;
            colors[index][2] = b;
        }
    }

    @Override
    public void setAllRGB(int r, int g, int b) {
        for (int i = 0; i < length; i++) {
            colors[i][0] = r;
            colors[i][1] = g;
            colors[i][2] = b;
        }
    }

    @Override
    public void setData() {
        // No-op in simulation, colors are already stored
    }

    @Override
    public int getLength() {
        return length;
    }

    public int[] getColor(int index) {
        if (index >= 0 && index < length) {
            return colors[index].clone();
        }
        return new int[]{0, 0, 0};
    }
}
