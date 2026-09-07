package frc.robot.subsystems.hood;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

class HoodIOSimTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void positiveTargetMovesHoodUpward() {
        HoodIOSim io = new HoodIOSim();
        HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();
        io.updateInputs(inputs);
        double initialAngle = inputs.hoodAngleDegrees;

        io.setTargetAngle(initialAngle + 2.0);
        for (int i = 0; i < 50; i++) {
            io.updateInputs(inputs);
        }

        assertTrue(
            inputs.hoodAngleDegrees > initialAngle,
            "hood did not move upward: initial=" + initialAngle
                + ", current=" + inputs.hoodAngleDegrees
                + ", volts=" + inputs.motorAppliedVolts);
    }
}
