package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.*;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.hal.HAL;
import frc.robot.Constants.IntakeConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class IntakeRollerConfigurationTest {
    @BeforeAll static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test void independentRollersHaveOppositeConfiguredPositiveDirections() {
        var left = IntakeIOTalonFX.rollerConfiguration(IntakeConstants.LEFT_ROLLER_MOTOR_INVERTED);
        var right = IntakeIOTalonFX.rollerConfiguration(IntakeConstants.RIGHT_ROLLER_MOTOR_INVERTED);
        assertEquals(InvertedValue.CounterClockwise_Positive, left.MotorOutput.Inverted);
        assertEquals(InvertedValue.Clockwise_Positive, right.MotorOutput.Inverted);
        assertEquals(IntakeConstants.ROLLER_GEARING, left.Feedback.SensorToMechanismRatio);
        assertEquals(IntakeConstants.ROLLER_GEARING, right.Feedback.SensorToMechanismRatio);
        assertTrue(left.CurrentLimits.SupplyCurrentLimitEnable);
        assertTrue(right.CurrentLimits.StatorCurrentLimitEnable);
        assertEquals(IntakeConstants.ROLLER_kV * IntakeConstants.ROLLER_GEARING, left.Slot0.kV);
    }

    @Test void simulatedRollersRespondToEnableSpeedAndStopCommand() {
        var io = new IntakeIOSim();
        var inputs = new IntakeIO.IntakeIOInputs();
        io.setRollerRPM(1000.0);
        for (int i = 0; i < 150; i++) io.updateInputs(inputs);
        assertTrue(inputs.rollerConnected && inputs.rollerConfigured);
        assertEquals(1000.0, inputs.rollerVelocityRPS * 60.0, 150.0);
        assertEquals(1000.0, inputs.rightRollerVelocityRPS * 60.0, 150.0);
        io.stop();
        io.updateInputs(inputs);
        assertEquals(0.0, inputs.rollerAppliedVolts);
        assertEquals(0.0, inputs.rightRollerAppliedVolts);
    }
}
