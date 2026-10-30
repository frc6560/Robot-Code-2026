package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
        assertEquals(40, left.CurrentLimits.StatorCurrentLimit);
        assertEquals(40, right.CurrentLimits.StatorCurrentLimit);
        assertEquals(40, left.CurrentLimits.SupplyCurrentLimit);
        assertEquals(40, right.CurrentLimits.SupplyCurrentLimit);
        assertEquals(40, IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT);
        assertTrue(IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT <= 40);
        assertEquals(NeutralModeValue.Brake, left.MotorOutput.NeutralMode);
        assertEquals(NeutralModeValue.Brake, right.MotorOutput.NeutralMode);
        assertTrue(left.CurrentLimits.SupplyCurrentLimitEnable);
        assertTrue(right.CurrentLimits.StatorCurrentLimitEnable);
        assertEquals(IntakeConstants.ROLLER_kV * IntakeConstants.ROLLER_GEARING, left.Slot0.kV);
    }

    @Test void simulatedRollersRespondToEnableSpeedAndStopCommand() {
        var io = new IntakeIOSim();
        var inputs = new IntakeIO.IntakeIOInputs();
        io.setRollerRPM(2500.0);
        for (int i = 0; i < 150; i++) io.updateInputs(inputs);
        assertTrue(inputs.rollerConnected && inputs.rollerConfigured);
        assertEquals(2500.0, inputs.rollerVelocityRPS * 60.0, 150.0);
        assertEquals(2500.0, inputs.rightRollerVelocityRPS * 60.0, 150.0);
        io.stop();
        io.updateInputs(inputs);
        assertEquals(0.0, inputs.rollerAppliedVolts);
        assertEquals(0.0, inputs.rightRollerAppliedVolts);
    }
    @Test void hardwareBoundaryRejectsRollersUnlessRackFeedbackConfirmsExtension() {
        double end = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        assertTrue(IntakeIOTalonFX.rackAllowsRollers(true, true, false, end, end, 0));
        assertFalse(IntakeIOTalonFX.rackAllowsRollers(true, true, false, end, 0, 0));
        assertFalse(IntakeIOTalonFX.rackAllowsRollers(true, true, false, 0, end, 0));
        assertFalse(IntakeIOTalonFX.rackAllowsRollers(false, true, false, end, end, 0));
        assertFalse(IntakeIOTalonFX.rackAllowsRollers(true, false, false, end, end, 0));
        assertFalse(IntakeIOTalonFX.rackAllowsRollers(true, true, true, end, end, 0));
        assertFalse(IntakeIOTalonFX.rackAllowsRollers(true, true, false, end, end, 1));
        assertFalse(IntakeIOTalonFX.rackAllowsRollers(true, true, false, end, Double.NaN, 0));
        assertFalse(IntakeIOTalonFX.rackAllowsRollers(true, true, false, end, end, Double.NaN));
    }
    @Test void rackAndRollersKeepContinuousCurrentLimits() {
        var startup = IntakeIOTalonFX.deployCurrentLimits(IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT);
        var running = IntakeIOTalonFX.deployCurrentLimits(40);
        assertEquals(40, startup.StatorCurrentLimit);
        assertEquals(40, running.StatorCurrentLimit);
        assertEquals(IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT, startup.SupplyCurrentLimit);
        assertEquals(IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT, running.SupplyCurrentLimit);
        assertEquals(40, IntakeIOTalonFX.rollerConfiguration(false).CurrentLimits.StatorCurrentLimit);
    }
}
