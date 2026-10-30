package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.*;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.IndexerConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class IndexerMotorConfigurationTest {
    @BeforeAll static void initializeHal() { assertTrue(HAL.initialize(500, 0)); }

    @Test void floorMotorsUseConfirmedIdsAndOppositePhysicalDirections() {
        assertEquals(25, IndexerConstants.FLOOR_MOTOR_ID);
        assertEquals(0, IndexerConstants.FLOOR_SECONDARY_MOTOR_ID);
        assertEquals(16, IndexerConstants.TUNNEL_MOTOR_ID);
        var primary = IndexerIOTalonFX.motorConfiguration(IndexerConstants.FLOOR_GEAR_RATIO,
            IndexerConstants.FLOOR_MOTOR_INVERTED, true);
        var secondary = IndexerIOTalonFX.motorConfiguration(IndexerConstants.FLOOR_GEAR_RATIO,
            IndexerConstants.FLOOR_SECONDARY_MOTOR_INVERTED, true);
        assertEquals(InvertedValue.Clockwise_Positive, primary.MotorOutput.Inverted);
        assertEquals(InvertedValue.CounterClockwise_Positive, secondary.MotorOutput.Inverted);
        assertEquals(primary.Feedback.SensorToMechanismRatio, secondary.Feedback.SensorToMechanismRatio);
        assertEquals(60, primary.CurrentLimits.StatorCurrentLimit);
        assertEquals(40, primary.CurrentLimits.SupplyCurrentLimit);
        assertEquals(60, secondary.CurrentLimits.StatorCurrentLimit);
        var tunnel = IndexerIOTalonFX.motorConfiguration(IndexerConstants.TUNNEL_GEAR_RATIO, false);
        assertEquals(40, tunnel.CurrentLimits.StatorCurrentLimit);
        assertTrue(primary.CurrentLimits.StatorCurrentLimitEnable);
        assertTrue(secondary.CurrentLimits.SupplyCurrentLimitEnable);
    }

    @Test void bothSimulatedFloorMotorsRespondToOneTargetAndStopTogether() {
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        try {
            var io = new IndexerIOSim();
            var inputs = new IndexerIO.IndexerIOInputs();
            io.setFloorRPM(1000.0);
            io.updateInputs(inputs);
            assertTrue(inputs.floorConnected && inputs.floorSecondaryConnected);
            assertTrue(inputs.floorAppliedVolts > 0.0);
            assertTrue(inputs.floorSecondaryAppliedVolts > 0.0);
            // Encoders report positive mechanism velocity after each configured inversion.
            assertEquals(inputs.floorRPM, inputs.floorSecondaryRPM, 1e-9);
            io.stop();
            io.updateInputs(inputs);
            assertEquals(0.0, inputs.floorAppliedVolts);
            assertEquals(0.0, inputs.floorSecondaryAppliedVolts);
            io.setFloorRPM(1000.0);
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            io.updateInputs(inputs);
            assertEquals(0.0, inputs.floorAppliedVolts);
            assertEquals(0.0, inputs.floorSecondaryAppliedVolts);
        } finally {
            DriverStationSim.resetData();
            DriverStationSim.notifyNewData();
        }
    }
}
