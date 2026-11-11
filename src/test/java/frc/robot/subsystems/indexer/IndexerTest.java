package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;

class IndexerTest {
    @Test
    void floorAndTunnelCommandsReachIndependentOutputs() {
        assertTrue(HAL.initialize(500, 0));
        class FakeIO implements IndexerIO {
            double floorTarget;
            double tunnelTarget;

            @Override
            public void setFloorRPM(double rpm) { floorTarget = rpm; }

            @Override
            public void setTunnelRPM(double rpm) { tunnelTarget = rpm; }

            @Override
            public void stop() { floorTarget = 0; tunnelTarget = 0; }

            @Override
            public void updateInputs(IndexerIOInputs inputs) {
                inputs.floorConnected = true;
                inputs.floorRPM = 950;
                inputs.floorPositionRotations = 12.5;
                inputs.floorSecondaryPositionRotations = 12.4;
                inputs.tunnelPositionRotations = 8.25;
                inputs.floorAppliedVolts = 5;
                inputs.floorCurrentAmps = 12;
                inputs.appliedVolts = 0; // Tunnel stopped, floor running.
            }
        }
        FakeIO io = new FakeIO();
        Indexer indexer = new Indexer(io);
        try {
            indexer.setFloorRPM(1000);
            indexer.setTunnelRPM(1500);
            assertEquals(1500, io.tunnelTarget);
            indexer.setTunnelRPM(0);
            assertEquals(1000, io.floorTarget);
            assertEquals(0, io.tunnelTarget);
            indexer.periodic();
            assertEquals(1000 * frc.robot.Constants.IndexerConstants.FLOOR_GEAR_RATIO,
                SmartDashboard.getNumber("Indexer/FloorMotorTargetRPM", -1));
            assertEquals(12.5, SmartDashboard.getNumber("Indexer/FloorPositionRotations", -1));
            assertEquals(12.4, SmartDashboard.getNumber("Indexer/FloorSecondaryPositionRotations", -1));
            assertEquals(8.25, SmartDashboard.getNumber("Indexer/TunnelPositionRotations", -1));
            assertEquals(950 * frc.robot.Constants.IndexerConstants.FLOOR_GEAR_RATIO,
                SmartDashboard.getNumber("Indexer/FloorMotorRPM", -1));
            assertEquals(5, SmartDashboard.getNumber("Indexer/FloorVolts", -1));
            assertEquals(12, SmartDashboard.getNumber("Indexer/FloorAmps", -1));
            indexer.stop();
            assertEquals(0, io.floorTarget);
            assertEquals(0, io.tunnelTarget);
        } finally {
            CommandScheduler.getInstance().unregisterSubsystem(indexer);
        }
    }
    @Test void sustainedLowSpeedAndHighCurrentDoNotCancelFloorRequest() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        class BlockedIO implements IndexerIO {
            double requested;
            @Override public void setFloorRPM(double rpm) { requested = rpm; }
            @Override public void updateInputs(IndexerIOInputs inputs) {
                inputs.floorRPM = 100; // First motor moves, secondary is stationary.
                inputs.floorSecondaryRPM = 0;
                inputs.floorSecondaryCurrentAmps = 40;
            }
        }
        var io = new BlockedIO();
        var indexer = new Indexer(io);
        try {
            indexer.setFloorRPM(800); indexer.periodic();
            SimHooks.stepTiming(0.5); indexer.periodic();
            assertEquals(800, io.requested);
            SimHooks.stepTiming(2.0); indexer.periodic();
            assertEquals(800, io.requested);
            assertEquals(800, indexer.getFloorTargetRPM());
            indexer.setFloorRPM(0); assertEquals(0, io.requested);
            indexer.setFloorRPM(800); assertEquals(800, io.requested);
        } finally {
            CommandScheduler.getInstance().unregisterSubsystem(indexer);
            DriverStationSim.resetData(); DriverStationSim.notifyNewData();
            SimHooks.resumeTiming();
        }
    }
}
