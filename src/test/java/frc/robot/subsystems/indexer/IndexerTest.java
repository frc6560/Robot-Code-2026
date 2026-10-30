package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
                inputs.floorAppliedVolts = 5;
                inputs.floorCurrentAmps = 12;
                inputs.appliedVolts = 0; // Tunnel stopped, floor running.
                inputs.floorConfigurationStatus = "OK";
                inputs.floorControlStatus = "OK";
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
            assertEquals(1000, SmartDashboard.getNumber("Indexer/FloorTargetRPM", -1));
            assertEquals(950, SmartDashboard.getNumber("Indexer/FloorRPM", -1));
            assertEquals(5, SmartDashboard.getNumber("Indexer/FloorVolts", -1));
            assertEquals(12, SmartDashboard.getNumber("Indexer/FloorAmps", -1));
            assertTrue(SmartDashboard.getBoolean("Indexer/FloorConnected", false));
            assertEquals("OK", SmartDashboard.getString("Indexer/FloorConfigurationStatus", ""));
            assertEquals("OK", SmartDashboard.getString("Indexer/FloorControlStatus", ""));
            indexer.stop();
            assertEquals(0, io.floorTarget);
            assertEquals(0, io.tunnelTarget);
        } finally {
            CommandScheduler.getInstance().unregisterSubsystem(indexer);
        }
    }
    @Test void eitherStalledFloorMotorStopsBothAndRequiresDisableToReset() {
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
                inputs.floorRPM = 100; // First motor moves; secondary alone can trip the guard.
                inputs.floorSecondaryRPM = 0;
                inputs.floorSecondaryCurrentAmps = 60;
            }
        }
        var io = new BlockedIO();
        var indexer = new Indexer(io);
        try {
            indexer.setFloorRPM(800); indexer.periodic();
            SimHooks.stepTiming(0.5); indexer.periodic();
            assertEquals(800, io.requested);
            SimHooks.stepTiming(0.6); indexer.periodic();
            assertEquals(0, io.requested);
            indexer.setFloorRPM(800); assertEquals(0, io.requested);
            assertTrue(SmartDashboard.getBoolean("Indexer/FloorStallFault", false));
            DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData();
            indexer.periodic();
            indexer.setFloorRPM(800); assertEquals(800, io.requested);
        } finally {
            CommandScheduler.getInstance().unregisterSubsystem(indexer);
            DriverStationSim.resetData(); DriverStationSim.notifyNewData();
            SimHooks.resumeTiming();
        }
    }
}
