package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;

class IndexerTest {
    @Test
    void floorCommandsReachIOAndDiagnosticsAreSeparateFromTunnel() {
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
                inputs.floorTelemetryStatus = "OK";
                inputs.tunnelConnected = true;
                inputs.tunnelRPM = 1200;
                inputs.tunnelAppliedVolts = 4;
                inputs.tunnelCurrentAmps = 7;
                inputs.tunnelConfigurationStatus = "Tunnel config status";
                inputs.tunnelControlStatus = "Tunnel control status";
                inputs.tunnelTelemetryStatus = "Tunnel refresh status";
            }
        }
        FakeIO io = new FakeIO();
        Indexer indexer = new Indexer(io);
        try {
            indexer.setFloorRPM(1000);
            indexer.setTunnelRPM(1500);
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
            assertEquals("OK", SmartDashboard.getString("Indexer/FloorTelemetryStatus", ""));
            assertEquals(0, SmartDashboard.getNumber("Indexer/TunnelTargetRPM", -1));
            assertEquals(1200, SmartDashboard.getNumber("Indexer/TunnelRPM", -1));
            assertEquals(4, SmartDashboard.getNumber("Indexer/TunnelVolts", -1));
            assertEquals(7, SmartDashboard.getNumber("Indexer/TunnelAmps", -1));
            assertTrue(SmartDashboard.getBoolean("Indexer/TunnelConnected", false));
            assertEquals("Tunnel config status", SmartDashboard.getString("Indexer/TunnelConfigurationStatus", ""));
            assertEquals("Tunnel control status", SmartDashboard.getString("Indexer/TunnelControlStatus", ""));
            assertEquals("Tunnel refresh status", SmartDashboard.getString("Indexer/TunnelTelemetryStatus", ""));
            indexer.stop();
            assertEquals(0, io.floorTarget);
            assertEquals(0, io.tunnelTarget);
        } finally {
            CommandScheduler.getInstance().unregisterSubsystem(indexer);
        }
    }
}
