package frc.robot.subsystems.feeder;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FeederConstants;
import org.junit.jupiter.api.Test;

class FeederTest {
    @Test
    void wallRunsOnlyInEnabledTeleopAndStopsForPitCoast() {
        assertTrue(HAL.initialize(500, 0));
        class FakeIO implements FeederIO {
            double wallRPM;
            double panRPM;
            double pusherRPM;
            @Override public void setWallRPM(double rpm) { wallRPM = rpm; }
            @Override public void setPanRPM(double rpm) { panRPM = rpm; }
            @Override public void setPusherRPM(double rpm) { pusherRPM = rpm; }
            @Override public void stop() { wallRPM = panRPM = pusherRPM = 0; }
        }
        FakeIO io = new FakeIO();
        Feeder feeder = new Feeder(io);
        try {
            DriverStationSim.resetData();
            DriverStationSim.setDsAttached(true);
            DriverStationSim.notifyNewData();
            feeder.periodic();
            assertEquals(0, io.wallRPM);

            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
            feeder.periodic();
            assertEquals(FeederConstants.WALL_RPM, io.wallRPM);
            assertEquals(FeederConstants.WALL_RPM,
                SmartDashboard.getNumber("Feeder/WallTargetRPM", -1));
            assertEquals(0, io.panRPM);
            assertEquals(0, io.pusherRPM);

            feeder.setPitCoastMode(true);
            assertEquals(0, io.wallRPM);
            feeder.periodic();
            assertEquals(0, io.wallRPM);
            feeder.setPitCoastMode(false);
            feeder.periodic();
            assertEquals(FeederConstants.WALL_RPM, io.wallRPM);

            DriverStationSim.setAutonomous(true);
            DriverStationSim.notifyNewData();
            feeder.periodic();
            assertEquals(0, io.wallRPM);
            DriverStationSim.setAutonomous(false);
            DriverStationSim.setTest(true);
            DriverStationSim.notifyNewData();
            feeder.periodic();
            assertEquals(0, io.wallRPM);
            DriverStationSim.setTest(false);
            DriverStationSim.notifyNewData();
            feeder.periodic();
            assertEquals(FeederConstants.WALL_RPM, io.wallRPM);
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            feeder.periodic();
            assertEquals(0, io.wallRPM);
        } finally {
            CommandScheduler.getInstance().unregisterSubsystem(feeder);
            DriverStationSim.resetData();
            DriverStationSim.notifyNewData();
        }
    }
}
