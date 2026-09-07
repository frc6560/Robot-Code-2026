package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.feeder.Feeder;

class RobotContainerDriverControlsTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void resetState() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test
    void driverRightTriggerFeedsImmediatelyAndReleaseStopsIt() throws Exception {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setJoystickAxisCount(0, 6);
        DriverStationSim.setJoystickIsXbox(0, true);
        DriverStationSim.setJoystickAxis(0, XboxController.Axis.kRightTrigger.value, 0.0);
        DriverStationSim.notifyNewData();

        RobotContainer container = new RobotContainer();
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.run();

        DriverStationSim.setJoystickAxis(0, XboxController.Axis.kRightTrigger.value, 1.0);
        DriverStationSim.notifyNewData();
        scheduler.run();

        Feeder feeder = getFeeder(container);
        assertNotNull(feeder);
        assertTrue(feeder.isShooting());

        DriverStationSim.setJoystickAxis(0, XboxController.Axis.kRightTrigger.value, 0.0);
        DriverStationSim.notifyNewData();
        scheduler.run();

        assertFalse(feeder.isShooting());
    }

    private static Feeder getFeeder(RobotContainer container) throws ReflectiveOperationException {
        Field field = RobotContainer.class.getDeclaredField("feeder");
        field.setAccessible(true);
        return (Feeder) field.get(container);
    }
}
