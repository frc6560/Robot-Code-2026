package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class RedHubSimpleAutoTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void reset() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test
    void startsFromTheRedPoseAndLeavesMotorsSafeWhenCancelled() throws Exception {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();

        RobotContainer robot = new RobotContainer();
        Shooter shooter = getField(robot, "shooter", Shooter.class);
        Indexer indexer = getField(robot, "indexer", Indexer.class);
        Command auto = RedHubSimpleAuto.create(robot.getDrivebase(), shooter, indexer);
        CommandScheduler.getInstance().schedule(auto);
        CommandScheduler.getInstance().run();

        assertEquals(12.8781423569, robot.getDrivebase().getPose().getX(), 0.01);
        assertEquals(7.3673672676, robot.getDrivebase().getPose().getY(), 0.01);
        assertEquals(0.0, robot.getDrivebase().getPose().getRotation().getDegrees(), 1.0);
        assertEquals(0.0, shooter.getGoalRPM(), 1.0);
        assertEquals(0.0, indexer.getTunnelTargetRPM(), 1.0);

        CommandScheduler.getInstance().cancel(auto);
        assertEquals(0.0, shooter.getGoalRPM(), 1.0);
        assertEquals(0.0, indexer.getTunnelTargetRPM(), 1.0);
    }

    private static <T> T getField(RobotContainer robot, String name, Class<T> type)
            throws ReflectiveOperationException {
        Field field = RobotContainer.class.getDeclaredField(name);
        field.setAccessible(true);
        return type.cast(field.get(robot));
    }
}
