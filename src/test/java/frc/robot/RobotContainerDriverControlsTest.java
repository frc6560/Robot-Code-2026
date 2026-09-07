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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
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
        DriverStationSim.setJoystickButtonCount(0, 10);
        DriverStationSim.setJoystickIsXbox(0, true);
        DriverStationSim.setJoystickAxis(0, XboxController.Axis.kRightTrigger.value, 0.0);
        DriverStationSim.notifyNewData();

        RobotContainer container = new RobotContainer();
        // The default field origin is just outside the calibrated HUB range. An ungated driver
        // shot must use the nearest setpoint instead of retracting the hood and idling the wheel.
        container.getDrivebase().resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d()));
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.run();

        Shooter shooter = getField(container, "shooter", Shooter.class);
        Hood hood = getField(container, "hood", Hood.class);
        double initialHoodAngle = hood.getHoodAngle();

        DriverStationSim.setJoystickAxis(0, XboxController.Axis.kRightTrigger.value, 1.0);
        DriverStationSim.notifyNewData();
        for (int i = 0; i < 50; i++) {
            scheduler.run();
        }

        Feeder feeder = getField(container, "feeder", Feeder.class);
        assertNotNull(feeder);
        assertTrue(feeder.isShooting());
        assertTrue(shooter.getGoalRPM() > Constants.ShooterConstants.FLYWHEEL_IDLE_RPM);
        assertTrue(shooter.getCurrentRPM() > 0.0);
        assertTrue(hood.getTargetAngle() > Constants.HoodConstants.HOOD_MIN_ANGLE);
        assertTrue(
            hood.getHoodAngle() > initialHoodAngle,
            "hood did not move: initial=" + initialHoodAngle
                + ", current=" + hood.getHoodAngle()
                + ", target=" + hood.getTargetAngle());

        DriverStationSim.setJoystickAxis(0, XboxController.Axis.kRightTrigger.value, 0.0);
        DriverStationSim.notifyNewData();
        scheduler.run();

        assertFalse(feeder.isShooting());
    }

    private static <T> T getField(RobotContainer container, String name, Class<T> type)
            throws ReflectiveOperationException {
        Field field = RobotContainer.class.getDeclaredField(name);
        field.setAccessible(true);
        return type.cast(field.get(container));
    }
}
