package frc.robot.commands.scoring;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.utility.Shooter.ShotCalculator;

class ShotCommandTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void resetDriverStation() {
        DriverStationSim.resetData();
    }

    @Test
    void missingAllianceImmediatelyClosesTheFeederGate() {
        Feeder feeder = new Feeder(new FeederIO() {});
        Hood hood = new Hood(new HoodIO() {});
        Shooter shooter = new Shooter(new ShooterIO() {});
        Turret turret = new Turret(new TurretIO() {});
        ShotCalculator calculator = new ShotCalculator();
        LED led = new LED(new LEDIO() {}, hood, shooter, turret, calculator);
        ShotCommand command = new ShotCommand(
            feeder,
            turret,
            hood,
            shooter,
            calculator,
            Pose2d::new,
            led
        );

        feeder.setShooting(true);
        command.execute();

        assertFalse(feeder.isShooting());
    }
}
