package frc.robot.diagnostics;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public interface SimScenario {
    String getName();

    Command configure(RobotContainer container);

    double getDurationSeconds();

    default boolean isAutonomous() {
        return true;
    }
}
