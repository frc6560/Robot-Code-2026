package frc.robot.diagnostics.scenarios;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.diagnostics.SimScenario;

public class TimedRunScenario implements SimScenario {
    private final String name;
    private final double duration;
    private final boolean autonomous;

    public TimedRunScenario(String name, double duration, boolean autonomous) {
        this.name = name;
        this.duration = duration;
        this.autonomous = autonomous;
    }

    @Override
    public String getName() { return name; }

    @Override
    public double getDurationSeconds() { return duration; }

    @Override
    public boolean isAutonomous() { return autonomous; }

    @Override
    public Command configure(RobotContainer container) {
        return null;
    }
}
