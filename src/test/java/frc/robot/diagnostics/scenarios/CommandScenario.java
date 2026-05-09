package frc.robot.diagnostics.scenarios;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.diagnostics.SimScenario;

import java.util.function.Function;

public class CommandScenario implements SimScenario {
    private final String name;
    private final double duration;
    private final Function<RobotContainer, Command> commandFactory;

    public CommandScenario(String name, double duration, Function<RobotContainer, Command> commandFactory) {
        this.name = name;
        this.duration = duration;
        this.commandFactory = commandFactory;
    }

    @Override
    public String getName() { return name; }

    @Override
    public double getDurationSeconds() { return duration; }

    @Override
    public Command configure(RobotContainer container) {
        return commandFactory.apply(container);
    }
}
