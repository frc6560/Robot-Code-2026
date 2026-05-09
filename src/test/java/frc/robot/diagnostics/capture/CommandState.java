package frc.robot.diagnostics.capture;

import java.util.List;

public record CommandState(
    List<ScheduledCommandInfo> running,
    List<CommandEvent> events
) {}
