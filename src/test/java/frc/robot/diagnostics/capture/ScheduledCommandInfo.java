package frc.robot.diagnostics.capture;

import java.util.Set;

public record ScheduledCommandInfo(
    String name,
    String type,
    Set<String> requirements
) {}
