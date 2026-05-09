package frc.robot.diagnostics.capture;

public record CommandEvent(
    String name,
    String eventType,
    double timestamp
) {}
