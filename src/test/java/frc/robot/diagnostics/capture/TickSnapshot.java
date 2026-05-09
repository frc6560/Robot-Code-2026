package frc.robot.diagnostics.capture;

public record TickSnapshot(
    int tick,
    double timestampSeconds,
    DriveState drive,
    SuperstructureState superstructure,
    CommandState commands,
    TimingState timing
) {}
