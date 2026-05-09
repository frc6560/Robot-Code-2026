package frc.robot.diagnostics.capture;

public record ModuleSnapshot(
    int index,
    double speedRadPerSec,
    double angleDeg,
    double driveCurrentAmps,
    double driveAppliedVolts,
    double turnCurrentAmps,
    double turnAppliedVolts
) {}
