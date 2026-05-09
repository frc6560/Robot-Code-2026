package frc.robot.diagnostics.capture;

import java.util.List;

public record DriveState(
    double poseX,
    double poseY,
    double headingDeg,
    double vxMps,
    double vyMps,
    double omegaRadPerSec,
    List<ModuleSnapshot> modules
) {}
