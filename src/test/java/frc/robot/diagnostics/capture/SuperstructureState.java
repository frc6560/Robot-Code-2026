package frc.robot.diagnostics.capture;

public record SuperstructureState(
    double hoodAngleDeg,
    double hoodTargetDeg,
    boolean hoodAtTarget,
    double shooterRPM,
    double shooterTargetRPM,
    boolean shooterAtTarget,
    double turretAngleDeg,
    double turretTargetDeg,
    boolean turretAtTarget,
    String feederState,
    boolean feederIntaking,
    boolean feederShooting,
    boolean intakeActive
) {}
