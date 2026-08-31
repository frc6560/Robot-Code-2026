package frc.robot.intake;

/** State-feedback controller for the rack extension and intake roller. */
public final class IntakeStateSpaceController {
  public enum Mode { RETRACTED, EXTENDED, EXTENDED_SPINNING, OUTTAKE, OSCILLATING }

  public static final double MIN_EXTENSION_METERS = 0.0;
  public static final double MAX_EXTENSION_METERS = 0.50;
  public static final double OSCILLATION_AMPLITUDE_METERS = 0.05;
  public static final double OSCILLATION_CENTER_METERS = 0.45;
  public static final double HALF_SPEED_VOLTAGE_LIMIT = 6.0;
  public static final double HALF_SPEED_ROLLER_RPS = (2500.0 / 60.0) * 0.5;
  public static final double OSCILLATION_FREQUENCY_HZ = 0.625;

  private Mode mode = Mode.RETRACTED;
  private double oscillationTimeSeconds = 0.0;

  public void setMode(Mode mode) {
    if (mode == null) throw new IllegalArgumentException("mode cannot be null");
    this.mode = mode;
    oscillationTimeSeconds = 0.0;
  }

  public Mode getMode() { return mode; }

  public Output calculate(double extensionMeters, double extensionVelocityMetersPerSecond,
                          double rollerVelocityRps, double dtSeconds) {
    double xRef;
    double vRef;
    double rollerRef;
    switch (mode) {
      case RETRACTED -> { xRef = MIN_EXTENSION_METERS; vRef = 0.0; rollerRef = 0.0; }
      case EXTENDED -> { xRef = MAX_EXTENSION_METERS; vRef = 0.0; rollerRef = 0.0; }
      case EXTENDED_SPINNING -> {
        xRef = MAX_EXTENSION_METERS; vRef = 0.0; rollerRef = HALF_SPEED_ROLLER_RPS;
      }
      case OUTTAKE -> {
        xRef = MAX_EXTENSION_METERS; vRef = 0.0; rollerRef = -HALF_SPEED_ROLLER_RPS;
      }
      case OSCILLATING -> {
        double phase = 2.0 * Math.PI * OSCILLATION_FREQUENCY_HZ * oscillationTimeSeconds;
        xRef = OSCILLATION_CENTER_METERS + OSCILLATION_AMPLITUDE_METERS * Math.sin(phase);
        vRef = 2.0 * Math.PI * OSCILLATION_FREQUENCY_HZ
            * OSCILLATION_AMPLITUDE_METERS * Math.cos(phase);
        rollerRef = 0.0;
      }
      default -> throw new IllegalStateException("Unhandled intake mode");
    }

    xRef = clamp(xRef, MIN_EXTENSION_METERS, MAX_EXTENSION_METERS);
    double deploymentVolts = clamp(
        18.0 * (xRef - extensionMeters) + 2.0 * (vRef - extensionVelocityMetersPerSecond),
        -HALF_SPEED_VOLTAGE_LIMIT, HALF_SPEED_VOLTAGE_LIMIT);
    if (extensionMeters <= MIN_EXTENSION_METERS && deploymentVolts < 0.0) deploymentVolts = 0.0;
    if (extensionMeters >= MAX_EXTENSION_METERS && deploymentVolts > 0.0) deploymentVolts = 0.0;

    double rollerVolts = clamp(0.75 * (rollerRef - rollerVelocityRps),
        -HALF_SPEED_VOLTAGE_LIMIT, HALF_SPEED_VOLTAGE_LIMIT);
    oscillationTimeSeconds += Math.max(0.0, dtSeconds);
    return new Output(xRef, vRef, rollerRef, deploymentVolts, rollerVolts, mode);
  }

  private static double clamp(double value, double minimum, double maximum) {
    return Math.min(maximum, Math.max(minimum, value));
  }

  public record Output(double referenceExtensionMeters, double referenceVelocityMetersPerSecond,
      double referenceRollerRps, double deploymentVolts, double rollerVolts, Mode mode) {}
}
