package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Drives a spinning-wheel Mechanism2d from a physical model of the tube flywheel,
 * so spin-up, firing droop and recovery can be watched in AdvantageScope.
 *
 * <p>This is a self-contained simulation, not a view onto the real shooter. It
 * runs a repeating demonstration — idle, climb through five target RPMs, fire a
 * four-ball volley at each target, recover from the RPM drop, then return to
 * idle — and publishes both the spinning wheel and the numbers behind it.
 *
 * <p>The model mirrors {@code tools/shot-calibrator/shotlab/flywheel.py}: a
 * stator-current-limited torque plateau below the knee, the voltage-limited
 * motor curve above it, and battery sag folded in as extra series resistance.
 * Firing removes energy in one step rather than over the few milliseconds of
 * ball contact, which is the same conservative simplification the sizing tool
 * makes.
 */
public class FlywheelVisualizer {

  // --- Mechanism as tuned by the wall-thickness optimiser -------------------
  private static final double WHEEL_DIAMETER_M = 4.0 * 0.0254;
  private static final double TUBE_LENGTH_M = 26.0 * 0.0254;
  private static final double WALL_THICKNESS_M = 0.100 * 0.0254;
  private static final double STEEL_DENSITY = 7850.0;

  private static final int MOTOR_COUNT = 2;
  /** Motor turns per flywheel turn: a 15 T pinion driving an 18 T gear. */
  private static final double GEAR_RATIO = 18.0 / 15.0;
  private static final double GEAR_EFFICIENCY = 0.97;
  private static final double STATOR_CURRENT_LIMIT = 40.0;

  private static final double OPEN_CIRCUIT_VOLTS = 12.0;
  private static final double BATTERY_RESISTANCE = 0.015;
  private static final double CHANNEL_RESISTANCE = 0.010;
  private static final double DRAG_TORQUE_AT_FREE_SPEED = 0.05;

  // --- Shot schedule, from the trajectory model ----------------------------
  private static final double IDLE_RPM = 800.0;
  private static final double SETTLE_BAND_RPM = 25.0;
  private static final int BALLS_PER_VOLLEY = 4;

  /** Five increasing distance/RPM targets spanning the calculated shot range. */
  private static final double[][] SETPOINTS = {
    // {distance m, target RPM}
    {1.63, 2301.0},
    {2.74, 2712.0},
    {3.84, 3156.0},
    {4.95, 3587.0},
    {6.05, 4018.0},
  };
  private static final double MAX_TARGET_RPM = SETPOINTS[SETPOINTS.length - 1][1];
  private static final int DEMONSTRATION_BALLS = SETPOINTS.length * BALLS_PER_VOLLEY;

  private static final double BALL_MASS_KG = 0.215;
  private static final double BALL_INERTIA_FACTOR = 2.0 / 3.0;
  private static final double EXIT_SPEED_RATIO = 0.50;
  private static final double TRANSFER_EFFICIENCY = 0.60;

  /**
   * Rotation shown per real rotation. At 4000 RPM the wheel turns more than one
   * full revolution per 50 Hz frame, so drawing true angle would alias into a
   * stationary or backwards wheel. Scaling keeps the direction and the relative
   * speed readable; the logged RPM stays exact.
   */
  private static final double DISPLAY_SPIN_SCALE = 0.06;

  private static final int SPOKE_COUNT = 6;
  private static final double SECONDS_AT_IDLE_BEFORE_START = 1.5;
  private static final double SECONDS_SETTLED_BEFORE_SHOT = 0.30;
  private static final double SECONDS_SETTLED_AFTER_RECOVERY = 0.45;
  private static final double SECONDS_TO_HOLD_TARGET = 0.65;

  /**
   * Proportional gain of the speed controller, in stator amps per rad/s of error.
   * Sized to saturate the current limit beyond roughly 50 RPM of error, so the
   * wheel uses everything it has while climbing and settles to a small holding
   * current once it arrives. Without a controller the model would command full
   * current even at setpoint and sit there chattering between drive and brake.
   */
  private static final double VELOCITY_KP_AMPS_PER_RAD_PER_SEC = 8.0;

  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(2.0, 2.0);
  private final LoggedMechanismLigament2d[] spokes = new LoggedMechanismLigament2d[SPOKE_COUNT];
  private final LoggedMechanismLigament2d tachNeedle;
  private final LoggedMechanismLigament2d targetNeedle;

  private final double inertia;
  private final double dragCoefficient;

  private double velocityRadPerSec = 0.0;
  private double angleRad = 0.0;
  private int setpointIndex = 0;
  private int ballsRemaining = DEMONSTRATION_BALLS;
  private double phaseElapsedSeconds = 0.0;
  private double settledElapsedSeconds = 0.0;
  private double lastDroopRpm = 0.0;
  private double flashTimer = 0.0;
  private int volleysFired = 0;

  private enum Phase {
    IDLING,
    SPINNING_UP,
    FIRING,
    RECOVERING,
    HOLDING,
    RETURNING_TO_IDLE
  }

  private Phase phase = Phase.IDLING;

  public FlywheelVisualizer() {
    double outerRadius = WHEEL_DIAMETER_M / 2.0;
    double innerRadius = Math.max(0.0, outerRadius - WALL_THICKNESS_M);
    double mass =
        STEEL_DENSITY
            * Math.PI
            * (outerRadius * outerRadius - innerRadius * innerRadius)
            * TUBE_LENGTH_M;
    inertia = 0.5 * mass * (outerRadius * outerRadius + innerRadius * innerRadius);

    double freeSpeedRadPerSec = DCMotor.getKrakenX60(1).freeSpeedRadPerSec / GEAR_RATIO;
    dragCoefficient = DRAG_TORQUE_AT_FREE_SPEED / freeSpeedRadPerSec;

    LoggedMechanismRoot2d hub = mechanism.getRoot("hub", 0.65, 1.0);
    for (int i = 0; i < SPOKE_COUNT; i++) {
      spokes[i] =
          hub.append(
              new LoggedMechanismLigament2d(
                  "spoke" + i, 0.45, i * (360.0 / SPOKE_COUNT), 7.0, new Color8Bit(17, 121, 238)));
    }

    // A gauge beside the wheel: the needle sweeps with speed, and a second
    // marker sits at the commanded speed so droop is visible as a gap.
    LoggedMechanismRoot2d gauge = mechanism.getRoot("gauge", 1.55, 0.75);
    targetNeedle =
        gauge.append(
            new LoggedMechanismLigament2d("target", 0.40, 0.0, 4.0, new Color8Bit(120, 130, 140)));
    tachNeedle =
        gauge.append(new LoggedMechanismLigament2d("tach", 0.36, 0.0, 8.0, new Color8Bit(0, 186, 255)));

    velocityRadPerSec = IDLE_RPM * Math.PI / 30.0;
  }

  /** Step the model and republish. Call from {@code simulationPeriodic()}. */
  public void update(double dtSeconds) {
    double commandRpm = advanceSequence(dtSeconds);

    integrate(dtSeconds, commandRpm);

    angleRad = (angleRad + velocityRadPerSec * dtSeconds * DISPLAY_SPIN_SCALE) % (2.0 * Math.PI);
    double rpm = velocityRadPerSec * 30.0 / Math.PI;

    publish(rpm, commandRpm);
  }

  /**
   * Runs the five-point demonstration. Each target jump is held until the wheel
   * settles, then one volley removes energy, making the RPM drop visible before
   * the controller recovers. After the fifth target the wheel returns to idle.
   *
   * @return the speed currently being commanded, in flywheel RPM
   */
  private double advanceSequence(double dtSeconds) {
    phaseElapsedSeconds += dtSeconds;
    flashTimer = Math.max(0.0, flashTimer - dtSeconds);
    double rpm = velocityRadPerSec * 30.0 / Math.PI;

    if (phase == Phase.IDLING) {
      if (phaseElapsedSeconds >= SECONDS_AT_IDLE_BEFORE_START) {
        transitionTo(Phase.SPINNING_UP);
      }
      return IDLE_RPM;
    }

    if (phase == Phase.RETURNING_TO_IDLE) {
      if (Math.abs(rpm - IDLE_RPM) <= SETTLE_BAND_RPM) {
        setpointIndex = 0;
        ballsRemaining = DEMONSTRATION_BALLS;
        transitionTo(Phase.IDLING);
      }
      return IDLE_RPM;
    }

    double commandRpm = SETPOINTS[setpointIndex][1];
    boolean atSpeed = Math.abs(rpm - commandRpm) <= SETTLE_BAND_RPM;
    settledElapsedSeconds = atSpeed ? settledElapsedSeconds + dtSeconds : 0.0;

    switch (phase) {
      case SPINNING_UP -> {
        if (settledElapsedSeconds >= SECONDS_SETTLED_BEFORE_SHOT) {
          fireVolley();
          volleysFired++;
          transitionTo(Phase.FIRING);
        }
      }
      case FIRING -> transitionTo(Phase.RECOVERING);
      case RECOVERING -> {
        if (settledElapsedSeconds >= SECONDS_SETTLED_AFTER_RECOVERY) {
          transitionTo(Phase.HOLDING);
        }
      }
      case HOLDING -> {
        if (phaseElapsedSeconds >= SECONDS_TO_HOLD_TARGET) {
          if (setpointIndex < SETPOINTS.length - 1) {
            setpointIndex++;
            transitionTo(Phase.SPINNING_UP);
          } else {
            transitionTo(Phase.RETURNING_TO_IDLE);
          }
        }
      }
      default -> {
        // IDLING and RETURNING_TO_IDLE are handled above.
      }
    }
    return commandRpm;
  }

  private void transitionTo(Phase nextPhase) {
    phase = nextPhase;
    phaseElapsedSeconds = 0.0;
    settledElapsedSeconds = 0.0;
  }

  /**
   * Removes a volley's worth of energy in one step.
   *
   * <p>Each ball leaves with translation and backspin, and the wheel gives up
   * more than the balls receive because contact slips. Subtracting energy rather
   * than speed is what makes a fast wheel droop less than a slow one.
   */
  private void fireVolley() {
    double before = velocityRadPerSec;
    double surfaceSpeed = velocityRadPerSec * (WHEEL_DIAMETER_M / 2.0);
    double exitSpeed = EXIT_SPEED_RATIO * surfaceSpeed;
    double perBall = 0.5 * BALL_MASS_KG * exitSpeed * exitSpeed * (1.0 + BALL_INERTIA_FACTOR);
    double removed = BALLS_PER_VOLLEY * perBall / TRANSFER_EFFICIENCY;

    double stored = 0.5 * inertia * before * before;
    double remaining = Math.max(0.0, stored - removed);
    velocityRadPerSec = Math.sqrt(2.0 * remaining / inertia);

    ballsRemaining -= BALLS_PER_VOLLEY;
    lastDroopRpm = (before - velocityRadPerSec) * 30.0 / Math.PI;
    flashTimer = 0.15;
  }

  /** One step of the torque model, matching the Python sizing tool. */
  private void integrate(double dtSeconds, double commandRpm) {
    DCMotor motor = DCMotor.getKrakenX60(1);
    double effectiveResistance =
        motor.rOhms + CHANNEL_RESISTANCE + MOTOR_COUNT * BATTERY_RESISTANCE;

    double motorSpeed = velocityRadPerSec * GEAR_RATIO;
    double backEmf = motorSpeed / motor.KvRadPerSecPerVolt;
    double targetRadPerSec = commandRpm * Math.PI / 30.0;

    // Proportional speed control, clamped by both the stator limit and what the
    // motor curve can actually deliver at this speed. Saturates while climbing,
    // then trims back to a small holding current at setpoint.
    double error = targetRadPerSec - velocityRadPerSec;
    double available = Math.max(0.0, (OPEN_CIRCUIT_VOLTS - backEmf) / effectiveResistance);
    double ceiling = Math.min(STATOR_CURRENT_LIMIT, available);
    double stator =
        Math.max(-STATOR_CURRENT_LIMIT, Math.min(ceiling, VELOCITY_KP_AMPS_PER_RAD_PER_SEC * error));

    double gross = MOTOR_COUNT * motor.KtNMPerAmp * stator * GEAR_RATIO * GEAR_EFFICIENCY;
    double friction =
        MOTOR_COUNT
            * motor.KtNMPerAmp
            * DCMotor.getKrakenX60(1).freeCurrentAmps
            * GEAR_RATIO
            * GEAR_EFFICIENCY;
    double net = gross - Math.signum(velocityRadPerSec) * friction - dragCoefficient * velocityRadPerSec;

    velocityRadPerSec = Math.max(0.0, velocityRadPerSec + net / inertia * dtSeconds);

    // Report the sag the rest of the robot would feel while this is happening.
    double applied = backEmf + Math.abs(stator) * motor.rOhms;
    double duty = OPEN_CIRCUIT_VOLTS > 0.0 ? Math.min(1.0, applied / OPEN_CIRCUIT_VOLTS) : 0.0;
    double supply = MOTOR_COUNT * Math.abs(stator) * duty;
    RoboRioSim.setVInVoltage(OPEN_CIRCUIT_VOLTS - supply * BATTERY_RESISTANCE);

    Logger.recordOutput("Flywheel/StatorCurrentAmps", Math.abs(stator));
    Logger.recordOutput("Flywheel/SupplyCurrentAmps", supply);
    Logger.recordOutput("Flywheel/BusVoltage", OPEN_CIRCUIT_VOLTS - supply * BATTERY_RESISTANCE);
  }

  private void publish(double rpm, double commandRpm) {
    double bandLow = commandRpm - SETTLE_BAND_RPM;
    double bandHigh = commandRpm + SETTLE_BAND_RPM;
    boolean inBand = rpm >= bandLow && rpm <= bandHigh;

    // Blue while it is where it should be, amber the instant a volley pulls it
    // out of band, grey while idling or returning to idle.
    Color8Bit color;
    if (flashTimer > 0.0 || !inBand) {
      boolean resting = phase == Phase.IDLING || phase == Phase.RETURNING_TO_IDLE;
      color = resting ? new Color8Bit(120, 130, 140) : new Color8Bit(217, 119, 6);
    } else {
      color = new Color8Bit(17, 121, 238);
    }

    double displayDegrees = Math.toDegrees(angleRad);
    for (int i = 0; i < SPOKE_COUNT; i++) {
      spokes[i].setAngle(displayDegrees + i * (360.0 / SPOKE_COUNT));
      spokes[i].setColor(color);
    }

    double sweep = 270.0 / MAX_TARGET_RPM;
    tachNeedle.setAngle(rpm * sweep);
    tachNeedle.setColor(color);
    targetNeedle.setAngle(commandRpm * sweep);

    Logger.recordOutput("Flywheel/Mechanism", mechanism);
    Logger.recordOutput("Flywheel/RPM", rpm);
    Logger.recordOutput("Flywheel/CommandRPM", commandRpm);
    Logger.recordOutput("Flywheel/BandLowRPM", bandLow);
    Logger.recordOutput("Flywheel/BandHighRPM", bandHigh);
    Logger.recordOutput("Flywheel/InBand", inBand);
    Logger.recordOutput("Flywheel/LastDroopRPM", lastDroopRpm);
    Logger.recordOutput("Flywheel/BallsRemaining", ballsRemaining);
    Logger.recordOutput("Flywheel/TargetIndex", setpointIndex + 1);
    Logger.recordOutput("Flywheel/TargetCount", SETPOINTS.length);
    Logger.recordOutput("Flywheel/ShotDistanceMeters", SETPOINTS[setpointIndex][0]);
    Logger.recordOutput("Flywheel/Phase", phase.toString());
    Logger.recordOutput("Flywheel/InertiaKgM2", inertia);
    Logger.recordOutput("Flywheel/VolleysFired", volleysFired);
  }

  int getCurrentTargetNumber() {
    return setpointIndex + 1;
  }

  int getTargetCount() {
    return SETPOINTS.length;
  }

  int getVolleysFired() {
    return volleysFired;
  }

  double getLastDroopRpm() {
    return lastDroopRpm;
  }

  boolean isReturningToIdle() {
    return phase == Phase.RETURNING_TO_IDLE;
  }
}
