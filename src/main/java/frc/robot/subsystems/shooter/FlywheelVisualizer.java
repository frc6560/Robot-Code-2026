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
 * runs a scripted match sequence — idle, climb to a shot speed, fire a four-ball
 * volley, recover, repeat until the hopper is empty, then move to a new
 * distance — and publishes both the wheel and the numbers behind it.
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
  private static final double IDLE_RPM = 3332.0;
  private static final double SETTLE_BAND_RPM = 25.0;
  private static final int BALLS_PER_VOLLEY = 4;
  private static final int HOPPER_BALLS = 60;

  /** Distance, band centre and band half-width for each scripted shot. */
  private static final double[][] SHOTS = {
    // {distance m, command RPM, band low RPM, band high RPM}
    {6.05, 4018.0, 3848.0, 4188.0},
    {3.29, 2917.0, 2743.0, 3091.0},
    {1.63, 2301.0, 2183.0, 2418.0},
  };

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
  private static final double SECONDS_BETWEEN_VOLLEYS = 0.35;
  private static final double SECONDS_AT_IDLE_BEFORE_START = 1.0;
  /** Pause on an empty hopper before reloading, so the run repeats for watching. */
  private static final double SECONDS_BEFORE_RELOAD = 3.0;

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
  private double elapsed = 0.0;

  private int shotIndex = 0;
  private int ballsRemaining = HOPPER_BALLS;
  private double volleyCooldown = 0.0;
  private double lastDroopRpm = 0.0;
  private double flashTimer = 0.0;
  private double emptyTimer = 0.0;
  private int volleysFired = 0;

  private enum Phase {
    IDLING,
    SPINNING_UP,
    FIRING,
    EMPTY
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
    elapsed += dtSeconds;
    double commandRpm = advanceSequence(dtSeconds);

    integrate(dtSeconds, commandRpm);

    angleRad = (angleRad + velocityRadPerSec * dtSeconds * DISPLAY_SPIN_SCALE) % (2.0 * Math.PI);
    double rpm = velocityRadPerSec * 30.0 / Math.PI;

    publish(rpm, commandRpm);
  }

  /**
   * Runs the scripted hopper: hold idle, climb to the shot, fire a volley every
   * time the wheel settles back inside its band, then move to the next distance.
   *
   * @return the speed currently being commanded, in flywheel RPM
   */
  private double advanceSequence(double dtSeconds) {
    if (phase == Phase.EMPTY) {
      // Reload and start over, so the sequence can be watched on a loop.
      emptyTimer += dtSeconds;
      if (emptyTimer >= SECONDS_BEFORE_RELOAD) {
        ballsRemaining = HOPPER_BALLS;
        shotIndex = 0;
        emptyTimer = 0.0;
        phase = Phase.IDLING;
      }
      return IDLE_RPM;
    }
    if (elapsed < SECONDS_AT_IDLE_BEFORE_START) {
      phase = Phase.IDLING;
      return IDLE_RPM;
    }

    double commandRpm = SHOTS[shotIndex][1];
    double rpm = velocityRadPerSec * 30.0 / Math.PI;
    volleyCooldown = Math.max(0.0, volleyCooldown - dtSeconds);
    flashTimer = Math.max(0.0, flashTimer - dtSeconds);

    boolean atSpeed = rpm >= commandRpm - SETTLE_BAND_RPM;
    if (atSpeed && volleyCooldown <= 0.0 && ballsRemaining > 0) {
      fireVolley(commandRpm);
      volleysFired++;
      phase = Phase.FIRING;
      volleyCooldown = SECONDS_BETWEEN_VOLLEYS;
      // A third of the hopper per distance, then move on.
      if (ballsRemaining % (HOPPER_BALLS / SHOTS.length) == 0 && ballsRemaining > 0) {
        shotIndex = Math.min(SHOTS.length - 1, shotIndex + 1);
      }
    } else if (!atSpeed) {
      phase = Phase.SPINNING_UP;
    }
    if (ballsRemaining <= 0) {
      phase = Phase.EMPTY;
    }
    return commandRpm;
  }

  /**
   * Removes a volley's worth of energy in one step.
   *
   * <p>Each ball leaves with translation and backspin, and the wheel gives up
   * more than the balls receive because contact slips. Subtracting energy rather
   * than speed is what makes a fast wheel droop less than a slow one.
   */
  private void fireVolley(double commandRpm) {
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
    double bandLow = SHOTS[shotIndex][2];
    double bandHigh = SHOTS[shotIndex][3];
    boolean inBand = rpm >= bandLow && rpm <= bandHigh;

    // Blue while it is where it should be, amber the instant a volley pulls it
    // out of band, grey while idling between shots.
    Color8Bit color;
    if (flashTimer > 0.0 || !inBand) {
      boolean resting = phase == Phase.IDLING || phase == Phase.EMPTY;
      color = resting ? new Color8Bit(120, 130, 140) : new Color8Bit(217, 119, 6);
    } else {
      color = new Color8Bit(17, 121, 238);
    }

    double displayDegrees = Math.toDegrees(angleRad);
    for (int i = 0; i < SPOKE_COUNT; i++) {
      spokes[i].setAngle(displayDegrees + i * (360.0 / SPOKE_COUNT));
      spokes[i].setColor(color);
    }

    double sweep = 270.0 / SHOTS[0][1];
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
    Logger.recordOutput("Flywheel/ShotDistanceMeters", SHOTS[shotIndex][0]);
    Logger.recordOutput("Flywheel/Phase", phase.toString());
    Logger.recordOutput("Flywheel/InertiaKgM2", inertia);
    Logger.recordOutput("Flywheel/VolleysFired", volleysFired);
  }
}
