package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOOutputMode;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOOutputs;

/**
 * Flywheel control modelled on 6328's 2026 robot. The goal is never sent to the Talon directly:
 * a rate limiter walks a setpoint toward it no faster than the power budget can accelerate the
 * wheel, and the Talon tracks that setpoint with a Java-side kS/kV/kA feedforward plus its own kP.
 */
public class Shooter extends SubsystemBase {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final ShooterIOOutputs outputs = new ShooterIOOutputs();

    private final Debouncer leaderConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer followerConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
    private final Alert leaderDisconnected = new Alert("Flywheel leader motor disconnected!", AlertType.kError);
    private final Alert followerDisconnected = new Alert("Flywheel follower motor disconnected!", AlertType.kError);

    // Requested behavior, set by commands.
    private double goalRadsPerSec = 0.0;
    private boolean bangBang = false;
    private boolean idle = false;
    private double characterizationVolts = Double.NaN;
    private boolean pitCoastMode = false;

    // Rate limiter state.
    private double setpointRadsPerSec = 0.0;
    private double filteredAccel = 0.0;
    private boolean nonZeroAccel = false;
    private boolean atGoal = false;

    private boolean withinTolerancekS = false;
    private boolean withinTolerancekV = false;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    /**
     * Track the given flywheel speed with the Talon velocity loop.
     * @param rpm Desired flywheel RPM
     */
    public void setGoal(double rpm) {
        setGoal(rpm, false);
    }

    /**
     * @param rpm Desired flywheel RPM
     * @param bangBang Open-loop feedforward, multiplied by {@link ShooterConstants#BANGBANG_CONSTANT}
     *     while below the setpoint. Recovers faster between long shots at the cost of some ripple.
     */
    public void setGoal(double rpm, boolean bangBang) {
        goalRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rpm);
        this.bangBang = bangBang;
        idle = false;
    }

    /** Hold idle speed open-loop on feedforward alone. */
    public void setIdle() {
        goalRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(ShooterConstants.FLYWHEEL_IDLE_RPM);
        bangBang = false;
        idle = true;
    }

    public void stop() {
        goalRadsPerSec = 0.0;
        bangBang = false;
        idle = false;
    }

    /** Zero output so the wheel can be turned by hand. Flywheels already use coast as their neutral mode. */
    public void setPitCoastMode(boolean enabled) {
        pitCoastMode = enabled;
        stop();
    }

    /** Whether the distance to the target calls for bang-bang control on a hub shot. */
    public static boolean useBangBang(double distanceMeters) {
        return distanceMeters > ShooterConstants.BANGBANG_MIN_DISTANCE_METERS;
    }

    public double getGoalRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(goalRadsPerSec);
    }

    public double getSetpointRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(setpointRadsPerSec);
    }

    public double getCurrentRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadsPerSec);
    }

    /**
     * True once the rate-limited setpoint has reached a non-idle goal. This deliberately ignores the
     * encoder, so a ball strike mid-volley doesn't drop it; gate the actual release on measured speed
     * (e.g. {@code ShotCalculator.measuredControlsScore}) or {@link #withinTolerance}.
     */
    public boolean atTarget() {
        return atGoal && !idle && getGoalRPM() >= 60.0;
    }

    /** Measured speed is within the given band of the goal. */
    public boolean withinTolerance(double toleranceRPM) {
        return Math.abs(getCurrentRPM() - getGoalRPM()) < toleranceRPM;
    }

    public boolean isWithinTolerancekS() {
        return withinTolerancekS;
    }

    public boolean isWithinTolerancekV() {
        return withinTolerancekV;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        leaderDisconnected.set(!leaderConnectedDebouncer.calculate(inputs.leaderConnected));
        followerDisconnected.set(!followerConnectedDebouncer.calculate(inputs.followerConnected));

        outputs.kP = ShooterConstants.kP;
        outputs.kD = ShooterConstants.kD;

        if (DriverStation.isDisabled() || pitCoastMode || (goalRadsPerSec <= 0.0 && Double.isNaN(characterizationVolts))) {
            stopOutputs();
        } else if (!Double.isNaN(characterizationVolts)) {
            outputs.mode = ShooterIOOutputMode.VOLTAGE;
            outputs.voltage = characterizationVolts;
            setpointRadsPerSec = inputs.velocityRadsPerSec;
            atGoal = false;
        } else {
            runVelocity(goalRadsPerSec, bangBang, idle);
        }

        io.applyOutputs(outputs);

        Logger.recordOutput("Shooter/GoalRPM", getGoalRPM());
        Logger.recordOutput("Shooter/SetpointRPM", getSetpointRPM());
        Logger.recordOutput("Shooter/CurrentRPM", getCurrentRPM());
        Logger.recordOutput("Shooter/ErrorRPM", getCurrentRPM() - getGoalRPM());
        Logger.recordOutput("Shooter/AtTarget", atTarget());
        Logger.recordOutput("Shooter/SetpointAccel", filteredAccel);
        Logger.recordOutput("Shooter/Feedforward", outputs.feedforwardVolts);
        Logger.recordOutput("Shooter/BangBang", bangBang);
        Logger.recordOutput("Shooter/Idle", idle);
        Logger.recordOutput("Shooter/Mode", outputs.mode.toString());
        Logger.recordOutput("Shooter/PitCoastMode", pitCoastMode);
    }

    private void runVelocity(double goal, boolean bangBang, boolean idle) {
        // Acceleration ceiling from the power budget. Electrical power in is I*(I*R + backEmf),
        // so a fixed watt budget is a quadratic in stator current; the bus voltage caps it again
        // once back-EMF eats most of the bus. The Talon's stator limit is the last ceiling.
        // Hub shots run at up to ~85% of free speed through the 12:18 reduction, so planning against
        // a pessimistic bus (6328 used 10 V in sim) would stall the ramp short of the goal.
        double vBus = RobotController.getBatteryVoltage();
        double supplyBudget = DriverStation.isAutonomous()
            ? ShooterConstants.FLYWHEEL_SUPPLY_BUDGET_AUTO
            : ShooterConstants.FLYWHEEL_SUPPLY_BUDGET_TELEOP;
        double powerBudget = supplyBudget * vBus * ShooterConstants.FLYWHEEL_EFFICIENCY;
        var gearbox = ShooterConstants.FLYWHEEL_GEARBOX;
        double backEmf = setpointRadsPerSec / gearbox.KvRadPerSecPerVolt;

        double maxStatorCurrent =
            (-backEmf + Math.sqrt(backEmf * backEmf + 4.0 * gearbox.rOhms * powerBudget))
                / (2.0 * gearbox.rOhms);
        double voltageLimitedCurrent = Math.max(0.0, (vBus - backEmf) / gearbox.rOhms);
        maxStatorCurrent = Math.min(maxStatorCurrent, voltageLimitedCurrent);
        maxStatorCurrent = Math.min(maxStatorCurrent,
            ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT * ShooterConstants.FLYWHEEL_MOTOR_COUNT);

        // Friction eats the torque that kS volts buys.
        double maxAccelFromCurrent =
            (gearbox.getTorque(maxStatorCurrent) - gearbox.getTorque(ShooterConstants.FF_kS / gearbox.rOhms))
                / ShooterConstants.FLYWHEEL_MOI;
        maxAccelFromCurrent = MathUtil.clamp(maxAccelFromCurrent, 0.0, ShooterConstants.FLYWHEEL_MAX_ACCELERATION);

        // Walk the setpoint toward the goal.
        double maxStep = maxAccelFromCurrent * LOOP_PERIOD_SECS;
        double error = goal - setpointRadsPerSec;
        double rawAccel;
        if (Math.abs(error) <= maxStep) {
            setpointRadsPerSec = goal;
            rawAccel = error / LOOP_PERIOD_SECS;
        } else {
            setpointRadsPerSec += Math.copySign(maxStep, error);
            rawAccel = Math.copySign(maxAccelFromCurrent, error);
        }
        // The limiter's implied acceleration is a square wave; low-pass it before it hits kA.
        if (!nonZeroAccel) {
            filteredAccel = rawAccel;
        } else {
            filteredAccel += (rawAccel - filteredAccel) * LOOP_PERIOD_SECS
                / ShooterConstants.FLYWHEEL_ACCEL_FILTER_TIME_CONSTANT;
        }
        nonZeroAccel = true;

        atGoal = Math.abs(setpointRadsPerSec - goal) <= ShooterConstants.AT_GOAL_EPSILON;

        outputs.mode = bangBang || idle ? ShooterIOOutputMode.VOLTAGE : ShooterIOOutputMode.VELOCITY;
        outputs.velocityRadsPerSec = setpointRadsPerSec;
        outputs.feedforwardVolts = Math.signum(setpointRadsPerSec) * ShooterConstants.FF_kS
            + setpointRadsPerSec * ShooterConstants.FF_kV
            + filteredAccel * ShooterConstants.FF_kA;
        outputs.voltage = outputs.feedforwardVolts;
        if (bangBang) {
            if (inputs.velocityRadsPerSec < setpointRadsPerSec) {
                outputs.voltage *= ShooterConstants.BANGBANG_CONSTANT;
            }
        } else if (!idle) {
            outputs.velocityRadsPerSec += ShooterConstants.PID_SETPOINT_OFFSET;
        }
    }

    /** Coast, and re-seed the rate limiter from the measured speed so the next spin-up starts there. */
    private void stopOutputs() {
        outputs.mode = ShooterIOOutputMode.COAST;
        outputs.velocityRadsPerSec = 0.0;
        outputs.feedforwardVolts = 0.0;
        outputs.voltage = 0.0;
        atGoal = false;
        setpointRadsPerSec = inputs.velocityRadsPerSec;
    }

    /**
     * Ramps voltage slowly and fits kS (intercept) and kV (slope) by least squares when cancelled,
     * then checks the fit against the configured feedforward.
     */
    public Command feedforwardCharacterizationCommand() {
        List<Double> velocitySamples = new ArrayList<>();
        List<Double> voltageSamples = new ArrayList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                    characterizationVolts = 0.0;
                }),
                run(() -> characterizationVolts = 0.0)
                    .withTimeout(ShooterConstants.FF_CHARACTERIZATION_START_DELAY),
                runOnce(timer::restart),
                run(() -> {
                    characterizationVolts =
                        Math.min(timer.get() * ShooterConstants.FF_CHARACTERIZATION_RAMP_RATE, 12.0);
                    velocitySamples.add(inputs.velocityRadsPerSec);
                    voltageSamples.add(characterizationVolts);
                }))
            .finallyDo(() -> {
                characterizationVolts = Double.NaN;
                stop();
                reportCharacterization(velocitySamples, voltageSamples);
            })
            .withName("Shooter FF Characterization");
    }

    private void reportCharacterization(List<Double> velocities, List<Double> voltages) {
        int n = velocities.size();
        if (n < 2) {
            System.out.println("[Shooter] FF characterization: not enough samples.");
            return;
        }
        double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0;
        for (int i = 0; i < n; i++) {
            double x = velocities.get(i);
            double y = voltages.get(i);
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }
        double denominator = n * sumX2 - sumX * sumX;
        if (denominator == 0.0) {
            System.out.println("[Shooter] FF characterization: flywheel never moved.");
            return;
        }
        double kV = (n * sumXY - sumX * sumY) / denominator;
        double kS = (sumY - kV * sumX) / n;

        withinTolerancekS = Math.abs(kS - ShooterConstants.FF_kS) <= ShooterConstants.FF_kS_TOLERANCE;
        withinTolerancekV = Math.abs(kV - ShooterConstants.FF_kV) <= ShooterConstants.FF_kV_TOLERANCE;
        Logger.recordOutput("Shooter/Characterization/kS", kS);
        Logger.recordOutput("Shooter/Characterization/kV", kV);
        Logger.recordOutput("Shooter/Characterization/WithinTolerancekS", withinTolerancekS);
        Logger.recordOutput("Shooter/Characterization/WithinTolerancekV", withinTolerancekV);
        System.out.printf(
            "[Shooter] FF characterization: kS=%.5f V (%s), kV=%.5f V/(rad/s) (%s)%n",
            kS, withinTolerancekS ? "ok" : "OUT OF TOLERANCE",
            kV, withinTolerancekV ? "ok" : "OUT OF TOLERANCE");
    }
}
