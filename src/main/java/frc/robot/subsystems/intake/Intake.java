package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeEndStopTracker.Direction;
import frc.robot.subsystems.intake.IntakeEndStopTracker.Endpoint;

/** One reciprocating intake motor. Endpoints are inferred from current, never encoder position. */
public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeEndStopTracker tracker;
    private final IntakeEndStopTracker.Settings settings;
    private final DoubleSupplier clock;
    private final BooleanSupplier outputsEnabled;

    // Retained for existing operator controls and autonomous commands.
    public enum State { RETRACTED_STOPPED, EXTENDED_STOPPED, EXTENDED_SPINNING, EXTENDED_OUTTAKING }
    private State state = State.RETRACTED_STOPPED;
    private boolean shootingOscillation;
    private boolean pitCoastMode;
    private Direction manualDirection = Direction.STOPPED;
    private Direction cycleDirection = Direction.RETRACTING;
    private boolean visitedExtended;
    private int completedCycles;
    private double commandedVolts;

    public Intake(IntakeIO io) {
        this(io, Timer::getFPGATimestamp,
            () -> IntakeConstants.ENABLED && DriverStation.isEnabled(),
            new IntakeEndStopTracker.Settings(
                IntakeConstants.EXTEND_END_STOP_CURRENT_AMPS,
                IntakeConstants.RETRACT_END_STOP_CURRENT_AMPS,
                IntakeConstants.STARTUP_IGNORE_SECONDS,
                IntakeConstants.END_STOP_CONFIRM_SECONDS,
                IntakeConstants.TRAVEL_TIMEOUT_SECONDS,
                IntakeConstants.REVERSAL_PAUSE_SECONDS));
    }

    Intake(IntakeIO io, DoubleSupplier clock, BooleanSupplier outputsEnabled,
           IntakeEndStopTracker.Settings settings) {
        this.io = io;
        this.clock = clock;
        this.outputsEnabled = outputsEnabled;
        this.settings = settings;
        tracker = new IntakeEndStopTracker(settings);
    }

    public void activate() { setExtendedSpinning(); }
    public void deactivate() { setRetracted(); }
    public void activateOuttake() {
        state = State.EXTENDED_OUTTAKING;
        manualDirection = Direction.RETRACTING;
    }

    /** Cycle between both inferred end stops while this operating state is requested. */
    public void setExtendedSpinning() {
        if (state != State.EXTENDED_SPINNING) startCycle();
        state = State.EXTENDED_SPINNING;
    }

    /** Releasing the intake button stops it at its current location. */
    public void setExtendedStopped() {
        state = State.EXTENDED_STOPPED;
        manualDirection = Direction.STOPPED;
    }

    /** Retract until rear-stop current is detected, then leave the motor stopped. */
    public void setRetracted() {
        state = State.RETRACTED_STOPPED;
        manualDirection = Direction.RETRACTING;
    }

    private void startCycle() {
        cycleDirection = tracker.endpoint() == Endpoint.RETRACTED
            ? Direction.EXTENDING : Direction.RETRACTING;
        visitedExtended = false;
    }

    public void startShootingOscillation() {
        if (!shootingOscillation) startCycle();
        shootingOscillation = true;
    }

    public void stopShootingOscillation() {
        shootingOscillation = false;
    }

    public State getState() { return state; }
    public boolean isActive() { return state == State.EXTENDED_SPINNING; }
    public boolean isShootingOscillationActive() { return shootingOscillation; }
    public String getInferredEndpoint() { return tracker.endpoint().toString(); }
    public String getFault() { return tracker.fault(); }
    public int getCompletedCycles() { return completedCycles; }

    /** An inferred endpoint, not a measured continuous position. */
    public boolean atDeployTarget() {
        Direction target = isCycling() ? cycleDirection : manualDirection;
        return target == Direction.EXTENDING && tracker.endpoint() == Endpoint.EXTENDED
            || target == Direction.RETRACTING && tracker.endpoint() == Endpoint.RETRACTED;
    }

    private boolean isCycling() {
        return shootingOscillation || state == State.EXTENDED_SPINNING;
    }

    public void setPitCoastMode(boolean enabled) {
        pitCoastMode = enabled;
        shootingOscillation = false;
        state = State.RETRACTED_STOPPED;
        manualDirection = Direction.STOPPED;
        tracker.invalidate(clock.getAsDouble());
        commandedVolts = 0;
        io.stop();
        io.setCoastMode(enabled);
    }

    /** Explicit operator reset; faults never automatically retry a stalled mechanism. */
    public void clearFault() {
        tracker.clearFault(clock.getAsDouble());
        shootingOscillation = false;
        state = State.RETRACTED_STOPPED;
        manualDirection = Direction.STOPPED;
        visitedExtended = false;
        commandedVolts = 0;
        io.stop();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        double now = clock.getAsDouble();
        boolean enabled = outputsEnabled.getAsBoolean() && !pitCoastMode;
        boolean cycling = isCycling();
        Direction requested = cycling ? cycleDirection : manualDirection;
        if (!enabled) {
            tracker.invalidate(now);
            cycleDirection = Direction.RETRACTING;
            visitedExtended = false;
            commandedVolts = 0;
            io.stop();
        } else {
            int previousStops = tracker.endStopCount();
            Direction permitted = tracker.update(
                requested, inputs.motorStatorCurrentAmps,
                inputs.motorConnected && inputs.motorCurrentValid, now);
            commandedVolts = switch (permitted) {
                case EXTENDING -> IntakeConstants.EXTEND_VOLTS;
                case RETRACTING -> -IntakeConstants.RETRACT_VOLTS;
                case STOPPED -> 0.0;
            };
            if (permitted == Direction.STOPPED) io.stop();
            else io.setMotorVoltage(commandedVolts);
            if (cycling && tracker.endStopCount() != previousStops) {
                if (tracker.endpoint() == Endpoint.EXTENDED) {
                    visitedExtended = true;
                    cycleDirection = Direction.RETRACTING;
                } else {
                    if (visitedExtended) completedCycles++;
                    visitedExtended = false;
                    cycleDirection = Direction.EXTENDING;
                }
            }
        }
        Logger.recordOutput("Intake/OutputsEnabled", enabled);
        Logger.recordOutput("Intake/DetectionConfigured", settings.configured());
        Logger.recordOutput("Intake/PitCoastMode", pitCoastMode);
        Logger.recordOutput("Intake/State", state.toString());
        Logger.recordOutput("Intake/ShootingOscillation", shootingOscillation);
        Logger.recordOutput("Intake/Cycling", cycling);
        Logger.recordOutput("Intake/RequestedDirection", requested.toString());
        Logger.recordOutput("Intake/MovingDirection", tracker.moving().toString());
        Logger.recordOutput("Intake/InferredEndpoint", tracker.endpoint().toString());
        Logger.recordOutput("Intake/EndpointKnown", tracker.endpoint() != Endpoint.UNKNOWN);
        Logger.recordOutput("Intake/EndStopCount", tracker.endStopCount());
        Logger.recordOutput("Intake/CompletedCycles", completedCycles);
        Logger.recordOutput("Intake/CommandedVolts", commandedVolts);
        Logger.recordOutput("Intake/Fault", tracker.fault());
        Logger.recordOutput("Intake/AtDeployTarget", atDeployTarget());
        Logger.recordOutput("Intake/ExtendThresholdAmps", settings.extendCurrentAmps());
        Logger.recordOutput("Intake/RetractThresholdAmps", settings.retractCurrentAmps());
    }
}
