package frc.robot.subsystems.superstructure;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerRumbleSubsystem extends SubsystemBase {

    private static final double WARNING_LEAD_SECONDS = 6.0;
    private static final double RUMBLE_DURATION_SECONDS = 0.5;
    private static final double RUMBLE_STRENGTH = 1.0;

    private final GenericHID[] controllers;

    private double rumbleStopTimestamp = -1.0;
    private double lastWarnedBoundaryMatchTime = Double.NaN;

    public ControllerRumbleSubsystem(CommandXboxController... controllers) {
        this.controllers = new GenericHID[controllers.length];
        for (int i = 0; i < controllers.length; i++) {
            this.controllers[i] = controllers[i].getHID();
        }
    }

    @Override
    public void periodic() {
        if (!DriverStation.isTeleopEnabled()) {
            stopRumble();
            lastWarnedBoundaryMatchTime = Double.NaN;
            publishTelemetry(Optional.empty(), -1.0);
            return;
        }

        Optional<Alliance> allianceOpt = DriverStation.getAlliance();
        Optional<Double> boundaryOpt = allianceOpt.flatMap(alliance ->
                LEDSubsystem.getUpcomingAllianceActivationBoundary(
                        alliance,
                        DriverStation.getGameSpecificMessage(),
                        DriverStation.getMatchTime()));

        double timeToBoundary = -1.0;
        if (boundaryOpt.isPresent()) {
            double boundaryMatchTime = boundaryOpt.get();
            timeToBoundary = DriverStation.getMatchTime() - boundaryMatchTime;

            if (!sameBoundary(boundaryMatchTime, lastWarnedBoundaryMatchTime)
                    && timeToBoundary >= 0.0
                    && timeToBoundary <= WARNING_LEAD_SECONDS) {
                rumbleStopTimestamp = Timer.getFPGATimestamp() + RUMBLE_DURATION_SECONDS;
                lastWarnedBoundaryMatchTime = boundaryMatchTime;
            }
        }

        boolean rumbling = Timer.getFPGATimestamp() < rumbleStopTimestamp;
        setRumble(rumbling ? RUMBLE_STRENGTH : 0.0);
        if (!rumbling) {
            rumbleStopTimestamp = -1.0;
        }

        publishTelemetry(boundaryOpt, timeToBoundary);
    }

    private void stopRumble() {
        rumbleStopTimestamp = -1.0;
        setRumble(0.0);
    }

    private void setRumble(double value) {
        for (GenericHID controller : controllers) {
            controller.setRumble(RumbleType.kBothRumble, value);
        }
    }

    private boolean sameBoundary(double a, double b) {
        return Math.abs(a - b) < 1e-6;
    }

    private void publishTelemetry(Optional<Double> boundaryOpt, double timeToBoundary) {
        SmartDashboard.putBoolean("Rumble/TeleopEnabled", DriverStation.isTeleopEnabled());
        SmartDashboard.putBoolean("Rumble/Active", rumbleStopTimestamp > Timer.getFPGATimestamp());
        SmartDashboard.putNumber("Rumble/TimeToActivationBoundary", timeToBoundary);
        SmartDashboard.putNumber("Rumble/ActivationBoundaryMatchTime", boundaryOpt.orElse(-1.0));
        SmartDashboard.putNumber("Rumble/LastWarnedBoundaryMatchTime", lastWarnedBoundaryMatchTime);
    }
}
