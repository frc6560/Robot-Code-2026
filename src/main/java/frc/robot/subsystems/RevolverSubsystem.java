package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Revolver subsystem using a robust state machine:
 * IDLE -> SPINNING_UP -> FEEDING -> STAGED
 * OVERRIDE bypasses beam breaker.
 */
public class RevolverSubsystem extends SubsystemBase {

    /* ======================= Hardware ======================= */
    private final TalonFX panMotor = new TalonFX(14);
    private final TalonFX pusherMotor = new TalonFX(23);
    private final DigitalInput beamBreaker = new DigitalInput(0); // DIO 0

    /* ======================= RPM Targets ======================= */
    private static final double PAN_RUNNING_RPM = 180.0;
    private static final double PUSHER_RUNNING_RPM = 2500.0;
    private static final double IDLE_RPM = 10.0;

    /* ======================= Gearing ======================= */
    private static final double PAN_GEAR_RATIO = 1.0;
    private static final double PUSHER_GEAR_RATIO = 1.0;

    /* ======================= Tolerances ======================= */
    private static final double PAN_SPEED_TOLERANCE_RPM = 20.0;

    private final VelocityVoltage panRequest = new VelocityVoltage(0);
    private final VelocityVoltage pusherRequest = new VelocityVoltage(0);

    /* ======================= State Machine ======================= */
    public enum RevolverState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        STAGED,
        OVERRIDE
    }

    private RevolverState state = RevolverState.IDLE;
    private boolean beamBreakOverride = false;

    /* ======================= Constructor ======================= */
    public RevolverSubsystem() {
        configureMotor(panMotor, 0.25, 40);
        configureMotor(pusherMotor, 0.15, 60);
    }

    private void configureMotor(TalonFX motor, double kP, int currentLimit) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kV = 0.12;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    /* ======================= Motor Helpers ======================= */
    private double rpmToRps(double rpm, double gearRatio) {
        return (rpm / 60.0) / gearRatio;
    }

    private void setPanRPM(double rpm) {
        panMotor.setControl(
            panRequest.withVelocity(rpmToRps(rpm, PAN_GEAR_RATIO))
        );
    }

    private void setPusherRPM(double rpm) {
        pusherMotor.setControl(
            pusherRequest.withVelocity(rpmToRps(rpm, PUSHER_GEAR_RATIO))
        );
    }

    /* ======================= Sensors ======================= */
    public boolean hasBall() {
        return !beamBreaker.get(); // beam broken = ball present
    }

    /* ======================= Velocity Feedback ======================= */
    public double getPanActualRPM() {
        return panMotor.getVelocity().getValueAsDouble()
                * 60.0
                * PAN_GEAR_RATIO;
    }

    public double getPusherActualRPM() {
        return pusherMotor.getVelocity().getValueAsDouble()
                * 60.0
                * PUSHER_GEAR_RATIO;
    }

    public boolean panAtSpeed() {
        return Math.abs(getPanActualRPM() - PAN_RUNNING_RPM)
                < PAN_SPEED_TOLERANCE_RPM;
    }

    /* ======================= Requests ======================= */
    public void requestFeed() {
        if (state == RevolverState.IDLE) {
            state = RevolverState.SPINNING_UP;
        }
    }

    public void requestStop() {
        state = RevolverState.IDLE;
    }

    public void enableBeamBreakOverride() {
        beamBreakOverride = true;
        state = RevolverState.OVERRIDE;
    }

    public void disableBeamBreakOverride() {
        beamBreakOverride = false;

        if (state == RevolverState.OVERRIDE) {
            state = hasBall() ? RevolverState.STAGED : RevolverState.FEEDING;
        }
    }

    /* ======================= Periodic State Machine ======================= */
    @Override
    public void periodic() {

        switch (state) {
            case IDLE:
                setPanRPM(IDLE_RPM);
                setPusherRPM(IDLE_RPM);
                break;

            case SPINNING_UP:
                setPanRPM(PAN_RUNNING_RPM);
                setPusherRPM(IDLE_RPM);
                if (panAtSpeed()) {
                    state = RevolverState.FEEDING;
                }
                break;

            case FEEDING:
                setPanRPM(PAN_RUNNING_RPM);
                if (!hasBall() || beamBreakOverride) {
                    setPusherRPM(PUSHER_RUNNING_RPM);
                } else {
                    setPusherRPM(IDLE_RPM);
                    state = RevolverState.STAGED;
                }
                break;

            case STAGED:
                setPanRPM(PAN_RUNNING_RPM);
                setPusherRPM(IDLE_RPM);
                break;

            case OVERRIDE:
                setPanRPM(PAN_RUNNING_RPM);
                setPusherRPM(PUSHER_RUNNING_RPM);
                break;
        }

        /* ======================= Telemetry ======================= */
        SmartDashboard.putString("Revolver/State", state.name());
        SmartDashboard.putBoolean("Revolver/Has Ball", hasBall());
        SmartDashboard.putBoolean("Revolver/Beam Override", beamBreakOverride);

        SmartDashboard.putNumber("Revolver/Pan Actual RPM", getPanActualRPM());
        SmartDashboard.putNumber("Revolver/Pusher Actual RPM", getPusherActualRPM());
        SmartDashboard.putBoolean("Revolver/Pan At Speed", panAtSpeed());
    }

    /* ======================= Accessor ======================= */
    public RevolverState getState() {
        return state;
    }
} 