package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Feeder extends SubsystemBase {

    private final TalonFX panMotor = new TalonFX(14);
    private final TalonFX pusherMotor = new TalonFX(23);

    private static final double PAN_RUNNING_RPM = 60.0;
    private static final double PUSHER_RUNNING_RPM = 3000.0;
    private static final double IDLE_RPM = 0.0;

    private static final double PAN_GEAR_RATIO = 1.0 / 33.14;
    private static final double PUSHER_GEAR_RATIO = 1.0 / 2.5;

    private static final double PAN_SPEED_TOLERANCE_RPM = 20.0;

    private final VelocityVoltage panRequest = new VelocityVoltage(0);
    private final VelocityVoltage pusherRequest = new VelocityVoltage(0);

    public enum RevolverState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        STAGED,
        OVERRIDE
    }

    private RevolverState state = RevolverState.IDLE;

    /* ======================= Constructor ======================= */
    public Feeder() {
        configureMotor(panMotor, 0.25, 40, true);
        configureMotor(pusherMotor, 0.15, 60, true);
    }

    private void configureMotor(TalonFX motor, double kP, int currentLimit, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kV = 0.12;
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
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
        return true; // Always assume ball is present (no beam break sensor)
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

    public void requestOverride() {
        state = RevolverState.OVERRIDE;
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
                setPusherRPM(PUSHER_RUNNING_RPM);
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


        SmartDashboard.putString("Revolver/State", state.name());

        SmartDashboard.putNumber("Revolver/Pan Actual RPM", getPanActualRPM());
        SmartDashboard.putNumber("Revolver/Pusher Actual RPM", getPusherActualRPM());
        SmartDashboard.putBoolean("Revolver/Pan At Speed", panAtSpeed());
    }

    public RevolverState getState() {
        return state;
    }
} 