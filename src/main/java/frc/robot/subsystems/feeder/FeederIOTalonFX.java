package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FeederIOTalonFX implements FeederIO {
    private final TalonFX panMotor;
    private final TalonFX pusherMotor;

    private static final int PAN_MOTOR_ID = 14;
    private static final int PUSHER_MOTOR_ID = 23;

    private static final double PAN_GEAR_RATIO =  2688/324;
    private static final double PUSHER_GEAR_RATIO = 1.0 / 2.5;

    private final VelocityVoltage panRequest = new VelocityVoltage(0);
    private final VelocityVoltage pusherRequest = new VelocityVoltage(0);

    private final StatusSignal<Angle> panPosition;
    private final StatusSignal<AngularVelocity> panVelocity;
    private final StatusSignal<Voltage> panVoltage;
    private final StatusSignal<Current> panCurrent;
    private final StatusSignal<Temperature> panTemp;

    private final StatusSignal<Angle> pusherPosition;
    private final StatusSignal<AngularVelocity> pusherVelocity;
    private final StatusSignal<Voltage> pusherVoltage;
    private final StatusSignal<Current> pusherCurrent;
    private final StatusSignal<Temperature> pusherTemp;

    public FeederIOTalonFX() {
        panMotor = new TalonFX(PAN_MOTOR_ID);
        pusherMotor = new TalonFX(PUSHER_MOTOR_ID);

        configureMotor(panMotor, 0.25, 40, true);
        configureMotor(pusherMotor, 0.15, 40, true);

        panPosition = panMotor.getPosition();
        panVelocity = panMotor.getVelocity();
        panVoltage = panMotor.getMotorVoltage();
        panCurrent = panMotor.getSupplyCurrent();
        panTemp = panMotor.getDeviceTemp();

        pusherPosition = pusherMotor.getPosition();
        pusherVelocity = pusherMotor.getVelocity();
        pusherVoltage = pusherMotor.getMotorVoltage();
        pusherCurrent = pusherMotor.getSupplyCurrent();
        pusherTemp = pusherMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            panPosition, panVelocity, panVoltage, panCurrent, panTemp,
            pusherPosition, pusherVelocity, pusherVoltage, pusherCurrent, pusherTemp
        );

        panMotor.optimizeBusUtilization();
        pusherMotor.optimizeBusUtilization();
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
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    private double rpmToRps(double rpm, double gearRatio) {
        return (rpm / 60.0) / gearRatio;
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            panPosition, panVelocity, panVoltage, panCurrent, panTemp,
            pusherPosition, pusherVelocity, pusherVoltage, pusherCurrent, pusherTemp
        );

        inputs.panPositionRotations = panPosition.getValueAsDouble();
        inputs.panVelocityRPS = panVelocity.getValueAsDouble();
        inputs.panAppliedVolts = panVoltage.getValueAsDouble();
        inputs.panCurrentAmps = panCurrent.getValueAsDouble();
        inputs.panTempCelsius = panTemp.getValueAsDouble();

        inputs.pusherPositionRotations = pusherPosition.getValueAsDouble();
        inputs.pusherVelocityRPS = pusherVelocity.getValueAsDouble();
        inputs.pusherAppliedVolts = pusherVoltage.getValueAsDouble();
        inputs.pusherCurrentAmps = pusherCurrent.getValueAsDouble();
        inputs.pusherTempCelsius = pusherTemp.getValueAsDouble();
    }

    @Override
    public void setPanRPM(double rpm) {
        panMotor.setControl(panRequest.withVelocity(rpmToRps(rpm, PAN_GEAR_RATIO)));
    }

    @Override
    public void setPusherRPM(double rpm) {
        pusherMotor.setControl(pusherRequest.withVelocity(rpmToRps(rpm, PUSHER_GEAR_RATIO)));
    }

    @Override
    public void stop() {
        panMotor.set(0);
        pusherMotor.set(0);
    }
}
