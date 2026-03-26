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
import frc.robot.Constants.FeederConstants;

public class FeederIOTalonFX implements FeederIO {
    private final TalonFX panMotor;
    private final TalonFX pusherMotor;
    private final TalonFX floorMotor;

    private final VelocityVoltage panRequest = new VelocityVoltage(0);
    private final VelocityVoltage pusherRequest = new VelocityVoltage(0);
    private final VelocityVoltage floorRequest = new VelocityVoltage(0);
    private final VelocityVoltage wallRequest = new VelocityVoltage(0);

    private static final double PAN_GEAR_RATIO = FeederConstants.PAN_GEAR_RATIO;
    private static final double PUSHER_GEAR_RATIO = FeederConstants.PUSHER_GEAR_RATIO;

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

    private final StatusSignal<Angle> floorPosition;
    private final StatusSignal<AngularVelocity> floorVelocity;
    private final StatusSignal<Voltage> floorVoltage;
    private final StatusSignal<Current> floorCurrent;
    private final StatusSignal<Temperature> floorTemp;

    public FeederIOTalonFX() {
        panMotor = new TalonFX(FeederConstants.PAN_MOTOR_ID, FeederConstants.CAN_BUS);
        pusherMotor = new TalonFX(FeederConstants.PUSHER_MOTOR_ID, FeederConstants.CAN_BUS);
        floorMotor = new TalonFX(FeederConstants.FLOOR_ID, FeederConstants.CAN_BUS);

        configureMotor(panMotor, FeederConstants.PAN_kP, FeederConstants.SUPPLY_CURRENT_LIMIT, FeederConstants.PAN_MOTOR_INVERTED);
        configureMotor(pusherMotor, FeederConstants.PUSHER_kP, FeederConstants.SUPPLY_CURRENT_LIMIT, FeederConstants.PUSHER_MOTOR_INVERTED);
        configureMotor(floorMotor, FeederConstants.FLOOR_kP, FeederConstants.SUPPLY_CURRENT_LIMIT, FeederConstants.FLOOR_MOTOR_INVERTED);

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

        floorPosition = floorMotor.getPosition();
        floorVelocity = floorMotor.getVelocity();
        floorVoltage = floorMotor.getMotorVoltage();
        floorCurrent = floorMotor.getSupplyCurrent();
        floorTemp = floorMotor.getDeviceTemp();


        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            panPosition, panVelocity, panVoltage, panCurrent, panTemp,
            pusherPosition, pusherVelocity, pusherVoltage, pusherCurrent, pusherTemp,
            floorPosition, floorVelocity, floorVoltage, floorCurrent, floorTemp
        );

        panMotor.optimizeBusUtilization();
        pusherMotor.optimizeBusUtilization();
        floorMotor.optimizeBusUtilization();
    }

    private void configureMotor(TalonFX motor, double kP, int currentLimit, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kV = FeederConstants.kV;
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = FeederConstants.STATOR_CURRENT_LIMIT;
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
            pusherPosition, pusherVelocity, pusherVoltage, pusherCurrent, pusherTemp,
            floorPosition, floorVelocity, floorVoltage, floorCurrent, floorTemp
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

        inputs.floorPositionRotations = floorPosition.getValueAsDouble();
        inputs.floorVelocityRPS = floorVelocity.getValueAsDouble();
        inputs.floorAppliedVolts = floorVoltage.getValueAsDouble();
        inputs.floorCurrentAmps = floorCurrent.getValueAsDouble();
        inputs.floorTempCelsius = floorTemp.getValueAsDouble();
    }

    @Override
    public void setPanRPM(double rpm) {
        panMotor.setControl(panRequest.withVelocity(rpm / 60.0));
    }

    @Override
    /** Mechanism only */
    public void setPusherRPM(double rpm) {
        pusherMotor.setControl(pusherRequest.withVelocity(rpm / 60.0));
    }

    @Override
    public void setFloorRPM(double rpm) {
        floorMotor.setControl(floorRequest.withVelocity(rpmToRps(rpm, FeederConstants.FLOOR_GEAR_RATIO)));
    }

    @Override
    public void stop() {
        panMotor.set(0);
        pusherMotor.set(0);
    }
}
