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
    private final TalonFX leftPusherAssistMotor;
    private final TalonFX rightPusherAssistMotor;

    private final VelocityVoltage panRequest = new VelocityVoltage(0);
    private final VelocityVoltage pusherRequest = new VelocityVoltage(0);
    private final VelocityVoltage leftPusherAssistRequest = new VelocityVoltage(0);
    private final VelocityVoltage rightPusherAssistRequest = new VelocityVoltage(0);

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

    private final StatusSignal<AngularVelocity> leftPusherAssistVelocity;
    private final StatusSignal<Voltage> leftPusherAssistVoltage;
    private final StatusSignal<Current> leftPusherAssistCurrent;
    private final StatusSignal<Temperature> leftPusherAssistTemp;

    private final StatusSignal<AngularVelocity> rightPusherAssistVelocity;
    private final StatusSignal<Voltage> rightPusherAssistVoltage;
    private final StatusSignal<Current> rightPusherAssistCurrent;
    private final StatusSignal<Temperature> rightPusherAssistTemp;

    public FeederIOTalonFX() {
        panMotor = new TalonFX(FeederConstants.PAN_MOTOR_ID, FeederConstants.CAN_BUS);
        pusherMotor = new TalonFX(FeederConstants.PUSHER_MOTOR_ID, FeederConstants.CAN_BUS);
        leftPusherAssistMotor = new TalonFX(FeederConstants.LEFT_PUSHER_ASSIST_MOTOR_ID, FeederConstants.CAN_BUS);
        rightPusherAssistMotor = new TalonFX(FeederConstants.RIGHT_PUSHER_ASSIST_MOTOR_ID, FeederConstants.CAN_BUS);

        configureMotor(
            panMotor,
            FeederConstants.PAN_kP,
            FeederConstants.SUPPLY_CURRENT_LIMIT,
            FeederConstants.PAN_MOTOR_INVERTED
        );
        configureMotor(
            pusherMotor,
            FeederConstants.PUSHER_kP,
            FeederConstants.SUPPLY_CURRENT_LIMIT,
            FeederConstants.PUSHER_MOTOR_INVERTED
        );
        configureMotor(
            leftPusherAssistMotor,
            FeederConstants.PUSHER_kP,
            FeederConstants.SUPPLY_CURRENT_LIMIT,
            FeederConstants.LEFT_PUSHER_ASSIST_MOTOR_INVERTED
        );
        configureMotor(
            rightPusherAssistMotor,
            FeederConstants.PUSHER_kP,
            FeederConstants.SUPPLY_CURRENT_LIMIT,
            FeederConstants.RIGHT_PUSHER_ASSIST_MOTOR_INVERTED
        );

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

        leftPusherAssistVelocity = leftPusherAssistMotor.getVelocity();
        leftPusherAssistVoltage = leftPusherAssistMotor.getMotorVoltage();
        leftPusherAssistCurrent = leftPusherAssistMotor.getSupplyCurrent();
        leftPusherAssistTemp = leftPusherAssistMotor.getDeviceTemp();

        rightPusherAssistVelocity = rightPusherAssistMotor.getVelocity();
        rightPusherAssistVoltage = rightPusherAssistMotor.getMotorVoltage();
        rightPusherAssistCurrent = rightPusherAssistMotor.getSupplyCurrent();
        rightPusherAssistTemp = rightPusherAssistMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            panPosition, panVelocity, panVoltage, panCurrent, panTemp,
            pusherPosition, pusherVelocity, pusherVoltage, pusherCurrent, pusherTemp,
            leftPusherAssistVelocity, leftPusherAssistVoltage, leftPusherAssistCurrent, leftPusherAssistTemp,
            rightPusherAssistVelocity, rightPusherAssistVoltage, rightPusherAssistCurrent, rightPusherAssistTemp
        );

        panMotor.optimizeBusUtilization();
        pusherMotor.optimizeBusUtilization();
        leftPusherAssistMotor.optimizeBusUtilization();
        rightPusherAssistMotor.optimizeBusUtilization();
    }

    private void configureMotor(TalonFX motor, double kP, int currentLimit, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kV = FeederConstants.VELOCITY_kV;
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

    private void setPusherMotorRPM(TalonFX motor, VelocityVoltage request, double rpm) {
        motor.setControl(request.withVelocity(rpmToRps(rpm, FeederConstants.PUSHER_GEAR_RATIO)));
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            panPosition, panVelocity, panVoltage, panCurrent, panTemp,
            pusherPosition, pusherVelocity, pusherVoltage, pusherCurrent, pusherTemp,
            leftPusherAssistVelocity, leftPusherAssistVoltage, leftPusherAssistCurrent, leftPusherAssistTemp,
            rightPusherAssistVelocity, rightPusherAssistVoltage, rightPusherAssistCurrent, rightPusherAssistTemp
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

        inputs.leftPusherAssistVelocityRPS = leftPusherAssistVelocity.getValueAsDouble();
        inputs.leftPusherAssistAppliedVolts = leftPusherAssistVoltage.getValueAsDouble();
        inputs.leftPusherAssistCurrentAmps = leftPusherAssistCurrent.getValueAsDouble();
        inputs.leftPusherAssistTempCelsius = leftPusherAssistTemp.getValueAsDouble();

        inputs.rightPusherAssistVelocityRPS = rightPusherAssistVelocity.getValueAsDouble();
        inputs.rightPusherAssistAppliedVolts = rightPusherAssistVoltage.getValueAsDouble();
        inputs.rightPusherAssistCurrentAmps = rightPusherAssistCurrent.getValueAsDouble();
        inputs.rightPusherAssistTempCelsius = rightPusherAssistTemp.getValueAsDouble();
    }

    @Override
    public void setPanRPM(double rpm) {
        panMotor.setControl(panRequest.withVelocity(rpmToRps(rpm, FeederConstants.PAN_GEAR_RATIO)));
    }

    @Override
    public void setPusherRPM(double rpm) {
        // The pusher path is now a matched three-motor group, with one assist roller reversed.
        setPusherMotorRPM(pusherMotor, pusherRequest, rpm);
        setPusherMotorRPM(
            leftPusherAssistMotor,
            leftPusherAssistRequest,
            rpm * FeederConstants.LEFT_PUSHER_ASSIST_DIRECTION
        );
        setPusherMotorRPM(
            rightPusherAssistMotor,
            rightPusherAssistRequest,
            rpm * FeederConstants.RIGHT_PUSHER_ASSIST_DIRECTION
        );
    }

    @Override
    public void stop() {
        panMotor.set(0.0);
        pusherMotor.set(0.0);
        leftPusherAssistMotor.set(0.0);
        rightPusherAssistMotor.set(0.0);
    }
}
