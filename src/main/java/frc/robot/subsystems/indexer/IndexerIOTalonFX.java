package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {
    private final TalonFX floorLeader;
    private final TalonFX tunnelMotor;

    private final VelocityVoltage floorRequest = new VelocityVoltage(0.0);
    private final VelocityVoltage tunnelRequest = new VelocityVoltage(0.0);
    private final NeutralOut floorStopRequest = new NeutralOut();
    private final NeutralOut tunnelStopRequest = new NeutralOut();
    private final StatusCode floorConfigurationStatus;
    private final StatusCode tunnelConfigurationStatus;
    private String tunnelControlStatus = "No request";
    private String floorControlStatus = "No request";

    private final StatusSignal<Angle> floorPosition;
    private final StatusSignal<AngularVelocity> floorVelocity;
    private final StatusSignal<Voltage> floorVoltage;
    private final StatusSignal<Current> floorLeaderCurrent;

    private final StatusSignal<Angle> tunnelPosition;
    private final StatusSignal<AngularVelocity> tunnelVelocity;
    private final StatusSignal<Voltage> tunnelVoltage;
    private final StatusSignal<Current> tunnelCurrent;

    public IndexerIOTalonFX() {
        this(IndexerConstants.FLOOR_MOTOR_ID, IndexerConstants.TUNNEL_MOTOR_ID);
    }

    public IndexerIOTalonFX(
            int floorLeaderId,
            int tunnelMotorId) {
        this(floorLeaderId, tunnelMotorId, new CANBus(IndexerConstants.CAN_BUS));
    }

    public IndexerIOTalonFX(
            int floorLeaderId,
            int tunnelMotorId,
            CANBus canBus) {
        floorLeader = new TalonFX(floorLeaderId, canBus);
        tunnelMotor = new TalonFX(tunnelMotorId, canBus);

        floorConfigurationStatus = configureMotor(floorLeader, IndexerConstants.FLOOR_GEAR_RATIO,
            IndexerConstants.FLOOR_MOTOR_INVERTED);
        tunnelConfigurationStatus = configureMotor(tunnelMotor, IndexerConstants.TUNNEL_GEAR_RATIO,
            IndexerConstants.TUNNEL_MOTOR_INVERTED);

        floorPosition = floorLeader.getPosition();
        floorVelocity = floorLeader.getVelocity();
        floorVoltage = floorLeader.getMotorVoltage();
        floorLeaderCurrent = floorLeader.getStatorCurrent();

        tunnelPosition = tunnelMotor.getPosition();
        tunnelVelocity = tunnelMotor.getVelocity();
        tunnelVoltage = tunnelMotor.getMotorVoltage();
        tunnelCurrent = tunnelMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                IndexerConstants.STATUS_FREQUENCY_HZ,
                floorPosition,
                floorVelocity,
                floorVoltage,
                floorLeaderCurrent,
                tunnelPosition,
                tunnelVelocity,
                tunnelVoltage,
                tunnelCurrent);

        floorLeader.optimizeBusUtilization();
        tunnelMotor.optimizeBusUtilization();
    }

    private static StatusCode configureMotor(TalonFX motor, double gearRatio, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = gearRatio;
        config.Slot0.kP = IndexerConstants.VELOCITY_KP;
        config.Slot0.kV = IndexerConstants.MOTOR_KV * gearRatio;
        config.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        StatusCode status = motor.getConfigurator().apply(config, 0.1);
        for (int attempt = 1; attempt < 3 && !status.isOK(); attempt++) {
            status = motor.getConfigurator().apply(config, 0.1);
        }
        return status;
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.floorTelemetryStatus = BaseStatusSignal.refreshAll(
                floorPosition,
                floorVelocity,
                floorVoltage,
                floorLeaderCurrent).toString();
        inputs.tunnelTelemetryStatus = BaseStatusSignal.refreshAll(
                tunnelPosition,
                tunnelVelocity,
                tunnelVoltage,
                tunnelCurrent).toString();

        inputs.appliedVolts = tunnelVoltage.getValueAsDouble();
        inputs.floorRPM = floorVelocity.getValueAsDouble() * 60.0;
        inputs.tunnelRPM = tunnelVelocity.getValueAsDouble() * 60.0;
        inputs.currentAmps = floorLeaderCurrent.getValueAsDouble() + tunnelCurrent.getValueAsDouble();
        inputs.floorConnected = floorLeader.isConnected();
        inputs.floorAppliedVolts = floorVoltage.getValueAsDouble();
        inputs.floorCurrentAmps = floorLeaderCurrent.getValueAsDouble();
        inputs.floorConfigurationStatus = floorConfigurationStatus.toString();
        inputs.floorControlStatus = floorControlStatus;
        inputs.tunnelConnected = tunnelMotor.isConnected();
        inputs.tunnelAppliedVolts = tunnelVoltage.getValueAsDouble();
        inputs.tunnelCurrentAmps = tunnelCurrent.getValueAsDouble();
        inputs.tunnelConfigurationStatus = tunnelConfigurationStatus.toString();
        inputs.tunnelControlStatus = tunnelControlStatus;

        Logger.recordOutput("Indexer/Connected/FloorLeader", floorLeader.isConnected());
        Logger.recordOutput("Indexer/Connected/TunnelMotor", tunnelMotor.isConnected());
    }

    @Override
    public void setFloorRPM(double rpm) {
        if (rpm == 0.0) {
            floorControlStatus = floorLeader.setControl(floorStopRequest).toString();
        } else {
            floorControlStatus = floorLeader.setControl(floorRequest.withVelocity(rpm / 60.0)).toString();
        }
    }

    @Override
    public void setTunnelRPM(double rpm) {
        if (rpm == 0.0) {
            tunnelControlStatus = tunnelMotor.setControl(tunnelStopRequest).toString();
        } else {
            tunnelControlStatus = tunnelMotor.setControl(tunnelRequest.withVelocity(rpm / 60.0)).toString();
        }
    }

    @Override
    public void stop() {
        setFloorRPM(0.0);
        setTunnelRPM(0.0);
    }
}
