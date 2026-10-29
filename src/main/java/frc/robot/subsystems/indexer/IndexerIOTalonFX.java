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
    private static final double CURRENT_LIMIT_AMPS = 40.0;
    private static final double STATUS_FREQUENCY_HZ = 50.0;

    private final TalonFX floorLeader;
    private final TalonFX floorSecondary;
    private final TalonFX tunnelMotor;

    private final VelocityVoltage floorRequest = new VelocityVoltage(0.0);
    private final VelocityVoltage floorSecondaryRequest = new VelocityVoltage(0.0);
    private final VelocityVoltage tunnelRequest = new VelocityVoltage(0.0);
    private final NeutralOut floorStopRequest = new NeutralOut();
    private final NeutralOut floorSecondaryStopRequest = new NeutralOut();
    private final StatusCode floorConfigurationStatus;
    private final StatusCode floorSecondaryConfigurationStatus;
    private String floorControlStatus = "No request";

    private final StatusSignal<Angle> floorPosition;
    private final StatusSignal<AngularVelocity> floorVelocity;
    private final StatusSignal<Voltage> floorVoltage;
    private final StatusSignal<Current> floorLeaderCurrent;
    private final StatusSignal<AngularVelocity> floorSecondaryVelocity;
    private final StatusSignal<Voltage> floorSecondaryVoltage;
    private final StatusSignal<Current> floorSecondaryCurrent;

    private final StatusSignal<Angle> tunnelPosition;
    private final StatusSignal<AngularVelocity> tunnelVelocity;
    private final StatusSignal<Voltage> tunnelVoltage;
    private final StatusSignal<Current> tunnelCurrent;

    public IndexerIOTalonFX(
            int floorLeaderId,
            int floorSecondaryId,
            int tunnelMotorId) {
        this(floorLeaderId, floorSecondaryId, tunnelMotorId, new CANBus(IndexerConstants.CAN_BUS));
    }

    public IndexerIOTalonFX(
            int floorLeaderId,
            int floorSecondaryId,
            int tunnelMotorId,
            CANBus canBus) {
        floorLeader = new TalonFX(floorLeaderId, canBus);
        floorSecondary = new TalonFX(floorSecondaryId, canBus);
        tunnelMotor = new TalonFX(tunnelMotorId, canBus);

        floorConfigurationStatus = configureMotor(floorLeader, IndexerConstants.FLOOR_GEAR_RATIO,
            IndexerConstants.FLOOR_MOTOR_INVERTED);
        floorSecondaryConfigurationStatus = configureMotor(floorSecondary, IndexerConstants.FLOOR_GEAR_RATIO,
            IndexerConstants.FLOOR_SECONDARY_MOTOR_INVERTED);
        configureMotor(tunnelMotor, IndexerConstants.TUNNEL_GEAR_RATIO, false);

        floorPosition = floorLeader.getPosition();
        floorVelocity = floorLeader.getVelocity();
        floorVoltage = floorLeader.getMotorVoltage();
        floorLeaderCurrent = floorLeader.getStatorCurrent();
        floorSecondaryVelocity = floorSecondary.getVelocity();
        floorSecondaryVoltage = floorSecondary.getMotorVoltage();
        floorSecondaryCurrent = floorSecondary.getStatorCurrent();

        tunnelPosition = tunnelMotor.getPosition();
        tunnelVelocity = tunnelMotor.getVelocity();
        tunnelVoltage = tunnelMotor.getMotorVoltage();
        tunnelCurrent = tunnelMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                STATUS_FREQUENCY_HZ,
                floorPosition,
                floorVelocity,
                floorVoltage,
                floorLeaderCurrent,
                floorSecondaryVelocity,
                floorSecondaryVoltage,
                floorSecondaryCurrent,
                tunnelPosition,
                tunnelVelocity,
                tunnelVoltage,
                tunnelCurrent);

        floorLeader.optimizeBusUtilization();
        floorSecondary.optimizeBusUtilization();
        tunnelMotor.optimizeBusUtilization();
    }

    static TalonFXConfiguration motorConfiguration(double gearRatio, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = gearRatio;
        config.Slot0.kP = IndexerConstants.VELOCITY_KP;
        config.Slot0.kV = IndexerConstants.MOTOR_KV * gearRatio;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        return config;
    }

    private static StatusCode configureMotor(TalonFX motor, double gearRatio, boolean inverted) {
        TalonFXConfiguration config = motorConfiguration(gearRatio, inverted);
        StatusCode status = motor.getConfigurator().apply(config, 0.1);
        for (int attempt = 1; attempt < 3 && !status.isOK(); attempt++) {
            status = motor.getConfigurator().apply(config, 0.1);
        }
        return status;
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                floorPosition,
                floorVelocity,
                floorVoltage,
                floorLeaderCurrent,
                floorSecondaryVelocity,
                floorSecondaryVoltage,
                floorSecondaryCurrent,
                tunnelPosition,
                tunnelVelocity,
                tunnelVoltage,
                tunnelCurrent);

        inputs.appliedVolts = tunnelVoltage.getValueAsDouble();
        inputs.floorRPM = floorVelocity.getValueAsDouble() * 60.0;
        inputs.tunnelRPM = tunnelVelocity.getValueAsDouble() * 60.0;
        inputs.currentAmps = floorLeaderCurrent.getValueAsDouble()
            + floorSecondaryCurrent.getValueAsDouble() + tunnelCurrent.getValueAsDouble();
        inputs.floorConnected = floorLeader.isConnected() && floorSecondary.isConnected();
        inputs.floorSecondaryConnected = floorSecondary.isConnected();
        inputs.floorSecondaryRPM = floorSecondaryVelocity.getValueAsDouble() * 60.0;
        inputs.floorSecondaryAppliedVolts = floorSecondaryVoltage.getValueAsDouble();
        inputs.floorSecondaryCurrentAmps = floorSecondaryCurrent.getValueAsDouble();
        inputs.floorAppliedVolts = floorVoltage.getValueAsDouble();
        inputs.floorCurrentAmps = floorLeaderCurrent.getValueAsDouble();
        inputs.floorConfigurationStatus = "ID25=" + floorConfigurationStatus
            + " ID0=" + floorSecondaryConfigurationStatus;
        inputs.floorControlStatus = floorControlStatus;

        Logger.recordOutput("Indexer/Connected/FloorLeader", floorLeader.isConnected());
        Logger.recordOutput("Indexer/Connected/FloorSecondary", floorSecondary.isConnected());
        Logger.recordOutput("Indexer/Connected/TunnelMotor", tunnelMotor.isConnected());
    }

    @Override
    public void setFloorRPM(double rpm) {
        if (rpm == 0.0 || !Double.isFinite(rpm) || !floorConfigurationStatus.isOK()
                || !floorSecondaryConfigurationStatus.isOK()) {
            StatusCode first = floorLeader.setControl(floorStopRequest);
            StatusCode second = floorSecondary.setControl(floorSecondaryStopRequest);
            floorControlStatus = "Stopped: ID25=" + first + " ID0=" + second;
        } else {
            // Same positive mechanism RPM, opposite configured physical directions.
            StatusCode first = floorLeader.setControl(floorRequest.withVelocity(rpm / 60.0));
            StatusCode second = floorSecondary.setControl(floorSecondaryRequest.withVelocity(rpm / 60.0));
            floorControlStatus = "ID25=" + first + " ID0=" + second;
            if (!first.isOK() || !second.isOK()) {
                floorLeader.setControl(floorStopRequest);
                floorSecondary.setControl(floorSecondaryStopRequest);
            }
        }
    }

    @Override
    public void setTunnelRPM(double rpm) {
        if (rpm == 0.0) {
            tunnelMotor.stopMotor();
        } else {
            tunnelMotor.setControl(tunnelRequest.withVelocity(rpm / 60.0));
        }
    }

    @Override
    public void stop() {
        setFloorRPM(0.0);
        tunnelMotor.stopMotor();
    }
}
