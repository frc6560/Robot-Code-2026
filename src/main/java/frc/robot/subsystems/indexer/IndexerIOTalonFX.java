package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
    private final Follower floorFollowerRequest;
    private final StatusCode floorSecondaryConfigurationStatus;
    private final TalonFX tunnelMotor;
    private final StatusCode tunnelConfigurationStatus;

    private final VelocityVoltage floorRequest = new VelocityVoltage(0.0);
    private final VelocityVoltage tunnelRequest = new VelocityVoltage(0.0);
    private final NeutralOut floorStopRequest = new NeutralOut();
    private final NeutralOut floorFollowerStopRequest = new NeutralOut();
    private final NeutralOut tunnelStopRequest = new NeutralOut();
    private final StatusCode floorConfigurationStatus;
    private String floorControlStatus = "No request";
    private String tunnelControlStatus = "No request";
    private double requestedFloorRPM, requestedTunnelRPM;

    private final StatusSignal<Angle> floorPosition;
    private final StatusSignal<AngularVelocity> floorVelocity;
    private final StatusSignal<Voltage> floorVoltage;
    private final StatusSignal<Current> floorLeaderCurrent;

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
        floorFollowerRequest = new Follower(floorLeaderId, MotorAlignmentValue.Aligned);
        tunnelMotor = new TalonFX(tunnelMotorId, canBus);

        floorConfigurationStatus = configureMotor(floorLeader, IndexerConstants.FLOOR_GEAR_RATIO,
            IndexerConstants.FLOOR_MOTOR_INVERTED, true);
        // ID 0 mirrors leader voltage; it never runs its own RPM controller.
        TalonFXConfiguration followerConfig = motorConfiguration(IndexerConstants.FLOOR_GEAR_RATIO,
            IndexerConstants.FLOOR_SECONDARY_MOTOR_INVERTED, true);
        followerConfig.Slot0.kP = 0;
        followerConfig.Slot0.kV = 0;
        StatusCode followerStatus = floorSecondary.getConfigurator().apply(followerConfig, 0.1);
        for (int attempt = 1; attempt < 3 && !followerStatus.isOK(); attempt++) {
            followerStatus = floorSecondary.getConfigurator().apply(followerConfig, 0.1);
        }
        floorSecondaryConfigurationStatus = followerStatus;
        tunnelConfigurationStatus = configureMotor(tunnelMotor, IndexerConstants.TUNNEL_GEAR_RATIO, false, false);

        floorPosition = floorLeader.getPosition();
        floorVelocity = floorLeader.getVelocity();
        floorVoltage = floorLeader.getMotorVoltage();
        floorLeaderCurrent = floorLeader.getStatorCurrent();

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
                floorSecondary.getVelocity(),
                floorSecondary.getMotorVoltage(),
                floorSecondary.getStatorCurrent(),
                tunnelPosition,
                tunnelVelocity,
                tunnelVoltage,
                tunnelCurrent);

        floorLeader.optimizeBusUtilization();
        floorSecondary.optimizeBusUtilization();
        tunnelMotor.optimizeBusUtilization();
        frc.robot.utility.CurrentUsageMonitor.register("Floor leader", floorLeader);
        frc.robot.utility.CurrentUsageMonitor.register("Floor follower", floorSecondary);
        frc.robot.utility.CurrentUsageMonitor.register("Wall tunnel", tunnelMotor);
    }

    static TalonFXConfiguration motorConfiguration(double gearRatio, boolean inverted) {
        return motorConfiguration(gearRatio, inverted, false);
    }

    static TalonFXConfiguration motorConfiguration(double gearRatio, boolean inverted, boolean floor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = gearRatio;
        config.Slot0.kP = IndexerConstants.VELOCITY_KP;
        config.Slot0.kV = IndexerConstants.MOTOR_KV * gearRatio;
        config.CurrentLimits.StatorCurrentLimit = floor ? IndexerConstants.FLOOR_STATOR_CURRENT_LIMIT : CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = floor ? IndexerConstants.FLOOR_SUPPLY_CURRENT_LIMIT : CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        return config;
    }

    private void stopFloorOutputs() {
        floorLeader.setControl(floorStopRequest);
        floorSecondary.setControl(floorFollowerStopRequest);
    }

    private static StatusCode configureMotor(TalonFX motor, double gearRatio, boolean inverted, boolean floor) {
        TalonFXConfiguration config = motorConfiguration(gearRatio, inverted, floor);
        StatusCode status = motor.getConfigurator().apply(config, 0.1);
        for (int attempt = 1; attempt < 3 && !status.isOK(); attempt++) {
            status = motor.getConfigurator().apply(config, 0.1);
        }
        return status;
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.floorStatorLimitAmps = IndexerConstants.FLOOR_STATOR_CURRENT_LIMIT;
        inputs.tunnelStatorLimitAmps = CURRENT_LIMIT_AMPS;
        inputs.floorCurrentLimitOK = floorConfigurationStatus.isOK() && floorSecondaryConfigurationStatus.isOK();
        inputs.tunnelCurrentLimitOK = tunnelConfigurationStatus.isOK();
        BaseStatusSignal.refreshAll(
                floorPosition,
                floorVelocity,
                floorVoltage,
                floorLeaderCurrent,
                floorSecondary.getVelocity(),
                floorSecondary.getMotorVoltage(),
                floorSecondary.getStatorCurrent(),
                tunnelPosition,
                tunnelVelocity,
                tunnelVoltage,
                tunnelCurrent);

        inputs.appliedVolts = tunnelVoltage.getValueAsDouble();
        inputs.floorRPM = floorVelocity.getValueAsDouble() * 60.0;
        inputs.tunnelConnected = tunnelMotor.isConnected();
        inputs.tunnelConfigured = tunnelConfigurationStatus.isOK();
        inputs.tunnelRPM = tunnelVelocity.getValueAsDouble() * 60.0;
        inputs.tunnelCurrentAmps = tunnelCurrent.getValueAsDouble();
        inputs.tunnelConfigurationStatus = tunnelConfigurationStatus.toString();
        inputs.tunnelControlStatus = tunnelControlStatus;
        inputs.currentAmps = floorLeaderCurrent.getValueAsDouble()
            + floorSecondary.getStatorCurrent().getValueAsDouble() + tunnelCurrent.getValueAsDouble();
        inputs.floorConnected = floorLeader.isConnected() && floorSecondary.isConnected();
        inputs.floorSecondaryConnected = floorSecondary.isConnected();
        inputs.floorSecondaryRPM = floorSecondary.getVelocity().getValueAsDouble() * 60;
        inputs.floorSecondaryAppliedVolts = floorSecondary.getMotorVoltage().getValueAsDouble();
        inputs.floorSecondaryCurrentAmps = floorSecondary.getStatorCurrent().getValueAsDouble();
        inputs.floorAppliedVolts = floorVoltage.getValueAsDouble();
        inputs.floorCurrentAmps = floorLeaderCurrent.getValueAsDouble();
        inputs.floorConfigurationStatus = "ID25=" + floorConfigurationStatus
            + "; ID0 follower=" + floorSecondaryConfigurationStatus;
        inputs.floorControlStatus = floorControlStatus;

        Logger.recordOutput("Indexer/Connected/FloorLeader", floorLeader.isConnected());
        Logger.recordOutput("Indexer/Connected/FloorSecondary", floorSecondary.isConnected());
        Logger.recordOutput("Indexer/Connected/TunnelMotor", tunnelMotor.isConnected());
    }

    @Override
    public void setFloorRPM(double rpm) {
        requestedFloorRPM = DriverStation.isEnabled() && Double.isFinite(rpm)
            && floorConfigurationStatus.isOK() && floorSecondaryConfigurationStatus.isOK() ? rpm : 0;
        rpm = requestedFloorRPM;
        if (rpm == 0.0) {
            stopFloorOutputs();
            floorControlStatus = "Stopped: leader and follower";
        } else {
            StatusCode follower = floorSecondary.setControl(floorFollowerRequest);
            if (!follower.isOK()) {
                stopFloorOutputs();
                floorControlStatus = "ID0 follower request failed: " + follower;
                return;
            }
            StatusCode leader = floorLeader.setControl(floorRequest.withVelocity(rpm / 60.0));
            floorControlStatus = "ID25=" + leader + "; ID0 follower=" + follower;
            if (!leader.isOK()) stopFloorOutputs();
        }
    }

    @Override
    public void setTunnelRPM(double rpm) {
        requestedTunnelRPM = DriverStation.isEnabled() && Double.isFinite(rpm)
            && tunnelConfigurationStatus.isOK() ? rpm : 0;
        rpm = requestedTunnelRPM;
        if (rpm == 0.0 || !Double.isFinite(rpm) || !tunnelConfigurationStatus.isOK()) {
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
