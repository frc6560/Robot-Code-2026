package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import frc.robot.Constants.StartupCurrentConstants;
import frc.robot.utility.StartupCurrentLimit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
    private final TalonFX tunnelMotor;
    private final StatusCode tunnelConfigurationStatus;

    private final VelocityVoltage floorRequest = new VelocityVoltage(0.0);
    private final VelocityVoltage tunnelRequest = new VelocityVoltage(0.0);
    private final NeutralOut floorStopRequest = new NeutralOut();
    private final NeutralOut tunnelStopRequest = new NeutralOut();
    private final StatusCode floorConfigurationStatus;
    private String floorControlStatus = "No request";
    private String tunnelControlStatus = "No request";
    private final StartupCurrentLimit floorLimit;
    private final StartupCurrentLimit tunnelLimit;
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
        // Temporary hardware setup: ID 0 is removed. Do not create or depend on its controller.
        // Keep the constructor argument for compatibility with existing wiring code.
        floorLeader = new TalonFX(floorLeaderId, canBus);
        tunnelMotor = new TalonFX(tunnelMotorId, canBus);

        floorConfigurationStatus = configureMotor(floorLeader, IndexerConstants.FLOOR_GEAR_RATIO,
            IndexerConstants.FLOOR_MOTOR_INVERTED, true);
        tunnelConfigurationStatus = configureMotor(tunnelMotor, IndexerConstants.TUNNEL_GEAR_RATIO, false, false);

        floorLimit = new StartupCurrentLimit(IndexerConstants.FLOOR_STATOR_CURRENT_LIMIT,
            StartupCurrentConstants.PEAK_AMPS, StartupCurrentConstants.DURATION_SECONDS,
            Timer::getFPGATimestamp, amps -> {
                CurrentLimitsConfigs limits = currentLimits(amps, IndexerConstants.FLOOR_SUPPLY_CURRENT_LIMIT);
                StatusCode first = floorLeader.getConfigurator().apply(limits, StartupCurrentConstants.CONFIG_TIMEOUT_SECONDS);
                return first.isOK();
            }, this::stopFloorOutputs);
        tunnelLimit = new StartupCurrentLimit(CURRENT_LIMIT_AMPS,
            StartupCurrentConstants.PEAK_AMPS, StartupCurrentConstants.DURATION_SECONDS,
            Timer::getFPGATimestamp,
            amps -> tunnelMotor.getConfigurator().apply(currentLimits(amps, CURRENT_LIMIT_AMPS),
                StartupCurrentConstants.CONFIG_TIMEOUT_SECONDS).isOK(),
            () -> tunnelMotor.setControl(tunnelStopRequest));

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
                tunnelPosition,
                tunnelVelocity,
                tunnelVoltage,
                tunnelCurrent);

        floorLeader.optimizeBusUtilization();
        tunnelMotor.optimizeBusUtilization();
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

    static CurrentLimitsConfigs currentLimits(double statorAmps, double supplyAmps) {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.StatorCurrentLimitEnable = true;
        limits.StatorCurrentLimit = statorAmps;
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = supplyAmps;
        return limits;
    }

    private void stopFloorOutputs() {
        floorLeader.setControl(floorStopRequest);
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
        // Also check the deadline here, even if no command issues another RPM request.
        floorLimit.update(DriverStation.isEnabled() && requestedFloorRPM != 0, false, true);
        tunnelLimit.update(DriverStation.isEnabled() && requestedTunnelRPM != 0, false, true);
        inputs.floorStatorLimitAmps = floorLimit.appliedAmps();
        inputs.tunnelStatorLimitAmps = tunnelLimit.appliedAmps();
        inputs.floorCurrentLimitOK = floorLimit.confirmed();
        inputs.tunnelCurrentLimitOK = tunnelLimit.confirmed();
        BaseStatusSignal.refreshAll(
                floorPosition,
                floorVelocity,
                floorVoltage,
                floorLeaderCurrent,
                tunnelPosition,
                tunnelVelocity,
                tunnelVoltage,
                tunnelCurrent);

        inputs.appliedVolts = tunnelVoltage.getValueAsDouble();
        inputs.floorRPM = floorVelocity.getValueAsDouble() * 60.0;
        inputs.tunnelConnected = tunnelMotor.isConnected();
        inputs.tunnelConfigured = tunnelConfigurationStatus.isOK() && tunnelLimit.confirmed();
        inputs.tunnelRPM = tunnelVelocity.getValueAsDouble() * 60.0;
        inputs.tunnelCurrentAmps = tunnelCurrent.getValueAsDouble();
        inputs.tunnelConfigurationStatus = tunnelConfigurationStatus.toString();
        inputs.tunnelControlStatus = tunnelControlStatus;
        inputs.currentAmps = floorLeaderCurrent.getValueAsDouble() + tunnelCurrent.getValueAsDouble();
        inputs.floorConnected = floorLeader.isConnected();
        inputs.floorSecondaryConnected = false;
        inputs.floorSecondaryRPM = 0;
        inputs.floorSecondaryAppliedVolts = 0;
        inputs.floorSecondaryCurrentAmps = 0;
        inputs.floorAppliedVolts = floorVoltage.getValueAsDouble();
        inputs.floorCurrentAmps = floorLeaderCurrent.getValueAsDouble();
        inputs.floorConfigurationStatus = "ID25=" + floorConfigurationStatus + "; ID0 removed";
        inputs.floorControlStatus = floorControlStatus;

        Logger.recordOutput("Indexer/Connected/FloorLeader", floorLeader.isConnected());
        Logger.recordOutput("Indexer/Connected/FloorSecondary", false);
        Logger.recordOutput("Indexer/Connected/TunnelMotor", tunnelMotor.isConnected());
    }

    @Override
    public void setFloorRPM(double rpm) {
        requestedFloorRPM = DriverStation.isEnabled() && Double.isFinite(rpm)
            && floorConfigurationStatus.isOK() ? rpm : 0;
        if (!floorLimit.update(requestedFloorRPM != 0, false, true)) {
            floorControlStatus = "Current-limit update failed; motors stopped";
            return;
        }
        rpm = requestedFloorRPM;
        if (rpm == 0.0 || !Double.isFinite(rpm) || !floorConfigurationStatus.isOK()) {
            floorControlStatus = "Stopped: ID25=" + floorLeader.setControl(floorStopRequest);
        } else {
            StatusCode status = floorLeader.setControl(floorRequest.withVelocity(rpm / 60.0));
            floorControlStatus = "ID25=" + status;
            if (!status.isOK()) floorLeader.setControl(floorStopRequest);
        }
    }

    @Override
    public void setTunnelRPM(double rpm) {
        requestedTunnelRPM = DriverStation.isEnabled() && Double.isFinite(rpm)
            && tunnelConfigurationStatus.isOK() ? rpm : 0;
        if (!tunnelLimit.update(requestedTunnelRPM != 0, false, true)) {
            tunnelControlStatus = "Current-limit update failed; motor stopped";
            return;
        }
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
