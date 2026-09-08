package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class IndexerIOTalonFX implements IndexerIO {
    private static final double FLOOR_GEAR_RATIO = 2.5;
    private static final double TOWER_GEAR_RATIO = 5.0 / 3.0;
    private static final double CURRENT_LIMIT_AMPS = 40.0;
    private static final double STATUS_FREQUENCY_HZ = 50.0;
    private static final double GAME_PIECE_DISTANCE_METERS = 0.1;

    private final TalonFX floorLeader;
    private final TalonFX floorFollower;
    private final TalonFX towerMotor;
    private final CANrange towerSensor;

    private final DutyCycleOut floorRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut towerRequest = new DutyCycleOut(0.0);

    private final StatusSignal<Angle> floorPosition;
    private final StatusSignal<AngularVelocity> floorVelocity;
    private final StatusSignal<Voltage> floorVoltage;
    private final StatusSignal<Current> floorLeaderCurrent;
    private final StatusSignal<Current> floorFollowerCurrent;

    private final StatusSignal<Angle> towerPosition;
    private final StatusSignal<AngularVelocity> towerVelocity;
    private final StatusSignal<Voltage> towerVoltage;
    private final StatusSignal<Current> towerCurrent;
    private final StatusSignal<Distance> towerDistance;

    public IndexerIOTalonFX(
            int floorLeaderId,
            int floorFollowerId,
            int towerMotorId,
            int towerSensorId) {
        this(floorLeaderId, floorFollowerId, towerMotorId, towerSensorId, CANBus.roboRIO());
    }

    public IndexerIOTalonFX(
            int floorLeaderId,
            int floorFollowerId,
            int towerMotorId,
            int towerSensorId,
            CANBus canBus) {
        floorLeader = new TalonFX(floorLeaderId, canBus);
        floorFollower = new TalonFX(floorFollowerId, canBus);
        towerMotor = new TalonFX(towerMotorId, canBus);
        towerSensor = new CANrange(towerSensorId, canBus);

        configureMotor(floorLeader, FLOOR_GEAR_RATIO);
        configureMotor(floorFollower, FLOOR_GEAR_RATIO);
        configureMotor(towerMotor, TOWER_GEAR_RATIO);
        floorFollower.setControl(new StrictFollower(floorLeaderId));

        floorPosition = floorLeader.getPosition();
        floorVelocity = floorLeader.getVelocity();
        floorVoltage = floorLeader.getMotorVoltage();
        floorLeaderCurrent = floorLeader.getStatorCurrent();
        floorFollowerCurrent = floorFollower.getStatorCurrent();

        towerPosition = towerMotor.getPosition();
        towerVelocity = towerMotor.getVelocity();
        towerVoltage = towerMotor.getMotorVoltage();
        towerCurrent = towerMotor.getStatorCurrent();
        towerDistance = towerSensor.getDistance();

        BaseStatusSignal.setUpdateFrequencyForAll(
                STATUS_FREQUENCY_HZ,
                floorPosition,
                floorVelocity,
                floorVoltage,
                floorLeaderCurrent,
                floorFollowerCurrent,
                towerPosition,
                towerVelocity,
                towerVoltage,
                towerCurrent,
                towerDistance);

        floorLeader.optimizeBusUtilization();
        floorFollower.optimizeBusUtilization();
        towerMotor.optimizeBusUtilization();
        towerSensor.optimizeBusUtilization();
    }

    private static void configureMotor(TalonFX motor, double gearRatio) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = gearRatio;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                floorPosition,
                floorVelocity,
                floorVoltage,
                floorLeaderCurrent,
                floorFollowerCurrent,
                towerPosition,
                towerVelocity,
                towerVoltage,
                towerCurrent,
                towerDistance);

        inputs.appliedVolts = towerVoltage.getValueAsDouble();
        inputs.currentAmps = floorLeaderCurrent.getValueAsDouble()
                + floorFollowerCurrent.getValueAsDouble()
                + towerCurrent.getValueAsDouble();
        inputs.sensorTriggered = towerDistance.getValueAsDouble() <= GAME_PIECE_DISTANCE_METERS;

        Logger.recordOutput("Indexer/Connected/FloorLeader", floorLeader.isConnected());
        Logger.recordOutput("Indexer/Connected/FloorFollower", floorFollower.isConnected());
        Logger.recordOutput("Indexer/Connected/TowerMotor", towerMotor.isConnected());
        Logger.recordOutput("Indexer/Connected/TowerSensor", towerSensor.isConnected());
    }

    @Override
    public void setSpeed(double dutyCycle) {
        floorLeader.setControl(floorRequest.withOutput(dutyCycle));
        towerMotor.setControl(towerRequest.withOutput(dutyCycle));
    }

    @Override
    public void stop() {
        floorLeader.setControl(floorRequest.withOutput(0.0));
        towerMotor.setControl(towerRequest.withOutput(0.0));
    }
}
