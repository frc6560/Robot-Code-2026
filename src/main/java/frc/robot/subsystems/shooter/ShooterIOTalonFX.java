package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final NeutralOut coastControl = new NeutralOut();

    private String controlMode = "Off";

    private final StatusSignal<Angle> leaderPosition;
    private final StatusSignal<AngularVelocity> leaderVelocity;
    private final StatusSignal<Voltage> leaderVoltage;
    private final StatusSignal<Current> leaderCurrent;
    private final StatusSignal<Temperature> leaderTemp;

    private final StatusSignal<AngularVelocity> followerVelocity;
    private final StatusSignal<Voltage> followerVoltage;
    private final StatusSignal<Current> followerCurrent;
    private final StatusSignal<Temperature> followerTemp;

    public ShooterIOTalonFX() {
        leaderMotor = new TalonFX(ShooterConstants.LEFT_FLYWHEEL_ID, "rio");
        followerMotor = new TalonFX(ShooterConstants.RIGHT_FLYWHEEL_ID, "rio");

        configureLeaderMotor();
        configureFollowerMotor();

        leaderPosition = leaderMotor.getPosition();
        leaderVelocity = leaderMotor.getVelocity();
        leaderVoltage = leaderMotor.getMotorVoltage();
        leaderCurrent = leaderMotor.getSupplyCurrent();
        leaderTemp = leaderMotor.getDeviceTemp();

        followerVelocity = followerMotor.getVelocity();
        followerVoltage = followerMotor.getMotorVoltage();
        followerCurrent = followerMotor.getSupplyCurrent();
        followerTemp = followerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            leaderPosition, leaderVelocity, leaderVoltage, leaderCurrent, leaderTemp,
            followerVelocity, followerVoltage, followerCurrent, followerTemp
        );

        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }

    private void configureLeaderMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = ShooterConstants.kP;
        config.Slot0.kI = ShooterConstants.kI;
        config.Slot0.kD = ShooterConstants.kD;
        config.Slot0.kV = ShooterConstants.kV;
        config.Slot0.kS = ShooterConstants.kS;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = ShooterConstants.LEFT_FLYWHEEL_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;

        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

        leaderMotor.getConfigurator().apply(config);
    }

    private void configureFollowerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;

        followerMotor.getConfigurator().apply(config);
        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leaderPosition, leaderVelocity, leaderVoltage, leaderCurrent, leaderTemp,
            followerVelocity, followerVoltage, followerCurrent, followerTemp
        );

        inputs.leaderPositionRotations = leaderPosition.getValueAsDouble();
        inputs.leaderVelocityRPS = leaderVelocity.getValueAsDouble();
        inputs.leaderAppliedVolts = leaderVoltage.getValueAsDouble();
        inputs.leaderCurrentAmps = leaderCurrent.getValueAsDouble();
        inputs.leaderTempCelsius = leaderTemp.getValueAsDouble();

        inputs.followerVelocityRPS = followerVelocity.getValueAsDouble();
        inputs.followerAppliedVolts = followerVoltage.getValueAsDouble();
        inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();
        inputs.followerTempCelsius = followerTemp.getValueAsDouble();

        inputs.controlMode = controlMode;
    }

    @Override
    public void setVelocityRPS(double rps) {
        double rpsTolerance = ShooterConstants.FLYWHEEL_RPM_TOLERANCE / 60.0;
        if (Math.abs(rps) < 1.0) {
            controlMode = "Off";
            leaderMotor.setControl(coastControl);
            return;
        }

        // Convert motor velocity to mechanism velocity for comparison
        double currentMechanismRPS = leaderMotor.getVelocity().getValueAsDouble() * ShooterConstants.FLYWHEEL_GEAR_RATIO;

        // Applies bang bang control, unless we're within tolerance.
        if (currentMechanismRPS < rps - rpsTolerance) {
            // Below target - full power
            controlMode = "BangBang_FullPower";
            leaderMotor.setControl(voltageControl.withOutput(12.0));
        } else if (currentMechanismRPS > rps + rpsTolerance) {
            // Above target - coast
            controlMode = "BangBang_Coast";
            leaderMotor.setControl(voltageControl.withOutput(0.0));
        } else {
            // Within tolerance - velocity PID
            controlMode = "VelocityPID";
            double motorRPS = rps / ShooterConstants.FLYWHEEL_GEAR_RATIO;
            leaderMotor.setControl(velocityControl.withVelocity(motorRPS));
        }
    }

    @Override
    public void setVoltage(double volts) {
        leaderMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void stop() {
        leaderMotor.setControl(coastControl);
    }
}
