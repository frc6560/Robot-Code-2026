package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOKraken implements ClimbIO {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final DigitalInput retractLimitSwitch;

    private final MotionMagicVoltage motionMagicReq = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageReq = new VoltageOut(0);

    // Signals
    private final StatusSignal<Angle> leaderPos, followerPos;
    private final StatusSignal<AngularVelocity> leaderVel, followerVel;
    private final StatusSignal<Voltage> leaderVolts, followerVolts;
    private final StatusSignal<Current> leaderCurrent, followerCurrent;
    private final StatusSignal<Temperature> leaderTemp, followerTemp;

    public ClimbIOKraken() {
        leaderMotor = new TalonFX(ClimbConstants.LEFT_MOTOR_ID, ClimbConstants.CAN_BUS);
        followerMotor = new TalonFX(ClimbConstants.RIGHT_MOTOR_ID, ClimbConstants.CAN_BUS);
        retractLimitSwitch = new DigitalInput(ClimbConstants.RETRACT_LIMIT_SWITCH_DIO);

        configureLeaderMotor();
        configureFollowerMotor();

        leaderPos = leaderMotor.getPosition(); followerPos = followerMotor.getPosition();
        leaderVel = leaderMotor.getVelocity(); followerVel = followerMotor.getVelocity();
        leaderVolts = leaderMotor.getMotorVoltage(); followerVolts = followerMotor.getMotorVoltage();
        leaderCurrent = leaderMotor.getSupplyCurrent(); followerCurrent = followerMotor.getSupplyCurrent();
        leaderTemp = leaderMotor.getDeviceTemp(); followerTemp = followerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
            leaderPos, followerPos, leaderVel, followerVel, leaderVolts, followerVolts, leaderCurrent, followerCurrent, leaderTemp, followerTemp
        );

        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }

    private void configureLeaderMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = ClimbConstants.LEFT_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.CLIMB_SOFT_LIMIT_FORWARD;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.CLIMB_SOFT_LIMIT_REVERSE;
        
        config.Slot0.kP = ClimbConstants.kP;
        config.Slot0.kI = ClimbConstants.kI;
        config.Slot0.kD = ClimbConstants.kD;
        config.Slot0.kV = ClimbConstants.kV;
        config.Slot0.kG = ClimbConstants.kG;
        config.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.CRUISE_VELOCITY_RPS;
        config.MotionMagic.MotionMagicAcceleration = ClimbConstants.ACCELERATION_RPS2;

        leaderMotor.getConfigurator().apply(config);
        leaderMotor.setPosition(0.0);
    }

    private void configureFollowerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;

        followerMotor.getConfigurator().apply(config);
        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Aligned)); 
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        BaseStatusSignal.refreshAll(leaderPos, followerPos, leaderVel, followerVel, leaderVolts, followerVolts, leaderCurrent, followerCurrent, leaderTemp, followerTemp);

        inputs.leftPositionRotations = leaderPos.getValueAsDouble();
        inputs.rightPositionRotations = followerPos.getValueAsDouble();
        inputs.leftVelocityRPS = leaderVel.getValueAsDouble();
        inputs.rightVelocityRPS = followerVel.getValueAsDouble();

        inputs.appliedVolts[0] = leaderVolts.getValueAsDouble(); inputs.appliedVolts[1] = followerVolts.getValueAsDouble();
        inputs.currentAmps[0] = leaderCurrent.getValueAsDouble(); inputs.currentAmps[1] = followerCurrent.getValueAsDouble();
        inputs.tempCelsius[0] = leaderTemp.getValueAsDouble(); inputs.tempCelsius[1] = followerTemp.getValueAsDouble();

        boolean rawSwitch = retractLimitSwitch.get();
        inputs.retractLimitSwitch = ClimbConstants.RETRACT_LIMIT_SWITCH_INVERTED ? !rawSwitch : rawSwitch;
    }

    @Override
    public void setTarget(double target) { leaderMotor.setControl(motionMagicReq.withPosition(target)); }
    
    @Override
    public void setPercent(double pct) { leaderMotor.set(pct); }

    @Override
    public void setVoltage(double volts) { leaderMotor.setControl(voltageReq.withOutput(volts)); }

    @Override
    public void zeroPosition() { leaderMotor.setPosition(0.0); }

    @Override
    public void setSoftLimitsEnabled(boolean enabled) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        leaderMotor.getConfigurator().refresh(config);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enabled;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enabled;
        leaderMotor.getConfigurator().apply(config);
    }
}
