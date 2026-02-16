package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.*;

public class Turret extends SubsystemBase {
    private final TalonFX m_turretMotor = new TalonFX(TurretConstants.MOTOR_ID, "rio");
    private final CANcoder m_turretEncoder = new CANcoder(TurretConstants.ENCODER_ID, "rio");

    private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    private double m_goalDegrees = 0.0;

    public Turret() {
        // Configure TalonFX
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // PID and feedforward configuration
        Slot0Configs slot0 = talonFXConfigs.Slot0;
        slot0.kS = TurretConstants.kS;
        slot0.kV = TurretConstants.kV;
        slot0.kA = TurretConstants.kA;
        slot0.kP = TurretConstants.kP;
        slot0.kI = TurretConstants.kI;
        slot0.kD = TurretConstants.kD;

        // Motion Magic configuration (values are in motor rotations and rotations/s)
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = TurretConstants.kMaxV * TurretConstants.MOTOR_GEAR_RATIO / 360.0; // deg/s -> rot/s
        motionMagicConfigs.MotionMagicAcceleration = TurretConstants.kMaxA * TurretConstants.MOTOR_GEAR_RATIO / 360.0;   // deg/s^2 -> rot/s^2
        motionMagicConfigs.MotionMagicJerk = 0; // 0 = trapezoidal (no jerk limit)

        // Set motor to brake mode
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = TurretConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40; // Amps

        m_turretMotor.getConfigurator().apply(talonFXConfigs);

        // Configure CANcoder
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = TurretConstants.ABSOLUTE_ENCODER_OFFSET;
        cancoderConfigs.MagnetSensor.SensorDirection = TurretConstants.ABSOLUTE_ENCODER_REVERSED
            ? com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive
            : com.ctre.phoenix6.signals.SensorDirectionValue.Clockwise_Positive;

        m_turretEncoder.getConfigurator().apply(cancoderConfigs);

        seedMotorEncoder();
    }

    /**
     * Seeds the motor encoder from the CANcoder absolute position.
     * Accounts for gear ratios between CANcoder, turret, and motor.
     */
    public void seedMotorEncoder() {
        Timer.delay(0.25); // Wait for CANcoder to stabilize
        double cancoderRotations = m_turretEncoder.getAbsolutePosition().getValueAsDouble();
        double turretRotations = cancoderRotations / TurretConstants.ENCODER_GEAR_RATIO;
        double motorRotations = turretRotations * TurretConstants.MOTOR_GEAR_RATIO;
        m_turretMotor.setPosition(motorRotations);
    }

    /**
     * Set the target angle for the turret.
     * @param goal Target angle in degrees
     */
    public void setGoal(double goal) {
        m_goalDegrees = MathUtil.clamp(goal, TurretConstants.LOWER_SOFT_LIMIT, TurretConstants.UPPER_SOFT_LIMIT);
    }

    public double getGoalDegrees() {
        return m_goalDegrees;
    }

    /** Gets the turret angle in degrees */
    public double getTurretAngle() {
        double motorRotations = m_turretMotor.getPosition().getValueAsDouble();
        return motorRotations * 360.0 / TurretConstants.MOTOR_GEAR_RATIO;
    }

    /** Gets the turret velocity in degrees per second */
    public double getTurretVelocity() {
        double motorRotPerSec = m_turretMotor.getVelocity().getValueAsDouble();
        return motorRotPerSec * 360.0 / TurretConstants.MOTOR_GEAR_RATIO;
    }

    public void stopMotor() {
        m_turretMotor.set(0);
    }

    @Override
    public void periodic() {
        // logs!
        SmartDashboard.putNumber("Turret/Current Angle (deg)", getTurretAngle());
        SmartDashboard.putNumber("Turret/Goal Angle (deg)", m_goalDegrees);
        SmartDashboard.putNumber("Turret/Velocity (deg per s)", getTurretVelocity());
        SmartDashboard.putNumber("Turret/Absolute Encoder (rots)", m_turretEncoder.getAbsolutePosition().getValueAsDouble());
        //runMotionMagic();
    }

    /** Runs Motion Magic to the current goal position. */
    private void runMotionMagic() {
        double targetMotorRotations = m_goalDegrees * TurretConstants.MOTOR_GEAR_RATIO / 360.0;
        m_turretMotor.setControl(m_motionMagicRequest.withPosition(targetMotorRotations));
    }
}
