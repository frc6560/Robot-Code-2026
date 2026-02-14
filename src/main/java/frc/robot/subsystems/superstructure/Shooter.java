// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

// --- SYSID IMPORTS ---
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut; 
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utility.Shooter.ShotCalculator;

public class Shooter extends SubsystemBase {
    // --- VISUALIZATION ---
    private final Mechanism2d mech2d;
    private final MechanismRoot2d flywheelRoot;
    private final MechanismLigament2d leftFlywheelVisual;
    private final MechanismLigament2d rightFlywheelVisual;
    private double visualAngle = 0.0;

    // --- HARDWARE ---
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final NetworkTable limelightTable;

    // --- CONTROL ---
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut coastControl = new NeutralOut(); 
    
    // --- SYSID COMPONENTS ---
    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine sysIdRoutine;

    // --- STATE ---
    private double targetRPS = 0.0;
    private final PoseSupplier poseSupplier;

    public interface PoseSupplier { Pose2d getPose(); }

    /** Creates a new Flywheel. */
    public Shooter(PoseSupplier poseSupplier) {
        this.poseSupplier = poseSupplier;
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        // 1. Initialize Motors
        leaderMotor = new TalonFX(ShooterConstants.LEFT_FLYWHEEL_ID, "rio");
        followerMotor = new TalonFX(ShooterConstants.RIGHT_FLYWHEEL_ID, "rio");

        configureLeaderMotor();
        configureFollowerMotor();

        // 2. SysId Setup
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second), 
                Volts.of(4),    
                null,           
                (state) -> SignalLogger.writeString("SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> leaderMotor.setControl(sysIdControl.withOutput(voltage.in(Volts))),
                null,
                this
            )
        );

        // 3. Visualization Setup
        mech2d = new Mechanism2d(200, 200);
        flywheelRoot = mech2d.getRoot("Flywheel Root", 100, 100);
        leftFlywheelVisual = flywheelRoot.append(new MechanismLigament2d("Left Flywheel", 50, 0));
        leftFlywheelVisual.setColor(new Color8Bit(0, 0, 255));
        rightFlywheelVisual = flywheelRoot.append(new MechanismLigament2d("Right Flywheel", 50, 0));
        rightFlywheelVisual.setColor(new Color8Bit(255, 0, 0));
        SmartDashboard.putData("Flywheel Mechanism", mech2d);
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
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;

        // UPDATED: Faster Ramp (0.02s) for responsive shooting
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02; 

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

    // --- SYSID METHODS ---
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    // --- CONTROL METHODS ---

    public void setRPS(double rps) {
        targetRPS = rps;
        
        if (Math.abs(rps) < 1.0) { // Increased deadband slightly
            // Smart Stop: Coast logic
            leaderMotor.setControl(coastControl);
        } else {
            // Normal Run
            double motorRPS = rps * ShooterConstants.FLYWHEEL_GEAR_RATIO;
            leaderMotor.setControl(velocityControl.withVelocity(motorRPS));
        }
    }

    public void setRPM(double rpm) {
        setRPS(rpm / 60.0);
    }

    public void setRPMFromCalculator(ShotCalculator calculator) {
        setRPM(calculator.getFlywheelRPM());
    }

    public void setIdle() {
        setRPM(ShooterConstants.FLYWHEEL_IDLE_RPM);
    }

    public void stop() {
        targetRPS = 0.0;
        leaderMotor.setControl(coastControl);
    }

    public double getCurrentRPS() {
        double motorRPS = leaderMotor.getVelocity().getValueAsDouble();
        return motorRPS / ShooterConstants.FLYWHEEL_GEAR_RATIO;
    }

    public double getCurrentRPM() {
        return getCurrentRPS() * 60.0;
    }

    public boolean atTarget() {
        // FIXED: Cannot be "at target" if we are stopped/idling
        if (Math.abs(targetRPS) < 1.0) return false;
        return Math.abs(getCurrentRPS() - targetRPS) < (ShooterConstants.FLYWHEEL_RPM_TOLERANCE / 60.0);
    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public Pose2d getRobotPose() {
        return poseSupplier.getPose();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Flywheel/Periodic Running", true);
        
        SmartDashboard.putNumber("Flywheel/Current RPS", getCurrentRPS());
        SmartDashboard.putNumber("Flywheel/Current RPM", getCurrentRPM());
        SmartDashboard.putNumber("Flywheel/Target RPS", targetRPS);
        SmartDashboard.putBoolean("Flywheel/At Target", atTarget());

        SmartDashboard.putNumber("Flywheel/Leader Voltage", leaderMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Leader Current", leaderMotor.getSupplyCurrent().getValueAsDouble());

        // Visualization
        double rpmToUse = targetRPS * 60.0; 
        if (Math.abs(rpmToUse) < 1.0) rpmToUse = 1000.0;
        
        double degPerSec = rpmToUse * 360.0 / 60.0; 
        double degPerTick = degPerSec * 0.02;
        visualAngle += degPerTick;
        visualAngle %= 360.0;

        leftFlywheelVisual.setAngle(visualAngle);
        rightFlywheelVisual.setAngle(-visualAngle);
        SmartDashboard.putNumber("Flywheel/Visual Angle", visualAngle);
    }
}