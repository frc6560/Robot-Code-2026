// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.ShotCalculator;

public class Flywheel extends SubsystemBase {
    // Mechanism2D visualization components
    private final Mechanism2d mech2d;
    private final MechanismRoot2d flywheelRoot;
    private final MechanismLigament2d leftFlywheelVisual;
    private final MechanismLigament2d rightFlywheelVisual;

    // Motors - leader and follower
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;

    // Control
    private final VelocityVoltage velocityControl;
    private final NetworkTable limelightTable;

    // Pose supplier
    private final PoseSupplier poseSupplier;

    // Get pose from swerve subsystem
    public interface PoseSupplier {
        Pose2d getPose();
    }

    // State
    private double targetRPM = 0.0;
    private double visualAngle = 0.0;

    /** Creates a new Flywheel. */
    public Flywheel(PoseSupplier poseSupplier) {
        this.poseSupplier = poseSupplier;

        // Initialize limelight network table
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        // Initialize motors - left is leader, right is follower
        leaderMotor = new TalonFX(FlywheelConstants.LEFT_FLYWHEEL_ID, "rio");
        followerMotor = new TalonFX(FlywheelConstants.RIGHT_FLYWHEEL_ID, "rio");

        // Configure motors
        configureLeaderMotor();
        configureFollowerMotor();

        // Initialize Mechanism2D (200x200 px)
        mech2d = new Mechanism2d(200, 200);

        // Add a root point for the flywheel at the center
        flywheelRoot = mech2d.getRoot("Flywheel Root", 100, 100);

        // Create ligaments representing the flywheels
        leftFlywheelVisual = flywheelRoot.append(
            new MechanismLigament2d("Left Flywheel", 50, 0));
        leftFlywheelVisual.setColor(new Color8Bit(0, 0, 255));

        rightFlywheelVisual = flywheelRoot.append(
            new MechanismLigament2d("Right Flywheel", 50, 0));
        rightFlywheelVisual.setColor(new Color8Bit(255, 0, 0));

        // Push the Mechanism2D to SmartDashboard
        SmartDashboard.putData("Flywheel Mechanism", mech2d);

        // Initialize control
        velocityControl = new VelocityVoltage(0.0).withSlot(0);
    }

    /**
     * Configure the leader motor with PID, inversion, and current limits
     */
    private void configureLeaderMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID configuration
        config.Slot0.kP = FlywheelConstants.kP;
        config.Slot0.kI = FlywheelConstants.kI;
        config.Slot0.kD = FlywheelConstants.kD;
        config.Slot0.kV = FlywheelConstants.kV;
        config.Slot0.kS = FlywheelConstants.kS; // Add kS if you have it in constants

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Set inversion ONCE in configuration
        config.MotorOutput.Inverted = FlywheelConstants.LEFT_FLYWHEEL_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        // Current limits - both stator and supply
        config.CurrentLimits.StatorCurrentLimitEnable = FlywheelConstants.FLYWHEEL_STATOR_CURRENT_LIMIT > 0;
        config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = FlywheelConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT > 0;
        config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;

        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;

        // Apply configuration
        leaderMotor.getConfigurator().apply(config);
    }

    /**
     * Configure the follower motor to follow the leader
     */
    private void configureFollowerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limits (same as leader)
        config.CurrentLimits.StatorCurrentLimitEnable = FlywheelConstants.FLYWHEEL_STATOR_CURRENT_LIMIT > 0;
        config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = FlywheelConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT > 0;
        config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;

        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;

        // Apply configuration
        followerMotor.getConfigurator().apply(config);

        // Set follower to follow leader (opposed means it spins opposite direction)
        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * Set flywheel RPM from ShotCalculator
     * @param calculator The ShotCalculator instance
     */
    public void setRPMFromCalculator(ShotCalculator calculator) {
        setRPM(calculator.getFlywheelRPM());
    }

    /**
     * Set flywheel to idle speed
     */
    public void setIdle() {
        setRPM(FlywheelConstants.FLYWHEEL_IDLE_RPM);
    }

    /**
     * Set target RPM for the flywheel
     * @param rpm Target RPM (flywheel output speed, not motor speed)
     */
    public void setRPM(double rpm) {
        targetRPM = rpm;
        // Convert to motor velocity accounting for gear ratio
        // DIRECT RPS: We want 70 input -> 70 output
  double motorRPS = rpm * FlywheelConstants.FLYWHEEL_GEAR_RATIO;

        // Send command to leader motor only - follower automatically follows
        leaderMotor.setControl(velocityControl.withVelocity(motorRPS));
    }

    /**
     * Set flywheel to direct percent output (for testing)
     * @param percent Percent output (-1.0 to 1.0)
     */
    public void setPercent(double percent) {
        leaderMotor.set(percent);
    }

    /**
     * Stop the flywheel
     */
    public void stop() {
        targetRPM = 0.0;
        leaderMotor.stopMotor();
        // Re-establish follower relationship after stop
        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * Get current flywheel RPM (output speed, not motor speed)
     */
    public double getCurrentRPM() {
        double motorRPS = leaderMotor.getVelocity().getValueAsDouble();
        return motorRPS / FlywheelConstants.FLYWHEEL_GEAR_RATIO;
    }

    /**
     * Get target flywheel RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Check if flywheel is at target speed
     */
    public boolean atTargetRPM() {
        double currentRPM = getCurrentRPM();
        return Math.abs(currentRPM - targetRPM) < FlywheelConstants.FLYWHEEL_RPM_TOLERANCE;
    }

    /**
     * Gets current robot pose (for debugging)
     */
    public Pose2d getRobotPose() {
        return poseSupplier.getPose();
    }

    /**
     * Check if limelight has a valid target
     */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    @Override
    public void periodic() {
        // Debug: Verify periodic is being called
        SmartDashboard.putBoolean("Flywheel/Periodic Running", true);

        // Telemetry for AdvantageScope and SmartDashboard
        SmartDashboard.putNumber("Flywheel/Current RPM", getCurrentRPM());
        SmartDashboard.putNumber("Flywheel/Target RPM", targetRPM);
        SmartDashboard.putBoolean("Flywheel/At Target", atTargetRPM());
        SmartDashboard.putBoolean("Flywheel/Has Target", hasTarget());
        SmartDashboard.putNumber("Flywheel/Leader Voltage", leaderMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Leader Output", leaderMotor.get());
        SmartDashboard.putNumber("Flywheel/Follower Output", followerMotor.get());
        SmartDashboard.putNumber("Flywheel/Leader Current", leaderMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Follower Current", followerMotor.getSupplyCurrent().getValueAsDouble());

        // Automatic demo spin for visualization
        double rpmToUse = targetRPM;
        // If targetRPM is zero, use demo RPM for visualization
        if (Math.abs(rpmToUse) < 1e-6) {
            rpmToUse = 1000.0;
        }

        // Compute incremental rotation (periodic runs ~20ms per tick)
        double degPerSec = rpmToUse * 360.0 / 60.0; // 360° per revolution
        double degPerTick = degPerSec * 0.02; // 20ms tick
        visualAngle += degPerTick;
        visualAngle %= 360.0;

        leftFlywheelVisual.setAngle(visualAngle);
        rightFlywheelVisual.setAngle(-visualAngle); // opposite for variety

        // Debug: expose visual angle
        SmartDashboard.putNumber("Flywheel/Visual Angle", visualAngle);
    }
}