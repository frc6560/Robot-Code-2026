package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOSim implements ClimbIO {
    private final DCMotorSim leftSim;
    private final DCMotorSim rightSim;
    
    private final ProfiledPIDController leftMotionMagicSim;
    private final ProfiledPIDController rightMotionMagicSim;
    
    private boolean leftClosedLoop = false;
    private boolean rightClosedLoop = false;
    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double leftTargetPosition = 0.0;
    private double rightTargetPosition = 0.0;

    private static final double CLIMB_GEARING = 25.0; 
    private static final double CLIMB_MOI = 0.005;

    public ClimbIOSim() {
        leftSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), CLIMB_MOI, CLIMB_GEARING), DCMotor.getKrakenX60(1));
        rightSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), CLIMB_MOI, CLIMB_GEARING), DCMotor.getKrakenX60(1));

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(ClimbConstants.CRUISE_VELOCITY_RPS, ClimbConstants.ACCELERATION_RPS2);
        leftMotionMagicSim = new ProfiledPIDController(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD, constraints);
        rightMotionMagicSim = new ProfiledPIDController(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD, constraints);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        if (leftClosedLoop) {
            double currentPosition = leftSim.getAngularPositionRad() / (2.0 * Math.PI);
            double pidOutput = leftMotionMagicSim.calculate(currentPosition, leftTargetPosition);
            double ffOutput = ClimbConstants.kV * leftMotionMagicSim.getSetpoint().velocity;
            leftAppliedVolts = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
        }
        
        if (rightClosedLoop) {
            double currentPosition = rightSim.getAngularPositionRad() / (2.0 * Math.PI);
            double pidOutput = rightMotionMagicSim.calculate(currentPosition, rightTargetPosition);
            double ffOutput = ClimbConstants.kV * rightMotionMagicSim.getSetpoint().velocity;
            rightAppliedVolts = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
        }

        leftSim.setInputVoltage(leftAppliedVolts);
        rightSim.setInputVoltage(rightAppliedVolts);
        leftSim.update(0.02);
        rightSim.update(0.02);

        inputs.leftPositionRotations = leftSim.getAngularPositionRad() / (2.0 * Math.PI);
        inputs.rightPositionRotations = rightSim.getAngularPositionRad() / (2.0 * Math.PI);
        inputs.leftVelocityRPS = leftSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        inputs.rightVelocityRPS = rightSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        
        inputs.appliedVolts[0] = leftAppliedVolts; inputs.appliedVolts[1] = rightAppliedVolts;
        inputs.currentAmps[0] = Math.abs(leftSim.getCurrentDrawAmps()); inputs.currentAmps[1] = Math.abs(rightSim.getCurrentDrawAmps());
        inputs.tempCelsius[0] = 25.0; inputs.tempCelsius[1] = 25.0;

        inputs.leftLimitSwitch = inputs.leftPositionRotations <= 0.05;
        inputs.rightLimitSwitch = inputs.rightPositionRotations <= 0.05;
    }

    @Override
    public void setLeftTarget(double target) { leftClosedLoop = true; leftTargetPosition = target; }
    @Override
    public void setRightTarget(double target) { rightClosedLoop = true; rightTargetPosition = target; }
    @Override
    public void setLeftPercent(double pct) { leftClosedLoop = false; leftAppliedVolts = MathUtil.clamp(pct * 12.0, -12.0, 12.0); }
    @Override
    public void setRightPercent(double pct) { rightClosedLoop = false; rightAppliedVolts = MathUtil.clamp(pct * 12.0, -12.0, 12.0); }

    @Override
    public void resetLeftPosition() {
        leftSim.setState(0.0, leftSim.getAngularVelocityRadPerSec());
        leftMotionMagicSim.reset(0.0, leftSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI));
    }
    
    @Override
    public void resetRightPosition() {
        rightSim.setState(0.0, rightSim.getAngularVelocityRadPerSec());
        rightMotionMagicSim.reset(0.0, rightSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI));
    }
}